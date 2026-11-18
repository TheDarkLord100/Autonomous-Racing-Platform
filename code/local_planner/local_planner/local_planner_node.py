#!/usr/bin/env python3
"""Local planner: keep the car off the walls the LIVE SCAN sees, staying on the raceline
whenever it is safe.

Subscribes: /global_waypoints (race_msgs/WaypointArray, latched), pose (nav_msgs/Odometry),
            /scan (sensor_msgs/LaserScan)
Publishes:  /local_waypoints (race_msgs/WaypointArray, latched QoS to match pure pursuit),
            /local_planner/mode (std_msgs/String), /local_planner/compute_ms (std_msgs/Float32),
            /local_planner/markers (visualization_msgs/MarkerArray)

Every scan:
    1. pose -> nearest raceline index -> raceline segment over `horizon` m, in base_link
    2. scan -> obstacle points in base_link (laser offset from TF)
    3. gate: does every raceline point keep `car_half_width + clearance_margin` from the scan?
    4. PASS   if planner == passthrough, or (gate free AND car within rejoin_tol of the line
              AND no recent plan)  -> publish the raceline segment as is
       PLAN   otherwise -> frenet | rrt; speed = raceline speed capped by curvature
       FALLBACK if the planner finds nothing -> the candidate with most clearance, slowed down
Output is always in the map frame with s_m starting at 0 and increasing (no start-line wrap),
so pure pursuit tracks it unchanged.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from race_msgs.msg import Waypoint, WaypointArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from local_planner.corridor import (Raceline, RefPath, Obstacles, check_corridor, scan_to_car,
                                    car_to_map, heading_curvature, speed_profile)
from local_planner.frenet import FrenetPlanner
from local_planner.rrt import RRTStarPlanner


def yaw_from_quat(q):
    """Return the yaw angle of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def apply_tf(tf, x, y, yaw):
    """Apply a TransformStamped to a 2D pose (x, y, yaw)."""
    t, th = tf.transform.translation, yaw_from_quat(tf.transform.rotation)
    c, s = math.cos(th), math.sin(th)
    return t.x + c * x - s * y, t.y + s * x + c * y, yaw + th


COMMON = {
    'planner': 'frenet',              # passthrough | frenet | rrt
    'pose_topic': '/ego_racecar/odom',
    'scan_topic': '/scan',
    'waypoints_topic': '/global_waypoints',
    'local_topic': '/local_waypoints',
    'base_frame': 'ego_racecar/base_link',
    'horizon': 5.0,                   # m, fixed
    'car_half_width': 0.155,          # gym width 0.31
    'clearance_margin': 0.10,         # keep < global --margin (0.15); see config
    'wheelbase': 0.3302,
    'max_steer': 0.4189,
    'near_radius': 0.5,               # m around the car where only near_margin is required
    'near_margin': 0.02,              # so a car already inside the margin can still plan out
    'rejoin_tol': 0.15,               # m off the raceline -> keep planning to rejoin smoothly
    'hold_cycles': 3,                 # keep planning this many scans after the gate frees
    'mu': 1.0489,
    'grip': 0.7,                      # same as optimise_raceline.py --grip
    'a_brake': 5.0,                   # m/s^2, backward pass of the speed profile
    'max_speed': 5.0,
    'fallback_speed': 1.0,            # m/s when no feasible path exists
    'debug_markers': True,            # candidates / RRT tree (heavier)
}
FRENET = {
    'ds': 0.1, 'd_step': 0.1, 'end_stations': [2.0, 3.0, 4.0, 5.0],
    'w_offset': 3.0, 'w_smooth': 2.0, 'w_consist': 0.5, 'w_prox': 5.0, 'prox_dist': 0.5,
}
RRT = {
    'ds': 0.1, 'max_iters': 1000, 'time_budget': 0.05, 'max_step': 0.5,
    'neighbour_radius': 1.0, 'goal_bias': 0.1, 'goal_tol': 0.3, 'edge_res': 0.05,
    'turn_slack': 1.5, 'stop_at_first': False, 'seed': 42,
}


class LocalPlanner(Node):
    """ROS 2 node: gate the raceline ahead against the scan and publish the local path.

    Subscribes to the global raceline, the pose and the scan; publishes
    ``/local_waypoints``, ``/local_planner/mode``, ``/local_planner/compute_ms`` and
    ``/local_planner/markers``.
    """

    def __init__(self):
        """Declare parameters, create the planners, publishers, subscribers and TF listener."""
        super().__init__('local_planner')
        self.p = self.declare_group('', COMMON)
        fr = self.declare_group('frenet.', FRENET)
        rr = self.declare_group('rrt.', RRT)
        for g in (fr, rr):
            g['horizon'] = self.p['horizon']
            g['near_radius'] = self.p['near_radius']
            g['near_threshold'] = self.p['car_half_width'] + self.p['near_margin']

        self.threshold = self.p['car_half_width'] + self.p['clearance_margin']
        self.kappa_max = math.tan(self.p['max_steer']) / self.p['wheelbase']
        self.a_lat = self.p['mu'] * 9.81 * self.p['grip']
        self.frenet = FrenetPlanner(fr)
        self.rrt = RRTStarPlanner(rr)

        self.raceline = None
        self.frame = 'map'
        self.pose = None
        self.idx = None
        self.laser_in_base = None
        self.hold = 0
        self.times = []

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(WaypointArray, self.p['waypoints_topic'], self.on_raceline,
                                 latched)
        self.create_subscription(Odometry, self.p['pose_topic'], self.on_pose, 10)
        self.create_subscription(LaserScan, self.p['scan_topic'], self.on_scan, 10)
        # latched to match pure pursuit's subscription QoS (volatile pub would be incompatible)
        self.path_pub = self.create_publisher(WaypointArray, self.p['local_topic'], latched)
        self.mode_pub = self.create_publisher(String, '/local_planner/mode', 10)
        self.ms_pub = self.create_publisher(Float32, '/local_planner/compute_ms', 10)
        self.mk_pub = self.create_publisher(MarkerArray, '/local_planner/markers', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(
            f'planner {self.p["planner"]}, horizon {self.p["horizon"]} m, clearance '
            f'{self.threshold:.3f} m, kappa_max {self.kappa_max:.2f} 1/m; waiting for '
            f'{self.p["waypoints_topic"]} ...')

    def declare_group(self, prefix, defaults):
        """Declare the parameters in ``defaults`` under ``prefix`` and return their values."""
        out = {}
        for k, v in defaults.items():
            self.declare_parameter(prefix + k, v)
            out[k] = self.get_parameter(prefix + k).value
        return out

    # ------------------------------------------------------------------ inputs
    def on_raceline(self, msg):
        """Store the received global raceline and its frame."""
        if len(msg.waypoints) < 2:
            self.get_logger().error('raceline has fewer than 2 waypoints')
            return
        self.raceline = Raceline.from_msg(msg)
        self.frame = msg.header.frame_id or 'map'
        self.source = msg.source
        self.idx = None
        self.get_logger().info(f'raceline "{msg.source}": {self.raceline.n} pts, '
                               f'{self.raceline.length:.2f} m')

    def on_pose(self, msg):
        """Store the latest pose, transformed into the raceline frame if needed."""
        pp = msg.pose.pose
        x, y, yaw = pp.position.x, pp.position.y, yaw_from_quat(pp.orientation)
        if msg.header.frame_id and msg.header.frame_id != self.frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.frame, msg.header.frame_id, Time())
            except TransformException as e:
                self.get_logger().warn(f'TF {msg.header.frame_id} -> {self.frame}: {e}',
                                       throttle_duration_sec=2.0)
                return
            x, y, yaw = apply_tf(tf, x, y, yaw)
        self.pose = (x, y, yaw)

    def laser_offset(self, laser_frame):
        """Look up the laser pose in the base frame; return False if TF is not ready."""
        try:
            tf = self.tf_buffer.lookup_transform(self.p['base_frame'], laser_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'waiting for TF {self.p["base_frame"]} -> {laser_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return False
        t = tf.transform.translation
        self.laser_in_base = (t.x, t.y, yaw_from_quat(tf.transform.rotation))
        self.get_logger().info('laser in base: x={:.3f} y={:.3f} yaw={:.3f}'.format(
            *self.laser_in_base))
        return True

    # -------------------------------------------------------------------- cycle
    def on_scan(self, scan):
        """Run one planning cycle on a new scan.

        Builds the raceline segment ahead, runs the clearance gate, selects pass-through,
        the planner or the fallback, and publishes the path, mode, timing and markers.
        """
        if self.raceline is None or self.pose is None:
            return
        if self.laser_in_base is None and not self.laser_offset(scan.header.frame_id):
            return
        t0 = time.perf_counter()
        pose = self.pose
        rl = self.raceline

        self.idx = rl.nearest_index(pose[:2], self.idx)
        seg_idx, seg_s = rl.segment_ahead(self.idx, self.p['horizon'])
        ref = RefPath(rl, seg_idx, seg_s, pose)
        obs = Obstacles(scan_to_car(scan.ranges, scan.angle_min, scan.angle_increment,
                                    scan.range_min, scan.range_max, self.laser_in_base))
        gate = check_corridor(ref, obs, self.threshold)
        d0, _ = ref.car_frenet_state()

        need_plan = (not gate.free) or abs(d0) > self.p['rejoin_tol']
        self.hold = self.p['hold_cycles'] if need_plan else max(self.hold - 1, 0)
        planner = self.p['planner']
        debug = None

        if planner == 'passthrough' or self.hold == 0:
            mode = 'pass'
            path_car, v = ref.xy, ref.vx
            s_out = ref.s
        else:
            path_car, debug, mode = self.run_planner(planner, ref, obs)
            s_out = np.concatenate(([0.0], np.cumsum(
                np.linalg.norm(np.diff(path_car, axis=0), axis=1))))
            _, kap = heading_curvature(path_car)
            v_ref = ref.v_at(np.minimum(s_out, ref.length))
            v = speed_profile(v_ref, kap, s_out, self.a_lat, self.p['a_brake'],
                              self.p['max_speed'])
            if mode == 'fallback':
                v = np.minimum(v, self.p['fallback_speed'])

        self.publish_path(path_car, s_out, v, pose, ref, scan.header.stamp)
        ms = (time.perf_counter() - t0) * 1e3
        self.mode_pub.publish(String(data=mode))
        self.ms_pub.publish(Float32(data=float(ms)))
        self.log_timing(ms, mode, gate)
        self.publish_markers(ref, gate, path_car, debug, mode, Time().to_msg())

    def run_planner(self, planner, ref, obs):
        """Run the configured planner and return (path, debug, mode).

        When the planner finds no feasible path, returns the Frenet best-effort candidate
        (if any) or the raceline segment, with mode ``fallback``.
        """
        if planner == 'frenet':
            res = self.frenet.plan(ref, obs, self.threshold, self.kappa_max)
            if res.path is not None:
                return res.path, ('frenet', res), 'frenet'
            best = self.frenet.best_effort(res, obs)
            if best is not None:
                self.get_logger().warn('frenet: no feasible candidate, fallback',
                                       throttle_duration_sec=0.5)
                return best, ('frenet', res), 'fallback'
        elif planner == 'rrt':
            res = self.rrt.plan(ref, obs, self.threshold, self.kappa_max)
            if res.path is not None:
                return res.path, ('rrt', res), 'rrt' if res.smoothed else 'rrt_raw'
            self.get_logger().warn(f'rrt: no path after {res.iters} iters, fallback',
                                   throttle_duration_sec=0.5)
        else:
            self.get_logger().error(f'unknown planner "{planner}"', throttle_duration_sec=5.0)
        # nothing: hold the raceline but slow right down
        return ref.xy, None, 'fallback'

    # ------------------------------------------------------------------ outputs
    def publish_path(self, path_car, s, v, pose, ref, stamp):
        """Transform a car-frame path to the raceline frame and publish it as waypoints."""
        pts = car_to_map(path_car, pose)
        psi, kap = heading_curvature(pts)
        # widths: raceline widths shifted by the lateral offset (approximate, for logging only)
        wl = np.interp(np.minimum(s, ref.length), ref.s, ref.w_left)
        wr = np.interp(np.minimum(s, ref.length), ref.s, ref.w_right)
        msg = WaypointArray()
        msg.header.frame_id = self.frame
        msg.header.stamp = stamp
        msg.source = f'local_{self.p["planner"]}'
        for i in range(len(pts)):
            msg.waypoints.append(Waypoint(
                s_m=float(s[i]), x_m=float(pts[i, 0]), y_m=float(pts[i, 1]),
                psi_rad=float(psi[i]), kappa_radpm=float(kap[i]), vx_mps=float(v[i]),
                ax_mps2=0.0, w_left_m=float(wl[i]), w_right_m=float(wr[i])))
        self.path_pub.publish(msg)

    def log_timing(self, ms, mode, gate):
        """Record the cycle time, log blocked gates, and log timing stats every 100 cycles."""
        self.times.append(ms)
        if not gate.free:
            self.get_logger().info(f'[{mode}] blocked at s={gate.first_blocked_s:.2f} m '
                                   f'(min clearance {gate.clearance.min():.3f} m)',
                                   throttle_duration_sec=0.5)
        if len(self.times) >= 100:
            t = np.array(self.times)
            self.get_logger().info(f'compute: mean {t.mean():.1f} ms, p95 '
                                   f'{np.percentile(t, 95):.1f} ms, max {t.max():.1f} ms')
            self.times = []

    def publish_markers(self, ref, gate, path_car, debug, mode, stamp):
        """Publish the corridor, local path and candidate/tree markers in base_link."""
        ma = MarkerArray()
        base = self.p['base_frame']

        def strip(ns, mid, pts, colors=None, rgba=(0.0, 0.4, 1.0, 1.0), w=0.05,
                  mtype=Marker.LINE_STRIP):
            """Return a marker of type mtype through the given points."""
            m = Marker()
            m.header.frame_id, m.header.stamp = base, stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.scale.x = w
            m.pose.orientation.w = 1.0
            m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            for p in pts:
                m.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.0))
            if colors is not None:
                m.colors = colors
            return m

        clear = Marker()
        clear.header.frame_id, clear.header.stamp = base, stamp
        clear.ns, clear.action = '', Marker.DELETEALL
        ma.markers.append(clear)

        cols = [ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0) if c >= self.threshold
                else ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0) for c in gate.clearance]
        ma.markers.append(strip('corridor', 0, ref.xy, colors=cols, w=0.04))
        path_rgba = {'pass': (0.0, 0.4, 1.0, 0.0), 'fallback': (1.0, 0.5, 0.0, 1.0)}.get(
            mode, (0.0, 0.4, 1.0, 1.0))
        ma.markers.append(strip('local_path', 0, path_car, rgba=path_rgba, w=0.07))

        if self.p['debug_markers'] and debug is not None:
            kind, res = debug
            if kind == 'frenet' and len(res.all_paths):
                segs = []
                for pth, ok in zip(res.all_paths[::2], res.feasible_mask[::2]):
                    if ok:
                        segs.extend(np.repeat(pth, 2, axis=0)[1:-1])
                if segs:
                    ma.markers.append(strip('debug', 1, segs, rgba=(0.6, 0.6, 0.6, 0.4),
                                            w=0.01, mtype=Marker.LINE_LIST))
            elif kind == 'rrt' and len(res.tree_edges):
                ma.markers.append(strip('debug', 2, res.tree_edges.reshape(-1, 2),
                                        rgba=(0.6, 0.6, 0.6, 0.4), w=0.01,
                                        mtype=Marker.LINE_LIST))
        self.mk_pub.publish(ma)


def main(args=None):
    """Run the local planner node until shutdown."""
    rclpy.init(args=args)
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
