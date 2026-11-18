"""ROS 2 node: PL-ICP scan matching localization.

in : /scan (LaserScan), /ego_racecar/odom_noisy (Odometry, initial guess),
     /initialpose (PoseWithCovarianceStamped, RViz "2D Pose Estimate")
out: /scan_match/odom (Odometry), /scan_match/pose (PoseStamped),
     /scan_match/path (Path), optional TF (publish_tf, off by default)
All algorithm work is in core.py / tracker.py; this file only converts
messages and parameters.
"""

import math
import re
from collections import Counter, deque
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               TransformStamped)
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import tf2_ros

from . import core
from .tracker import ScanMatchTracker, interpolate_pose


def yaw_from_quat(q):
    """Return the yaw angle of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def fill_quat(q, yaw):
    """Write a yaw-only rotation into a quaternion message in place."""
    q.x, q.y = 0.0, 0.0
    q.z, q.w = math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def stamp_to_sec(stamp):
    """Return a builtin_interfaces/Time as float seconds."""
    return stamp.sec + 1e-9 * stamp.nanosec


class ScanMatchingNode(Node):
    """ROS 2 node: PL-ICP scan matching with odometry as the initial guess.

    Publishes the pose on ``output_odom_topic``, ``output_pose_topic`` and
    ``output_path_topic``, and optionally a TF to ``tf_child_frame``.
    """

    def __init__(self):
        """Declare parameters, create publishers, subscribers, TF and the stats timer."""
        super().__init__('scan_matching')
        p = self.declare_parameter
        # Topics and frames
        self.scan_topic = p('scan_topic', '/scan').value
        self.odom_topic = p('odom_topic', '/ego_racecar/odom_noisy').value
        self.initialpose_topic = p('initialpose_topic', '/initialpose').value
        self.out_odom_topic = p('output_odom_topic', '/scan_match/odom').value
        self.out_pose_topic = p('output_pose_topic', '/scan_match/pose').value
        self.out_path_topic = p('output_path_topic', '/scan_match/path').value
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'ego_racecar/base_link').value
        self.publish_tf = p('publish_tf', False).value
        self.tf_child_frame = p('tf_child_frame', 'scan_match/base_link').value
        self.path_max_len = p('path_max_length', 3000).value
        # Starting pose until the first /initialpose
        x0 = p('initial_pose_x', 0.0).value
        y0 = p('initial_pose_y', 0.0).value
        th0 = p('initial_pose_theta', 0.0).value
        # Scan preprocessing
        self.max_range = p('max_range', 0.0).value     # 0 = use msg.range_max
        scan_params = {
            'normal_length': p('normal_length', 0.4).value,
            'normal_max_gap': p('normal_max_gap', 0.2).value,
        }
        icp_params = {
            'max_iterations': p('max_iterations', 30).value,
            'max_corr_dist': p('max_correspondence_dist', 0.5).value,
            'outlier_quantile': p('outlier_quantile', 0.7).value,
            'outlier_mult': p('outlier_mult', 2.0).value,
            'outlier_min_dist': p('outlier_min_dist', 0.02).value,
            'min_correspondences': p('min_correspondences', 50).value,
            'epsilon_xy': p('epsilon_xy', 1e-3).value,
            'epsilon_theta': p('epsilon_theta', 1e-3).value,
            'degeneracy_min_eig': p('degeneracy_min_eig', 10.0).value,
            'odom_prior_weight': p('odom_prior_weight', 1.0).value,
            'search': p('correspondence_search', 'jump').value,
        }
        self.kf_lin = p('kf_dist_linear', 0.0).value
        self.kf_ang = p('kf_dist_angular', 0.0).value
        self.max_corr_lin = p('max_correction_linear', 0.5).value
        self.max_corr_ang = p('max_correction_angular', 0.3).value
        self.scan_params, self.icp_params = scan_params, icp_params

        # Odometry buffer: ~2 s at 100 Hz
        self.odom_t = deque(maxlen=200)
        self.odom_p = deque(maxlen=200)

        self.tracker = None                 # created once the laser TF is known
        self.start_pose = np.array([x0, y0, th0])
        self.pending_reset = None
        self.pending_scans = deque()        # wait for odometry to cover them
        self.prev_stamp = None
        self.stats = {'n': 0, 'fallback': 0, 'degenerate': 0, 'skipped': 0,
                      'ms_sum': 0.0, 'ms_max': 0.0, 'it_sum': 0}
        self.fail_reasons = Counter()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) \
            if self.publish_tf else None

        self.create_subscription(LaserScan, self.scan_topic, self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_subscription(PoseWithCovarianceStamped,
                                 self.initialpose_topic, self.on_initialpose, 1)
        self.odom_pub = self.create_publisher(Odometry, self.out_odom_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped,
                                              self.out_pose_topic, 10)
        self.path_pub = self.create_publisher(Path, self.out_path_topic, 1)
        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.create_timer(5.0, self.log_stats)
        t0 = time.perf_counter()
        backend = 'numba' if core.warmup() else 'plain Python (numba not found)'
        self.get_logger().info(
            f'correspondence search: {backend}, ready in '
            f'{time.perf_counter() - t0:.1f} s')
        self.get_logger().info(
            f'scan_matching: {self.scan_topic} + {self.odom_topic} -> '
            f'{self.out_odom_topic}; keyframes lin={self.kf_lin} m, '
            f'ang={self.kf_ang} rad; search={icp_params["search"]}; '
            f'publish_tf={self.publish_tf}')

    # ----------------------------------------------------------- inputs ----

    def on_odom(self, msg):
        """Buffer an odometry pose and process any scans it now covers."""
        p = msg.pose.pose
        self.odom_t.append(stamp_to_sec(msg.header.stamp))
        self.odom_p.append(np.array([p.position.x, p.position.y,
                                     yaw_from_quat(p.orientation)]))
        self.try_process()

    def on_initialpose(self, msg):
        """Schedule a pose reset for the next scan and drop waiting scans."""
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f'/initialpose in frame "{msg.header.frame_id}", expected '
                f'"{self.map_frame}"; using it as {self.map_frame} anyway')
        p = msg.pose.pose
        pose = np.array([p.position.x, p.position.y,
                         yaw_from_quat(p.orientation)])
        # Applied at the next scan, so it cannot interleave with a match.
        # Scans still waiting were taken before the new pose: drop them.
        self.pending_reset = pose
        self.pending_scans.clear()
        self.path.poses.clear()
        self.get_logger().info(
            f'initial pose: x={pose[0]:.2f} y={pose[1]:.2f} '
            f'theta={pose[2]:.2f}')

    def lookup_laser_in_base(self, laser_frame):
        """Return the laser pose [x, y, yaw] in the base frame, or None if TF is not ready."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, laser_frame,
                                                 Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f'waiting for TF {self.base_frame} -> {laser_frame}: {e}',
                throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        yaw = yaw_from_quat(tf.transform.rotation)
        self.get_logger().info(
            f'laser in base: x={t.x:.3f} y={t.y:.3f} yaw={yaw:.3f}')
        return np.array([t.x, t.y, yaw])

    def on_scan(self, msg):
        """Queue a scan (at most 3 waiting) and process the ones odometry covers."""
        self.pending_scans.append(msg)
        # If the node falls behind, drop the oldest waiting scans.
        while len(self.pending_scans) > 3:
            self.pending_scans.popleft()
            self.stats['skipped'] += 1
        self.try_process()

    def try_process(self):
        """Match waiting scans, oldest first, once odometry covering each
        scan's timestamp has arrived. Processing a scan before that would
        interpolate past the newest odometry, i.e. take the guess from an
        earlier moment; while a match runs (tens of ms) odometry keeps
        queueing, so the guess could be 100 ms stale - enough to point the
        wrong way when the car swerves.
        """
        while self.pending_scans:
            if not self.odom_t:
                self.get_logger().warn(f'no odometry on {self.odom_topic} yet',
                                       throttle_duration_sec=2.0)
                return
            t_scan = stamp_to_sec(self.pending_scans[0].header.stamp)
            if self.odom_t[-1] < t_scan:
                return                      # wait for newer odometry
            msg = self.pending_scans.popleft()
            if t_scan < self.odom_t[0]:
                self.stats['skipped'] += 1  # older than the odometry buffer
                continue
            self.process_scan(msg, t_scan)

    def process_scan(self, msg, t_scan):
        """Match one scan, update the pose and publish it."""
        if self.tracker is None:
            laser_in_base = self.lookup_laser_in_base(msg.header.frame_id)
            if laser_in_base is None:
                return
            self.tracker = ScanMatchTracker(
                laser_in_base, self.scan_params, self.icp_params,
                self.kf_lin, self.kf_ang, self.max_corr_lin, self.max_corr_ang)
            self.tracker.reset(self.start_pose)

        odom_pose = interpolate_pose(list(self.odom_t), list(self.odom_p),
                                     t_scan)

        if self.pending_reset is not None:
            self.tracker.reset(self.pending_reset)
            self.pending_reset = None
            self.prev_stamp = None

        range_max = msg.range_max if self.max_range <= 0.0 \
            else min(self.max_range, msg.range_max)
        t0 = time.perf_counter()
        out = self.tracker.process(msg.ranges, msg.angle_min,
                                   msg.angle_increment, msg.range_min,
                                   range_max, odom_pose)
        ms = 1e3 * (time.perf_counter() - t0)

        if out.icp is not None:
            s = self.stats
            s['n'] += 1
            s['ms_sum'] += ms
            s['ms_max'] = max(s['ms_max'], ms)
            s['it_sum'] += out.icp.iterations
            s['degenerate'] += int(out.icp.degenerate)
            if out.used_fallback:
                s['fallback'] += 1
                self.fail_reasons[re.sub(r'[\d.]+', 'N', out.reason)] += 1

        dt = None
        if self.prev_stamp is not None:
            dt = t_scan - self.prev_stamp
        self.prev_stamp = t_scan
        self.publish(msg.header.stamp, out, dt)

    # ---------------------------------------------------------- outputs ----

    def publish(self, stamp, out, dt):
        """Publish the pose, odometry (with twist from the step), path and optional TF."""
        x, y, th = out.pose
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.map_frame
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        fill_quat(ps.pose.orientation, float(th))
        self.pose_pub.publish(ps)

        od = Odometry()
        od.header = ps.header
        od.child_frame_id = self.base_frame
        od.pose.pose = ps.pose
        if dt is not None and dt > 1e-6:
            d = out.base_delta
            od.twist.twist.linear.x = float(d[0] / dt)
            od.twist.twist.linear.y = float(d[1] / dt)
            od.twist.twist.angular.z = float(d[2] / dt)
        self.odom_pub.publish(od)

        self.path.header.stamp = stamp
        self.path.poses.append(ps)
        if len(self.path.poses) > self.path_max_len:
            del self.path.poses[0]
        self.path_pub.publish(self.path)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = ps.header
            tf.child_frame_id = self.tf_child_frame
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            fill_quat(tf.transform.rotation, float(th))
            self.tf_broadcaster.sendTransform(tf)

    def log_stats(self):
        """Log match time, iterations, degenerate matches, fallbacks, skipped scans; reset."""
        s = self.stats
        if s['n'] == 0:
            return
        reasons = f' {dict(self.fail_reasons)}' if self.fail_reasons else ''
        self.get_logger().info(
            f'{s["n"]} scans: match {s["ms_sum"] / s["n"]:.1f} ms mean, '
            f'{s["ms_max"]:.1f} ms max, {s["it_sum"] / s["n"]:.1f} iters, '
            f'{s["degenerate"]} degenerate (odometry along track), '
            f'{s["fallback"]} odometry fallbacks{reasons}, '
            f'{s["skipped"]} scans skipped (node too slow)')
        self.stats = {'n': 0, 'fallback': 0, 'degenerate': 0, 'skipped': 0,
                      'ms_sum': 0.0, 'ms_max': 0.0, 'it_sum': 0}
        self.fail_reasons.clear()


def main(args=None):
    """Run the scan-matching node until shutdown."""
    rclpy.init(args=args)
    node = ScanMatchingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()