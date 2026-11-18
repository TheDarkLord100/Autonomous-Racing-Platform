#!/usr/bin/env python3
"""Lap logger: records how the car actually tracks the published raceline.

Subscribes: waypoints (race_msgs/WaypointArray, latched), pose (nav_msgs/Odometry),
            drive (ackermann_msgs/AckermannDriveStamped, the controller's command)
Writes:     <log_dir>/<source>[_<tag>]_<YYYYmmdd_HHMMSS>.csv, one row per pose message:
            t, lap, s, x, y, e_lat, v_plan, v_act, v_cmd, steer_cmd, kappa_plan, kappa_act, yaw_rate,
            gap_left, gap_right
Prints:     per-lap summary (lap time, lateral error max / RMS and where, speed over plan,
            min wall gap), a warning on estimated wall contact, and one when the car stops.

Wall gap = distance from the car's SIDE to the wall, estimated from the raceline's
w_left_m / w_right_m (measured along the raceline normal) and e_lat. It is approximate
at tight corners; the gym's own "hit something" message is the definitive contact check.

Sign conventions: e_lat > 0 = car is LEFT of the path; kappa > 0 = turning left (CCW).
In sim the pose should be ground truth (/ego_racecar/odom), so this is the true error.
"""
import csv
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from race_msgs.msg import WaypointArray


def yaw_from_quat(q):
    """Return the yaw angle of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def apply_tf(tf, x, y, yaw):
    """Apply a TransformStamped to a 2D pose (x, y, yaw)."""
    t, th = tf.transform.translation, yaw_from_quat(tf.transform.rotation)
    c, s = math.cos(th), math.sin(th)
    return t.x + c * x - s * y, t.y + s * x + c * y, yaw + th


COLUMNS = ['t', 'lap', 's', 'x', 'y', 'e_lat', 'v_plan', 'v_act', 'v_cmd', 'steer_cmd',
           'kappa_plan', 'kappa_act', 'yaw_rate', 'gap_left', 'gap_right']


class LapLogger(Node):
    """ROS 2 node: log the car's pose against the raceline to CSV, one row per pose message.

    Also counts laps on start-line crossings, logs a per-lap summary and warns when the car
    stops after having moved.
    """

    def __init__(self):
        """Declare parameters and create subscribers and the TF listener."""
        super().__init__('lap_logger')
        defaults = {
            'pose_topic': '/ego_racecar/odom',
            'waypoints_topic': '/global_waypoints',
            'drive_topic': '/drive',
            'log_dir': str(Path.home() / 'race_logs'),
            'tag': '',                 # extra label in the file name, e.g. g07_scale10
            'search_back': 10,
            'search_ahead': 60,
            'relocalise_dist': 2.0,
            'stop_speed': 0.2,         # m/s; below this for stop_time after moving = stopped
            'stop_time': 1.0,          # s
            'half_width': 0.155,       # m, car half width (gym 0.31 / 2)
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        self.xy = None
        self.idx = None
        self.file = None
        self.writer = None
        self.v_cmd = float('nan')
        self.steer_cmd = float('nan')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(WaypointArray, self.p['waypoints_topic'],
                                 self.on_waypoints, latched)
        self.create_subscription(Odometry, self.p['pose_topic'], self.on_pose, 10)
        self.create_subscription(AckermannDriveStamped, self.p['drive_topic'], self.on_drive, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(f'waiting for {self.p["waypoints_topic"]} ...')

    # ---------- path + file ----------
    def on_waypoints(self, msg):
        """Store the raceline and open a new log file named after its source."""
        wp = msg.waypoints
        if len(wp) < 2:
            return
        self.frame = msg.header.frame_id
        self.xy = np.array([[w.x_m, w.y_m] for w in wp])
        self.psi = np.array([w.psi_rad for w in wp])
        self.kappa = np.array([w.kappa_radpm for w in wp])
        self.v = np.array([w.vx_mps for w in wp])
        self.wl = np.array([w.w_left_m for w in wp])
        self.wr = np.array([w.w_right_m for w in wp])
        s = np.array([w.s_m for w in wp])
        self.s = s - s[0]
        self.lap_len = self.s[-1] + float(np.hypot(*(self.xy[0] - self.xy[-1])))
        self.idx = None
        self.open_log(msg.source or 'path')
        self.reset_lap_state()

    def open_log(self, source):
        """Open ``<log_dir>/<source>[_<tag>]_<time>.csv`` and write the header."""
        if self.file:
            self.file.close()
        log_dir = Path(self.p['log_dir']).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        tag = f'_{self.p["tag"]}' if self.p['tag'] else ''
        path = log_dir / f'{source}{tag}_{time.strftime("%Y%m%d_%H%M%S")}.csv'
        self.file = open(path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(COLUMNS)
        self.rows = 0
        self.get_logger().info(f'logging to {path}')

    def reset_lap_state(self):
        """Reset the lap counter, lap timer and stop detection."""
        self.lap = 0                   # 0 = out-lap from spawn until the first line crossing
        self.lap_t0 = None
        self.prev_s = None
        self.moving = False
        self.slow_since = None
        self.stopped_reported = False
        self.reset_lap_stats()

    def reset_lap_stats(self):
        """Reset the per-lap error, speed and wall-gap statistics."""
        self.e_max, self.e_max_s, self.e_sq, self.n = 0.0, 0.0, 0.0, 0
        self.over_max, self.over_max_s = 0.0, 0.0
        self.gap_min, self.gap_min_s, self.gap_min_side = float('inf'), 0.0, ''

    def on_drive(self, msg):
        """Store the latest commanded speed and steering angle."""
        self.v_cmd = msg.drive.speed
        self.steer_cmd = msg.drive.steering_angle

    # ---------- geometry ----------
    def nearest(self, x, y):
        """Return the index of the raceline point nearest to (x, y).

        Searches a window around the previous index; falls back to a full search at start
        or when the window minimum is farther than ``relocalise_dist``.
        """
        n = len(self.xy)
        if self.idx is not None:
            cand = np.arange(self.idx - self.p['search_back'],
                             self.idx + self.p['search_ahead'] + 1) % n
            d = np.hypot(self.xy[cand, 0] - x, self.xy[cand, 1] - y)
            k = int(np.argmin(d))
            if d[k] < self.p['relocalise_dist']:
                self.idx = int(cand[k])
                return self.idx
        d = np.hypot(self.xy[:, 0] - x, self.xy[:, 1] - y)
        self.idx = int(np.argmin(d))
        return self.idx

    # ---------- per pose ----------
    def on_pose(self, msg):
        """Compute the row for one pose, update lap statistics and write it."""
        if self.xy is None:
            return
        pp = msg.pose.pose
        x, y, yaw = pp.position.x, pp.position.y, yaw_from_quat(pp.orientation)
        if msg.header.frame_id != self.frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.frame, msg.header.frame_id, Time())
            except TransformException as e:
                self.get_logger().warn(f'TF {msg.header.frame_id} -> {self.frame}: {e}',
                                       throttle_duration_sec=2.0)
                return
            x, y, yaw = apply_tf(tf, x, y, yaw)

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        i = self.nearest(x, y)
        c, sn = math.cos(self.psi[i]), math.sin(self.psi[i])
        dx, dy = x - self.xy[i, 0], y - self.xy[i, 1]
        e_lat = -sn * dx + c * dy                                # + = left of path
        s_now = (self.s[i] + c * dx + sn * dy) % self.lap_len

        tw = msg.twist.twist
        v_act = math.hypot(tw.linear.x, tw.linear.y)
        yaw_rate = tw.angular.z
        kappa_act = yaw_rate / v_act if v_act > 0.3 else float('nan')
        v_plan = float(self.v[i])
        gap_l = float(self.wl[i]) - e_lat - self.p['half_width']   # car side to left wall
        gap_r = float(self.wr[i]) + e_lat - self.p['half_width']   # car side to right wall
        if min(gap_l, gap_r) < 0.0:
            side = 'left' if gap_l < gap_r else 'right'
            self.get_logger().warn(f'contact? {side} side, gap {min(gap_l, gap_r):+.3f} m '
                                   f'at s = {s_now:.1f}', throttle_duration_sec=1.0)

        self.check_lap(t, s_now)
        self.check_stopped(t, v_act, s_now)

        if self.lap > 0:                                         # stats on timed laps only
            self.n += 1
            self.e_sq += e_lat * e_lat
            if abs(e_lat) > abs(self.e_max):
                self.e_max, self.e_max_s = e_lat, s_now
            if v_act - v_plan > self.over_max:
                self.over_max, self.over_max_s = v_act - v_plan, s_now
            for g, side in ((gap_l, 'left'), (gap_r, 'right')):
                if g < self.gap_min:
                    self.gap_min, self.gap_min_s, self.gap_min_side = g, s_now, side

        self.writer.writerow([f'{t:.4f}', self.lap, f'{s_now:.3f}', f'{x:.4f}', f'{y:.4f}',
                              f'{e_lat:.4f}', f'{v_plan:.3f}', f'{v_act:.3f}',
                              f'{self.v_cmd:.3f}', f'{self.steer_cmd:.4f}',
                              f'{float(self.kappa[i]):.4f}', f'{kappa_act:.4f}',
                              f'{yaw_rate:.4f}', f'{gap_l:.4f}', f'{gap_r:.4f}'])
        self.rows += 1
        if self.rows % 200 == 0:
            self.file.flush()

    def check_lap(self, t, s_now):
        """Detect a start-line crossing, log the finished lap and start the next."""
        crossed = self.prev_s is not None and (self.prev_s - s_now) > self.lap_len / 2
        self.prev_s = s_now
        if not crossed:
            return
        if self.lap > 0 and self.lap_t0 is not None:
            rms = math.sqrt(self.e_sq / max(self.n, 1))
            self.get_logger().info(
                f'lap {self.lap}: {t - self.lap_t0:.2f} s | e_lat max {self.e_max:+.3f} m '
                f'at s = {self.e_max_s:.1f}, rms {rms:.3f} m | speed over plan max '
                f'{self.over_max:.2f} m/s at s = {self.over_max_s:.1f} | min wall gap '
                f'{self.gap_min:+.3f} m ({self.gap_min_side}) at s = {self.gap_min_s:.1f}')
        self.lap += 1
        self.lap_t0 = t
        self.reset_lap_stats()
        self.file.flush()

    def check_stopped(self, t, v_act, s_now):
        """Warn once if the car stays below ``stop_speed`` for ``stop_time`` after moving."""
        if v_act > 1.0:
            self.moving = True
            self.slow_since = None
            self.stopped_reported = False
            return
        if not self.moving or v_act > self.p['stop_speed']:
            return
        if self.slow_since is None:
            self.slow_since = t
        elif t - self.slow_since > self.p['stop_time'] and not self.stopped_reported:
            self.get_logger().warn(f'car stopped (crash?) on lap {self.lap} at s = {s_now:.1f} m')
            self.stopped_reported = True
            self.file.flush()

    def destroy_node(self):
        """Close the log file and destroy the node."""
        if self.file:
            self.file.close()
        super().destroy_node()


def main(args=None):
    """Run the lap logger node until shutdown."""
    rclpy.init(args=args)
    node = LapLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()