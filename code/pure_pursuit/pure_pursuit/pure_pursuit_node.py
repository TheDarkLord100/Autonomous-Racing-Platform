#!/usr/bin/env python3
"""Pure pursuit path tracker with understeer compensation.

Subscribes: waypoints (race_msgs/WaypointArray, latched), pose (nav_msgs/Odometry)
Publishes:  drive (ackermann_msgs/AckermannDriveStamped), lookahead marker

Steering:   delta = atan(L_wb * kappa) + K * v^2 * kappa,   kappa = 2 * l_y / l_d^2
            The first term is classic (kinematic) pure pursuit. The second adds the extra
            steering a real (dynamic) car needs when it understeers; K is the understeer
            gradient [rad per m/s^2]. K = 0 gives plain pure pursuit.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from race_msgs.msg import WaypointArray
from visualization_msgs.msg import Marker


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def apply_tf(tf, x, y, yaw):
    """Transform a 2D pose with a TransformStamped (source -> target frame)."""
    t, th = tf.transform.translation, yaw_from_quat(tf.transform.rotation)
    c, s = math.cos(th), math.sin(th)
    return t.x + c * x - s * y, t.y + s * x + c * y, yaw + th


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        defaults = {
            'pose_topic': '/ego_racecar/odom',
            'waypoints_topic': '/global_waypoints',
            'drive_topic': '/drive',
            'wheelbase': 0.3302,
            'max_steer': 0.4189,
            'rear_axle_offset': 0.0,     # m from pose origin back to the rear axle
            'understeer_gradient': 0.0,  # K [rad per m/s^2]; extra steer = K * v^2 * kappa
            'lookahead_base': 0.5,       # L0 [m]
            'lookahead_gain': 0.25,      # k [s]
            'lookahead_min': 0.5,
            'lookahead_max': 2.5,
            'speed_mode': 'constant',    # 'constant' | 'path'
            'constant_speed': 2.0,
            'speed_scale': 1.0,          # multiplies path speed
            'max_speed': 5.0,
            'search_back': 10,           # waypoints behind last nearest index
            'search_ahead': 60,          # waypoints ahead of last nearest index
            'relocalise_dist': 2.0,      # m; beyond this, do a full nearest search
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        self.xy = None
        self.idx = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(WaypointArray, self.p['waypoints_topic'],
                                 self.on_waypoints, latched)
        self.create_subscription(Odometry, self.p['pose_topic'], self.on_pose, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.p['drive_topic'], 10)
        self.marker_pub = self.create_publisher(Marker, '/pp/lookahead', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(
            f'speed_mode {self.p["speed_mode"]}, understeer_gradient '
            f'{self.p["understeer_gradient"]}; waiting for {self.p["waypoints_topic"]} ...')

    # ---------- path ----------
    def on_waypoints(self, msg):
        wp = msg.waypoints
        if len(wp) < 2:
            self.get_logger().error('received fewer than 2 waypoints')
            return
        self.frame = msg.header.frame_id
        self.xy = np.array([[w.x_m, w.y_m] for w in wp])
        self.v = np.array([w.vx_mps for w in wp])
        s = np.array([w.s_m for w in wp])
        s -= s[0]
        self.lap_len = s[-1] + float(np.hypot(*(self.xy[0] - self.xy[-1])))
        self.s = s
        # closed-loop copies for interpolation
        self.s_ext = np.append(s, self.lap_len)
        self.xy_ext = np.vstack([self.xy, self.xy[:1]])
        self.idx = None
        self.get_logger().info(f'path "{msg.source}": {len(wp)} pts, '
                               f'{self.lap_len:.2f} m, frame "{self.frame}"')

    def nearest(self, x, y):
        n = len(self.xy)
        if self.idx is not None:
            cand = np.arange(self.idx - self.p['search_back'],
                             self.idx + self.p['search_ahead'] + 1) % n
            d = np.hypot(self.xy[cand, 0] - x, self.xy[cand, 1] - y)
            k = int(np.argmin(d))
            if d[k] < self.p['relocalise_dist']:
                self.idx = int(cand[k])
                return self.idx
        d = np.hypot(self.xy[:, 0] - x, self.xy[:, 1] - y)   # full search (start / reset)
        self.idx = int(np.argmin(d))
        return self.idx

    def point_at(self, s_query):
        s_q = s_query % self.lap_len
        return (float(np.interp(s_q, self.s_ext, self.xy_ext[:, 0])),
                float(np.interp(s_q, self.s_ext, self.xy_ext[:, 1])))

    # ---------- control ----------
    def on_pose(self, msg):
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

        # pure pursuit geometry is about the rear axle
        x -= self.p['rear_axle_offset'] * math.cos(yaw)
        y -= self.p['rear_axle_offset'] * math.sin(yaw)

        v_now = abs(msg.twist.twist.linear.x)
        L = float(np.clip(self.p['lookahead_base'] + self.p['lookahead_gain'] * v_now,
                          self.p['lookahead_min'], self.p['lookahead_max']))

        i = self.nearest(x, y)
        tx, ty = self.point_at(self.s[i] + L)

        # target in vehicle frame
        dx, dy = tx - x, ty - y
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
        ld2 = max(lx * lx + ly * ly, 1e-6)
        kappa_pp = 2.0 * ly / ld2                                    # pure pursuit arc curvature
        steer = (math.atan(self.p['wheelbase'] * kappa_pp)           # kinematic part
                 + self.p['understeer_gradient'] * v_now ** 2 * kappa_pp)   # understeer comp.
        steer = float(np.clip(steer, -self.p['max_steer'], self.p['max_steer']))

        if self.p['speed_mode'] == 'path':
            speed = float(self.v[i]) * self.p['speed_scale']
        else:
            speed = self.p['constant_speed']
        speed = min(speed, self.p['max_speed'])

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.steering_angle = steer
        cmd.drive.speed = speed
        self.drive_pub.publish(cmd)
        self.publish_marker(tx, ty)

    def publish_marker(self, x, y):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'pp', 0, Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 1.0
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()