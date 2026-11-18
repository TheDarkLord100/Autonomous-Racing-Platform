#!/usr/bin/env python3
"""Localizer pose (per scan) -> Odometry at odometry rate, for the planner + controller.

Cartographer (/carto/pose, PoseStamped) and scan matching (/scan_match/odom, Odometry)
publish one estimate per scan. The local planner and pure pursuit expect an Odometry
stream with velocity, like the particle filter's /pf/odom. This node does what the PF
does internally:

    estimate_now = estimate_at_scan  (+)  (odometry_now  (-)  odometry_at_scan)

using the same odometry the localizer reads (GT or odom_noise), and copies the
odometry twist. Optionally it also publishes map -> odom (REP 105) for the belief view.

Logs "first estimate" once, which the bringup uses as the readiness signal.
"""
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    """Return the yaw angle of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """Return the composition a (+) b of two 2D poses."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    """Return the inverse of a 2D pose."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def stamp_sec(st):
    """Return a builtin_interfaces/Time as float seconds."""
    return st.sec + st.nanosec * 1e-9


class LocalizationRelay(Node):
    """ROS 2 node: republish a per-scan pose estimate at the odometry rate.

    The latest estimate is moved forward by the odometry since its stamp and published as
    nav_msgs/Odometry on ``output_topic`` on every odometry message.
    """

    def __init__(self):
        """Declare parameters, create the odometry buffer, publishers and subscribers."""
        super().__init__('localization_relay')
        p = self.declare_parameter
        self.input_topic = p('input_topic', '/carto/pose').value
        self.input_type = p('input_type', 'pose').value          # pose | odometry
        odom_topic = p('odom_topic', '/ego_racecar/odom').value
        self.output_topic = p('output_topic', '/loc/odom').value
        self.base_frame = p('base_frame', 'ego_racecar/base_link').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.publish_tf = p('publish_tf', False).value

        self.odom = deque(maxlen=300)        # (t, pose) ~3 s at 100 Hz
        self.anchor_est = None
        self.anchor_odom = None
        self.anchor_t = None
        self.frame = None
        self.br = TransformBroadcaster(self) if self.publish_tf else None

        self.pub = self.create_publisher(Odometry, self.output_topic, 10)
        msg_type = PoseStamped if self.input_type == 'pose' else Odometry
        self.create_subscription(msg_type, self.input_topic, self.on_estimate, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 50)
        self.get_logger().info(f'{self.input_topic} ({self.input_type}) + {odom_topic} -> '
                               f'{self.output_topic}; publish_tf={self.publish_tf}')

    def odom_at(self, t):
        """Odometry pose interpolated at time t (clamped to the buffer)."""
        if not self.odom:
            return None
        if t <= self.odom[0][0]:
            return self.odom[0][1]
        if t >= self.odom[-1][0]:
            return self.odom[-1][1]
        for (t0, p0), (t1, p1) in zip(self.odom, list(self.odom)[1:]):
            if t0 <= t <= t1:
                a = (t - t0) / max(t1 - t0, 1e-9)
                dyaw = math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2]))
                return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]),
                        p0[2] + a * dyaw)
        return self.odom[-1][1]

    def on_estimate(self, msg):
        """Anchor a new estimate to the odometry at its stamp."""
        pose = msg.pose if self.input_type == 'pose' else msg.pose.pose
        t = stamp_sec(msg.header.stamp)
        if self.anchor_t is not None and t <= self.anchor_t:
            return                            # same scan republished
        od = self.odom_at(t)
        if od is None:
            return
        self.anchor_est = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
        self.anchor_odom, self.anchor_t = od, t
        if self.frame is None:
            self.frame = msg.header.frame_id or 'map'
            self.get_logger().info(f'first estimate in "{self.frame}": '
                                   f'x={self.anchor_est[0]:.2f} y={self.anchor_est[1]:.2f}')
        if self.br is not None:               # map -> odom = estimate (+) inverse(odom)
            m2o = compose(self.anchor_est, inverse(od))
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id, tf.child_frame_id = self.frame, self.odom_frame
            tf.transform.translation.x, tf.transform.translation.y = m2o[0], m2o[1]
            tf.transform.rotation.z = math.sin(m2o[2] / 2)
            tf.transform.rotation.w = math.cos(m2o[2] / 2)
            self.br.sendTransform(tf)

    def on_odom(self, msg):
        """Buffer odometry and publish the anchored estimate moved forward to this message."""
        pp = msg.pose.pose
        pose = (pp.position.x, pp.position.y, yaw_of(pp.orientation))
        self.odom.append((stamp_sec(msg.header.stamp), pose))
        if self.anchor_est is None:
            return
        est = compose(self.anchor_est, compose(inverse(self.anchor_odom), pose))
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x, out.pose.pose.position.y = est[0], est[1]
        out.pose.pose.orientation.z = math.sin(est[2] / 2)
        out.pose.pose.orientation.w = math.cos(est[2] / 2)
        out.twist = msg.twist
        self.pub.publish(out)


def main(args=None):
    """Run the localization relay node until shutdown."""
    rclpy.init(args=args)
    node = LocalizationRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
