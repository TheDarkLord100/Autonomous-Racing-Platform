"""Republishes the sim's ground-truth odometry as drifting wheel odometry.

in:  input_topic  (Odometry, ground truth in the sim's map frame)
out: output_topic (Odometry, frame odom_frame, starts at the origin and drifts)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from .noise_model import OdometryNoise


def _quat_to_yaw(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class OdomNoiseNode(Node):

    def __init__(self):
        super().__init__('odom_noise')
        p = self.declare_parameter
        self.input_topic = p('input_topic', '/ego_racecar/odom').value
        self.output_topic = p('output_topic', '/ego_racecar/odom_noisy').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'ego_racecar/base_link').value
        seed = p('random_seed', 42).value

        self.noise = OdometryNoise(
            trans_scale_error=p('trans_scale_error', 0.05).value,
            rot_scale_error=p('rot_scale_error', 0.05).value,
            yaw_drift_per_m=p('yaw_drift_per_m', 0.005).value,
            trans_noise=p('trans_noise', 0.02).value,
            rot_noise_per_rad=p('rot_noise_per_rad', 0.02).value,
            rot_noise_per_m=p('rot_noise_per_m', 0.005).value,
            teleport_threshold=p('teleport_threshold', 1.0).value,
            seed=seed)

        self.pub = self.create_publisher(Odometry, self.output_topic, 20)
        self.create_subscription(Odometry, self.input_topic, self.cb, 20)
        self.get_logger().info(
            f'odom noise: {self.input_topic} -> {self.output_topic} '
            f'(seed {"random" if seed < 0 else seed})')

    def cb(self, msg):
        pp = msg.pose.pose
        true_pose = (pp.position.x, pp.position.y, _quat_to_yaw(pp.orientation))
        pose, teleported = self.noise.step(true_pose)
        if teleported:
            self.get_logger().info('car teleported (sim reset / initial pose): '
                                   'jump not added to odometry')

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = float(pose[0])
        out.pose.pose.position.y = float(pose[1])
        out.pose.pose.orientation.z = float(np.sin(0.5 * pose[2]))
        out.pose.pose.orientation.w = float(np.cos(0.5 * pose[2]))
        t = msg.twist.twist
        vx, vy, wz = self.noise.corrupt_twist(t.linear.x, t.linear.y, t.angular.z)
        out.twist.twist.linear.x = float(vx)
        out.twist.twist.linear.y = float(vy)
        out.twist.twist.angular.z = float(wz)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNoiseNode()
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