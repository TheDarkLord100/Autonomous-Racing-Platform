#!/usr/bin/env python3
"""odom -> base_link TF from an Odometry topic: the odometry half of REP 105.

Used by the bringup when run.tf_source is 'localization': the gym stops publishing
its ground-truth map -> base_link, this node publishes odom -> base_link from the same
odometry the particle filter reads (GT or odom_noise), and the particle filter
publishes carto_map -> odom. base_link then sits where the stack believes the car is.

The message's own frame ids are ignored; the TF always uses odom_frame / base_frame,
so the tree matches what the particle filter assumes.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        topic = self.get_parameter('odom_topic').value
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self.on_odom, 20)
        self.get_logger().info(f'{topic} -> TF {self.odom_frame} -> {self.base_frame}')

    def on_odom(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t.transform.translation.x, t.transform.translation.y = p.x, p.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
