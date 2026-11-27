"""Rotate the raw LiDAR scan into the base_link orientation.

Subscribes to scan_raw (sllidar driver, laser frame) and republishes on scan
with angle 0 pointing straight ahead and positive angles to the left, so
algorithms see the same scan convention as in simulation.

Only a full 360 degree scan can be rotated by reordering the beams; for a
partial scan the yaw offset is applied to angle_min instead.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):

    def __init__(self):
        super().__init__('scan_filter_node')
        # Yaw of the laser frame relative to base_link (same as the static TF).
        self.yaw_offset = self.declare_parameter('yaw_offset', math.pi).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value

        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos_profile_sensor_data)
        self.warned_partial = False

    def on_scan(self, msg):
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment <= 0.0:
            return
        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        full_scan = abs(n * msg.angle_increment - 2.0 * math.pi) < 3.0 * msg.angle_increment
        if full_scan:
            # Shift the beams by a whole number of steps (like numpy.roll) and put the
            # leftover fraction of a step into angle_min, so angles stay in the same
            # [angle_min, angle_max] window and the rotation is exact.
            shift = round(self.yaw_offset / msg.angle_increment)
            residual = self.yaw_offset - shift * msg.angle_increment
            k = shift % n
            out.ranges = list(msg.ranges[n - k:]) + list(msg.ranges[:n - k])
            if len(msg.intensities) == n:
                out.intensities = (list(msg.intensities[n - k:])
                                   + list(msg.intensities[:n - k]))
            out.angle_min = msg.angle_min + residual
            out.angle_max = msg.angle_max + residual
        else:
            if not self.warned_partial:
                self.get_logger().warn('Partial scan: applying yaw offset to angle_min only')
                self.warned_partial = True
            out.ranges = msg.ranges
            out.intensities = msg.intensities
            out.angle_min = msg.angle_min + self.yaw_offset
            out.angle_max = msg.angle_max + self.yaw_offset
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
