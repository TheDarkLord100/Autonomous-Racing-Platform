#!/usr/bin/env python3
"""Load an offline raceline CSV and publish it once (latched).

CSV columns (from planning_scripts/optimise_raceline.py):
s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, w_left_m, w_right_m
"""
import math
from pathlib import Path as FsPath

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from race_msgs.msg import Waypoint, WaypointArray

N_COLS = 9


class RacelinePublisher(Node):
    """Load a raceline CSV once and publish it as a latched WaypointArray and Path."""

    def __init__(self):
        """Declare parameters, load and check the CSV, and publish both messages.

        Raises RuntimeError if ``csv_path`` is empty or the CSV does not have the expected
        number of columns.
        """
        super().__init__('raceline_publisher')
        self.declare_parameter('csv_path', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('waypoints_topic', '/global_waypoints')
        self.declare_parameter('path_topic', '/global_path')

        csv_path = self.get_parameter('csv_path').value
        frame_id = self.get_parameter('frame_id').value
        if not csv_path:
            raise RuntimeError('parameter csv_path is empty')

        data = np.loadtxt(csv_path, delimiter=',', comments='#', ndmin=2)
        if data.shape[1] != N_COLS:
            raise RuntimeError(f'{csv_path}: expected {N_COLS} columns, got {data.shape[1]}')

        # Latched: late subscribers (pure pursuit, RViz) still get the one message
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.wp_pub = self.create_publisher(
            WaypointArray, self.get_parameter('waypoints_topic').value, qos)
        self.path_pub = self.create_publisher(
            Path, self.get_parameter('path_topic').value, qos)

        stamp = self.get_clock().now().to_msg()
        self.wp_pub.publish(self.to_waypoints(data, frame_id, stamp, FsPath(csv_path).stem))
        self.path_pub.publish(self.to_path(data, frame_id, stamp))

        self.get_logger().info(
            f'published {len(data)} waypoints from {csv_path} '
            f'(length {data[-1, 0]:.2f} m, v {data[:, 5].min():.2f}-{data[:, 5].max():.2f} m/s)')

    @staticmethod
    def to_waypoints(data, frame_id, stamp, source):
        """Convert CSV rows to a race_msgs/WaypointArray."""
        msg = WaypointArray()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.source = source
        for r in data:
            msg.waypoints.append(Waypoint(
                s_m=r[0], x_m=r[1], y_m=r[2], psi_rad=r[3], kappa_radpm=r[4],
                vx_mps=r[5], ax_mps2=r[6], w_left_m=r[7], w_right_m=r[8]))
        return msg

    @staticmethod
    def to_path(data, frame_id, stamp):
        """Convert CSV rows to a nav_msgs/Path (position and heading only)."""
        msg = Path()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        for r in data:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = r[1]
            p.pose.position.y = r[2]
            p.pose.orientation.z = math.sin(r[3] / 2.0)
            p.pose.orientation.w = math.cos(r[3] / 2.0)
            msg.poses.append(p)
        return msg


def main(args=None):
    """Run the raceline publisher node until shutdown."""
    rclpy.init(args=args)
    node = RacelinePublisher()
    try:
        rclpy.spin(node)          # keep alive so the latched message stays available
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()