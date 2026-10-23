#!/usr/bin/env python3
"""
Standalone reactive Follow-The-Gap node -- thin ROS 2 orchestrator.

Owns the ROS interfaces and declared parameters, and wires together
the pure-logic modules that do the actual work. It contains no
algorithmic logic of its own beyond the pipeline order:

    ScanProcessor     -- clean/clip, FOV mask, disparity extension, safety bubble
    GapFinder         -- largest free run, edge trimming, physical gap width
    TargetSelector    -- target pick, centering blend, steering clamp, side safety
    SpeedPlanner      -- min(steering-based, gap-width-based) speed
    MarkerVisualizer  -- rviz debug markers

Subscribes:  /scan  (sensor_msgs/LaserScan)
Publishes:   /drive (ackermann_msgs/AckermannDriveStamped)
             /gap_follow/steering_marker (visualization_msgs/Marker)
             /gap_follow/debug_markers   (visualization_msgs/MarkerArray)
"""

import math

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from gap_follow.gap_finder import GapFinder, GapFinderConfig
from gap_follow.marker_visualizer import MarkerVisualizer, MarkerVisualizerConfig
from gap_follow.scan_processor import ScanProcessor, ScanProcessorConfig
from gap_follow.speed_planner import SpeedPlanner, SpeedPlannerConfig
from gap_follow.target_selector import TargetSelector, TargetSelectorConfig


class GapFollowNode(Node):

    def __init__(self):
        super().__init__('gap_follow_node')

        # ---------------- declare parameters ----------------
        max_range = self.declare_parameter('max_range', 4.0).value
        bubble_radius = self.declare_parameter('bubble_radius', 0.35).value
        min_gap_points = self.declare_parameter('min_gap_points', 5).value
        max_speed = self.declare_parameter('max_speed', 4.0).value
        min_speed = self.declare_parameter('min_speed', 1.0).value
        fov_deg = self.declare_parameter('fov_deg', 180.0).value
        car_width = self.declare_parameter('car_width', 0.30).value
        disparity_threshold = self.declare_parameter('disparity_threshold', 0.3).value
        gap_edge_margin = self.declare_parameter('gap_edge_margin', 0.10).value
        speed_scale_angle_deg = self.declare_parameter('speed_scale_angle_deg', 20.0).value
        center_gain = self.declare_parameter('center_gain', 0.35).value
        center_blend_angle_deg = self.declare_parameter('center_blend_angle_deg', 15.0).value
        side_probe_window_deg = self.declare_parameter('side_probe_window_deg', 8.0).value
        max_steering_angle_deg = self.declare_parameter('max_steering_angle_deg', 24.0).value
        min_safe_clearance = self.declare_parameter('min_safe_clearance', 0.3).value
        base_frame_id = self.declare_parameter(
            'base_frame_id', 'ego_racecar/base_link').value
        marker_length_min = self.declare_parameter('marker_length_min', 0.3).value
        marker_length_max = self.declare_parameter('marker_length_max', 1.0).value
        marker_shaft_diameter = self.declare_parameter('marker_shaft_diameter', 0.03).value
        marker_head_diameter = self.declare_parameter('marker_head_diameter', 0.06).value
        marker_head_length = self.declare_parameter('marker_head_length', 0.1).value

        # Half-width the disparity extender / gap trim sweep outward, plus a
        # small safety pad. Shared by ScanProcessor and GapFinder.
        car_half_width = car_width / 2.0 + 0.05

        # ---------------- wire up modules ----------------
        scan_cfg = ScanProcessorConfig()
        scan_cfg.max_range = max_range
        scan_cfg.fov_deg = fov_deg
        scan_cfg.bubble_radius = bubble_radius
        scan_cfg.disparity_threshold = disparity_threshold
        scan_cfg.car_half_width = car_half_width
        self.scan_processor = ScanProcessor(scan_cfg)

        gap_cfg = GapFinderConfig()
        gap_cfg.min_gap_points = min_gap_points
        gap_cfg.gap_edge_margin = gap_edge_margin
        gap_cfg.car_half_width = car_half_width
        self.gap_finder = GapFinder(gap_cfg)

        target_cfg = TargetSelectorConfig()
        target_cfg.center_gain = center_gain
        target_cfg.center_blend_angle_deg = center_blend_angle_deg
        target_cfg.side_probe_window_deg = side_probe_window_deg
        target_cfg.max_steering_angle_deg = max_steering_angle_deg
        target_cfg.max_range = max_range
        self.target_selector = TargetSelector(target_cfg)

        speed_cfg = SpeedPlannerConfig()
        speed_cfg.max_speed = max_speed
        speed_cfg.min_speed = min_speed
        speed_cfg.speed_scale_angle_deg = speed_scale_angle_deg
        speed_cfg.min_safe_clearance = min_safe_clearance
        speed_cfg.car_width = car_width
        self.speed_planner = SpeedPlanner(speed_cfg)

        marker_cfg = MarkerVisualizerConfig()
        marker_cfg.base_frame_id = base_frame_id
        marker_cfg.marker_length_min = marker_length_min
        marker_cfg.marker_length_max = marker_length_max
        marker_cfg.marker_shaft_diameter = marker_shaft_diameter
        marker_cfg.marker_head_diameter = marker_head_diameter
        marker_cfg.marker_head_length = marker_head_length
        self.marker_visualizer = MarkerVisualizer(marker_cfg)

        # ---------------- ROS interfaces ----------------
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/gap_follow/steering_marker', 10)
        self.debug_marker_pub = self.create_publisher(
            MarkerArray, '/gap_follow/debug_markers', 10)

        self.get_logger().info('Gap Follow node started')

    def publish_drive(self, steering_angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg: LaserScan):
        t_start = self.get_clock().now()
        stamp = msg.header.stamp
        sp = self.scan_processor
        ts = self.target_selector
        viz = self.marker_visualizer

        ranges, angles, mask = sp.preprocess(
            msg.ranges, msg.angle_min, msg.angle_increment)

        # Measure true left/right wall distance from the unmutated scan, before
        # extend_disparities/zero_bubble alter ranges for gap-finding purposes.
        side_window_rad = ts.side_probe_window_rad()
        left_dist = ts.compute_wall_distance(ranges, angles, math.pi / 2.0, side_window_rad)
        right_dist = ts.compute_wall_distance(ranges, angles, -math.pi / 2.0, side_window_rad)

        sp.extend_disparities(ranges, mask, msg.angle_increment)

        closest_idx, closest_dist = sp.find_closest_point(ranges, mask)
        sp.zero_bubble(ranges, closest_idx, closest_dist, msg.angle_increment)

        gap = self.gap_finder.find_max_gap(ranges, mask)
        if gap is None:
            # No viable gap in FOV: brake hard, hold last heading
            debug_markers = MarkerArray()
            debug_markers.markers.append(
                viz.make_scan_points_marker(ranges, angles, stamp))
            debug_markers.markers.append(
                viz.make_text_marker('NO GAP', 1.0, 0.0, 0.0, 0.5, stamp))
            self.debug_marker_pub.publish(debug_markers)

            self.publish_drive(0.0, self.speed_planner.min_speed() * 0.3)
            return

        raw_gap_start, raw_gap_end = gap
        gap_start, gap_end = self.gap_finder.trim_gap_edges(
            ranges, raw_gap_start, raw_gap_end, msg.angle_increment)

        target_idx = ts.best_point_in_gap(ranges, gap_start, gap_end)
        ftg_angle = float(angles[target_idx])

        # Blend in a centering correction that fades out as the FTG-demanded
        # turn gets sharper -- gentle sections get pulled toward corridor
        # center, aggressive turns are left entirely to the gap target.
        final_angle = ts.compute_final_angle(ftg_angle, left_dist, right_dist)
        # Tweak 3 hard override: abort the turn if we're about to clip a wall
        # on the side we're turning into.
        if abs(final_angle) > 1e-3:
            turn_sign = 1.0 if final_angle > 0.0 else -1.0
            if ts.side_too_close(ranges, angles, turn_sign):
                final_angle = 0.0
        max_steering_rad = ts.max_steering_rad()

        self.marker_pub.publish(viz.make_marker(final_angle, max_steering_rad, stamp))

        # Speed floor based on the gap's real physical width, not just the
        # steering angle -- a narrow needle-shaped opening (like a switchback
        # apex) can demand only a modest angle while still requiring a hard
        # slowdown, which the angle-based limit alone can't see.
        gap_width = self.gap_finder.compute_gap_width(
            ranges, angles, raw_gap_start, raw_gap_end)
        speed = self.speed_planner.compute_speed(final_angle, gap_width)

        debug_markers = MarkerArray()
        debug_markers.markers.append(viz.make_scan_points_marker(ranges, angles, stamp))
        debug_markers.markers.append(viz.make_gap_marker(
            ranges, angles, raw_gap_start, raw_gap_end, 0, 1.0, 1.0, 0.0, stamp))
        debug_markers.markers.append(viz.make_gap_marker(
            ranges, angles, gap_start, gap_end, 1, 0.0, 1.0, 0.0, stamp))
        debug_markers.markers.append(viz.make_target_marker(
            float(ranges[target_idx]), ftg_angle, stamp))
        debug_markers.markers.append(viz.make_speed_text_marker(
            speed, self.speed_planner.min_speed(), self.speed_planner.max_speed(), stamp))
        self.debug_marker_pub.publish(debug_markers)

        self.publish_drive(final_angle, speed)

        elapsed_ms = (self.get_clock().now() - t_start).nanoseconds / 1e6
        if elapsed_ms > 10.0:
            self.get_logger().warn(
                f'scan_callback took {elapsed_ms:.2f} ms (budget: 10ms for 40Hz LiDAR)')


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowNode()
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
