#!/usr/bin/env python3
"""
Pure-logic marker builder for pid_wall, mirroring gap_follow_pkg's
MarkerVisualizer pattern: no ROS node dependency, data in / Marker
objects out.

Publishes (wired in by pid_node.py):
    /pid_wall/debug_markers   (MarkerArray) -- rays, target line, status text
    /pid_wall/steering_marker (Marker)      -- steering direction arrow
"""

import math

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class MarkerVisualizerConfig:
    """Frame and sizes for the wall-following RViz markers."""

    def __init__(self):
        """Set the default frame and marker sizes."""
        self.base_frame_id = 'ego_racecar/base_link'
        self.ray_line_width = 0.02
        self.target_line_length = 3.0     # m, how far forward to draw the "hold this line" marker
        self.arrow_length_min = 0.3
        self.arrow_length_max = 1.0
        self.arrow_shaft_diameter = 0.03
        self.arrow_head_diameter = 0.06
        self.arrow_head_length = 0.1
        self.text_height = 0.3
        self.text_z_offset = 0.6


class MarkerVisualizer:
    """Build RViz markers for the wall-following controller."""

    def __init__(self, config: MarkerVisualizerConfig):
        """Store the configuration."""
        self.config = config

    def _base_marker(self, marker_id, ns, marker_type, stamp):
        """Return a marker with the common header, namespace, id and type."""
        m = Marker()
        m.header.frame_id = self.config.base_frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def make_ray_marker(self, angle, distance, marker_id, r, g, b, stamp):
        """LINE_STRIP from the car origin to the LiDAR hit point for ray a or b."""
        m = self._base_marker(marker_id, 'rays', Marker.LINE_STRIP, stamp)
        m.scale.x = self.config.ray_line_width
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.9
        m.points = [
            Point(x=0.0, y=0.0, z=0.05),
            Point(x=distance * math.cos(angle), y=distance * math.sin(angle), z=0.05),
        ]
        return m

    def make_target_line_marker(self, wall_side, desired_distance, stamp):
        """
        Line at y = +-desired_distance (car frame), running forward --
        the line the controller is trying to hold the car on.
        """
        m = self._base_marker(0, 'target_line', Marker.LINE_STRIP, stamp)
        m.scale.x = self.config.ray_line_width
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 1.0, 0.6
        y = desired_distance if wall_side == 'left' else -desired_distance
        m.points = [
            Point(x=0.0, y=y, z=0.02),
            Point(x=self.config.target_line_length, y=y, z=0.02),
        ]
        return m

    def make_status_text_marker(self, error, alpha, Dt, speed, stamp):
        """Return a text marker with error, look-ahead distance, wall angle and speed."""
        m = self._base_marker(0, 'status_text', Marker.TEXT_VIEW_FACING, stamp)
        m.scale.z = self.config.text_height
        err_frac = min(abs(error) / 0.5, 1.0)  # green when tracking well, red as error grows
        m.color.r, m.color.g, m.color.b, m.color.a = err_frac, 1.0 - err_frac, 0.0, 1.0
        m.pose.position.z = self.config.text_z_offset
        m.text = (
            f'err={error:+.2f}m  Dt={Dt:.2f}m  '
            f'alpha={math.degrees(alpha):+.1f}deg  v={speed:.2f}m/s'
        )
        return m

    def make_steering_arrow_marker(self, steering_angle, max_steering_angle, stamp):
        """Return an arrow marker along the steering angle, scaled by its magnitude."""
        m = self._base_marker(0, 'steering_arrow', Marker.ARROW, stamp)
        frac = min(abs(steering_angle) / max(max_steering_angle, 1e-6), 1.0)
        length = self.config.arrow_length_min + frac * (
            self.config.arrow_length_max - self.config.arrow_length_min)
        m.scale.x = self.config.arrow_shaft_diameter
        m.scale.y = self.config.arrow_head_diameter
        m.scale.z = self.config.arrow_head_length
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.9
        m.points = [
            Point(x=0.0, y=0.0, z=0.1),
            Point(x=length * math.cos(steering_angle), y=length * math.sin(steering_angle), z=0.1),
        ]
        return m