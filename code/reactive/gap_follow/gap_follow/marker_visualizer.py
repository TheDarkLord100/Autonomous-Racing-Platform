#!/usr/bin/env python3
"""
Builds visualization_msgs/Marker messages for every stage of the
pipeline: the commanded-steering arrow, the processed scan points, the
raw/trimmed gap, the raw target point, and a speed readout. Only builds
messages -- publishing stays with the node, so this class has no
dependency on a live ROS graph.
"""

import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
import numpy as np
from visualization_msgs.msg import Marker


class MarkerVisualizerConfig:
    """Frame and sizes for the gap-follow RViz markers."""

    def __init__(self):
        """Set the default frame and marker sizes."""
        self.base_frame_id = 'ego_racecar/base_link'
        self.marker_length_min = 0.3
        self.marker_length_max = 1.0
        self.marker_shaft_diameter = 0.03
        self.marker_head_diameter = 0.06
        self.marker_head_length = 0.1
        self.scan_point_scale = 0.03
        self.gap_point_scale = 0.04
        self.target_sphere_scale = 0.12
        self.text_height = 0.25


class MarkerVisualizer:
    """Build RViz markers for the gap-follow controller."""

    LIFETIME = Duration(sec=0, nanosec=300_000_000)  # 0.3 s

    def __init__(self, config: MarkerVisualizerConfig):
        """Store the configuration."""
        self.config = config

    def _base_marker(self, ns, marker_id, marker_type, stamp):
        """Return a marker with the common header, namespace, id, type and lifetime."""
        m = Marker()
        m.header.frame_id = self.config.base_frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.lifetime = self.LIFETIME
        return m

    @staticmethod
    def _to_points(ranges, angles, z):
        """(x, y, z) Points for every beam with range > 0."""
        valid = ranges > 0.0  # blanked by bubble/disparity -- not "seen" free space
        r = ranges[valid].astype(np.float64)
        a = angles[valid].astype(np.float64)
        xs = (r * np.cos(a)).tolist()
        ys = (r * np.sin(a)).tolist()
        return [Point(x=x, y=y, z=z) for x, y in zip(xs, ys)]

    def make_marker(self, angle, max_steering_rad, stamp):
        """
        Arrow rooted at the car's origin, pointing along the commanded
        (final, centering-blended) steering angle. Length and color scale
        with turn intensity.
        """
        m = self._base_marker('gap_follow', 0, Marker.ARROW, stamp)

        angle_frac = min(abs(angle) / max_steering_rad, 1.0)
        length = self.config.marker_length_min + (
            self.config.marker_length_max - self.config.marker_length_min) * angle_frac

        start = Point(x=0.0, y=0.0, z=0.15)  # lift slightly above the car body
        end = Point(x=length * math.cos(angle), y=length * math.sin(angle), z=0.15)
        m.points = [start, end]

        m.scale.x = self.config.marker_shaft_diameter
        m.scale.y = self.config.marker_head_diameter
        m.scale.z = self.config.marker_head_length

        m.color.r = float(angle_frac)
        m.color.g = float(1.0 - angle_frac)
        m.color.b = 0.0
        m.color.a = 1.0
        return m

    def make_scan_points_marker(self, ranges, angles, stamp):
        """
        Every valid (range > 0) beam in the processed scan -- what the
        algorithm actually sees after clipping, disparity extension and
        the safety bubble, as opposed to the raw incoming LiDAR scan.
        """
        m = self._base_marker('gap_follow_scan', 0, Marker.POINTS, stamp)
        m.scale.x = self.config.scan_point_scale
        m.scale.y = self.config.scan_point_scale
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.8, 1.0, 0.8
        m.points = self._to_points(ranges, angles, 0.05)
        return m

    def make_gap_marker(self, ranges, angles, start, end, marker_id, r, g, b, stamp):
        """
        Index range [start, end) as (x, y) points. Used for both the raw
        found gap and the trimmed search gap; caller picks id/color.
        """
        m = self._base_marker('gap_follow_gap', marker_id, Marker.POINTS, stamp)
        m.scale.x = self.config.gap_point_scale
        m.scale.y = self.config.gap_point_scale
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.9
        # z slightly above the scan-points layer so it renders on top
        m.points = self._to_points(ranges[start:end], angles[start:end], 0.06)
        return m

    def make_target_marker(self, rng, angle, stamp):
        """
        Sphere at the raw FTG-selected target point (before the centering
        blend), so the gap search's own pick is visible independently of
        the final blended steering arrow.
        """
        m = self._base_marker('gap_follow_target', 0, Marker.SPHERE, stamp)
        m.pose.position.x = rng * math.cos(angle)
        m.pose.position.y = rng * math.sin(angle)
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        s = self.config.target_sphere_scale
        m.scale.x, m.scale.y, m.scale.z = s, s, s
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0
        return m

    def make_speed_text_marker(self, speed, min_speed, max_speed, stamp):
        """Current commanded speed, colored red (min_speed) to green (max_speed)."""
        span = max(max_speed - min_speed, 1e-6)
        frac = max(0.0, min(1.0, (speed - min_speed) / span))
        return self.make_text_marker(
            f'{speed:.2f} m/s', 1.0 - frac, frac, 0.0, 0.5, stamp)

    def make_text_marker(self, text, r, g, b, z, stamp):
        """Generic floating text marker, e.g. for a "no gap found" alert."""
        m = self._base_marker('gap_follow_text', 0, Marker.TEXT_VIEW_FACING, stamp)
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.text = text
        m.scale.z = self.config.text_height
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        return m
