#!/usr/bin/env python3
"""
Computes commanded speed as the minimum of two independent limits: one
from how sharp the commanded steering angle is, one from how physically
narrow the chosen gap is. A narrow needle-shaped opening can demand only
a modest steering angle while still requiring a hard slowdown, which the
angle-based limit alone can't see. Pure data in / data out -- no ROS
node dependency.
"""

import math


class SpeedPlannerConfig:
    """Speed limits and scaling settings for SpeedPlanner."""

    def __init__(self):
        """Set the default speed settings."""
        self.max_speed = 4.0               # m/s, straight-line cap
        self.min_speed = 1.0               # m/s, sharp-turn / narrow-gap floor
        self.speed_scale_angle_deg = 20.0  # steering angle (deg) at which speed hits min_speed
        self.min_safe_clearance = 0.3      # (m) clearance beyond car_width for max_speed
        self.car_width = 0.30              # actual physical car width (m)


class SpeedPlanner:
    """Compute the speed command from the steering angle and the gap width."""

    def __init__(self, config: SpeedPlannerConfig):
        """Store the configuration."""
        self.config = config

    def speed_from_steering(self, steering_angle):
        """Return a speed that falls linearly from max to min as the steering angle grows."""
        angle_frac = min(
            abs(steering_angle) / math.radians(self.config.speed_scale_angle_deg), 1.0)
        return self.config.max_speed - angle_frac * (
            self.config.max_speed - self.config.min_speed)

    def speed_from_gap_width(self, gap_width):
        """Return a speed that rises linearly from min to max with the clearance in the gap."""
        clearance = max(0.0, gap_width - self.config.car_width)
        safety_frac = min(clearance / self.config.min_safe_clearance, 1.0)
        return self.config.min_speed + (
            self.config.max_speed - self.config.min_speed) * safety_frac

    def compute_speed(self, steering_angle, gap_width):
        """min(speed_from_steering, speed_from_gap_width)."""
        return min(self.speed_from_steering(steering_angle),
                   self.speed_from_gap_width(gap_width))

    def min_speed(self):
        """Return the configured minimum speed."""
        return self.config.min_speed

    def max_speed(self):
        """Return the configured maximum speed."""
        return self.config.max_speed
