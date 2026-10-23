#!/usr/bin/env python3
"""
Two-ray wall-tracking geometry from the Penn "Reactive Methods for
Navigation - PID for Wall-following" lecture:

    - Cast two LiDAR rays on the side we're following:
        b : perpendicular to the car (straight left or straight right)
        a : `theta` degrees away from b, angled towards the front
    - alpha = angle of the car's heading relative to the wall
    - Dt     = current perpendicular distance to the wall
    - Dt1    = Dt projected forward by `lookahead_time` seconds, to
               compensate for the car having moved by the time the
               steering command actually takes effect
    - error  = desired_distance - Dt1   (>0 means "too close to the wall")

Pure data in / data out -- takes a ScanProcessor as a dependency for
the actual beam lookups. Independently unit-testable outside a ROS graph.
"""

import math


class WallErrorEstimatorConfig:
    def __init__(self):
        self.wall_side = 'left'
        self.theta_deg = 50.0
        self.desired_distance = 1.0
        self.lookahead_time = 0.3
        self.median_spread_deg = 2.0


class WallErrorEstimator:

    def __init__(self, config: WallErrorEstimatorConfig, scan_processor):
        self.config = config
        self.scan_processor = scan_processor
        self._configure()

    def _configure(self):
        wall_side = self.config.wall_side.lower()
        if wall_side not in ('left', 'right'):
            wall_side = 'left'  # node-level caller logs the warning; see pid_node.py
        self.config.wall_side = wall_side

        self.theta = math.radians(self.config.theta_deg)

        # Steering sign convention (REP103 / ackermann_msgs):
        #   +steering_angle -> front wheels turn LEFT (CCW yaw)
        # For the LEFT wall: too close (error > 0) must steer RIGHT (negative).
        # For the RIGHT wall: too close (error > 0) must steer LEFT (positive).
        # Consumed by PIDController as config.side_sign.
        self.side_sign = -1.0 if wall_side == 'left' else 1.0

        # b is the perpendicular ray: +90 deg (left) or -90 deg (right).
        # a sits `theta` closer to the front of the car than b.
        if wall_side == 'left':
            self.b_angle = math.pi / 2.0
            self.a_angle = math.pi / 2.0 - self.theta
        else:
            self.b_angle = -math.pi / 2.0
            self.a_angle = -math.pi / 2.0 + self.theta

    def compute_error(self, range_data, angle_min, angle_increment, num_ranges, velocity):
        """Returns (error, alpha, Dt, a, b)."""
        a = self.scan_processor.get_range_median(
            range_data, self.a_angle, angle_min, angle_increment, num_ranges,
            self.config.median_spread_deg)
        b = self.scan_processor.get_range_median(
            range_data, self.b_angle, angle_min, angle_increment, num_ranges,
            self.config.median_spread_deg)

        alpha = math.atan2(a * math.cos(self.theta) - b, a * math.sin(self.theta))
        Dt = b * math.cos(alpha)

        # Side-agnostic: a_angle/b_angle are already mirrored per side above,
        # so alpha already carries the correct sign for whichever wall we're
        # tracking. Do NOT multiply by side_sign here -- see PIDController,
        # which is the only place side_sign gets applied.
        Dt1 = Dt + velocity * self.config.lookahead_time * math.sin(alpha)

        error = self.config.desired_distance - Dt1
        return error, alpha, Dt, a, b