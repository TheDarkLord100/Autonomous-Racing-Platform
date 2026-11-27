#!/usr/bin/env python3
"""
Forward-looking speed governor, independent of the wall-tracking PID
loop: linearly scales velocity down as the closest reading in a small
forward cone shrinks, so the car brakes for a hairpin/apex regardless
of what the wall-following error currently says. Pure data in / data
out -- takes a ScanProcessor as a dependency.
"""

import math


class WallSpeedPlannerConfig:
    def __init__(self):
        self.cruise_velocity = 1.0
        self.min_velocity = 0.8
        self.front_cone_deg = 20.0
        self.min_front_distance = 1.2
        self.slow_front_distance = 3.0


class WallSpeedPlanner:

    def __init__(self, config: WallSpeedPlannerConfig, scan_processor):
        self.config = config
        self.scan_processor = scan_processor
        self.front_cone = math.radians(config.front_cone_deg)

    def compute_velocity(self, range_data, angle_min, angle_increment, num_ranges):
        front_min = min(
            self.scan_processor.get_range(
                range_data, a, angle_min, angle_increment, num_ranges)
            for a in (-self.front_cone, -self.front_cone / 2, 0.0,
                      self.front_cone / 2, self.front_cone)
        )
        if front_min >= self.config.slow_front_distance:
            return self.config.cruise_velocity
        if front_min <= self.config.min_front_distance:
            return self.config.min_velocity
        frac = (front_min - self.config.min_front_distance) / (
            self.config.slow_front_distance - self.config.min_front_distance)
        return self.config.min_velocity + frac * (
            self.config.cruise_velocity - self.config.min_velocity)