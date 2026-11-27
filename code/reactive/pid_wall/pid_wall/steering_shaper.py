#!/usr/bin/env python3
"""
Rate-limits the commanded steering angle so a sudden PID-output jump
(noisy alpha estimate, a curve caught early by the two-ray geometry)
can't jerk the servo in one control cycle. Pure data in / data out --
holds only its own previous-angle state.
"""


class SteeringShaperConfig:
    def __init__(self):
        self.max_steering_rate = 3.0  # rad/s


class SteeringShaper:

    def __init__(self, config: SteeringShaperConfig):
        self.config = config
        self.prev_angle = 0.0

    def shape(self, target_angle, dt):
        max_delta = self.config.max_steering_rate * max(dt, 1e-3)
        angle = max(self.prev_angle - max_delta,
                    min(self.prev_angle + max_delta, target_angle))
        self.prev_angle = angle
        return angle