#!/usr/bin/env python3
"""Standard PID with anti-windup clamp and a steering-limit clamp.
Pure data in / data out -- holds only its own integral/prev_error
state, no ROS dependency.
"""


class PIDControllerConfig:
    def __init__(self):
        self.kp = 1.2
        self.ki = 0.0
        self.kd = 0.15
        self.integral_limit = 3.0
        self.max_steering_angle = 0.4189
        self.side_sign = -1.0  # set from WallErrorEstimator.side_sign


class PIDController:

    def __init__(self, config: PIDControllerConfig):
        self.config = config
        self.integral = 0.0
        self.prev_error = None

    def compute(self, error, dt):
        if dt <= 0.0:
            dt = 1e-3

        self.integral += error * dt
        self.integral = max(-self.config.integral_limit,
                             min(self.config.integral_limit, self.integral))

        if self.prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        raw = (self.config.kp * error
               + self.config.ki * self.integral
               + self.config.kd * derivative)

        steering_angle = self.config.side_sign * raw
        steering_angle = max(-self.config.max_steering_angle,
                              min(self.config.max_steering_angle, steering_angle))
        return steering_angle