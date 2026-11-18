#!/usr/bin/env python3
"""
Standard PID with anti-windup clamp, a steering-limit clamp, and
speed-based gain scheduling. Pure data in / data out -- holds only its
own integral/prev_error state, no ROS dependency.

Gain scheduling: the same steering angle moves the car sideways faster
the faster it drives (lateral response grows ~v^2), so fixed gains that
are calm at low speed oscillate at high speed. All three gains are
multiplied by

    scale = clamp((gain_ref_velocity / v)^2, gain_scale_min, gain_scale_max)

so kp/ki/kd act exactly as written at gain_ref_velocity, get stronger
in slow corners and softer on fast straights. Scaling P and D together
keeps both the loop's natural frequency and its damping roughly
constant across speeds. gain_ref_velocity <= 0 disables scheduling.
"""


class PIDControllerConfig:
    """Gains, limits and gain-scheduling settings for PIDController."""

    def __init__(self):
        """Set the default gains and limits."""
        self.kp = 1.2
        self.ki = 0.0
        self.kd = 0.15
        self.integral_limit = 3.0
        self.max_steering_angle = 0.4189
        self.side_sign = -1.0  # set from WallErrorEstimator.side_sign
        self.gain_ref_velocity = 0.0  # (m/s) <= 0 disables gain scheduling
        self.gain_scale_min = 0.5
        self.gain_scale_max = 3.0


class PIDController:
    """PID on the wall-distance error with speed-scheduled gains."""

    def __init__(self, config: PIDControllerConfig):
        """Store the configuration and reset the integral and previous error."""
        self.config = config
        self.integral = 0.0
        self.prev_error = None  # no derivative until we have a previous sample

    def gain_scale(self, velocity):
        """Return the gain scale (gain_ref_velocity / velocity)^2, clamped.

        Returns 1.0 when scheduling is disabled or the velocity is near zero.
        """
        c = self.config
        if c.gain_ref_velocity <= 0.0 or velocity <= 1e-3:
            return 1.0
        scale = (c.gain_ref_velocity / velocity) ** 2
        return max(c.gain_scale_min, min(c.gain_scale_max, scale))

    def compute(self, error, dt, velocity):
        """Return the steering angle for ``error`` after one PID step of length ``dt``."""
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

        scale = self.gain_scale(velocity)
        raw = scale * (self.config.kp * error
                       + self.config.ki * self.integral
                       + self.config.kd * derivative)

        steering_angle = self.config.side_sign * raw
        steering_angle = max(-self.config.max_steering_angle,
                             min(self.config.max_steering_angle, steering_angle))
        return steering_angle