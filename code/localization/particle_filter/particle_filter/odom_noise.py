"""Synthetic wheel-odometry errors. No ROS imports.

Corrupts each true local motion step (dx, dy, dtheta) the way wheel odometry
goes wrong, then integrates the corrupted steps into a drifting odom pose.

Per step, with trans = hypot(dx, dy):
    dx'     = dx (1 + trans_scale_error) + N(0, trans_noise * trans)
    dy'     = dy                          + N(0, trans_noise * trans)
    dtheta' = dtheta (1 + rot_scale_error) + yaw_drift_per_m * dx
              + N(0, rot_noise_per_rad * |dtheta| + rot_noise_per_m * trans)

The scale errors and yaw drift are systematic, so they dominate over a lap, as
they do on a real car (wheel slip, wrong wheel radius, steering offset).
The white-noise terms add per-step jitter.
"""

import numpy as np

from .geometry import compose, relative_pose


class OdometryNoise:

    def __init__(self, trans_scale_error=0.05, rot_scale_error=0.05,
                 yaw_drift_per_m=0.005, trans_noise=0.02,
                 rot_noise_per_rad=0.02, rot_noise_per_m=0.005, rng=None):
        self.trans_scale_error = trans_scale_error
        self.rot_scale_error = rot_scale_error
        self.yaw_drift_per_m = yaw_drift_per_m
        self.trans_noise = trans_noise
        self.rot_noise_per_rad = rot_noise_per_rad
        self.rot_noise_per_m = rot_noise_per_m
        self.rng = rng if rng is not None else np.random.default_rng()

        self._last_true = None
        self.pose = np.zeros(3)   # drifting odom pose, starts at the odom-frame origin

    def corrupt(self, delta):
        """Corrupt one local motion step (3,) -> (3,)."""
        dx, dy, dth = float(delta[0]), float(delta[1]), float(delta[2])
        trans = float(np.hypot(dx, dy))
        sd_t = self.trans_noise * trans
        sd_r = self.rot_noise_per_rad * abs(dth) + self.rot_noise_per_m * trans
        n = self.rng.normal(0.0, 1.0, 3)
        return np.array([
            dx * (1.0 + self.trans_scale_error) + sd_t * n[0],
            dy + sd_t * n[1],
            dth * (1.0 + self.rot_scale_error) + self.yaw_drift_per_m * dx + sd_r * n[2],
        ])

    def step(self, true_pose):
        """Feed the next true pose; returns the updated noisy odom pose."""
        true_pose = np.asarray(true_pose, dtype=float)
        if self._last_true is not None:
            delta = relative_pose(self._last_true, true_pose)
            self.pose = compose(self.pose, self.corrupt(delta))
        self._last_true = true_pose
        return self.pose.copy()

    def corrupt_twist(self, vx, vy, wz):
        """Apply the same systematic errors to a body twist (no white noise)."""
        return (vx * (1.0 + self.trans_scale_error), vy,
                wz * (1.0 + self.rot_scale_error) + self.yaw_drift_per_m * vx)