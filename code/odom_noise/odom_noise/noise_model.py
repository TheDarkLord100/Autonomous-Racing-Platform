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
The white-noise terms add per-step jitter. With a fixed seed the white noise
is the same sequence every run, so the whole error is repeatable.

Steps longer than teleport_threshold (m) are treated as a simulator reset or
teleport, not motion: real wheel odometry would not see them, so the odom
pose is left unchanged.
"""

import numpy as np


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def _relative_pose(p0, p1):
    """Motion from p0 to p1 in p0's frame: (dx forward, dy left, dtheta)."""
    c, s = np.cos(p0[2]), np.sin(p0[2])
    ex, ey = p1[0] - p0[0], p1[1] - p0[1]
    return np.array([c * ex + s * ey, -s * ex + c * ey, _wrap(p1[2] - p0[2])])


def _compose(a, b):
    """Apply local motion b on top of pose a."""
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1],
                     a[1] + s * b[0] + c * b[1],
                     _wrap(a[2] + b[2])])


class OdometryNoise:

    def __init__(self, trans_scale_error=0.05, rot_scale_error=0.05,
                 yaw_drift_per_m=0.005, trans_noise=0.02,
                 rot_noise_per_rad=0.02, rot_noise_per_m=0.005,
                 teleport_threshold=1.0, seed=42):
        self.trans_scale_error = trans_scale_error
        self.rot_scale_error = rot_scale_error
        self.yaw_drift_per_m = yaw_drift_per_m
        self.trans_noise = trans_noise
        self.rot_noise_per_rad = rot_noise_per_rad
        self.rot_noise_per_m = rot_noise_per_m
        self.teleport_threshold = teleport_threshold
        self.rng = np.random.default_rng(None if seed is None or seed < 0 else seed)

        self._last_true = None
        self.pose = np.zeros(3)     # drifting odom pose, starts at the odom-frame origin
        self.teleports = 0          # number of steps rejected as teleports

    def corrupt(self, delta):
        """Corrupt one local motion step (3,) -> (3,)."""
        dx, dy, dth = float(delta[0]), float(delta[1]), float(delta[2])
        trans = float(np.hypot(dx, dy))
        sd_t = self.trans_noise * trans
        sd_r = self.rot_noise_per_rad * abs(dth) + self.rot_noise_per_m * trans
        n = self.rng.normal(0.0, 1.0, 3)    # always 3 draws per step: keeps the sequence aligned
        return np.array([
            dx * (1.0 + self.trans_scale_error) + sd_t * n[0],
            dy + sd_t * n[1],
            dth * (1.0 + self.rot_scale_error) + self.yaw_drift_per_m * dx + sd_r * n[2],
        ])

    def step(self, true_pose):
        """Feed the next true pose; returns (noisy odom pose, teleported?)."""
        true_pose = np.asarray(true_pose, dtype=float)
        teleported = False
        if self._last_true is not None:
            delta = _relative_pose(self._last_true, true_pose)
            if np.hypot(delta[0], delta[1]) > self.teleport_threshold:
                teleported = True
                self.teleports += 1
            else:
                self.pose = _compose(self.pose, self.corrupt(delta))
        self._last_true = true_pose
        return self.pose.copy(), teleported

    def corrupt_twist(self, vx, vy, wz):
        """Apply the same systematic errors to a body twist (no white noise)."""
        return (vx * (1.0 + self.trans_scale_error), vy,
                wz * (1.0 + self.rot_scale_error) + self.yaw_drift_per_m * vx)