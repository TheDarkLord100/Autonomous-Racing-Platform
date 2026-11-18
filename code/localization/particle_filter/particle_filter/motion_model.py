"""Odometry motion model (sampling form).

The odometry delta u = (dx, dy, dtheta) is measured in the robot's frame at
the start of the motion. Each particle receives its own noisy copy of u,
which is then composed onto its pose:

    trans = hypot(dx, dy),  rot = |dtheta|
    sigma_long = a_tt * trans + a_tr * rot        (longitudinal)
    sigma_lat  = a_lat * sigma_long               (lateral; small for a car)
    sigma_rot  = a_rt * trans + a_rr * rot        (heading)

    u_i = u + [N(0, sigma_long), N(0, sigma_lat), N(0, sigma_rot)]
    x_i <- x_i (+) u_i

With odometry scale calibration on, each particle i uses k_i * (dx, dy)
instead of (dx, dy) (see filter.py); rotation is not scaled.

Noise scales with the motion, so a stationary car keeps its particle spread.
A small additive floor (min_sigma_*) keeps the cloud from collapsing when
odometry is very good (or perfect, as in simulation).
"""

import numpy as np

from .geometry import compose


class OdometryMotionModel:
    """Odometry motion model: propagate particles by an odometry increment with noise."""

    def __init__(self, alpha_trans_trans=0.1, alpha_trans_rot=0.05,
                 alpha_lat=0.3, alpha_rot_trans=0.1, alpha_rot_rot=0.1,
                 min_sigma_xy=0.0, min_sigma_theta=0.0, rng=None):
        """Store the noise coefficients and lower bounds on the noise."""
        self.a_tt = alpha_trans_trans
        self.a_tr = alpha_trans_rot
        self.a_lat = alpha_lat
        self.a_rt = alpha_rot_trans
        self.a_rr = alpha_rot_rot
        self.min_sigma_xy = min_sigma_xy
        self.min_sigma_theta = min_sigma_theta
        self.rng = rng if rng is not None else np.random.default_rng()

    def apply(self, particles, delta, scale=None):
        """Propagate particles (N, 3) in place by local odometry delta (3,).

        scale: optional (N,) per-particle odometry scale factor k. Each particle's
        translation becomes k_i * (dx, dy); rotation is not scaled.
        """
        n = particles.shape[0]
        trans = float(np.hypot(delta[0], delta[1]))
        rot = abs(float(delta[2]))

        sigma_long = self.a_tt * trans + self.a_tr * rot + self.min_sigma_xy
        sigma_lat = self.a_lat * (self.a_tt * trans + self.a_tr * rot) + self.min_sigma_xy
        sigma_rot = self.a_rt * trans + self.a_rr * rot + self.min_sigma_theta

        k = 1.0 if scale is None else scale
        noisy = np.empty((n, 3))
        noisy[:, 0] = k * delta[0]
        noisy[:, 1] = k * delta[1]
        noisy[:, 2] = delta[2]
        if sigma_long > 0:
            noisy[:, 0] += self.rng.normal(0.0, sigma_long, n)
        if sigma_lat > 0:
            noisy[:, 1] += self.rng.normal(0.0, sigma_lat, n)
        if sigma_rot > 0:
            noisy[:, 2] += self.rng.normal(0.0, sigma_rot, n)

        particles[:] = compose(particles, noisy)
        return particles