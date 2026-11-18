"""Likelihood-field LiDAR sensor model (Thrun et al., Probabilistic Robotics, ch. 6.4).

For a particle pose x and beam k with range r_k and bearing phi_k (laser frame):

    endpoint = x (+) laser_offset (+) (r_k cos phi_k, r_k sin phi_k)
    d_k      = distance from endpoint to the nearest map obstacle
    p_k      = z_hit * exp(-d_k^2 / (2 sigma_hit^2)) + z_rand / range_max

    log w(x) = effective_beams * mean_k log p_k

Beams are not independent, so the raw product over K beams is far too
peaked. Instead the scan counts as `effective_beams` independent beams,
whatever K is: num_beams sets accuracy/cost, effective_beams alone sets
confidence, and confidence no longer drops in open sections where many
beams return max range and are skipped.
(Equivalent to a per-scan squash of effective_beams / K_valid.)
Invalid beams (NaN/inf, below range_min, at or beyond range_max) are skipped.
No ray casting is needed: d_k is a lookup in the precomputed distance field.
"""

import numpy as np


class LikelihoodFieldModel:
    """Likelihood-field scan model on an OccupancyMap distance field."""

    def __init__(self, occ_map, z_hit=0.9, z_rand=0.1, sigma_hit=0.1,
                 effective_beams=20.0, num_beams=60, laser_offset=(0.0, 0.0, 0.0),
                 penalize_non_free=True):
        """Store the map, mixture weights, beam settings and laser offset."""
        self.map = occ_map
        self.z_hit = z_hit
        self.z_rand = z_rand
        self.sigma_hit = sigma_hit
        self.effective_beams = effective_beams
        self.num_beams = num_beams
        self.laser_offset = np.asarray(laser_offset, dtype=float)
        self.penalize_non_free = penalize_non_free

        # set by log_likelihood(); used by recovery and diagnostics
        self.last_mean_beam_loglik = None   # (N,) mean_k log p_k per particle
        self.last_num_valid = 0

        self._beam_idx = None   # indices into the scan's range array
        self._cos_phi = None
        self._sin_phi = None
        self._scan_len = None

    def set_scan_geometry(self, angle_min, angle_increment, num_ranges):
        """Choose evenly spaced beams once per scan layout; cache their bearings."""
        k = min(self.num_beams, num_ranges)
        idx = np.linspace(0, num_ranges - 1, k).round().astype(np.int64)
        phi = angle_min + idx * angle_increment
        self._beam_idx = idx
        self._cos_phi = np.cos(phi)
        self._sin_phi = np.sin(phi)
        self._scan_len = num_ranges

    def log_likelihood(self, particles, ranges, range_min, range_max):
        """Return (N,) log-weights for particles (N, 3) given one scan."""
        ranges = np.asarray(ranges, dtype=float)
        if self._beam_idx is None or ranges.shape[0] != self._scan_len:
            raise RuntimeError('call set_scan_geometry() for this scan layout first')

        r = ranges[self._beam_idx]
        valid = np.isfinite(r) & (r >= range_min) & (r < range_max)
        n = particles.shape[0]
        self.last_num_valid = int(valid.sum())
        if not valid.any():
            self.last_mean_beam_loglik = None
            return np.zeros(n)

        r = r[valid]
        # beam endpoints in the laser frame, shape (K,)
        bx = r * self._cos_phi[valid]
        by = r * self._sin_phi[valid]

        # laser pose for every particle, shape (N,)
        c, s = np.cos(particles[:, 2]), np.sin(particles[:, 2])
        lx = particles[:, 0] + c * self.laser_offset[0] - s * self.laser_offset[1]
        ly = particles[:, 1] + s * self.laser_offset[0] + c * self.laser_offset[1]
        lth = particles[:, 2] + self.laser_offset[2]
        lc, ls = np.cos(lth), np.sin(lth)

        # endpoints in the world frame, shape (N, K)
        ex = lx[:, None] + lc[:, None] * bx[None, :] - ls[:, None] * by[None, :]
        ey = ly[:, None] + ls[:, None] * bx[None, :] + lc[:, None] * by[None, :]

        d = self.map.distance_at(ex, ey)
        p = self.z_hit * np.exp(-(d * d) / (2.0 * self.sigma_hit ** 2)) + self.z_rand / range_max
        mean_ll = np.log(p).mean(axis=1)
        self.last_mean_beam_loglik = mean_ll
        logw = self.effective_beams * mean_ll

        if self.penalize_non_free:
            free = self.map.is_free(particles[:, 0], particles[:, 1])
            logw[~free] = -1e9
        return logw