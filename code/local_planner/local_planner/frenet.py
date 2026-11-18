"""
frenet.py — path-only Frenet planner on the raceline (Werling-style lateral part).

Reference = the raceline segment ahead (d = 0 means "on the racing line").
Candidates: quintic d(s) from the car's current state (d0, d0', 0) to an end
offset d_f reached at s_f, then held at d_f until the horizon. Speed is NOT
sampled; it comes from the raceline profile capped by curvature (speed_profile).

Hard checks : scan clearance >= threshold at every point (only 'no contact' within
              near_radius of the car), |kappa| <= kappa_max
Cost        : w_offset*d_f^2 + w_smooth*int(d''^2) + w_consist*(d_f - prev)^2
              + w_prox*sum(max(0, prox_dist - clearance)^2)
"""

from dataclasses import dataclass

import numpy as np

from local_planner.corridor import (RefPath, Obstacles, lateral_bounds, heading_curvature,
                                    required_clearance)


def quintic_coeffs(d0, dd0, ddd0, df, sf):
    """d(s) = sum a_i s^i with d(0)=d0, d'(0)=dd0, d''(0)=ddd0, d(sf)=df, d'(sf)=0, d''(sf)=0.
    Vectorised over df (array) and sf (array, same shape)."""
    a0, a1, a2 = d0, dd0, ddd0 / 2.0
    s2, s3, s4, s5 = sf ** 2, sf ** 3, sf ** 4, sf ** 5
    # remaining 3 unknowns a3, a4, a5 from the end conditions
    b1 = df - (a0 + a1 * sf + a2 * s2)
    b2 = 0.0 - (a1 + 2 * a2 * sf)
    b3 = 0.0 - (2 * a2)
    a3 = (10 * b1 / s3) - (4 * b2 / s2) + (b3 / (2 * sf))
    a4 = (-15 * b1 / s4) + (7 * b2 / s3) - (b3 / s2)
    a5 = (6 * b1 / s5) - (3 * b2 / s4) + (b3 / (2 * s3))
    return a0, a1, a2, a3, a4, a5


@dataclass
class FrenetResult:
    """Chosen Frenet candidate and planning statistics."""

    path: np.ndarray            # (N, 2) best path in base_link, None if nothing feasible
    s: np.ndarray               # (N,) arc length along the reference
    d_f: float
    s_f: float
    n_candidates: int
    n_feasible: int
    all_paths: np.ndarray       # (M, N, 2) every candidate, for debug markers
    feasible_mask: np.ndarray   # (M,)


class FrenetPlanner:
    """Sample quintic lateral-offset paths from the raceline and pick the cheapest."""

    def __init__(self, p: dict):
        """Store the parameter dict and reset the previous end offset."""
        self.p = p
        self.prev_df = 0.0

    def plan(self, ref: RefPath, obstacles: Obstacles, threshold: float,
             kappa_max: float) -> FrenetResult:
        """Plan a path on ``ref`` that keeps ``threshold`` clearance from the scan.

        Candidates end at each configured end station with lateral offsets sampled inside
        the scan-derived bounds. Candidates that violate the clearance or exceed
        ``kappa_max`` are rejected; the remaining one with the lowest cost is returned.
        """
        p = self.p
        H = min(p['horizon'], ref.length)
        s = np.arange(0.0, H + 1e-9, p['ds'])
        d0, dd0 = ref.car_frenet_state()

        # lateral sampling range from the scan, at each candidate end station
        sf_list = np.array([sf for sf in p['end_stations'] if sf <= H + 1e-9] or [H])
        left, right = lateral_bounds(ref, obstacles, sf_list)
        df_all, sf_all = [], []
        for sf, lb, rb in zip(sf_list, left, right):
            lo, hi = rb + threshold, lb - threshold
            if hi < lo:                       # strip narrower than the car here
                continue
            dfs = np.arange(lo, hi + 1e-9, p['d_step'])
            if lo <= 0.0 <= hi:               # always offer "back on the raceline"
                dfs = np.append(dfs, 0.0)
            df_all.append(dfs)
            sf_all.append(np.full(len(dfs), sf))
        if not df_all:
            return FrenetResult(None, s, np.nan, np.nan, 0, 0, np.empty((0, len(s), 2)),
                                np.zeros(0, bool))
        df = np.concatenate(df_all)
        sf = np.concatenate(sf_all)
        M = len(df)

        # d(s) for every candidate: quintic up to sf, then hold
        a = quintic_coeffs(d0, dd0, 0.0, df, sf)
        S = np.minimum(s[None, :], sf[:, None])                         # (M, N)
        d = sum(a[i][:, None] * S ** i if np.ndim(a[i]) else a[i] * S ** i
                for i in range(6))
        dpp = (2 * a[2] + 6 * a[3][:, None] * S + 12 * a[4][:, None] * S ** 2
               + 20 * a[5][:, None] * S ** 3)
        dpp = np.where(s[None, :] <= sf[:, None], dpp, 0.0)

        # to base_link
        paths = ref.to_cartesian(np.broadcast_to(s, d.shape), d)       # (M, N, 2)

        # hard checks
        flat = paths.reshape(-1, 2)
        clear = obstacles.clearance(flat).reshape(M, len(s))
        need = required_clearance(flat, threshold, p['near_threshold'],
                                  p['near_radius']).reshape(M, len(s))
        kap = np.array([np.abs(heading_curvature(pth)[1][1:-1]).max() for pth in paths])
        feasible = (clear >= need).all(1) & (kap <= kappa_max)

        # cost
        smooth = (dpp ** 2).sum(1) * p['ds']
        prox = (np.clip(p['prox_dist'] - clear, 0.0, None) ** 2).sum(1)
        cost = (p['w_offset'] * df ** 2 + p['w_smooth'] * smooth
                + p['w_consist'] * (df - self.prev_df) ** 2 + p['w_prox'] * prox)

        if not feasible.any():
            return FrenetResult(None, s, np.nan, np.nan, M, 0, paths, feasible)

        best = int(np.argmin(np.where(feasible, cost, np.inf)))
        self.prev_df = float(df[best])
        return FrenetResult(paths[best], s, float(df[best]), float(sf[best]),
                            M, int(feasible.sum()), paths, feasible)

    def best_effort(self, res: FrenetResult, obstacles: Obstacles):
        """Nothing feasible: the candidate that stays furthest from everything."""
        if res.all_paths is None or len(res.all_paths) == 0:
            return None
        M, N, _ = res.all_paths.shape
        clear = obstacles.clearance(res.all_paths.reshape(-1, 2)).reshape(M, N).min(1)
        return res.all_paths[int(np.argmax(clear))]
