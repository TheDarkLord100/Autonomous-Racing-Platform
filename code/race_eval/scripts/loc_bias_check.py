#!/usr/bin/env python3
"""Constant offset check: mean error and best rigid fit (rotation + shift) per localizer.

  python3 loc_bias_check.py <replay_run_dir> [...]
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import eval_localization as E  # noqa: E402


def rigid_fit(P, Q):
    """Rotation R and shift t minimising |R P + t - Q| (2D Kabsch), plus residuals."""
    mp, mq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - mp).T @ (Q - mq))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = mq - mp @ R.T
    return R, t, np.hypot(*(P @ R.T + t - Q).T)


for run in sys.argv[1:]:
    method, gt, est, _ = E.evaluate(run)
    t, e_pos, e_yaw, d, keep = E.errors(gt, est)
    e = est[keep]
    gx = np.interp(t, gt[:, 1], gt[:, 2])
    gy = np.interp(t, gt[:, 1], gt[:, 3])
    gyaw = E.wrap(np.interp(t, gt[:, 1], np.unwrap(gt[:, 4])))
    dyaw = np.degrees(E.wrap(e[:, 4] - gyaw))
    R, sh, res = rigid_fit(e[:, 2:4], np.c_[gx, gy])
    print(f'{E.LABEL[method]:13s} mean dx {np.mean(e[:, 2] - gx):+.3f} m, '
          f'dy {np.mean(e[:, 3] - gy):+.3f} m, dyaw {dyaw.mean():+.2f} deg (std {dyaw.std():.2f}) | '
          f'fit: rot {math.degrees(math.atan2(R[1, 0], R[0, 0])):+.2f} deg, '
          f'shift ({sh[0]:+.2f}, {sh[1]:+.2f}) m, RMSE after fit {math.sqrt(np.mean(res ** 2)):.3f} m')