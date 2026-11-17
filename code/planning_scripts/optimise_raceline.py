#!/usr/bin/env python3
"""
Global planner (offline): track centreline CSV -> optimised raceline CSV.
  methods:  mincurv   minimise sum of squared 2nd differences (≈ squared curvature)
            shortest  minimise sum of squared 1st differences (≈ path length)
  Each pass is one sparse QP in the lateral offsets alpha_i along the reference normals,
  solved with OSQP. --iters N > 1 re-linearises around the previous solution (iterative
  variant): resample evenly, recompute normals, ray-cast widths to the ORIGINAL walls, solve.
  Wall clearance is a HARD requirement: after the passes, the last QP is re-solved with
  bounds tightened wherever the TRUE (shortest) distance to a wall is below car half-width
  + margin, until every point clears. A line that still fails is never written.
  Then a friction-circle velocity profile (flying lap) using only --grip of mu*g.
Input:   data/<track>/<map>/extraction/<track>_<map>_centreline.csv   (x, y, w_right, w_left)
Output:  data/<track>/<map>/raceline/<track>_<map>_<tag>.csv      tag = method[_iter][_suffix]
         data/<track>/<map>/plots/<track>_<map>_raceline_<tag>.png
Raceline CSV columns, map frame, ROS yaw convention (psi = 0 along +x, CCW positive):
  s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, w_left_m, w_right_m
Run from anywhere:  python3 optimise_raceline.py --map carto --method mincurv --iters 10
"""
import argparse
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import osqp
from scipy import sparse
from scipy.interpolate import splprep, splev

# code/planning_scripts/optimise_raceline.py -> parents[2] = repo root
REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / 'data'
G = 9.81


# ------------------------------------------------------------- track input ----
def load_track(path):
    """Centreline CSV -> points, left/right widths, unit left normals."""
    a = np.loadtxt(path, delimiter=',', comments='#')
    xy, wr, wl = a[:, :2], a[:, 2], a[:, 3]
    t = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)        # central difference, closed loop
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    nl = np.c_[-t[:, 1], t[:, 0]]
    return xy, wl, wr, nl


def left_normals(psi):
    return np.c_[-np.sin(psi), np.cos(psi)]


def ray_dist(p, d, poly, max_r=10.0, chunk=256):
    """Distance from each point p along unit direction d to the nearest crossing of the
    closed polyline poly (ray-segment intersection, vectorised in chunks)."""
    a = poly
    e = np.roll(poly, -1, axis=0) - a                           # segment vectors (M, 2)
    cross = lambda u, v: u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]
    out = np.empty(len(p))
    for i in range(0, len(p), chunk):
        pc, dc = p[i:i + chunk], d[i:i + chunk]
        ap = a[None] - pc[:, None]                              # (n, M, 2)
        den = cross(dc[:, None], e[None])                       # (n, M)
        with np.errstate(divide='ignore', invalid='ignore'):
            t = cross(ap, e[None]) / den
            u = cross(ap, dc[:, None]) / den
        ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        out[i:i + chunk] = np.where(ok, t, np.inf).min(axis=1)
    return np.minimum(out, max_r)


def poly_dist(p, poly, chunk=256):
    """True shortest distance from each point p to the closed polyline poly."""
    a = poly
    e = np.roll(poly, -1, axis=0) - a
    ee = (e ** 2).sum(axis=1)
    out = np.empty(len(p))
    for i in range(0, len(p), chunk):
        ap = p[i:i + chunk, None] - a[None]                     # (n, M, 2)
        t = np.clip((ap * e[None]).sum(-1) / ee[None], 0.0, 1.0)
        out[i:i + chunk] = np.linalg.norm(ap - t[..., None] * e[None], axis=-1).min(axis=1)
    return out


# --------------------------------------------------------------------- QP ----
def cyclic_diff(n, order):
    """Sparse cyclic difference matrix: order 1 -> x[i+1]-x[i], order 2 -> x[i+1]-2x[i]+x[i-1]."""
    I = sparse.identity(n, format='csr')
    S = sparse.csr_matrix((np.ones(n), (np.arange(n), (np.arange(n) + 1) % n)), shape=(n, n))
    return (S - I) if order == 1 else (S - 2 * I + S.T)


def solve_offsets(xy, wl, wr, nl, method, half_width, buf_l=0.0, buf_r=0.0):
    """min ||D (p + diag(alpha) n)||^2  s.t.  lo <= alpha <= hi, as an OSQP problem.
    buf_l / buf_r: extra bound tightening per side, scalar or one value per point."""
    n = len(xy)
    D = cyclic_diff(n, 2 if method == 'mincurv' else 1)
    M = (D.T @ D).tocsc()
    Nx, Ny = sparse.diags(nl[:, 0]), sparse.diags(nl[:, 1])
    P = 2 * (Nx @ M @ Nx + Ny @ M @ Ny)
    q = 2 * (Nx @ (M @ xy[:, 0]) + Ny @ (M @ xy[:, 1]))
    lo, hi = -(wr - half_width - buf_r), (wl - half_width - buf_l)

    if np.any(lo > hi):
        bad = np.flatnonzero(lo > hi)
        raise ValueError(f'track narrower than car + margins at {len(bad)} points '
                         f'(e.g. index {bad[0]}); reduce --margin')

    prob = osqp.OSQP()
    prob.setup(P=sparse.triu(P, format='csc'), q=q, A=sparse.identity(n, format='csc'),
               l=lo, u=hi, verbose=False, eps_abs=1e-7, eps_rel=1e-7,
               max_iter=200000, polish=True)
    t0 = time.perf_counter()
    res = prob.solve()
    return res.x, res.info.status, res.info.iter, time.perf_counter() - t0, lo, hi


def enforce_clearance(ref_pts, nl_r, wl_r, wr_r, left_b, right_b, half, method,
                      base_buf, dense_step, max_rounds, tol):
    """Hard wall clearance. Re-solve the last QP pass; measure the TRUE (shortest) distance
    from a dense resample of the result to each wall; wherever it is below `half`, tighten
    that point's bound by the shortfall (and its neighbours'); repeat until all points clear."""
    n = len(ref_pts)
    buf_l, buf_r = np.full(n, base_buf), np.full(n, base_buf)
    for rnd in range(1, max_rounds + 1):
        alpha, status, *_ = solve_offsets(ref_pts, wl_r, wr_r, nl_r, method, half, buf_l, buf_r)
        if status != 'solved':
            raise SystemExit(f'ERROR: OSQP "{status}" in clearance round {rnd}')
        cand = ref_pts + alpha[:, None] * nl_r

        s_d, p_d, _, _, L_d = resample_closed(cand, dense_step)
        def_l = half - poly_dist(p_d, left_b)
        def_r = half - poly_dist(p_d, right_b)
        worst = max(def_l.max(), def_r.max())
        print(f'[GP] clearance round {rnd}: worst shortfall {max(worst, 0.0) * 100:.1f} cm '
              f'(left {max(def_l.max(), 0) * 100:.1f}, right {max(def_r.max(), 0) * 100:.1f})')
        if worst <= tol:
            return cand

        idx = np.round(s_d / L_d * n).astype(int) % n           # dense point -> QP point
        for buf, dfc in ((buf_l, def_l), (buf_r, def_r)):
            bad = dfc > tol
            np.maximum.at(buf, idx[bad], buf[idx[bad]] + dfc[bad] + 0.01)
            grown = buf.copy()                                  # neighbours too: the line
            for k in (-2, -1, 1, 2):                            # bends between QP points
                grown = np.maximum(grown, np.roll(buf, k))
            buf[:] = grown
    raise SystemExit(f'ERROR: clearance not met after {max_rounds} rounds; CSV not written')


# ------------------------------------------------------ resample + geometry ----
def resample_closed(xy, step, n=None):
    """Interpolating periodic spline through xy, resampled at uniform arc length.
    Returns s, points, psi, kappa, lap length."""
    pts = np.vstack([xy, xy[:1]])                               # per=1 ignores last point
    tck, _ = splprep(pts.T, s=0, per=1)
    u = np.linspace(0, 1, 20 * len(xy), endpoint=False)
    x, y = splev(u, tck)
    d = np.hypot(np.diff(x, append=x[0]), np.diff(y, append=y[0]))
    s_dense = np.r_[0.0, np.cumsum(d)[:-1]]
    L = s_dense[-1] + d[-1]

    if n is None:
        n = int(round(L / step))
    s = np.arange(n) * (L / n)
    u_new = np.interp(s, s_dense, u)
    x, y = splev(u_new, tck)
    dx, dy = splev(u_new, tck, der=1)
    ddx, ddy = splev(u_new, tck, der=2)
    psi = np.arctan2(dy, dx)
    kappa = (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** 1.5
    return s, np.c_[x, y], psi, kappa, L


# --------------------------------------------------------- velocity profile ----
def velocity_profile(s, kappa, L, mu, v_max, ax_max, grip=1.0):
    """Friction circle (a_total = grip*mu*g) + accel/brake cap ax_max + top speed v_max.
    grip (0-1]: fraction of the tyre friction the plan may use.
    Flying lap: passes start at the slowest point so the loop closes consistently."""
    n = len(s)
    ds = np.diff(np.r_[s, L])                                   # ds[i]: i -> i+1 (wraps)
    a_tot = grip * mu * G
    k = np.abs(kappa)
    v_lim = np.full(n, float(v_max))
    m = k > 1e-9
    v_lim[m] = np.minimum(v_max, np.sqrt(a_tot / k[m]))

    order = np.roll(np.arange(n), -int(np.argmin(v_lim)))
    kk, dd = k[order], ds[order]
    v = v_lim[order].copy()

    def ax_avail(vi, ki):                                       # grip left after cornering
        return min(ax_max, np.sqrt(max(a_tot ** 2 - (vi ** 2 * ki) ** 2, 0.0)))

    for i in range(n - 1):                                      # forward: acceleration
        v[i + 1] = min(v[i + 1], np.sqrt(v[i] ** 2 + 2 * ax_avail(v[i], kk[i]) * dd[i]))
    for i in range(n - 1, -1, -1):                              # backward: braking (wraps)
        j = (i + 1) % n
        v[i] = min(v[i], np.sqrt(v[j] ** 2 + 2 * ax_avail(v[j], kk[j]) * dd[i]))

    vx = np.empty(n)
    vx[order] = v
    v_next = np.roll(vx, -1)
    ax = (v_next ** 2 - vx ** 2) / (2 * ds)
    lap_time = float(np.sum(ds / (0.5 * (vx + v_next))))
    return vx, ax, lap_time


# ------------------------------------------------------------------- main ----
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--track', default='Spielberg')
    ap.add_argument('--map', default='carto', help='map folder under data/<track>/, e.g. carto, GT')
    ap.add_argument('--method', default='mincurv', choices=['mincurv', 'shortest'])
    ap.add_argument('--iters', type=int, default=1, help='QP passes; >1 = iterative variant')
    ap.add_argument('--tol', type=float, default=0.005, help='stop when max |offset| < tol [m]')
    ap.add_argument('--opt-step', type=float, default=0.5, help='point spacing for the QP [m]')
    ap.add_argument('--step', type=float, default=0.2, help='output point spacing [m]')
    ap.add_argument('--veh-width', type=float, default=0.31, help='car width [m]')
    ap.add_argument('--margin', type=float, default=0.15, help='extra clearance per side [m]')
    ap.add_argument('--mu', type=float, default=1.0489, help='tyre friction coefficient')
    ap.add_argument('--v-max', type=float, default=5.0, help='top speed [m/s]')
    ap.add_argument('--ax-max', type=float, default=9.51, help='max accel/brake [m/s²]')
    ap.add_argument('--wheelbase', type=float, default=0.3302, help='[m], for steering check')
    ap.add_argument('--steer-max', type=float, default=0.4189, help='max steering angle [rad]')
    ap.add_argument('--damping', type=float, default=0.5, help='step size for passes >= 2 (0-1]')
    ap.add_argument('--buffer', type=float, default=0.0, help='extra QP-only bound tightening [m]')
    ap.add_argument('--clear-tol', type=float, default=0.005, help='allowed clearance shortfall [m]')
    ap.add_argument('--grip', type=float, default=0.7,
                    help='fraction of mu*g the velocity profile may use (0-1]')
    ap.add_argument('--suffix', default='', help='appended to the output tag, e.g. g070')
    ap.add_argument('--clear-rounds', type=int, default=30, help='max bound-tightening rounds')
    a = ap.parse_args()

    tag = a.method if a.iters == 1 else f'{a.method}_iter'
    if a.suffix:
        tag += f'_{a.suffix}'
    base = DATA_DIR / a.track / a.map
    track_csv = base / 'extraction' / f'{a.track}_{a.map}_centreline.csv'
    out_dir, plot_dir = base / 'raceline', base / 'plots'
    out_dir.mkdir(exist_ok=True)
    plot_dir.mkdir(exist_ok=True)

    xy, wl, wr, nl = load_track(track_csv)
    left_b, right_b = xy + wl[:, None] * nl, xy - wr[:, None] * nl      # original walls
    half = a.veh_width / 2 + a.margin
    print(f'track: {track_csv}  ({len(xy)} points)')
    print(f'[GP] {tag}; QP on {a.opt_step} m spacing; keep {half:.3f} m '
          f'(car {a.veh_width/2:.3f} + margin {a.margin:.3f}) from each wall')

    # 1. QP passes (1 = plain, >1 = iterative re-linearisation)
    ref, n_opt = xy, None
    for it in range(1, a.iters + 1):
        _, ref_pts, psi_r, _, _ = resample_closed(ref, a.opt_step, n_opt)
        n_opt = len(ref_pts)                                    # fixed from pass 1 on
        nl_r = left_normals(psi_r)
        wl_r = ray_dist(ref_pts, nl_r, left_b)
        wr_r = ray_dist(ref_pts, -nl_r, right_b)
        alpha, status, n_it, dt, lo, hi = solve_offsets(ref_pts, wl_r, wr_r, nl_r, a.method,
                                                        half, a.buffer, a.buffer)
        beta = 1.0 if it == 1 else a.damping
        ref = ref_pts + beta * alpha[:, None] * nl_r
        at_b = (np.mean(alpha <= lo + 1e-3) + np.mean(alpha >= hi - 1e-3)) * 100
        print(f'[GP] pass {it}: {len(ref_pts)} pts, OSQP {status} ({n_it} it, {dt*1000:.0f} ms), '
              f'max |offset| {np.abs(alpha).max():.3f} m, {at_b:.1f}% of points at a bound')
        if status != 'solved':
            print(f'  WARNING: OSQP status "{status}"')
        if it > 1 and np.abs(alpha).max() < a.tol:
            print(f'[GP] converged (max |offset| < {a.tol} m)')
            break

    # 1b. hard wall clearance (true shortest distance) before anything else uses the line
    ref = enforce_clearance(ref_pts, nl_r, wl_r, wr_r, left_b, right_b, half, a.method,
                            a.buffer, a.step, a.clear_rounds, a.clear_tol)

    # 2. final raceline geometry, exact wall clearances (and centreline for comparison)
    s, rxy, psi, kappa, L = resample_closed(ref, a.step)
    nl_o = left_normals(psi)
    wl_o = ray_dist(rxy, nl_o, left_b)
    wr_o = ray_dist(rxy, -nl_o, right_b)
    sc, _, _, kc, Lc = resample_closed(xy, a.step)

    # 3. velocity profiles
    vx, ax, lap = velocity_profile(s, kappa, L, a.mu, a.v_max, a.ax_max, a.grip)
    vxc, _, lapc = velocity_profile(sc, kc, Lc, a.mu, a.v_max, a.ax_max, a.grip)

    k_steer = np.tan(a.steer_max) / a.wheelbase
    print(f'[GP] length: centreline {Lc:.2f} m -> raceline {L:.2f} m ({(L-Lc)/Lc*100:+.2f}%)')
    print(f'[GP] max |κ|: centreline {np.abs(kc).max():.3f} -> raceline {np.abs(kappa).max():.3f} 1/m '
          f'(steering limit {k_steer:.3f}); Σκ²·ds: {np.sum(kc**2)*a.step:.2f} -> '
          f'{np.sum(kappa**2)*a.step:.2f}')
    if np.abs(kappa).max() > k_steer:
        print('  WARNING: raceline curvature exceeds the steering limit somewhere')
    dl_true, dr_true = poly_dist(rxy, left_b), poly_dist(rxy, right_b)
    for side, dist in (('left', dl_true), ('right', dr_true)):
        j = int(np.argmin(dist))
        print(f'[GP] min true clearance {side}: {dist[j]:.3f} m at s = {s[j]:.1f} m, '
              f'(x, y) = ({rxy[j, 0]:.2f}, {rxy[j, 1]:.2f})  (target {half:.3f})')
    shortfall = half - min(dl_true.min(), dr_true.min())
    if shortfall > a.clear_tol:
        raise SystemExit(f'ERROR: true clearance {shortfall * 100:.1f} cm below target; '
                         f'CSV not written')

    print(f'[GP] grip {a.grip:.2f} (lateral limit {a.grip * a.mu * G:.2f} m/s²)')
    print(f'[GP] speed {vx.min():.2f}-{vx.max():.2f} m/s; lap time: centreline {lapc:.2f} s -> '
          f'{tag} {lap:.2f} s ({(lap-lapc)/lapc*100:+.1f}%)')

    # 4. CSV
    out_csv = out_dir / f'{a.track}_{a.map}_{tag}.csv'
    np.savetxt(out_csv, np.c_[s, rxy, psi, kappa, vx, ax, wl_o, wr_o], delimiter=',', fmt='%.5f',
               header='s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,w_left_m,w_right_m',
               comments='# ')
    print(f'saved {out_csv}')

    # 5. plot
    close = lambda p: np.vstack([p, p[:1]])
    fig, axd = plt.subplot_mosaic([['map', 'k'], ['map', 'v']], figsize=(22, 11),
                                  layout='constrained', width_ratios=[1.3, 1])
    m = axd['map']
    m.plot(*close(left_b).T, 'k-', lw=0.8)
    m.plot(*close(right_b).T, 'k-', lw=0.8, label='track boundaries')
    m.plot(*close(xy).T, '--', c='gray', lw=0.8, label='centreline')
    sc_ = m.scatter(rxy[:, 0], rxy[:, 1], c=vx, cmap='plasma', s=4, label=f'{tag} raceline')
    fig.colorbar(sc_, ax=m, label='v [m/s]')
    m.plot(*rxy[0], 'o', c='tab:green', ms=9, label='start (s = 0)')
    m.set_aspect('equal')
    m.set_xlabel('x [m] (map frame)')
    m.set_ylabel('y [m] (map frame)')
    m.legend(loc='upper right', markerscale=3)
    m.set_title(f'{tag} raceline, coloured by speed')

    axd['k'].plot(sc, kc, c='gray', lw=0.8, label='centreline')
    axd['k'].plot(s, kappa, c='tab:red', lw=1, label=tag)
    axd['k'].axhline(k_steer, c='k', ls=':', lw=0.8)
    axd['k'].axhline(-k_steer, c='k', ls=':', lw=0.8, label='steering limit')
    axd['k'].set_ylabel('κ [1/m]')
    axd['k'].legend()
    axd['k'].grid(alpha=0.3)

    axd['v'].plot(sc, vxc, c='gray', lw=0.8, label=f'centreline ({lapc:.2f} s)')
    axd['v'].plot(s, vx, c='tab:red', lw=1, label=f'{tag} ({lap:.2f} s)')
    axd['v'].set_xlabel('s [m]')
    axd['v'].set_ylabel('v [m/s]')
    axd['v'].legend()
    axd['v'].grid(alpha=0.3)

    fig.suptitle(f'{a.track} {a.map}: global planner ({tag})', fontsize=16)
    out_png = plot_dir / f'{a.track}_{a.map}_raceline_{tag}.png'
    fig.savefig(out_png, dpi=150)
    print(f'saved {out_png}')
    plt.show()


if __name__ == '__main__':
    main()
