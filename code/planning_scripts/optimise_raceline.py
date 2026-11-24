#!/usr/bin/env python3
"""
Global planner (offline): track centreline CSV -> three racelines, compared.
  centreline  the extracted track centre (no optimisation)
  shortest    minimise sum of squared 1st differences (~ path length)
  mincurv     minimise sum of squared 2nd differences (~ squared curvature)
  shortest / mincurv are one sparse QP each in the lateral offsets alpha_i along the
  centreline normals, solved with OSQP. Wall clearance is a HARD requirement: the QP is
  re-solved with bounds tightened wherever the TRUE (shortest) distance to a wall is below
  car half-width + margin, until every point clears. A line that still fails is not written.
  vx_mps is the constant --speed; ax_mps2 is 0.  
  CSV / Waypoint format and every downstream node stay unchanged).
Input:   data/<track>/<map>/extraction/<track>_<map>_centreline.csv   (x, y, w_right, w_left)
Output:  data/<track>/<map>/raceline/<track>_<map>_<method>.csv       one per method
         data/<track>/<map>/plots/<track>_<map>_raceline_comparison.png   (lines drawn on
         the map image <track>_map.yaml when present, otherwise on the extracted walls)
         Raceline CSV columns, map frame, ROS yaw convention (psi = 0 along +x, CCW positive):
  s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, w_left_m, w_right_m
Run from anywhere:  python3 optimise_raceline.py --map carto
"""
import argparse
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import osqp
import yaml
from PIL import Image
from scipy import sparse
from scipy.interpolate import splprep, splev

# code/planning_scripts/optimise_raceline.py -> parents[2] = repo root
REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / 'data'
METHODS = ('centreline', 'shortest', 'mincurv')
LABEL = {'centreline': 'Centreline', 'shortest': 'Shortest path', 'mincurv': 'Minimum curvature'}
COLOR = {'centreline': '#8a8a85', 'shortest': '#eb6834', 'mincurv': '#2a78d6'}


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
    """Return unit left normals for headings ``psi``."""
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
    """Hard wall clearance. Solve the QP; measure the TRUE (shortest) distance
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


# ---------------------------------------------------------------- one line ----
def plan(method, xy, wl, wr, nl, left_b, right_b, half, a):
    """Return the closed reference points of one method (before final resampling)."""
    if method == 'centreline':
        return xy
    _, ref_pts, psi_r, _, _ = resample_closed(xy, a.opt_step)
    nl_r = left_normals(psi_r)
    wl_r = ray_dist(ref_pts, nl_r, left_b)
    wr_r = ray_dist(ref_pts, -nl_r, right_b)
    return enforce_clearance(ref_pts, nl_r, wl_r, wr_r, left_b, right_b, half, method,
                             a.buffer, a.step, a.clear_rounds, a.clear_tol)


# ------------------------------------------------------------------- main ----
def main():
    """Plan the three lines, check them, write one CSV each and the comparison plot."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--track', default='Spielberg')
    ap.add_argument('--map', default='carto', help='map folder under data/<track>/, e.g. carto, GT')
    ap.add_argument('--methods', nargs='+', default=list(METHODS), choices=METHODS)
    ap.add_argument('--opt-step', type=float, default=0.5, help='point spacing for the QP [m]')
    ap.add_argument('--step', type=float, default=0.2, help='output point spacing [m]')
    ap.add_argument('--veh-width', type=float, default=0.31, help='car width [m]')
    ap.add_argument('--margin', type=float, default=0.15, help='extra clearance per side [m]')
    ap.add_argument('--wheelbase', type=float, default=0.3302, help='[m], for steering check')
    ap.add_argument('--steer-max', type=float, default=0.4189, help='max steering angle [rad]')
    ap.add_argument('--speed', type=float, default=4.0, help='constant vx_mps written to the CSV')
    ap.add_argument('--buffer', type=float, default=0.0, help='extra QP-only bound tightening [m]')
    ap.add_argument('--clear-tol', type=float, default=0.005, help='allowed clearance shortfall [m]')
    ap.add_argument('--clear-rounds', type=int, default=30, help='max bound-tightening rounds')
    ap.add_argument('--no-show', action='store_true', help='save the plot without opening it')
    a = ap.parse_args()

    base = DATA_DIR / a.track / a.map
    track_csv = base / 'extraction' / f'{a.track}_{a.map}_centreline.csv'
    out_dir, plot_dir = base / 'raceline', base / 'plots'
    out_dir.mkdir(exist_ok=True)
    plot_dir.mkdir(exist_ok=True)

    xy, wl, wr, nl = load_track(track_csv)
    left_b, right_b = xy + wl[:, None] * nl, xy - wr[:, None] * nl      # original walls
    half = a.veh_width / 2 + a.margin
    k_steer = np.tan(a.steer_max) / a.wheelbase
    print(f'track: {track_csv}  ({len(xy)} points)')
    print(f'[GP] keep {half:.3f} m (car {a.veh_width / 2:.3f} + margin {a.margin:.3f}) from '
          f'each wall; steering limit |κ| <= {k_steer:.3f} 1/m; CSV speed {a.speed:g} m/s')

    lines = {}
    for method in a.methods:
        print(f'\n[GP] --- {method}')
        t0 = time.perf_counter()
        ref = plan(method, xy, wl, wr, nl, left_b, right_b, half, a)
        s, rxy, psi, kappa, L = resample_closed(ref, a.step)
        nl_o = left_normals(psi)
        wl_o, wr_o = ray_dist(rxy, nl_o, left_b), ray_dist(rxy, -nl_o, right_b)
        clear = min(poly_dist(rxy, left_b).min(), poly_dist(rxy, right_b).min())
        if method != 'centreline' and half - clear > a.clear_tol:
            raise SystemExit(f'ERROR: {method}: true clearance {clear:.3f} m below target '
                             f'{half:.3f} m; CSV not written')
        k_max = float(np.abs(kappa).max())
        lines[method] = dict(xy=rxy, s=s, kappa=kappa, L=L, k_max=k_max, clear=clear,
                             k2=float(np.sum(kappa ** 2) * a.step),
                             ms=1e3 * (time.perf_counter() - t0))
        if k_max > k_steer:
            print(f'  WARNING: max |κ| {k_max:.3f} exceeds the steering limit {k_steer:.3f}')

        out_csv = out_dir / f'{a.track}_{a.map}_{method}.csv'
        n = len(s)
        np.savetxt(out_csv, np.c_[s, rxy, psi, kappa, np.full(n, a.speed), np.zeros(n),
                                  wl_o, wr_o], delimiter=',', fmt='%.5f',
                   header='s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,w_left_m,w_right_m',
                   comments='# ')
        print(f'saved {out_csv}')

    # summary table (slide 12)
    print(f'\n{"":20s}{"length m":>10s}{"max |κ| 1/m":>13s}{"Σκ²·ds":>9s}{"min wall m":>12s}'
          f'{"time ms":>9s}')
    for m, d in lines.items():
        flag = ' >steer' if d['k_max'] > k_steer else ''
        print(f'{LABEL[m]:20s}{d["L"]:10.1f}{d["k_max"]:13.3f}{d["k2"]:9.2f}{d["clear"]:12.3f}'
              f'{d["ms"]:9.0f}{flag}')

    # comparison plot
    close = lambda p: np.vstack([p, p[:1]])     # noqa: E731
    fig, ax = plt.subplots(figsize=(12, 9), facecolor='#fcfcfb', layout='constrained')
    ax.set_facecolor('#fcfcfb')
    map_yaml = base / f'{a.track}_map.yaml'
    if map_yaml.exists():
        meta = yaml.safe_load(open(map_yaml))
        img = np.asarray(Image.open(map_yaml.parent / meta['image']).convert('L'))
        res, (ox, oy) = meta['resolution'], meta['origin'][:2]
        ax.imshow(img, cmap='gray', vmin=0, vmax=255, interpolation='nearest',
                  extent=(ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res))
    else:
        ax.plot(*close(left_b).T, c='#222222', lw=0.8)
        ax.plot(*close(right_b).T, c='#222222', lw=0.8, label='track walls')
    for m, d in lines.items():
        ax.plot(*close(d['xy']).T, c=COLOR[m], lw=1.6 if m != 'centreline' else 1.0,
                ls='--' if m == 'centreline' else '-', label=f'{LABEL[m]} ({d["L"]:.1f} m)')
    ax.plot(*xy[0], 'o', c='#222222', ms=6, label='start')
    walls = np.vstack([left_b, right_b])
    ax.set_xlim(walls[:, 0].min() - 2, walls[:, 0].max() + 2)
    ax.set_ylim(walls[:, 1].min() - 2, walls[:, 1].max() + 2)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m] (map frame)', color='#6b6b6b')
    ax.set_ylabel('y [m] (map frame)', color='#6b6b6b')
    for sp in ('top', 'right'):
        ax.spines[sp].set_visible(False)
    ax.grid(True, color='#e4e4e4', lw=0.8)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, loc='best')
    ax.set_title(f'{a.track} ({a.map} map): global planner comparison', loc='left')
    out_png = plot_dir / f'{a.track}_{a.map}_raceline_comparison.png'
    fig.savefig(out_png, dpi=200, facecolor=fig.get_facecolor())
    print(f'\nsaved {out_png}')
    if not a.no_show:
        plt.show()


if __name__ == '__main__':
    main()