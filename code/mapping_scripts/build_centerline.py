#!/usr/bin/env python3
"""
Track extraction, Part 2: pruned skeleton loop -> ordered, oriented, smoothed and
resampled centreline with left/right widths, written as a TUMFTM-style CSV
(x_m, y_m, w_tr_right_m, w_tr_left_m) in the MAP'S OWN FRAME.
Inputs:   data/<track>/<map>/extraction/<track>_<map>_part1.npz   (from extract_track.py)
          data/<track>/<map>/<track>_map.yaml
Outputs:  data/<track>/<map>/extraction/<track>_<map>_centreline.csv
          data/<track>/<map>/plots/<track>_<map>_P2_centreline.png
Run from anywhere:  python3 build_centreline.py --map GT
                    python3 build_centreline.py --map carto
"""
import argparse

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev

from map_utils import load_map, px_to_world, world_to_px, map_dir, out_dirs

# 4-neighbours first, so the walk doesn't cut staircase corners
NB = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


def order_loop(skel, start_rc):
    """Walk the skeleton loop from the pixel nearest start_rc. Returns (path_rc, coverage, gap_px)."""
    pts = np.argwhere(skel)
    cur = tuple(pts[np.argmin(((pts - np.asarray(start_rc)) ** 2).sum(1))])
    path, seen = [cur], {cur}
    Hs, Ws = skel.shape
    while True:
        for dr, dc in NB:
            nxt = (cur[0] + dr, cur[1] + dc)
            if 0 <= nxt[0] < Hs and 0 <= nxt[1] < Ws and skel[nxt] and nxt not in seen:
                break
        else:
            break
        path.append(nxt)
        seen.add(nxt)
        cur = nxt
    path = np.array(path)
    gap = int(np.abs(path[-1] - path[0]).max())         # end -> start, Chebyshev px
    return path, len(path) / skel.sum(), gap


def orient(xy, yaw, look_ahead_m=1.0):
    """Make the loop run in the driving direction given by yaw. Keeps xy[0] as the start."""
    seg = np.hypot(*np.diff(xy, axis=0).T)
    k = min(int(np.searchsorted(np.cumsum(seg), look_ahead_m)) + 1, len(xy) - 1)
    if np.dot(xy[k] - xy[0], [np.cos(yaw), np.sin(yaw)]) < 0:
        return np.vstack([xy[:1], xy[1:][::-1]]), True
    return xy, False


def smooth_resample(xy, thin, sigma, step):
    """Closed smoothing spline, then uniform arc-length resampling.
    sigma: allowed RMS deviation from the pixel path [m]. step: target spacing [m]."""
    pts = xy[::thin]
    pts = np.vstack([pts, pts[:1]])                     # per=1 ignores the last point: repeat first
    tck, _ = splprep(pts.T, s=len(pts) * sigma ** 2, per=1)

    u = np.linspace(0, 1, max(20000, 20 * len(pts)), endpoint=False)
    x, y = splev(u, tck)
    d = np.hypot(np.diff(x, append=x[0]), np.diff(y, append=y[0]))
    s_dense = np.r_[0.0, np.cumsum(d)[:-1]]
    L = s_dense[-1] + d[-1]

    n = int(round(L / step))
    s_new = np.arange(n) * (L / n)                      # spacing that closes the loop exactly
    u_new = np.interp(s_new, s_dense, u)
    cx, cy = splev(u_new, tck)
    dx, dy = splev(u_new, tck, der=1)
    ddx, ddy = splev(u_new, tck, der=2)
    kappa = (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** 1.5
    nrm = np.hypot(dx, dy)
    nl = np.c_[-dy / nrm, dx / nrm]                     # unit left normal (driving direction)
    return np.c_[cx, cy], nl, kappa, s_new, L


def ray_widths(xy, nl, track, meta, H, max_r):
    """Distance along +nl (left) and -nl (right) until leaving the track mask. Vectorised."""
    res = meta['resolution']
    step = res / 4
    r = np.arange(step, max_r, step)

    def side(sign):
        px = xy[:, 0, None] + sign * nl[:, 0, None] * r          # (N, R)
        py = xy[:, 1, None] + sign * nl[:, 1, None] * r
        rows, cols = world_to_px(px, py, meta, H)
        ri, ci = np.round(rows).astype(int), np.round(cols).astype(int)
        inb = (ri >= 0) & (ri < track.shape[0]) & (ci >= 0) & (ci < track.shape[1])
        on_track = np.zeros(inb.shape, bool)
        on_track[inb] = track[ri[inb], ci[inb]]
        hit = (~on_track).any(axis=1)
        first_out = np.argmax(~on_track, axis=1)
        return np.where(hit, r[first_out] - step / 2, max_r), hit

    wl, hit_l = side(+1)
    wr, hit_r = side(-1)
    return wl, wr, hit_l & hit_r


def plot_part2(track, meta, H, xy, nl, wl, wr, kappa, s, title, out_path):
    res = meta['resolution']
    ox, oy, oyaw = meta['origin']
    W = track.shape[1]
    if abs(oyaw) > 1e-9:
        print('  NOTE: map origin yaw != 0, background image extent will be off (lines are correct)')

    fig, ax = plt.subplot_mosaic([['map', 'w'], ['map', 'k']], figsize=(22, 11),
                                 layout='constrained', width_ratios=[1.4, 1])
    a = ax['map']
    a.imshow(track, cmap='gray', alpha=0.4, origin='upper',
             extent=[ox, ox + W * res, oy, oy + H * res])
    left, right = xy + wl[:, None] * nl, xy - wr[:, None] * nl
    close = lambda p: np.vstack([p, p[:1]])
    a.plot(*close(xy).T, 'r-', lw=1, label='centreline')
    a.plot(*close(left).T, '-', c='tab:blue', lw=0.8, label='left boundary')
    a.plot(*close(right).T, '-', c='tab:orange', lw=0.8, label='right boundary')
    every = max(1, len(xy) // 25)
    t = np.c_[nl[:, 1], -nl[:, 0]]                       # tangent = left normal rotated -90°
    a.quiver(xy[::every, 0], xy[::every, 1], t[::every, 0], t[::every, 1],
             color='r', scale=40, width=0.003)
    a.plot(*xy[0], 'o', c='tab:green', ms=10, label='start (s = 0)')
    a.set_aspect('equal')
    a.set_xlabel('x [m] (map frame)')
    a.set_ylabel('y [m] (map frame)')
    a.legend(loc='upper right')
    a.set_title('centreline + boundaries (arrows = driving direction)')

    ax['w'].plot(s, wl, c='tab:blue', lw=1, label='w_left')
    ax['w'].plot(s, wr, c='tab:orange', lw=1, label='w_right')
    ax['w'].plot(s, wl + wr, c='k', lw=1, label='total width')
    ax['w'].set_ylabel('width [m]')
    ax['w'].legend()
    ax['w'].grid(alpha=0.3)

    ax['k'].plot(s, kappa, c='tab:purple', lw=1)
    ax['k'].set_xlabel('s [m]')
    ax['k'].set_ylabel('curvature κ [1/m]  (+ = left turn)')
    ax['k'].grid(alpha=0.3)

    fig.suptitle(title, fontsize=16)
    fig.savefig(out_path, dpi=150)
    print(f'saved {out_path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--track', default='Spielberg')
    ap.add_argument('--map', default='GT', help='map folder under data/<track>/, e.g. GT, carto')
    ap.add_argument('--thin', type=int, default=3, help='use every n-th skeleton pixel for the fit')
    ap.add_argument('--smooth', type=float, default=0.05,
                    help='allowed RMS deviation of the spline from the pixel path [m]')
    ap.add_argument('--step', type=float, default=0.2, help='output point spacing [m]')
    ap.add_argument('--max-width', type=float, default=5.0, help='ray-cast limit per side [m]')
    a = ap.parse_args()

    yaml_path = map_dir(a.track, a.map) / f'{a.track}_map.yaml'
    plot_dir, out_dir = out_dirs(a.track, a.map)
    p1 = np.load(out_dir / f'{a.track}_{a.map}_part1.npz')
    track, skel, start_rc, start = p1['track'], p1['skel'], p1['start_rc'], p1['start']

    _, _, _, meta = load_map(yaml_path)
    H = track.shape[0]

    # 1. order
    path, coverage, gap = order_loop(skel, start_rc)
    print(f'[P2] ordered loop: {len(path)} px, coverage {coverage*100:.1f}% (expect >95%), '
          f'end->start gap {gap} px (expect <=2)')

    # 2. pixel -> map-frame metres
    x, y = px_to_world(path[:, 0], path[:, 1], meta, H)
    xy = np.c_[x, y]

    # 3. driving direction
    xy, flipped = orient(xy, start[2])
    print(f'[P2] direction: {"reversed to match" if flipped else "already matches"} '
          f'start yaw {start[2]:.3f} rad')

    # 4. smooth + resample
    xy, nl, kappa, s, L = smooth_resample(xy, a.thin, a.smooth, a.step)
    print(f'[P2] lap length {L:.2f} m, {len(xy)} points @ {L/len(xy):.3f} m spacing; '
          f'max |κ| {np.abs(kappa).max():.3f} 1/m (min radius {1/np.abs(kappa).max():.2f} m)')

    # 5. widths
    wl, wr, hit = ray_widths(xy, nl, track, meta, H, a.max_width)
    tot = wl + wr
    print(f'[P2] w_left  mean {wl.mean():.3f}, min {wl.min():.3f}, max {wl.max():.3f} m')
    print(f'[P2] w_right mean {wr.mean():.3f}, min {wr.min():.3f}, max {wr.max():.3f} m')
    print(f'[P2] total   mean {tot.mean():.3f}, min {tot.min():.3f}, max {tot.max():.3f} m; '
          f'mean |wl-wr| {np.abs(wl-wr).mean()*100:.1f} cm; rays without a wall: {(~hit).sum()}')

    ok = coverage > 0.95 and gap <= 2 and hit.all() and min(wl.min(), wr.min()) > 0.2
    print(f'[P2] {"PASS" if ok else "CHECK"}')

    # 6. CSV (map frame)
    csv_path = out_dir / f'{a.track}_{a.map}_centreline.csv'
    np.savetxt(csv_path, np.c_[xy, wr, wl], delimiter=',', fmt='%.4f',
               header='x_m,y_m,w_tr_right_m,w_tr_left_m', comments='# ')
    print(f'saved {csv_path}')

    plot_part2(track, meta, H, xy, nl, wl, wr, kappa, s,
               f'{a.track} {a.map}: Part 2 centreline (map frame)',
               plot_dir / f'{a.track}_{a.map}_P2_centreline.png')
    plt.show()


if __name__ == '__main__':
    main()