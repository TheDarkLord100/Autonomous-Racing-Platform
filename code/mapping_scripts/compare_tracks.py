#!/usr/bin/env python3
"""
Part 3: compare extracted centrelines.
  pair 1  GT extraction vs provided <track>_centerline.csv   -> validates the extractor
  pair 2  SLAM extraction (-> gym frame) vs GT extraction     -> measures mapping error
Wall errors use centreline offset + width, so they measure where the WALLS are,
not where each centreline happens to sit inside the band.
Outputs: data/<track>/<slam>/plots/<track>_<slam>_C_overlay.png
         data/<track>/<slam>/plots/<track>_<slam>_C_errors_vs_s.png
         data/<track>/<slam>/extraction/<track>_<slam>_metrics.json
Run from anywhere:
  python3 compare_tracks.py [--track Spielberg] [--gt-dir GT] [--slam-dir carto]
"""
import argparse
import json

import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.spatial import cKDTree

from map_utils import load_map, px_to_world, se2, map_dir, out_dirs


# ------------------------------------------------------------ line utils ----
def load_csv(path):
    """x, y, w_right, w_left -> dict."""
    a = np.loadtxt(path, delimiter=',', comments='#')
    return {'xy': a[:, :2].copy(), 'wr': a[:, 2].copy(), 'wl': a[:, 3].copy()}


def reverse(line):
    """Flip driving direction: keep the start point, swap left/right widths."""
    idx = np.r_[0, np.arange(len(line['xy']) - 1, 0, -1)]
    return {'xy': line['xy'][idx], 'wr': line['wl'][idx], 'wl': line['wr'][idx]}


def arc(xy):
    """Arc length of each point along a closed polyline, and the total lap length."""
    seg = np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T)
    return np.r_[0.0, np.cumsum(seg)[:-1]], seg.sum()


def tangents(xy):
    t = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    return t / np.linalg.norm(t, axis=1, keepdims=True)


def densify(line, ds=0.01):
    """Linear resampling of a closed line at ds, so nearest-point matching is ~exact."""
    s, L = arc(line['xy'])
    s_c = np.r_[s, L]
    sd = np.arange(0, L, ds)
    f = lambda v: np.interp(sd, s_c, np.r_[v, v[:1]])
    xy = np.c_[f(line['xy'][:, 0]), f(line['xy'][:, 1])]
    t = tangents(xy)
    return {'xy': xy, 'wr': f(line['wr']), 'wl': f(line['wl']), 's': sd, 'L': L,
            't': t, 'nl': np.c_[-t[:, 1], t[:, 0]]}


def stats(e):
    return {'mean': float(np.mean(e)), 'rms': float(np.sqrt(np.mean(e ** 2))),
            'max_abs': float(np.max(np.abs(e)))}


# ---------------------------------------------------------------- compare ----
def compare(test, ref_line, name):
    ref = densify(ref_line)
    _, i = cKDTree(ref['xy']).query(test['xy'])
    if np.mean(np.sum(tangents(test['xy']) * ref['t'][i], axis=1)) < 0:
        print(f'  [{name}] reference runs the other way -> reversed it')
        ref = densify(reverse(ref_line))
        _, i = cKDTree(ref['xy']).query(test['xy'])

    e_ct = np.sum((test['xy'] - ref['xy'][i]) * ref['nl'][i], axis=1)  # + = test left of ref
    e_left = (e_ct + test['wl']) - ref['wl'][i]                         # + = left wall further out
    e_right = (test['wr'] - e_ct) - ref['wr'][i]                        # + = right wall further out
    e_tot = (test['wl'] + test['wr']) - (ref['wl'][i] + ref['wr'][i])
    s_test, L_test = arc(test['xy'])

    m = {'cross_track': stats(e_ct), 'left_wall': stats(e_left), 'right_wall': stats(e_right),
         'total_width': stats(e_tot), 'lap_length_test': float(L_test),
         'lap_length_ref': float(ref['L']), 'lap_length_diff': float(L_test - ref['L'])}
    arrays = {'e_ct': e_ct, 'e_left': e_left, 'e_right': e_right,
              's_test': s_test, 's_ref': ref['s'][i]}
    return m, arrays


def report(name, m):
    cm = lambda d: f'mean {d["mean"]*100:+6.1f}, RMS {d["rms"]*100:5.1f}, max {d["max_abs"]*100:5.1f} cm'
    print(f'[{name}]')
    print(f'   cross-track  {cm(m["cross_track"])}')
    print(f'   left wall    {cm(m["left_wall"])}')
    print(f'   right wall   {cm(m["right_wall"])}')
    print(f'   total width  {cm(m["total_width"])}')
    print(f'   lap length   {m["lap_length_test"]:.2f} vs {m["lap_length_ref"]:.2f} m '
          f'({m["lap_length_diff"]:+.2f} m, {m["lap_length_diff"]/m["lap_length_ref"]*100:+.2f}%)')


def boundaries(line):
    nl = tangents(line['xy'])[:, ::-1] * [-1, 1]                       # left normal
    return line['xy'] + line['wl'][:, None] * nl, line['xy'] - line['wr'][:, None] * nl


# ------------------------------------------------------------------- main ----
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--track', default='Spielberg')
    ap.add_argument('--gt-dir', default='GT')
    ap.add_argument('--slam-dir', default='carto')
    a = ap.parse_args()

    gt, slam = map_dir(a.track, a.gt_dir), map_dir(a.track, a.slam_dir)
    _, gt_ex_dir = out_dirs(a.track, a.gt_dir)
    plot_dir, slam_ex_dir = out_dirs(a.track, a.slam_dir)

    provided = load_csv(gt / f'{a.track}_centerline.csv')
    gt_ex = load_csv(gt_ex_dir / f'{a.track}_{a.gt_dir}_centreline.csv')
    slam_ex = load_csv(slam_ex_dir / f'{a.track}_{a.slam_dir}_centreline.csv')
    with open(slam / f'{a.track}.yaml') as f:
        start_pose = yaml.safe_load(f)['origin']
    slam_ex['xy'] = np.c_[se2(slam_ex['xy'][:, 0], slam_ex['xy'][:, 1], start_pose)]  # -> gym

    m1, r1 = compare(gt_ex, provided, 'pair 1')
    m2, r2 = compare(slam_ex, gt_ex, 'pair 2')
    report(f'pair 1: {a.gt_dir} extraction vs provided centreline  (extractor error)', m1)
    report(f'pair 2: {a.slam_dir} extraction vs {a.gt_dir} extraction  (mapping error)', m2)

    metrics = {'track': a.track, 'gt_dir': a.gt_dir, 'slam_dir': a.slam_dir,
               'pair1_gt_extraction_vs_provided': m1, 'pair2_slam_vs_gt_extraction': m2}
    json_path = slam_ex_dir / f'{a.track}_{a.slam_dir}_metrics.json'
    with open(json_path, 'w') as f:
        json.dump(metrics, f, indent=2)
    print(f'saved {json_path}')

    # ---------- overlay (gym frame) ----------
    img, occ, _, meta = load_map(gt / f'{a.track}_map.yaml')
    wr_, wc_ = np.nonzero(occ)
    wx, wy = px_to_world(wr_, wc_, meta, img.shape[0])

    fig, ax = plt.subplots(figsize=(14, 11), layout='constrained')
    ax.scatter(wx, wy, s=0.2, c='lightgray', label='GT map walls')
    close = lambda p: np.vstack([p, p[:1]])
    ax.plot(*close(provided['xy']).T, 'k--', lw=1, label='provided centreline')
    for line, col, lab in ((gt_ex, 'tab:blue', f'{a.gt_dir} extraction'),
                           (slam_ex, 'tab:orange', f'{a.slam_dir} extraction (→ gym)')):
        ax.plot(*close(line['xy']).T, '-', c=col, lw=1.2, label=lab)
        for b in boundaries(line):
            ax.plot(*close(b).T, '-', c=col, lw=0.6, alpha=0.7)
    ax.plot(*start_pose[:2], 'o', c='tab:green', ms=10, label='start')
    ax.set_aspect('equal')
    ax.set_xlabel('x [m] (gym frame)')
    ax.set_ylabel('y [m] (gym frame)')
    ax.legend(loc='upper right')
    ax.set_title(f'{a.track}: centrelines + boundaries (thin lines) in the gym frame')
    out1 = plot_dir / f'{a.track}_{a.slam_dir}_C_overlay.png'
    fig.savefig(out1, dpi=150)
    print(f'saved {out1}')

    # ---------- errors vs s (both on the GT-extraction s axis) ----------
    fig, ax = plt.subplots(3, 1, figsize=(16, 11), sharex=True, layout='constrained')
    rows = (('e_ct', 'cross-track [cm]\n(+ = left of ref)'),
            ('e_left', 'left wall [cm]\n(+ = further out)'),
            ('e_right', 'right wall [cm]\n(+ = further out)'))
    for a_, (k, lab) in zip(ax, rows):
        a_.plot(r1['s_test'], r1[k] * 100, lw=0.8, c='tab:blue',
                label=f'pair 1: {a.gt_dir} extr. vs provided')
        a_.plot(r2['s_ref'], r2[k] * 100, '.', ms=2, c='tab:orange',
                label=f'pair 2: {a.slam_dir} vs {a.gt_dir} extr.')
        a_.axhline(0, c='k', lw=0.6)
        a_.set_ylabel(lab)
        a_.grid(alpha=0.3)
    ax[0].legend(loc='upper right', markerscale=6)
    ax[-1].set_xlabel(f's [m] along the {a.gt_dir} extraction (0 = start)')
    fig.suptitle(f'{a.track}: errors along the lap', fontsize=15)
    out2 = plot_dir / f'{a.track}_{a.slam_dir}_C_errors_vs_s.png'
    fig.savefig(out2, dpi=150)
    print(f'saved {out2}')
    plt.show()


if __name__ == '__main__':
    main()