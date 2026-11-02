#!/usr/bin/env python3
"""
Track extraction from an occupancy map.
  Part 1: classify -> unknown→occupied -> clean -> skeletonise -> prune spurs.
Outputs:  data/<track>/<map>/plots/<track>_<map>_P1_stages.png
          data/<track>/<map>/extraction/<track>_<map>_part1.npz
Run from anywhere:
  python3 extract_track.py --map GT
  python3 extract_track.py --map carto
  python3 extract_track.py --track Monza --map GT --start 0 0 1.57   # other track: pass its start
"""
import argparse

import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage as ndi
from skimage.measure import euler_number
from skimage.morphology import disk, skeletonize

from map_utils import load_map, world_to_px, map_dir, out_dirs

# Spielberg sim start pose (sx, sy, stheta from sim.yaml) in the gym/GT frame.
# For other tracks pass --start. SLAM maps start at (0, 0, 0) in their own frame.
SIM_START = [0.0, 0.0, -2.879]


# ---------------------------------------------------------------- Part 1 ----
def start_pixel(free, meta, H, start):
    r, c = world_to_px(np.array([start[0]]), np.array([start[1]]), meta, H)
    rc = (int(round(r[0])), int(round(c[0])))
    if not free[rc]:
        fr = np.argwhere(free)
        rc = tuple(fr[np.argmin(((fr - np.array(rc)) ** 2).sum(1))])
        print(f'  WARNING: start pixel not free, snapped to nearest free pixel {rc}')
    return rc


def build_track_mask(free, start_rc, res, open_radius_px, min_hole_m2):
    track = free.copy()                                   # unknown + occupied -> not track

    if open_radius_px > 0:                                # cut thin leaks through wall gaps
        track = ndi.binary_opening(track, structure=disk(open_radius_px))

    # 4-connectivity (scipy default): free space can't leak diagonally through a 1-px wall
    lab, n_comp = ndi.label(track)
    if lab[start_rc] == 0:
        raise RuntimeError('start pixel was removed by the opening; reduce --open-radius')
    track = lab == lab[start_rc]

    # fill holes smaller than min_hole_m2 (noise specks); the infield is far bigger and stays
    holes = ndi.binary_fill_holes(track) & ~track
    hl, n_holes = ndi.label(holes)
    sizes = ndi.sum(holes, hl, index=np.arange(1, n_holes + 1))
    small_ids = np.flatnonzero(sizes < min_hole_m2 / res ** 2) + 1
    track |= np.isin(hl, small_ids)

    return track, n_comp, len(small_ids)


def prune_spurs(skel, max_iter=100000):
    """Repeatedly delete endpoints (<=1 neighbour). A closed loop has none, so only it survives."""
    skel = skel.copy()
    K = np.ones((3, 3), np.uint8)
    K[1, 1] = 0
    removed = 0
    for _ in range(max_iter):
        nb = ndi.convolve(skel.astype(np.uint8), K, mode='constant')
        ends = skel & (nb <= 1)
        if not ends.any():
            break
        skel[ends] = False
        removed += int(ends.sum())
    return skel, removed


def part1(free, occ, meta, H, start, open_radius_px, min_hole_m2):
    res = meta['resolution']
    n = free.size
    print(f'[P1] pixels: free {free.sum()/n*100:.1f}%, occupied {occ.sum()/n*100:.1f}%, '
          f'unknown {(~free & ~occ).sum()/n*100:.1f}%')

    start_rc = start_pixel(free, meta, H, start)
    track, n_comp, n_filled = build_track_mask(free, start_rc, res, open_radius_px, min_hole_m2)
    eu = euler_number(track, connectivity=1)
    print(f'[P1] free components: {n_comp} -> kept 1 ({track.sum()} px, '
          f'{track.sum()*res**2:.1f} m²); small holes filled: {n_filled}')
    print(f'[P1] track Euler number: {eu}  (expect 0 = one band with one hole, the infield)')

    skel_raw = skeletonize(track)
    skel, removed = prune_spurs(skel_raw)
    _, n_loops = ndi.label(skel, structure=np.ones((3, 3)))
    print(f'[P1] skeleton: {skel_raw.sum()} px raw, {removed} px of spurs pruned, '
          f'{skel.sum()} px left in {n_loops} component(s)  (expect 1)')

    ok = (eu == 0) and (n_loops == 1) and skel.sum() > 0
    print(f'[P1] {"PASS" if ok else "FAIL"}')
    return track, skel_raw, skel, start_rc, ok


def plot_part1(free, occ, track, skel_raw, skel, start_rc, title, out_path):
    cls = np.full(free.shape, 0.5)                        # unknown = grey
    cls[free] = 1.0
    cls[occ] = 0.0

    fig, ax = plt.subplots(2, 2, figsize=(16, 16), layout='constrained')
    ax[0, 0].imshow(cls, cmap='gray', vmin=0, vmax=1)
    ax[0, 0].set_title('1. classified (white free, grey unknown, black occupied)')

    ax[0, 1].imshow(track, cmap='gray')
    ax[0, 1].set_title('2. track mask (cleaned, start component only)')

    for a, sk, t in ((ax[1, 0], skel_raw, '3. raw skeleton (with spurs)'),
                     (ax[1, 1], skel, '4. pruned skeleton (loop only)')):
        a.imshow(track, cmap='gray', alpha=0.35)
        r, c = np.nonzero(sk)
        a.scatter(c, r, s=0.3, c='red')
        a.set_title(t)

    for a in ax.flat:
        a.plot(start_rc[1], start_rc[0], 'o', c='tab:blue', ms=8)
        a.set_axis_off()
    fig.suptitle(title, fontsize=16)
    fig.savefig(out_path, dpi=150)
    print(f'saved {out_path}')


# ------------------------------------------------------------------ main ----
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--track', default='Spielberg')
    ap.add_argument('--map', default='GT', help='map folder under data/<track>/, e.g. GT, carto')
    ap.add_argument('--gt-dir', default='GT', help='which folder is the ground truth (gym frame)')
    ap.add_argument('--start', nargs=3, type=float, default=None, metavar=('X', 'Y', 'YAW'),
                    help="start pose in this map's frame "
                         "(default: SIM_START for the GT folder, 0 0 0 for SLAM folders)")
    ap.add_argument('--open-radius', type=int, default=1, help='opening radius [px], 0 = off')
    ap.add_argument('--min-hole', type=float, default=0.1, help='fill holes smaller than this [m²]')
    a = ap.parse_args()

    if a.start is not None:
        start = a.start
    elif a.map == a.gt_dir:
        start = SIM_START
    else:
        start = [0.0, 0.0, 0.0]

    yaml_path = map_dir(a.track, a.map) / f'{a.track}_map.yaml'
    plot_dir, out_dir = out_dirs(a.track, a.map)

    _, occ, free, meta = load_map(yaml_path)
    H = free.shape[0]
    print(f'map: {yaml_path}  ({free.shape[1]}x{H} px @ {meta["resolution"]} m), start {start}')

    track, skel_raw, skel, start_rc, ok = part1(free, occ, meta, H, start,
                                                a.open_radius, a.min_hole)

    plot_part1(free, occ, track, skel_raw, skel, start_rc,
               f'{a.track} {a.map}: Part 1 stages',
               plot_dir / f'{a.track}_{a.map}_P1_stages.png')

    npz_path = out_dir / f'{a.track}_{a.map}_part1.npz'
    np.savez_compressed(npz_path, track=track, skel=skel,
                        start_rc=np.array(start_rc), start=np.array(start))
    print(f'saved {npz_path}')
    plt.show()


if __name__ == '__main__':
    main()