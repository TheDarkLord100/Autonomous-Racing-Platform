#!/usr/bin/env python3
"""
Frame sanity check before track extraction.
  Plot A: GT map image (pixel space) + provided centreline mapped world -> pixel.
          Tests world_to_px.                                   -> data/<track>/<gt-dir>/plots/
  Plot B: GT walls vs SLAM walls, both in the gym frame.
          Tests px_to_world + the SLAM -> gym start-pose transform.
                                                               -> data/<track>/<slam-dir>/plots/
Run from anywhere:
  python3 check_frames.py [--track Spielberg] [--gt-dir GT] [--slam-dir carto]
"""
import argparse

import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy import ndimage as ndi
from scipy.spatial import cKDTree

from map_utils import load_map, px_to_world, world_to_px, se2, map_dir, out_dirs


def wall_surface(occ, free):
    """Occupied pixels touching free space = the wall surface the LiDAR sees."""
    return occ & ndi.binary_dilation(free, structure=np.ones((3, 3)))


def main():
    """Parse arguments and save the GT/SLAM wall overlay figures in the simulator frame."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--track', default='Spielberg')
    ap.add_argument('--gt-dir', default='GT', help='ground-truth map folder under data/<track>/')
    ap.add_argument('--slam-dir', default='carto', help='SLAM map folder under data/<track>/')
    a = ap.parse_args()

    gt, slam = map_dir(a.track, a.gt_dir), map_dir(a.track, a.slam_dir)
    gt_yaml       = gt / f'{a.track}_map.yaml'
    gt_centerline = gt / f'{a.track}_centerline.csv'
    slam_yaml     = slam / f'{a.track}_map.yaml'
    slam_meta     = slam / f'{a.track}.yaml'

    gt_plot_dir, _   = out_dirs(a.track, a.gt_dir)
    slam_plot_dir, _ = out_dirs(a.track, a.slam_dir)

    # ---------- load ----------
    gt_img, gt_occ, gt_free, gt_meta = load_map(gt_yaml)
    sl_img, sl_occ, sl_free, sl_meta = load_map(slam_yaml)
    H_gt, H_sl = gt_img.shape[0], sl_img.shape[0]
    with open(slam_meta) as f:
        start_pose = yaml.safe_load(f)['origin']          # SLAM map frame in gym frame
    cl = np.loadtxt(gt_centerline, delimiter=',', comments='#')

    # ---------- Plot A: centreline world -> GT pixels ----------
    rA, cA = world_to_px(cl[:, 0], cl[:, 1], gt_meta, H_gt)
    ri, ci = np.round(rA).astype(int), np.round(cA).astype(int)
    inside = (ri >= 0) & (ri < H_gt) & (ci >= 0) & (ci < gt_img.shape[1])
    on_free = np.zeros(len(cl), bool)
    on_free[inside] = gt_free[ri[inside], ci[inside]]
    print(f'[A] centreline points on free pixels: {on_free.mean()*100:.1f}%  (expect ~100%)')

    figA, axA = plt.subplots(figsize=(10, 10))
    axA.imshow(gt_img, cmap='gray')
    axA.plot(cA, rA, 'r-', lw=1, label='provided centreline (world→px)')
    axA.set_title(f'{a.track} {a.gt_dir} map (pixel space) + provided centreline\n'
                  f'{on_free.mean()*100:.1f}% of centreline on free pixels')
    axA.set_xlabel('col [px]')
    axA.set_ylabel('row [px]')
    axA.legend(loc='upper right')
    figA.tight_layout()
    outA = gt_plot_dir / f'{a.track}_{a.gt_dir}_A_centreline_px.png'
    figA.savefig(outA, dpi=150)
    print(f'saved {outA}')

    # ---------- Plot B: walls of both maps in gym frame ----------
    gr, gc = np.nonzero(wall_surface(gt_occ, gt_free))
    gx, gy = px_to_world(gr, gc, gt_meta, H_gt)

    sr, sc_ = np.nonzero(wall_surface(sl_occ, sl_free))
    sx_w, sy_w = px_to_world(sr, sc_, sl_meta, H_sl)       # pixel -> SLAM frame
    sx_w, sy_w = se2(sx_w, sy_w, start_pose)               # SLAM frame -> gym frame

    d, _ = cKDTree(np.c_[gx, gy]).query(np.c_[sx_w, sy_w])
    print(f'[B] {a.slam_dir} wall -> nearest GT wall: median {np.median(d)*100:.1f} cm, '
          f'95th pct {np.percentile(d, 95)*100:.1f} cm, max {d.max()*100:.1f} cm')

    figB, axB = plt.subplots(figsize=(12, 9))
    axB.scatter(gx, gy, s=0.3, c='lightgray', label='GT walls')
    sc = axB.scatter(sx_w, sy_w, s=0.5, c=d, cmap='viridis', vmin=0, vmax=0.5,
                     label=f'{a.slam_dir} walls (→ gym frame)')
    figB.colorbar(sc, ax=axB, label='distance to GT wall [m]')
    axB.plot(cl[:, 0], cl[:, 1], 'r-', lw=0.8, label='provided centreline')
    px0, py0, yaw0 = start_pose
    axB.arrow(px0, py0, 3 * np.cos(yaw0), 3 * np.sin(yaw0), width=0.25, color='tab:blue',
              length_includes_head=True, label='start pose')
    axB.set_aspect('equal')
    axB.set_xlabel('x [m] (gym frame)')
    axB.set_ylabel('y [m] (gym frame)')
    axB.set_title(f'{a.track} {a.slam_dir}: GT vs SLAM walls (gym frame)\n'
                  f'median {np.median(d)*100:.1f} cm, 95th {np.percentile(d, 95)*100:.1f} cm, '
                  f'max {d.max()*100:.1f} cm')
    axB.legend(loc='upper right', markerscale=10)
    figB.tight_layout()
    outB = slam_plot_dir / f'{a.track}_{a.slam_dir}_B_walls_gt_vs_slam.png'
    figB.savefig(outB, dpi=150)
    print(f'saved {outB}')

    plt.show()


if __name__ == '__main__':
    main()