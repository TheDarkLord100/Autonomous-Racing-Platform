"""Shared paths and map/coordinate helpers for the mapping scripts."""
import os
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

# code/mapping_scripts/map_utils.py -> parents[2] = repo root
REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / 'data'


def map_dir(track, name):
    """data/<track>/<name>, e.g. map_dir('Spielberg', 'GT') or map_dir('Spielberg', 'carto')."""
    d = DATA_DIR / track / name
    if not d.is_dir():
        raise FileNotFoundError(f'map folder not found: {d}')
    return d


def out_dirs(track, name):
    """Create and return (plots, extraction) folders inside data/<track>/<name>/."""
    d = map_dir(track, name)
    plots, extraction = d / 'plots', d / 'extraction'
    plots.mkdir(exist_ok=True)
    extraction.mkdir(exist_ok=True)
    return plots, extraction


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    # os.path.join returns meta['image'] unchanged if it's already absolute
    img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image'])
    img = np.array(Image.open(img_path).convert('L'), dtype=float)
    p = img / 255.0 if meta.get('negate', 0) else (255.0 - img) / 255.0  # occupancy prob
    occ = p > meta['occupied_thresh']
    free = p < meta['free_thresh']
    return img, occ, free, meta


def px_to_world(rows, cols, meta, H):
    """Pixel centres -> world. Row 0 is the TOP of the image, origin is BOTTOM-left."""
    res = meta['resolution']
    ox, oy, oyaw = meta['origin']
    xl = (cols + 0.5) * res
    yl = (H - rows - 0.5) * res
    c, s = np.cos(oyaw), np.sin(oyaw)
    return ox + c * xl - s * yl, oy + s * xl + c * yl


def world_to_px(x, y, meta, H):
    """Exact inverse of px_to_world (float pixel coords, not rounded)."""
    res = meta['resolution']
    ox, oy, oyaw = meta['origin']
    dx, dy = x - ox, y - oy
    c, s = np.cos(oyaw), np.sin(oyaw)
    xl = c * dx + s * dy
    yl = -s * dx + c * dy
    return H - 0.5 - yl / res, xl / res - 0.5   # rows, cols


def se2(x, y, pose):
    """Apply pose [px, py, yaw] to points (child frame -> parent frame)."""
    px, py, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    return px + c * x - s * y, py + s * x + c * y