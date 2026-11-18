# mapping_scripts

Offline track extraction: occupancy-grid map → centreline with left/right wall widths, the
input of the raceline optimiser (`code/planning_scripts`).

Not a ROS package: plain Python, run from a terminal. The same scripts work on the
simulator's ground-truth map and on a SLAM (Cartographer) map.

## Requirements

`numpy`, `scipy`, `scikit-image`, `Pillow`, `PyYAML`, `matplotlib`. In the workspace venv:

```bash
pip install scikit-image
```

## Expected data layout

All paths are resolved relative to the repository (`map_utils.py`: `REPO_ROOT/data`):

```
data/<track>/<map>/
├── <track>_map.yaml + .png/.pgm        the occupancy map (gym GT, or map_saver output)
├── <track>.yaml                        SLAM maps only: sidecar with pbstream + origin
├── extraction/                         written here
└── plots/                              written here
```

`<map>` is the folder name: `GT` for the simulator's map, `carto` (or any other name) for
a SLAM map. The SLAM sidecar's `origin` is the pose of the SLAM map frame in the
simulator frame (the car pose when mapping started); `check_frames.py` and
`compare_tracks.py` use it.

## Pipeline

Run in this order. Each step writes a plot to `plots/`; look at it before going on.

### 0. `check_frames.py`: frame sanity check (once per SLAM map)

```bash
python3 check_frames.py --track Spielberg --gt-dir GT --slam-dir carto
```

- Plot A: the provided GT centreline drawn on the GT map image, tests world ↔ pixel conversion.
- Plot B: GT walls and SLAM walls overlaid in the simulator frame, tests the SLAM → gym
  transform from the sidecar `origin`.

If Plot B is rotated or shifted, the sidecar origin is wrong; fix it before extracting.

### 1. `extract_track.py`: map → cleaned free space → skeleton

```bash
python3 extract_track.py --map GT
python3 extract_track.py --map carto
```

Classifies pixels, treats unknown as occupied, cleans small holes and noise, skeletonises the free space and prunes spurs until
one closed loop remains.

- `--start X Y YAW`: start pose in this map's frame. Default: the simulator spawn for the
  GT folder, `0 0 0` for SLAM folders (a SLAM map's origin is where mapping started).
- `--open-radius` (px, default 1), `--min-hole` (m², default 0.1): cleaning.
- Output: `extraction/<track>_<map>_part1.npz`, plot `P1_stages.png` (opens a window).

### 2. `build_centerline.py`: skeleton → ordered, smoothed centreline + widths

```bash
python3 build_centerline.py --map GT
python3 build_centerline.py --map carto
```

Walks the skeleton loop from the start pixel, orients it in the driving direction (from the
start yaw), fits a smoothing spline, resamples it evenly and ray-casts the distance to each
wall along the normal.

- `--step` (m, default 0.2) point spacing, `--smooth` spline smoothing, `--thin` use every
  n-th skeleton pixel for the fit, `--max-width` ray-cast limit per side.
- Output: `extraction/<track>_<map>_centreline.csv`, TUMFTM format in the map's own frame:

```
x_m, y_m, w_tr_right_m, w_tr_left_m
```

### 3. `compare_tracks.py`: validation and mapping error (optional)

```bash
python3 compare_tracks.py --track Spielberg --gt-dir GT --slam-dir carto
```

- Pair 1: GT extraction vs the provided `<track>_centerline.csv` (validates the extractor).
- Pair 2: SLAM extraction (moved to the gym frame) vs GT extraction. Wall errors are
  computed from centreline offset + width.
- Output: `C_overlay.png`, `C_errors_vs_s.png`, `extraction/<track>_<slam>_metrics.json`.

## Next step

The centreline CSV is the input of `code/planning_scripts/optimise_raceline.py`.

## Files

| File | Purpose |
|---|---|
| `map_utils.py` | repo paths, map loading, pixel ↔ world conversion, SE(2) helper |
| `check_frames.py` | step 0 |
| `extract_track.py` | step 1 |
| `build_centerline.py` | step 2 |
| `compare_tracks.py` | step 3 |
