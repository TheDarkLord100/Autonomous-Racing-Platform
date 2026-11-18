# data

Maps, extracted tracks and racelines used by the stack. Everything here is produced by
the project's own tools and is small enough to version; run logs and rosbags are **not**
stored here (they go to `~/f1tenth_simulation_logs/`).

## Layout

```
data/<track>/<map>/
├── <track>_map.yaml + .png / .pgm      occupancy map (image + metadata)
├── <track>.yaml + <track>.pbstream     SLAM maps only: Cartographer sidecar + pose graph
├── extraction/                         track extraction (code/mapping_scripts)
│   ├── <track>_<map>_part1.npz             step 1: cleaned free space + skeleton
│   ├── <track>_<map>_centreline.csv        step 2: x, y, w_right, w_left
│   └── <track>_<map>_metrics.json          step 3: SLAM vs GT errors (SLAM maps)
├── raceline/                           global planning (code/planning_scripts)
│   └── <track>_<map>_<method>[_iter][_suffix].csv
└── plots/                              figures from every step
```

## Contents

| Folder | What it is | Frame |
|---|---|---|
| `Spielberg/GT` | the simulator's ground-truth map (copied from `f1tenth_racetracks`), its provided centreline / raceline, our extraction and racelines | `map` (simulator) |
| `Spielberg/carto` | Cartographer map built at 3 m/s with clean odometry: pose graph, image, extraction, racelines | `carto_map` |
| `Spielberg/5ms` | Cartographer map built at 5 m/s (kept for comparison; not extracted) | `carto_map` |

`carto_map`'s pose in the simulator frame is in the sidecar (`origin`); for these maps it is
`[0.0, 0.0, -2.879]`, the spawn pose where mapping started.

## File formats

**Occupancy map:** ROS `map_server` format: a `.yaml` with `image`, `resolution`
(m/pixel), `origin` (x, y, yaw of the lower-left pixel), `occupied_thresh`,
`free_thresh`, and the image (`.png` / `.pgm`; dark = occupied).

**Cartographer sidecar** (`<track>.yaml`):

```yaml
pbstream: Spielberg.pbstream
origin: [0.0, 0.0, -2.879]     # pose of carto_map in the simulator frame
```

**Centreline CSV** (TUMFTM format), in the map's own frame:

```
x_m, y_m, w_tr_right_m, w_tr_left_m
```

**Raceline CSV**, in the map's own frame, one row every 0.2 m, ROS yaw convention:

```
s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, w_left_m, w_right_m
```

The raceline columns match `race_msgs/Waypoint` 1:1.

## Adding a track or a map

1. Map it: [docs/03_mapping.md](../docs/03_mapping.md) (writes `data/<track>/<new map>/`).
2. Extract it: [docs/04_track_extraction.md](../docs/04_track_extraction.md).
3. Optimise racelines: [docs/05_global_planning.md](../docs/05_global_planning.md).
4. Point a scenario file at it: [docs/09_running_simulations.md](../docs/09_running_simulations.md).
