# 4. Track extraction

Track extraction turns an occupancy map into a centreline with the distance to each wall,
the input of the raceline optimiser. The scripts run offline, on the simulator's
ground-truth map (`GT`) or a SLAM map (`carto`).

Full script reference: [`code/mapping_scripts/README.md`](../code/mapping_scripts/README.md).

## 4.1 Steps

```bash
cd ~/f1tenth_sim_ws/src/Autonomous-Racing-Platform/code/mapping_scripts

python3 check_frames.py --gt-dir GT --slam-dir carto     # 0. frames line up? (SLAM maps)
python3 extract_track.py --map carto                     # 1. map -> cleaned free space -> skeleton
python3 build_centerline.py --map carto                  # 2. skeleton -> centreline + widths
python3 compare_tracks.py --gt-dir GT --slam-dir carto   # 3. mapping error vs GT (optional)
```

Repeat 1–2 with `--map GT` for the ground-truth track. `--map` is the folder name under
`data/Spielberg/`; `--track` selects another track folder.

| Step | What it does | Plot shows |
|---|---|---|
| 0 | overlays GT walls and SLAM walls in the simulator frame, using the sidecar origin | both wall sets |
| 1 | unknown pixels → occupied, cleaning, skeleton, spur pruning | each stage, final skeleton loop |
| 2 | orders the loop in the driving direction, spline-smooths, resamples, ray-casts widths | centreline, start point (s = 0) |
| 3 | GT extraction vs provided centreline; SLAM extraction vs GT extraction | error vs s; `metrics.json` |

## 4.2 Output

`data/<track>/<map>/extraction/<track>_<map>_centreline.csv` with columns
`x_m, y_m, w_tr_right_m, w_tr_left_m` (TUMFTM format), every 0.2 m, in the map's own frame
(`map` for GT, `carto_map` for Cartographer).

## 4.3 Notes

- Unknown (grey) cells are treated as occupied.
- The centreline starts at the pixel nearest the start pose and runs in its direction. Default
  start pose: the simulator spawn for `GT`, `0 0 0` for a SLAM map. `--start X Y YAW` sets it.
- The CSV is written in the map's own frame (`carto_map` for a Cartographer map).

Next: [5. Global planning](05_global_planning.md).
