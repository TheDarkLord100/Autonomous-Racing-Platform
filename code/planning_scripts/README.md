# planning_scripts

Offline global planner. Turns a track centreline (with wall widths) into an optimised
raceline with a speed profile, and writes it as a CSV that `global_planner` publishes.

Not a ROS package: plain Python, run from a terminal.

## Requirements

`numpy`, `scipy`, `matplotlib`, `osqp` (QP solver), `PyYAML`. In the workspace venv:

```bash
pip install osqp
```

## Usage

```bash
cd <repo>/code/planning_scripts
python3 optimise_raceline.py --map carto --method mincurv --iters 10     # iterative min curvature
python3 optimise_raceline.py --map carto --method mincurv                # single-pass min curvature
python3 optimise_raceline.py --map carto --method shortest               # shortest path
python3 optimise_raceline.py --map GT --method mincurv --iters 10 --grip 0.8 --suffix g08
```

Paths are resolved relative to the repository, so the script runs from anywhere.

| | Path |
|---|---|
| Input | `data/<track>/<map>/extraction/<track>_<map>_centreline.csv` (from `code/mapping_scripts`) |
| Output | `data/<track>/<map>/raceline/<track>_<map>_<tag>.csv`, `tag = method[_iter][_suffix]` |
| Plot | `data/<track>/<map>/plots/<track>_<map>_raceline_<tag>.png` |

## Methods

The raceline is the centreline shifted sideways: each point *i* moves by a lateral offset
αᵢ along the centreline normal, bounded by the track widths. One pass is one sparse QP
in α, solved with OSQP.

| `--method` | Objective |
|---|---|
| `mincurv` | minimise Σ (second differences)² ≈ squared curvature |
| `shortest` | minimise Σ (first differences)² ≈ path length |

`--iters N > 1` is the **iterative** variant: after each pass the line is resampled, new
normals are computed, the widths are ray-cast again to the **original** walls, and the QP
is solved again around the previous solution (`--damping` sets the step size).

### Wall clearance check

After the optimisation passes, the true (shortest) distance from every raceline point to
the walls is checked against `veh_width / 2 + margin`. Wherever a point is too close, its
bounds are tightened and the last QP is solved again, for up to `--clear-rounds` rounds.
A line that still fails is **not written**.

### Speed profile

A friction-circle profile for a flying lap: lateral acceleration `v²κ` and longitudinal
acceleration share `grip · μ · g`, with a forward (acceleration) and backward (braking)
pass, capped at `--v-max`.

## Output CSV

```
# s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, w_left_m, w_right_m
```

Map frame of the input map (e.g. `carto_map` for a Cartographer map), ROS yaw convention
(ψ = 0 along +x, counter-clockwise positive), point spacing `--step` (0.2 m).
`w_left_m` / `w_right_m` are measured along the raceline normal.

## Parameters

| Flag | Default | Meaning |
|---|---|---|
| `--track` | `Spielberg` | folder under `data/` |
| `--map` | `carto` | map folder under `data/<track>/` (`GT`, `carto`, ...) |
| `--method` | `mincurv` | `mincurv` or `shortest` |
| `--iters` | `1` | QP passes; > 1 = iterative variant |
| `--tol` | `0.005` | stop iterating when max \|Δα\| < tol [m] |
| `--damping` | `0.5` | step size for passes ≥ 2 (0–1] |
| `--opt-step` | `0.5` | point spacing inside the QP [m] |
| `--step` | `0.2` | output point spacing [m] |
| `--veh-width` | `0.31` | car width [m] |
| `--margin` | `0.15` | extra clearance per side [m] |
| `--clear-tol` | `0.005` | allowed clearance shortfall [m] |
| `--clear-rounds` | `30` | max bound-tightening rounds |
| `--buffer` | `0.0` | extra bound tightening inside the QP only [m] |
| `--mu` | `1.0489` | tyre friction coefficient (f1tenth_gym default) |
| `--grip` | `0.7` | fraction of μg the speed profile may use |
| `--v-max` | `5.0` | top speed [m/s] |
| `--ax-max` | `9.51` | max acceleration / braking [m/s²] |
| `--wheelbase` | `0.3302` | [m], for the steering-feasibility check |
| `--steer-max` | `0.4189` | max steering angle [rad] |
| `--suffix` | `''` | appended to the output tag, e.g. `g08` |

The car geometry and friction defaults are the `f1tenth_gym` vehicle's values.
