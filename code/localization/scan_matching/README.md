# scan_matching

PL-ICP (point-to-line Iterative Closest Point) scan matching localization for the F1TENTH stack. Estimates the car's pose (x, y, yaw) by matching each 2D LiDAR scan against a reference scan, starting from a wheel-odometry guess. Frame-to-frame: no map is used; the pose is the chain of scan-to-scan motions from the initial pose.

Own implementation (ROS 2 Jazzy, `ament_python`). Ideas taken from: Censi, *An ICP variant using a point-to-line metric* (ICRA 2008) and his CSM / GPC reference code; F1TENTH Lab 5 (scan matching) and lectures 9–10 on fast correspondence search; `laser_scan_matcher` / `ros2_laser_scan_matcher` (ROS wrapper structure, keyframes, odometry guess).

---

## Folder structure

```
scan_matching/
├── package.xml                        ROS package manifest (deps: rclpy, tf2_ros, geometry_msgs,
│                                      nav_msgs, sensor_msgs, numpy, odom_noise)
├── setup.py                           install rules + executable scan_matching_node
├── setup.cfg                          script install location (ament_python standard)
├── resource/scan_matching             ament index marker
├── config/
│   └── scan_matching_params.yaml      all parameters
├── launch/
│   └── scan_matching_launch.py        starts the node (+ odom_noise with odom_noise:=true)
└── scan_matching/                     Python package
    ├── core.py                        scan preprocessing, correspondence search, solver, ICP loop
    ├── tracker.py                     keyframes, odometry guess, laser offset, fallback, reset
    └── scan_matching_node.py          ROS 2 node wrapping the tracker
```

Design rule: `core.py` and `tracker.py` are plain NumPy with **no ROS imports**. The algorithm can be run, tested and debugged offline (e.g. on recorded bags); the node only converts messages and calls into it.

Optional dependency: `numba` (compiles the two inner loops, ~5x faster per match). Without it the same code runs as plain Python with identical results.

---

## What each file does

### `core.py`

**SE(2) helpers** on poses `(x, y, theta)`:
- `wrap_angle(a)` → angle in [−π, π)
- `se2_compose(a, b)` → `a ⊕ b`, apply b in a's frame
- `se2_inverse(a)` → SE(2) inverse
- `transform_points(points, pose)` → move (N, 2) points by a pose

**`Scan`** — one LaserScan in its own laser frame, prepared for matching:
- ranges → points; rays outside `(range_min, range_max)` or non-finite are invalid
- Censi's **jump tables** (`up_bigger`, `up_smaller`, `down_bigger`, `down_smaller`): for each ray, the index of the next ray up/down with a larger/smaller range. Built once per reference scan.
- **surface normals**: for each ray, a line fit over the neighbouring points within ±`normal_length`/2 of path length along the same surface. Surfaces are split at invalid rays and at gaps > `normal_max_gap`; windows that are not close to a straight line (e.g. around a corner) get no normal.

**Correspondence search** — for each current point (already moved into the reference frame), the closest reference ray `j1` and its closer neighbour `j2`:
- `find_correspondences_jump` — Censi's fast search (below), the default
- `find_correspondences_brute` — compares against every ray; same answer, much slower; selectable with `correspondence_search: brute`
- `_jump_tables_kernel`, `_jump_search_kernel` — the two inner loops, written so the same source runs compiled by numba or as plain Python
- `warmup()` — compiles the kernels at node start-up so the first scan is not delayed

**`solve_point_to_line(p, q, n, prior_t, prior_w)`** — exact closed-form minimiser of the point-to-line cost plus an odometry prior on translation (below). Also returns λ_min, how well the translation is constrained.

**`icp(ref, cur, guess, ...)`** — the PL-ICP loop. Returns an `IcpResult`: pose of the current scan in the reference scan's frame, iterations, number of correspondences, mean squared error, `degenerate` flag, λ_min, and a reason if it failed.

### `tracker.py`

**`ScanMatchTracker`** — turns a stream of scans + odometry into a pose in the map:
- converts the odometry motion since the reference scan into an initial guess in the laser frame, runs `icp`, converts the result back to `base_link`
- keeps the reference (keyframe) scan and its pose
- falls back to the odometry motion if the match fails or disagrees too much with odometry
- `reset(pose)` for a new absolute pose (RViz "2D Pose Estimate")

**`interpolate_pose(times, poses, t)`** — odometry pose at the scan's timestamp, linear in x, y and shortest-way-round in θ.

### `scan_matching_node.py`

ROS 2 node: subscribes to scans, odometry and `/initialpose`, holds each scan until odometry covering its timestamp has arrived, calls the tracker, publishes pose / odometry / path, logs statistics every 5 s.

---

## The algorithm

### Per scan

Inputs: current scan `S_k`, reference scan `S_ref` (previous scan, or keyframe), odometry poses `o_ref`, `o_k` at both scans' timestamps, laser mounting `T_bl` (laser in `base_link`, from TF).

1. **Odometry guess.** Base motion since the reference: `Δ_odom = o_ref⁻¹ ⊕ o_k`. ICP works between laser frames, so the guess is `q₀ = T_bl⁻¹ ⊕ Δ_odom ⊕ T_bl`. (The laser sits ahead of `base_link`; when the car yaws it also moves sideways.)
2. **PL-ICP** from `q₀` → laser motion `L` (below).
3. **Back to base_link:** `Δ = T_bl ⊕ L ⊕ T_bl⁻¹`.
4. **Pose:** `pose_k = pose_ref ⊕ Δ`, where `pose_ref` is the map pose at the reference scan.
5. **Keyframe update:** the current scan becomes the new reference when it moved ≥ `kf_dist_linear` or turned ≥ `kf_dist_angular` from the reference (both 0: every scan, i.e. pure frame-to-frame).

If ICP fails (too few correspondences or inliers, singular system) or its result differs from the odometry guess by more than `max_correction_linear` / `max_correction_angular`, `Δ = Δ_odom` (odometry fallback) and the scan becomes the new reference.

### PL-ICP loop (`core.icp`)

Pose estimate `q = (t, θ)`, start `q = q₀`. Repeat up to `max_iterations`:

1. **Transform** the current points `pᵢ` into the reference frame with `q`.
2. **Correspondences:** closest valid reference ray `j1` for each point, within `max_correspondence_dist`, and only where `j1` has a surface normal `nᵢ`.
3. **Point-to-line error** (distance to the line through `q_j1` with normal `nᵢ`, not to the point itself):
   `eᵢ = nᵢ · (R(θ) pᵢ + t − q_j1)`
4. **Outlier trimming** (adaptive): keep `|eᵢ| ≤ max(outlier_mult · quantile(|e|, outlier_quantile), floor)`. The floor starts at `max_correspondence_dist` and halves every iteration down to `outlier_min_dist`: early iterations, with a rough guess, keep the few far points that fix one direction; later iterations reject real outliers.
5. **Solve** for the new `q` exactly (below).
6. **Stop** when the correspondences no longer change (PL-ICP then returns the same pose — finite convergence) or the step is below `epsilon_xy` / `epsilon_theta`.

### Closed-form solve (`core.solve_point_to_line`)

Unknowns `x = [tx, ty, c, s]` with `c = cos θ`, `s = sin θ`, so that `R pᵢ + t = Mᵢ x`, `Mᵢ = [[1, 0, pᵢₓ, −pᵢᵧ], [0, 1, pᵢᵧ, pᵢₓ]]`. Cost:

`J(x) = Σᵢ (nᵢᵀ (Mᵢ x − q_j1))² + w · |t − t_odom|²  =  xᵀ A x − 2 bᵀ x + const`

subject to `c² + s² = 1`.

- Lagrange multiplier λ on the constraint: `(A + λ W) x = b`, `W = diag(0, 0, 1, 1)`.
- Eliminate `t` → 2×2 system `(S + λ I) [c, s]ᵀ = v` (Schur complement).
- `c² + s² = 1` becomes a 4th-degree polynomial in λ: `|adj(S + λI) v|² − det(S + λI)² = 0`.
- Each real root gives a candidate `[c, s]` (normalised) and `t`; keep the one with the lowest cost. `θ = atan2(s, c)`.

No linearisation, no iteration inside the solve: exact for the given correspondences.

### Surface normals

Censi's original line goes through `j1` and its neighbour `j2`. With 1080 beams over 270°, neighbouring points on a wall 1 m away are 0.4 cm apart — less than the range noise — so a two-point line points in a near-random direction. Each reference ray instead gets a normal from a least-squares line fit (2×2 scatter matrix, smaller eigenvector) over `normal_length` of wall around it, computed for all rays at once with cumulative sums. A fixed length of wall rather than a fixed number of rays gives near and far walls equally good normals. `normal_length: 0` uses the `j1–j2` line instead.

### Fast correspondence search (jump tables)

Scan points are ordered by angle, so the closest reference ray is near the ray with the same bearing.

1. **Start** at the previous point's match + 1 (consecutive points match consecutive rays), else at the ray with the point's bearing.
2. **Walk** up and down from the start, alternating towards the side that is currently closer.
3. **Early stop:** a ray at angular gap Δφ from point `p` is at least `|p| · sin(Δφ)` away. Once that bound exceeds the best distance so far, nothing further in that direction can win.
4. **Jumps:** if the current ray's range is smaller than `|p|`, rays with even smaller ranges are further away, so jump to the next ray with a *larger* range (`up_bigger` / `down_bigger`); otherwise to the next *smaller* one.

Result: identical to brute force, about O(N) per iteration instead of O(N²).

### Odometry prior and degeneracy

`λ_min` = smaller eigenvalue of `Σ nᵢ nᵢᵀ` over the inliers ≈ how many points' worth of wall face the least-constrained translation direction. In turns it is ~40–100; on long straights (two parallel walls, nothing ahead within range) ~0.3: the walls fix heading and sideways position but not motion along the track.

The prior `w · |t − t_odom|²` (`odom_prior_weight`, same units) is tiny compared with the walls wherever they constrain the motion, and decides the along-track position where they do not. The match is therefore always used: heading and sideways position from the scan, along-track from odometry on straights. Matches with `λ_min < degeneracy_min_eig` are counted as degenerate in the log. With `odom_prior_weight: 0` a degenerate match is rejected instead and the whole step comes from odometry.

---

## Data flow

### Callbacks (`scan_matching_node.py`)

- **odometry** (100 Hz): appended to a 2 s buffer; then pending scans are processed.
- **scan** (10 Hz): appended to a queue of at most 3 (oldest dropped and counted as skipped); then pending scans are processed.
- **processing:** a scan is matched only once odometry with a timestamp ≥ the scan's has arrived, so the odometry guess is interpolated at exactly the scan time, never extrapolated from older odometry. Scans older than the odometry buffer are dropped.
- **/initialpose:** stored and applied before the next scan; clears the scan queue (those scans were taken before the new pose) and the published path.
- **timer** (5 s): logs statistics.

On the first scan the node looks up `base_link → laser` in TF and creates the tracker. At start-up it compiles the numba kernels (`warmup`) and logs which backend is in use.

### Topics and frames

| | Topic | Type | Frames |
|---|---|---|---|
| in | `/scan` | LaserScan | `ego_racecar/laser` |
| in | `/ego_racecar/odom_noisy` | Odometry (from `odom_noise`) | `odom` → `ego_racecar/base_link` |
| in | `/initialpose` | PoseWithCovarianceStamped | `map` |
| out | `/scan_match/odom` | Odometry; twist from consecutive poses | `map` → `ego_racecar/base_link` |
| out | `/scan_match/pose` | PoseStamped | `map` |
| out | `/scan_match/path` | Path, last `path_max_length` poses | `map` |
| out | TF (optional) | `publish_tf: true` | `map` → `scan_match/base_link` |

TF is off by default, so the node can run alongside other localizers and the simulator without conflicting transforms. The output is in `map`, directly comparable with the simulator's ground truth `/ego_racecar/odom`.

### Initial pose

Before the first `/initialpose` the pose starts at `initial_pose_x/y/theta`. RViz "2D Pose Estimate" sets the pose (the simulator also moves the car there); the next scan becomes the new reference. `odom_noise` ignores the teleport, so odometry continues without a jump.

---

## Parameters (`config/scan_matching_params.yaml`)

### Topics and frames

| Parameter | Default | Meaning |
|---|---|---|
| `scan_topic` | `/scan` | LiDAR input |
| `odom_topic` | `/ego_racecar/odom_noisy` | odometry for the initial guess |
| `initialpose_topic` | `/initialpose` | RViz 2D Pose Estimate |
| `output_odom_topic` | `/scan_match/odom` | pose + twist output |
| `output_pose_topic` | `/scan_match/pose` | pose output |
| `output_path_topic` | `/scan_match/path` | path for RViz |
| `map_frame` | `map` | frame of the outputs |
| `base_frame` | `ego_racecar/base_link` | car frame (laser offset looked up from it) |
| `publish_tf` | `false` | broadcast `map → tf_child_frame` |
| `tf_child_frame` | `scan_match/base_link` | child frame if TF is published |
| `path_max_length` | 3000 | poses kept in the published path |
| `initial_pose_x/y/theta` | 0, 0, 0 | pose before the first `/initialpose` |

### Scan preprocessing

| Parameter | Default | Meaning |
|---|---|---|
| `max_range` | 0.0 | ignore returns beyond this [m]; 0 = scan's `range_max` |
| `normal_length` | 0.4 | wall length per normal fit [m]; 0 = `j1–j2` line |
| `normal_max_gap` | 0.2 | gap between neighbouring points that splits two surfaces [m] |

### PL-ICP

| Parameter | Default | Meaning |
|---|---|---|
| `correspondence_search` | `jump` | `jump` (Censi's jump tables) or `brute` |
| `max_iterations` | 30 | ICP iteration limit |
| `max_correspondence_dist` | 0.5 | max point-to-ray distance for a correspondence [m] |
| `outlier_quantile` | 0.7 | trimming: quantile of the errors… |
| `outlier_mult` | 2.0 | …times this = threshold |
| `outlier_min_dist` | 0.02 | final trimming floor [m]; starts at `max_correspondence_dist`, halves per iteration |
| `min_correspondences` | 50 | fewer → match fails |
| `epsilon_xy` | 0.001 | stop when the translation step is smaller [m] |
| `epsilon_theta` | 0.001 | stop when the rotation step is smaller [rad] |
| `odom_prior_weight` | 0.25 | weight of the odometry prior on translation (points' worth); 0 = off |
| `degeneracy_min_eig` | 10.0 | λ_min below this → counted as degenerate (rejected only if the prior is off) |

Chosen on recorded Spielberg runs against ground truth: `odom_prior_weight` 0.25 gave the lowest mean error (1.0, 0.5 and 0.1 were worse); `normal_length` 0.4 beat 0.25 and 0.6; the 1 mm / 1 mrad stopping tolerance gives the same accuracy as 0.1 mm in ~4 iterations instead of ~14.

### Keyframes and sanity checks

| Parameter | Default | Meaning |
|---|---|---|
| `kf_dist_linear` | 0.0 | new reference scan after this distance [m]; 0 / 0 = every scan |
| `kf_dist_angular` | 0.0 | new reference scan after this rotation [rad] |
| `max_correction_linear` | 0.5 | ICP result further than this from the odometry guess → odometry used [m] |
| `max_correction_angular` | 0.3 | same, rotation [rad] |

### Stats log (every 5 s)

```
50 scans: match 12.1 ms mean, 65.2 ms max, 5.3 iters, 45 degenerate (odometry along track), 0 odometry fallbacks, 0 scans skipped (node too slow)
```
- **match** — time per scan inside the tracker
- **iters** — mean ICP iterations
- **degenerate** — matches where along-track position came from odometry (high on straights, ~0 in twisty sections)
- **odometry fallbacks** — matches rejected entirely, with the reason in braces
- **scans skipped** — scans dropped because the node fell behind (should stay 0)

---

## Build and run

```bash
f1sim                                            # venv + ROS + workspace
pip install numba --dry-run                      # check it does not change numpy, then:
pip install numba
cd ~/f1tenth_sim_ws && python3 -m colcon build --packages-select scan_matching
source install/setup.bash
ros2 launch scan_matching scan_matching_launch.py   # args: params_file, use_sim_time, odom_noise:=true|false
```

`odom_noise:=true` (default) also starts `odom_noise`; set it to false if it is already running. Then RViz "2D Pose Estimate" at the car. RViz displays: Path `/scan_match/path`, Pose `/scan_match/pose`.

Start-up log: `correspondence search: numba, ready in … s` (or `plain Python` if numba is not installed), then `laser in base: x=0.275 y=0.000 yaw=0.000`.

### Performance

- Laptop, 819-beam scans at 10 Hz: 8–18 ms per match with numba (100 ms available), ~4–6 ICP iterations; ~5x slower without numba.
- Numba compile at start-up: ~2–3 s the first time, < 1 s afterwards (cached).