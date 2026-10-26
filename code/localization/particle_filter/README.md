# particle_filter

Monte Carlo Localization (particle filter) for the F1TENTH stack. Estimates the car's pose (x, y, yaw) on a known occupancy-grid map from a 2D LiDAR and wheel odometry.

Own implementation (ROS 2 Jazzy, `ament_python`). Ideas taken from: MIT RACECAR particle filter (Walsh & Karaman), Nav2 AMCL, ForzaETH SynPF / TUM PF (Stahl et al.), Thrun et al. *Probabilistic Robotics* (ch. 5 motion models, ch. 6.4 likelihood field, ch. 8.3 augmented MCL).

---

## Folder structure

```
particle_filter/
├── package.xml                     ROS package manifest (deps: rclpy, tf2_ros, geometry_msgs,
│                                   nav_msgs, sensor_msgs, odom_noise, numpy, scipy)
├── setup.py                        install rules + executable pf_node
├── setup.cfg                       script install location (ament_python standard)
├── resource/particle_filter        ament index marker
├── config/
│   └── pf_params.yaml              all filter parameters
├── launch/
│   └── pf_launch.py                starts the filter (+ the odom_noise package's node with
│                                   odom_noise:=true)
├── test/                           offline tests (no ROS needed): sim_utils.py mini simulator,
│                                   test_filter.py
└── particle_filter/                Python package
    ├── geometry.py                 SE(2) math
    ├── occupancy_map.py            map, distance field, free-space queries
    ├── motion_model.py             odometry motion model (predict step)
    ├── sensor_model.py             likelihood-field LiDAR model (weight step)
    ├── resampling.py               N_eff, low-variance resampling
    ├── filter.py                   the particle filter itself (ties everything together)
    ├── odom_buffer.py              timestamped odometry history + interpolation
    └── pf_node.py                  ROS 2 node wrapping the filter
```

Design rule: every module except `pf_node.py` is plain NumPy/SciPy with **no ROS imports**. The algorithm can be run, tested and debugged offline; the nodes only convert messages and call into it.

---

## What each file does

### `geometry.py`
SE(2) helpers on poses `(x, y, theta)`.
- `wrap_angle(a)` → angle in [−π, π)
- `yaw_to_quat`, `quat_to_yaw` → conversions for ROS messages (rotation about z only)
- `relative_pose(p0, p1)` → motion from p0 to p1 expressed in p0's frame `(dx forward, dy left, dθ)`. Used to turn two odometry poses into a local motion step.
- `compose(a, b)` → `a ⊕ b`, apply local motion b on top of pose a. Vectorised: works on one pose or an (N, 3) array of particles.
- `interpolate(p0, p1, α)` → pose a fraction α between p0 and p1 (shortest way round for θ).
- `inverse(a)` → SE(2) inverse, used for the `map → odom` transform.

### `occupancy_map.py` — `OccupancyMap`
Built once when the map arrives.
- Classifies cells: occupied if value ≥ `occupied_threshold` (65), free if 0 ≤ value < `free_threshold` (25), else unknown.
- **Distance field**: for every cell, Euclidean distance (m) to the nearest occupied cell (`scipy.ndimage.distance_transform_edt`), clipped at `max_occ_dist`. This precomputation is what makes the sensor model a table lookup instead of ray casting. On the 2000×2000 Spielberg map it takes ~0.45 s.
- `world_to_grid(x, y)` → cell indices (handles non-zero map origin yaw).
- `distance_at(x, y)` → distance-field lookup; points off the map get `max_dist`.
- `is_free(x, y)` → known-free-space test (used to keep particles out of walls).
- `sample_free(n, rng)` → uniform random points in free space (used by global recovery).

### `motion_model.py` — `OdometryMotionModel`
Predict step. Moves every particle by the odometry motion plus noise. See *Algorithm → Predict*.

### `sensor_model.py` — `LikelihoodFieldModel`
Weight step. Scores every particle against the current scan. See *Algorithm → Weight*.
- `set_scan_geometry(angle_min, angle_increment, n)` picks `num_beams` evenly spaced beams and caches their bearings (redone only if the scan layout changes).
- `log_likelihood(particles, ranges, range_min, range_max)` → (N,) log-weights. Also stores `last_mean_beam_loglik` (per-particle average fit per beam), which recovery uses.

### `resampling.py`
- `effective_sample_size(w)` → N_eff = 1 / Σ wᵢ²
- `low_variance_resample(w, rng)` → systematic resampling (one random offset, N evenly spaced pointers into the weight CDF). Each particle is copied ⌊N·w⌋ or ⌈N·w⌉ times; O(N).

### `filter.py` — `ParticleFilter`, `RecoveryConfig`, `ScaleConfig`
The filter. Holds the particle set and runs the cycle.
- State: `particles` (N, 3), `weights` (N,), `scale` (N,) per-particle odometry scale, `estimate` (3,), `covariance` (3×3), `odom_scale` (learned odometry scale).
- `initialize(pose, std_xy, std_theta, is_free)` → Gaussian cloud around a pose; particles landing outside free space are redrawn.
- `predict(delta)` → motion update with an odometry step.
- `update(ranges, range_min, range_max)` → weight, estimate, resample, recover. Returns False if skipped because the car hasn't moved.
- `extrapolate(delta)` → current estimate moved by an odometry step using the learned scale (used for the high-rate output).
- Diagnostics: `last_neff`, `resampled`, `last_injected`, `p_inject`, `odom_scale`.

### `odom_buffer.py` — `OdomBuffer`
Last `odom_buffer_sec` seconds of `(timestamp, odom pose)`.
- `add(t, pose)` (out-of-order samples dropped, old ones pruned)
- `latest()`
- `pose_at(t)` → odom pose interpolated at time t, with status `ok` / `after` (newer than all samples) / `before` (older than the buffer) / `empty`.

Lets the filter use the odometry pose *at the scan's timestamp*, not whatever odom message arrived last.

### `pf_node.py` — `ParticleFilterNode` (executable `pf_node`)
ROS 2 wrapper. Reads parameters, converts messages, schedules the filter, publishes results. See *Data flow*.

### Odometry noise (separate package)
Drifting wheel odometry for simulation comes from the shared `odom_noise` package (`code/odom_noise`), so every localization package is tested against the same, repeatable odometry error. `pf_launch.py odom_noise:=true` starts it and points the filter at `/ego_racecar/odom_noisy`. The offline tests also use its `noise_model.py`.

---

## Algorithm

### Idea
Represent "where could the car be?" as N weighted guesses (particles). Odometry moves and spreads them; each LiDAR scan re-weights them by how well the scan fits the map from that pose; resampling concentrates them on the good ones. The weighted mean is the pose estimate.

### Cycle (once per scan, 10 Hz)

**0. Odometry step.** The node finds the odometry pose at the scan timestamp (interpolated from the buffer) and computes `delta = relative_pose(odom_at_last_scan, odom_at_this_scan)`.

**1. Predict** (`motion_model.py`, `filter.predict`). Each particle i gets its own noisy, scaled copy of the step:
```
trans = |(dx, dy)|,  rot = |dθ|
σ_long = α_tt·trans + α_tr·rot + σ_min_xy          (along the car)
σ_lat  = α_lat·(α_tt·trans + α_tr·rot) + σ_min_xy   (sideways; smaller for a car)
σ_rot  = α_rt·trans + α_rr·rot + σ_min_θ

uᵢ = (kᵢ·dx + N(0, σ_long),  kᵢ·dy + N(0, σ_lat),  dθ + N(0, σ_rot))
xᵢ ← xᵢ ⊕ uᵢ
```
`kᵢ` is the particle's odometry scale (see *Odometry scale calibration*). Noise grows with the motion; the σ_min floors keep the cloud from collapsing when odometry is very good. The estimate is also carried forward by `extrapolate(delta)` so it stays current if step 2 is skipped.

**2. Gate.** If the car has moved less than `update_min_d` and turned less than `update_min_a` since the last sensor update, stop here. Re-scoring the same view repeatedly would make the filter overconfident.

**3. Weight** (`sensor_model.py`, likelihood field). Take `num_beams` evenly spaced beams; drop invalid ones (NaN/inf, < range_min, ≥ range_max). For each particle and valid beam k with range r_k and bearing φ_k:
```
endpoint_k = particle ⊕ laser_offset ⊕ (r_k cos φ_k, r_k sin φ_k)
d_k        = distance_field(endpoint_k)                    (lookup, no ray casting)
p_k        = z_hit·exp(−d_k² / 2σ_hit²) + z_rand / range_max
log wᵢ     = effective_beams · mean_k log p_k
```
- Good pose → beam endpoints land on walls → d ≈ 0 → high p.
- `z_rand` term: a beam that hits something not on the map can't zero a particle.
- `effective_beams`: beams are correlated, so the raw product over K beams is far too confident. Instead the scan counts as `effective_beams` independent measurements, regardless of how many beams are valid. `num_beams` sets accuracy/cost; `effective_beams` alone sets confidence (higher = sharper weights, fewer surviving particles).
- Particles whose position is not in free space get log w = −10⁹.
- All N × K endpoints are computed in one vectorised NumPy operation.

New weights = old weights × likelihood (in log space, max-subtracted, normalised; old weights floored at 10⁻³⁰⁰).

**4. Estimate.** Weighted mean of x and y; circular mean of θ (`atan2(Σw sin θ, Σw cos θ)`, so −179° and +179° average to 180°); weighted covariance; `odom_scale` = weighted mean of k.

**5. Resample.** If N_eff < `resample_threshold`·N (or recovery is injecting), low-variance resampling; weights reset to 1/N; per-particle scales follow their particles.

**6. Recover** (see below). Replace a fraction of particles if the scan fit has dropped well below normal.

### Odometry scale calibration (`ScaleConfig`)
Why: along a long, featureless straight the walls look the same at every position, so the LiDAR cannot correct the position *along* the track. A systematic odometry scale error (wheel radius, VESC speed gain, tyre slip) then makes the estimate creep ahead of or behind the car until the next corner snaps it back.

How: each particle carries a factor kᵢ multiplying its odometry translation (rotation unscaled).
- Initialised as `odom_scale + N(0, init_std)` (`odom_scale` starts at 1 and is kept across re-initialisations).
- Random walk `kᵢ += N(0, walk_std_per_m · trans)` each step so it can follow changing grip; clipped to [`min`, `max`].
- At corners, particles with the wrong k end up off the walls and lose weight; resampling keeps the right k. The population converges to the true scale (offline on Spielberg: within ~30 s, to within ~0.005).
- `odom_scale` (logged) = learned correction. Odometry reporting 5% too much distance → `odom_scale ≈ 1/1.05 = 0.952`. On the real car this is a direct readout of the VESC odometry calibration error.

### Recovery — augmented MCL (`RecoveryConfig`)
Why: if the filter loses track (large slip, collision, wrong initial pose), all particles are in the wrong place and nothing brings them back.

How: track the average scan fit with a slow and a fast running average and compare them.
```
l_avg   = log meanᵢ exp(mean_k log p_ik)            per-beam fit, averaged over particles
w_slow ← w_slow + α_slow (w_avg − w_slow)            long-term "normal" fit (~100 s)
w_fast ← w_fast + α_fast (w_avg − w_fast)            recent fit (~1 s)
p_inject = 1 − w_fast / w_slow
```
- If `p_inject ≥ min_inject_ratio`: replace `min(p_inject, max_inject_ratio)·N` random particles with fresh ones — Gaussian around the current estimate (`local_std_xy`, `local_std_theta`) and/or, with share `global_fraction`, uniformly over free space with random heading. Fresh particles get scales drawn around `odom_scale`.
- Averages are kept in log space (no underflow).
- Warm-up: until 1/α updates have been seen, α = 1/count (plain running mean), so the baseline isn't anchored to the first, unconverged scans.
- Particles injected on the previous step are excluded from the next `l_avg` (they are mostly bad guesses; counting them would lower the fit and trigger more injection).
- Threshold choice: on the Spielberg map normal driving reaches p ≈ 0.075; a real loss of track passes 0.12 within ~3 scans. Default 0.12.

---

## Data flow (`pf_node.py`)

```
/map (OccupancyGrid, transient-local) ──► map_cb: build OccupancyMap, sensor model, filter
/initialpose (PoseWithCovarianceStamped) ► initialpose_cb: initialize the cloud
/ego_racecar/odom[_noisy] (Odometry, 100 Hz) ► odom_cb
/scan (LaserScan, 10 Hz) ────────────────► scan_cb
```

**`map_cb`**: builds map + distance field, sensor model and filter. Initialises immediately if an initial pose arrived earlier or `use_initial_pose_param` is true; otherwise waits for `/initialpose`.

**`odom_cb`** (every odom message):
1. Add `(stamp, pose)` to the odom buffer.
2. If a scan is waiting and this odom covers its stamp → process that scan now.
3. Publish `/pf/odom` = filter estimate ⊕ scale-corrected odometry since the last scan (high-rate pose for planning/control).

**`scan_cb`** (every scan):
1. First scans: read the LiDAR mounting offset from TF `base_frame → scan frame` (fallback `laser_offset_fallback` after 20 failed lookups).
2. If the scan is stamped up to `scan_wait_sec` after the newest odom (normal when scan and odom are published by separate timers), hold it until the next odom message arrives (≤ one odom period, 10 ms). Otherwise process immediately.

**`_process_scan`**: odom pose at scan stamp → `delta` → `predict` → `update` → publish `/pf/pose`, particles (every `publish_particles_every` scans), `map → odom` TF (if enabled), stats.

**Stats log** (every 100 sensor updates):
```
[100 updates] time mean / max ms | N_eff/N mean, min | resampled % |
recovery: events, particles | odom scale | scans waited for odom
```
Plus a warning if scans fell outside the odom buffer.

### Topics and frames
| Direction | Topic | Type | Rate | Use |
|---|---|---|---|---|
| in | `/scan` | LaserScan | 10 Hz | sensor update |
| in | `/ego_racecar/odom` (or `_noisy`) | Odometry | 100 Hz | motion |
| in | `/map` | OccupancyGrid | once | map |
| in | `/initialpose` | PoseWithCovarianceStamped | on demand | (re)initialise |
| out | `/pf/pose` | PoseStamped | per scan | filter estimate at the scan time (evaluation) |
| out | `/pf/odom` | Odometry | per odom msg | estimate extrapolated with odometry, with covariance (x, y, yaw) and twist (planning/control) |
| out | `/pf/particles` | PoseArray | every 2nd scan | RViz (≤ `max_viz_particles`) |
| out | TF `map → odom` | | per scan | only if `publish_tf: true`; `T_map_odom = T_map_base ⊕ T_odom_base⁻¹`, both at the scan stamp |

- In simulation `publish_tf` is false: the gym bridge publishes `map → ego_racecar/base_link` itself.
- On the real car `publish_tf` is true, with the VESC providing `odom → base_link`.
- The gym bridge also subscribes to `/initialpose` and teleports the car there. With the default topic, RViz "2D Pose Estimate" moves the car and re-initialises the filter together. Setting `initialpose_topic: /pf/initialpose` separates them: RViz then moves only the car, leaving the filter to find it via recovery (a kidnap test).

---

## Parameters (`config/pf_params.yaml`)

**Topics / frames**
- `scan_topic`, `odom_topic`, `map_topic`, `initialpose_topic`, `pose_topic`, `odom_out_topic`, `particles_topic`
- `map_frame` (map), `odom_frame` (odom), `base_frame` (ego_racecar/base_link)
- `publish_tf` (false in sim), `publish_on_odom` (publish `/pf/odom`)

**Filter**
- `num_particles` 2000
- `resample_threshold` 0.5 — resample when N_eff < 0.5·N
- `update_min_d` 0.02 m, `update_min_a` 0.02 rad — sensor-update gate
- `random_seed` −1 (≥ 0 for repeatable runs)

**Motion model (`motion.*`)**
- `alpha_trans_trans` 0.1, `alpha_trans_rot` 0.05 — along-track noise per m / per rad
- `alpha_lat` 0.3 — sideways noise as a fraction of along-track
- `alpha_rot_trans` 0.1, `alpha_rot_rot` 0.1 — heading noise per m / per rad
- `min_sigma_xy` 0.005 m, `min_sigma_theta` 0.005 rad — floors

**Sensor model (`sensor.*`)**
- `z_hit` 0.9, `z_rand` 0.1, `sigma_hit` 0.1 m
- `effective_beams` 20 — confidence of one scan
- `num_beams` 60 — beams evaluated per scan
- `max_occ_dist` 2.0 m — distance-field clip
- `occupied_threshold` 65, `free_threshold` 25
- `penalize_non_free` true
- `laser_offset_fallback` [0.275, 0, 0] — only if TF is unavailable

**Timing**
- `odom_buffer_sec` 1.0 — odom history kept
- `scan_wait_sec` 0.05 — max wait for odom covering a scan's stamp

**Odometry scale (`odom_scale.*`)**
- `enabled` true, `init_std` 0.05, `walk_std_per_m` 0.002, `min` 0.7, `max` 1.3

**Recovery (`recovery.*`)**
- `enabled` true
- `alpha_slow` 0.001, `alpha_fast` 0.1
- `min_inject_ratio` 0.12, `max_inject_ratio` 0.25
- `local_std_xy` 1.0 m, `local_std_theta` 0.4 rad
- `global_fraction` 0.0

**Initialisation**
- `use_initial_pose_param` false, `initial_pose` [x, y, yaw]
- `initial_std_xy` 0.25 m, `initial_std_theta` 0.1 rad

**Visualisation**
- `publish_particles_every` 2, `max_viz_particles` 500

### How the tuning was chosen
- `effective_beams` / `sigma_hit`: offline sweep with noisy odometry at 5 m/s. Higher confidence lowers error but leaves fewer surviving particles: 5 → 0.18 m mean error / 43% of particles alive, 20 → 0.12 m / 26%, 40 → 0.10 m / 16%. 20 is the middle.
- Reading the log: N_eff/N mean well below ~0.1 → lower `effective_beams`; estimate lagging in corners → raise it.
- `min_inject_ratio`: see *Recovery*.

---

## Build and run

```bash
cd ~/f1tenth_sim_ws
python3 -m colcon build --packages-select particle_filter
source install/setup.bash

ros2 launch particle_filter pf_launch.py                   # ground-truth sim odometry
ros2 launch particle_filter pf_launch.py odom_noise:=true  # drifting odometry (odom_noise node
                                                           # started, filter reads /ego_racecar/odom_noisy)
```

Launch arguments: `params_file`, `use_sim_time` (false), `odom_noise` (false). Noise parameters are in the `odom_noise` package's own config.

Offline tests (from the package root, with `code/odom_noise` next to it): `python3 -m pytest test/ -v`

Then initialise with RViz "2D Pose Estimate", or set `use_initial_pose_param: true` with `initial_pose` = the simulator's start pose (`sx`, `sy`, `stheta` in the gym's `sim.yaml`).

RViz displays: PoseArray `/pf/particles`, Pose `/pf/pose`, Odometry `/pf/odom`.

### Performance
- ~11–12 ms per sensor update on the laptop (2000 particles, 60 beams), vs. 100 ms available at 10 Hz.
- Map load ~0.45 s (Spielberg, 2000×2000).