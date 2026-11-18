# local_planner

Scan-aware local planner. Every scan, this node checks the next 5 m of the global
raceline against the live LiDAR scan. If the raceline keeps the required clearance from
every scan point, it is published unchanged; otherwise a local path that keeps the
clearance and rejoins the raceline is planned and published instead.

- **Build type:** `ament_python`
- **Node:** `local_planner` (executable `local_planner_node`)
- **Planners:** `passthrough`, `frenet` (Frenet-frame sampling), `rrt` (RRT* + smoothing)
- **Depends on:** `race_msgs`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`, `tf2_ros`,
  `numpy`, `scipy`

## Interface

| Direction | Topic | Type | Notes |
|---|---|---|---|
| subscribes | `waypoints_topic` (`/global_waypoints`) | `race_msgs/WaypointArray` | latched raceline |
| subscribes | `pose_topic` (`/ego_racecar/odom`) | `nav_msgs/Odometry` | GT, `/pf/odom` or `/loc/odom` |
| subscribes | `scan_topic` (`/scan`) | `sensor_msgs/LaserScan` | one planning cycle per scan |
| publishes | `local_topic` (`/local_waypoints`) | `race_msgs/WaypointArray` | reliable, transient local (matches `pure_pursuit`) |
| publishes | `/local_planner/mode` | `std_msgs/String` | `pass`, `frenet`, `rrt`, `rrt_raw`, `fallback` |
| publishes | `/local_planner/compute_ms` | `std_msgs/Float32` | time per cycle |
| publishes | `/local_planner/markers` | `visualization_msgs/MarkerArray` | corridor, local path, candidates / tree |

TF: the laser offset is read once from `base_frame → <scan frame>`. A pose in a different
frame from the raceline is transformed through TF.

The output path is in the raceline's frame, starts at the car with `s_m = 0` and never
wraps across the start line, so `pure_pursuit` tracks it unchanged.

## How it works

Every scan:

1. **Reference.** Nearest raceline point (windowed search, so the index cannot jump across
   the track at a hairpin), then the next `horizon` metres (5 m, about 10 car lengths), moved
   into the car frame (`base_link`) with the current pose.
2. **Obstacles.** Scan points in `base_link`, in a KD-tree. Collision checks use only the
   scan; the static map is not used.
3. **Gate.** Every reference point must keep `car_half_width + clearance_margin` from
   every scan point.
4. **Decision.**
   - gate free **and** car within `rejoin_tol` of the raceline **and** no plan in the last
     `hold_cycles` scans → **pass**: publish the raceline segment as is.
   - otherwise → run the selected planner.
   - planner finds nothing → **fallback**: the safest candidate (Frenet) or the raceline,
     at `fallback_speed`.
5. **Speed** (planned paths): raceline speed, capped by `√(μ·g·grip / |κ|)` on the new path,
   then a backward braking pass with `a_brake`.

### Frenet (`frenet`)

Path-only Frenet planner on the raceline (d = 0 means "on the racing line"):

- Start state from the car: lateral offset d₀ and slope d₀' = tan(heading error).
- Candidates: quintic d(s) to end offsets d_f (every `d_step`) reached at `end_stations`
  (2, 3, 4, 5 m), then held to the horizon.
- Sampling range across the track: the outermost scan return on each side (the walls),
  with map widths only where the scan has no return. Obstacles between the walls don't
  shrink the range; the clearance check rejects paths through them.
- Hard checks: clearance from the scan, and |κ| ≤ tan(max_steer) / wheelbase.
- Cost: `w_offset·d_f² + w_smooth·∫d''² + w_consist·(d_f − previous)² + w_prox·Σ max(0, prox_dist − clearance)²`.

### RRT* + smoothing (`rrt`)

- Samples in Frenet coordinates inside the scan-derived bounds (samples land on the track).
- Goal: the point at s = horizon closest to the raceline that has clearance; goal-biased sampling.
- Edges are straight and collision-checked every `edge_res`; an edge may turn at most
  `κ_max · length · turn_slack` from its parent edge.
- RRT* parent choice and rewiring within `neighbour_radius`; stops at `max_iters` or
  `time_budget`.
- Smoothing: greedy shortcut → cubic spline (start tangent = car heading) → resample;
  clearance and curvature re-checked. If that fails, the un-shortcut path is smoothed
  instead; if that fails too, the raw collision-free path is used (`rrt_raw`).

### Near-car relaxation

Within `near_radius` of the car the required clearance drops to
`car_half_width + near_margin` (no contact).

## Parameters (`config/local_planner_sim.yaml`)

| Name | Value | Meaning |
|---|---|---|
| `planner` | `frenet` | `passthrough` / `frenet` / `rrt` |
| `horizon` | `5.0` | planning horizon [m] |
| `car_half_width` | `0.155` | gym car width 0.31 m |
| `clearance_margin` | `0.10` | hard clearance beyond the half width [m] |
| `near_radius` / `near_margin` | `0.5` / `0.02` | near-car relaxation [m] |
| `rejoin_tol` | `0.15` | off the raceline by more than this → plan a rejoin [m] |
| `hold_cycles` | `3` | keep planning this many scans after the gate frees |
| `wheelbase` / `max_steer` | `0.3302` / `0.4189` | curvature limit |
| `mu` / `grip` / `a_brake` | `1.0489` / `0.7` / `5.0` | speed cap on planned paths |
| `max_speed` / `fallback_speed` | `5.0` / `1.0` | [m/s] |
| `debug_markers` | `true` | Frenet candidates / RRT tree in RViz |
| `frenet.*` | see file | `ds`, `d_step`, `end_stations`, cost weights `w_*`, `prox_dist` |
| `rrt.*` | see file | `max_iters`, `time_budget`, `max_step`, `neighbour_radius`, `goal_bias`, `goal_tol`, `edge_res`, `turn_slack`, `seed` |

`clearance_margin` (0.10 m) is set below the raceline optimiser's `--margin` (0.15 m,
`code/planning_scripts`).

## Run

```bash
ros2 launch local_planner local_planner_sim_launch.py \
  csv_path:=<raceline.csv> planner:=frenet frame_id:=map pose_topic:=/ego_racecar/odom tag:=<label>
```

This starts the raceline publisher, the local planner, `pure_pursuit` (tracking
`/local_waypoints`) and the lap logger. For full scenarios (maps, localizers, logging) use
`simulation_bringup` instead.

## RViz

`/local_planner/markers` is published in `base_link`:

- `corridor`: the raceline segment ahead, green where clear, red where blocked
- `local_path`: the path sent to the controller (blue; hidden in `pass` mode; orange in `fallback`)
- `debug`: feasible Frenet candidates or the RRT tree (grey)

The markers are in `base_link`, so RViz draws them at the pose TF gives `base_link`: the
simulator's ground-truth pose by default, or the localizer's estimate with
`simulation_bringup tf_source:=localization`.

## Files

| File | Purpose |
|---|---|
| `corridor.py` | raceline container, frame transforms, scan → obstacles, reference path, lateral bounds, gate, speed profile (no ROS) |
| `frenet.py` | Frenet planner (no ROS) |
| `rrt.py` | RRT* + smoothing (no ROS) |
| `local_planner_node.py` | ROS node: inputs, gate decision, outputs, markers |
