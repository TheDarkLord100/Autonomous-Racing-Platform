# cartographer_slam

Cartographer for the F1TENTH stack, in two modes:

- **mapping**: build a map of the track (pose graph `.pbstream` + occupancy grid);
- **localization**: localize against that frozen map while racing.

This package contains **no SLAM code**. It runs stock `cartographer_ros` with this car's
configuration, and adds a small node (`carto_node`) that handles frames, the initial pose
and map saving.

- **Build type:** `ament_python`
- **Node:** `carto_node` (executable `carto_node`), plus `cartographer_ros`'s
  `cartographer_node` and `cartographer_occupancy_grid_node`
- **Depends on:** `cartographer_ros`, `cartographer_ros_msgs`, `std_srvs`, `tf2_ros`,
  `odom_noise` (optional)

## Frames

Cartographer works in its own frame `carto_map`, whose origin is the car's pose when
mapping started. `carto_node` publishes the static
transform `map → carto_map` (`origin`) and converts every pose into `map`.

Cartographer does **not** publish TF (`publish_to_tf = false`); the pose is published on
topics only.

## Interface (`carto_node`)

| Direction | Topic / service | Type | Notes |
|---|---|---|---|
| subscribes | `/tracked_pose` | `geometry_msgs/PoseStamped` | from `cartographer_node`, in `carto_map` |
| subscribes | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | localization: restart at this pose (frame `map`) |
| publishes | `/carto/pose` | `geometry_msgs/PoseStamped` | pose in `map`, one per scan |
| publishes | `/carto/path` | `nav_msgs/Path` | trail of `/carto/pose` |
| publishes | `/carto/map` | `nav_msgs/OccupancyGrid` | from `cartographer_occupancy_grid_node` (remapped so the simulator's `/map` stays) |
| service | `/carto/save_map` | `std_srvs/Trigger` | mapping: finish the trajectory, optimise, write `<map_dir>/<map_name>.pbstream` + `<map_name>.yaml` |

The saved sidecar `<map_name>.yaml` holds `pbstream: <file>` and `origin: [x, y, yaw]`;
localization and the track-extraction scripts read it.

## Mapping

```bash
ros2 launch cartographer_slam carto_mapping_launch.py map_name:=Spielberg \
  origin_x:=<sx> origin_y:=<sy> origin_yaw:=<stheta>        # = spawn pose in the simulator's sim.yaml
# args: params_file, use_sim_time, odom_noise (false), map_name, map_dir (repo data/), origin_*
```

Drive the car around the track (e.g. with `gap_follow`), then save:

```bash
ros2 service call /carto/save_map std_srvs/srv/Trigger                          # pbstream + sidecar
ros2 run nav2_map_server map_saver_cli -t /carto/map -f <map_dir>/<map_name>_map  # image for planning
```

Mapping uses ground-truth odometry by default (`odom_noise:=false`). `simulation_bringup`'s `mapping_bringup_launch.py` does all of this
(gym, Cartographer, a reactive driver) in one command.

## Localization

```bash
ros2 launch cartographer_slam carto_localization_launch.py map_name:=Spielberg map_dir:=<folder with Spielberg.yaml>
# args: params_file, use_sim_time, odom_noise (true), map_name, map_dir
```

Loads the frozen `.pbstream` from the sidecar, then waits for an initial pose: RViz
"2D Pose Estimate", or `use_initial_pose_param: true` with `initial_pose` (in `map`). The
pose is published once per scan on `/carto/pose`; `simulation_bringup` turns it into a
100 Hz `nav_msgs/Odometry` for the planner (`localization_relay`).

## Configuration

`config/carto_mapping.lua`: stock Cartographer 2D with only what this car needs:
`map_frame = carto_map`, `tracking_frame = published_frame = ego_racecar/base_link`,
`provide_odom_frame = false`, `publish_to_tf = false`, `publish_tracked_pose = true`,
`use_pose_extrapolator = false` (tracked pose = scan-matched pose at scan time),
`use_odometry = true`, one laser scan, no IMU, ranges 0.1–10 m (the real RPLiDAR C1 reaches 12 m).

`config/carto_localization.lua` includes the mapping config and overrides:

| Option | Value |
|---|---|
| `pure_localization_trimmer.max_submaps_to_keep` | 3 |
| `POSE_GRAPH.optimize_every_n_nodes` | 20 |
| `use_online_correlative_scan_matching` | true (±20° search around the odometry guess) |
| `ceres_scan_matcher.rotation_weight` | 4 (stock 40) |
| `optimization_problem.odometry_translation_weight` / `_rotation_weight` | 0 (stock 1e5) |
| `fast_correlative_scan_matcher.linear_search_window` | 7 m (mapping config: 2 m) |

`config/carto_params.yaml` (`carto_node`): topics, `map_frame` / `carto_map_frame`,
`origin`, `use_initial_pose_param`, `initial_pose`, service timeouts.

## Files

| File | Purpose |
|---|---|
| `cartographer_slam/carto_node.py` | frames, pose conversion, initial pose, map saving |
| `launch/carto_mapping_launch.py` | mapping mode (+ `odom_noise`) |
| `launch/carto_localization_launch.py` | localization mode on a saved map (+ `odom_noise`) |
| `config/*.lua`, `config/carto_params.yaml` | configuration |
