# pure_pursuit

Path-tracking controller: follows a `race_msgs/WaypointArray` path (the global raceline,
or the local planner's 5 m segment) and outputs steering and speed. Pure pursuit with a
speed-scaled lookahead and an understeer correction.

- **Build type:** `ament_python`
- **Node:** `pure_pursuit` (executable `pure_pursuit_node`)
- **Depends on:** `race_msgs`, `nav_msgs`, `ackermann_msgs`, `visualization_msgs`, `tf2_ros`, `numpy`

## Interface

| Direction | Topic | Type | Notes |
|---|---|---|---|
| subscribes | `waypoints_topic` (`/global_waypoints`) | `race_msgs/WaypointArray` | reliable, transient local |
| subscribes | `pose_topic` (`/ego_racecar/odom`) | `nav_msgs/Odometry` | pose + speed (`twist.linear.x`) |
| publishes | `drive_topic` (`/drive`) | `ackermann_msgs/AckermannDriveStamped` | one command per pose message |
| publishes | `/pp/lookahead` | `visualization_msgs/Marker` | red sphere at the target point |

If the pose and the path are in different frames (e.g. pose in `map`, raceline in
`carto_map`), the pose is transformed into the path's frame through TF.

## How it works

On every pose message:

1. Move the pose back to the rear axle (`rear_axle_offset`).
2. Lookahead distance `L = clip(lookahead_base + lookahead_gain · v, lookahead_min, lookahead_max)`.
3. Find the nearest path point (windowed search around the last one, `search_back` /
   `search_ahead`; a full search on start-up or if the car is more than
   `relocalise_dist` from the window).
4. Target point = the point `L` metres further along the path (arc length).
5. Steering:

```
κ = 2 · l_y / l_d²                       (target in the car frame: l_x, l_y; l_d² = l_x² + l_y²)
δ = atan(wheelbase · κ) + K · v² · κ     (kinematic pure pursuit + understeer correction)
```

   `K` is the understeer gradient [rad per m/s²]; `K = 0` gives plain pure pursuit.
6. Speed: `speed_mode: path` uses the path's `vx_mps` at the nearest point × `speed_scale`;
   `constant` uses `constant_speed`. Both are capped at `max_speed`.

The path is treated as a closed loop (interpolation wraps from the last point to the
first).

## Parameters

Node defaults, with the simulation values from `config/pp_sim.yaml` where they differ:

| Name | Default | `pp_sim.yaml` | Meaning |
|---|---|---|---|
| `pose_topic` | `/ego_racecar/odom` | | `nav_msgs/Odometry` |
| `waypoints_topic` | `/global_waypoints` | | path to follow |
| `drive_topic` | `/drive` | | |
| `wheelbase` | `0.3302` | | [m] (f1tenth_gym) |
| `max_steer` | `0.4189` | | [rad] |
| `rear_axle_offset` | `0.0` | | [m] from the pose origin back to the rear axle |
| `understeer_gradient` | `0.0` | `0.0028` | K = (1/C_Sf − 1/C_Sr) / (μ g) with the gym's tyre stiffnesses |
| `lookahead_base` | `0.5` | | L0 [m] |
| `lookahead_gain` | `0.25` | `0.10` | k [s] |
| `lookahead_min` / `lookahead_max` | `0.5` / `2.5` | | [m] |
| `speed_mode` | `constant` | `path` | `constant` or `path` |
| `constant_speed` | `2.0` | `4.0` | [m/s] |
| `speed_scale` | `1.0` | | multiplies the path speed |
| `max_speed` | `5.0` | | [m/s] |
| `search_back` / `search_ahead` | `10` / `60` | | nearest-point window [points] |
| `relocalise_dist` | `2.0` | | [m] |

With `pp_sim.yaml` the lookahead at 5 m/s is 0.5 + 0.10 · 5 = 1.0 m.

## Run

On the global raceline directly (raceline publisher + controller + lap logger):

```bash
ros2 launch pure_pursuit pp_sim_launch.py csv_path:=<raceline.csv> tag:=<label>   # log:=false to skip the logger
```

In the full stack, `simulation_bringup` starts it with `waypoints_topic: /local_waypoints`
and the pose of the chosen localizer.
