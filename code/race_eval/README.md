# race_eval

Evaluation tools: a node that logs how the car tracks the raceline, and a script that
plots those logs.

- **Build type:** `ament_python`
- **Node:** `lap_logger` (executable `lap_logger`)
- **Script:** `scripts/plot_lap_log.py`
- **Depends on:** `race_msgs`, `nav_msgs`, `ackermann_msgs`, `tf2_ros`, `numpy`
  (script: `matplotlib`)

## `lap_logger`

| Direction | Topic | Type |
|---|---|---|
| subscribes | `waypoints_topic` (`/global_waypoints`) | `race_msgs/WaypointArray` (latched) |
| subscribes | `pose_topic` (`/ego_racecar/odom`) | `nav_msgs/Odometry` |
| subscribes | `drive_topic` (`/drive`) | `ackermann_msgs/AckermannDriveStamped` |

Writes `<log_dir>/<source>[_<tag>]_<YYYYmmdd_HHMMSS>.csv`, one row per pose message
(100 Hz in simulation):

| Column | Meaning |
|---|---|
| `t` | time [s] |
| `lap` | lap counter (0 = out-lap from the spawn to the start line) |
| `s` | arc length along the raceline [m] |
| `x`, `y` | pose in the raceline frame |
| `e_lat` | lateral error from the raceline [m], + = left |
| `v_plan`, `v_act`, `v_cmd` | planned, actual and commanded speed [m/s] |
| `steer_cmd` | commanded steering [rad] |
| `kappa_plan`, `kappa_act` | raceline curvature, actual curvature (yaw rate / v) |
| `yaw_rate` | [rad/s] |
| `gap_left`, `gap_right` | estimated car side to wall [m], from the raceline widths and `e_lat` |

It also prints a per-lap summary (lap time, max/RMS lateral error and where, speed vs plan,
min wall gap), warns on estimated wall contact, and reports when the car stops.

The wall gap is computed from the raceline's `w_left_m` / `w_right_m` and `e_lat`.

Parameters: `pose_topic`, `waypoints_topic`, `drive_topic`, `log_dir` (`~/race_logs`),
`tag`, `search_back` / `search_ahead` (10 / 60 points), `relocalise_dist` (2 m),
`stop_speed` (0.2 m/s) and `stop_time` (1 s) for stop detection, `half_width` (0.155 m).

```bash
ros2 run race_eval lap_logger --ros-args -p log_dir:=$HOME/race_logs -p tag:=test
```

`simulation_bringup` starts it automatically and writes into the run folder.

## `scripts/plot_lap_log.py`

Overlays one or more lap-logger CSVs against arc length: lateral error, speed (plan /
actual / command), curvature (plan vs actual), steering, wall gaps.

```bash
python3 plot_lap_log.py run_a.csv run_b.csv            # overlay two runs
python3 plot_lap_log.py run_a.csv --lap 2              # one lap
python3 plot_lap_log.py run_a.csv --s-range 95 125     # zoom on one corner
python3 plot_lap_log.py run_a.csv --save fig.png
```
