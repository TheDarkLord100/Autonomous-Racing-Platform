# 10. Evaluation tools

What each run records, and the tools that read it. Package reference:
[`code/race_eval/README.md`](../code/race_eval/README.md).

## 10.1 Run folder

`simulation_bringup` writes each run to `~/f1tenth_simulation_logs/<YYYYmmdd_HHMMSS>_<tag>/`:

| File | Written by | Content |
|---|---|---|
| `summary.csv` | `run_monitor` | per lap: lap time, collisions, % of scans in each local-planner mode, local-planner compute time (mean, p95, max) |
| `<raceline>_<tag>_<time>.csv` | `lap_logger` | one row per ground-truth odometry message (see 10.2) |
| `bag/` | `ros2 bag record` | `run.bag_topics` (default: GT and estimated pose, scan, drive, waypoints, local-planner mode and timing, collision, lap topics, TF) |
| `bringup_resolved.yaml`, `sim_resolved.yaml` | bringup | the resolved scenario and simulator parameters |
| `ros_logs/`, `launch_log` | ROS logging | per-node log files, raw process output |

## 10.2 Lap logger columns

`lap_logger` subscribes to the ground-truth odometry, the global raceline and `/drive`:

| Column | Meaning |
|---|---|
| `t`, `lap`, `s` | time, lap counter (0 = out-lap), arc length along the raceline |
| `x`, `y` | pose in the raceline frame |
| `e_lat` | lateral distance from the raceline, + = left |
| `v_plan`, `v_act`, `v_cmd` | planned, actual and commanded speed |
| `steer_cmd` | commanded steering |
| `kappa_plan`, `kappa_act`, `yaw_rate` | raceline curvature, yaw rate / speed, yaw rate |
| `gap_left`, `gap_right` | car side to wall from the raceline widths and `e_lat` |

It also prints a per-lap summary (lap time, max / RMS lateral error and position, speed vs
plan, minimum wall gap).

## 10.3 Plotting

```bash
python3 code/race_eval/scripts/plot_lap_log.py <run1.csv> [<run2.csv> ...] [--lap N] [--s-range S0 S1] [--save fig.png]
```

Plots, against arc length: lateral error, speed (plan / actual / command), curvature (plan vs
actual), steering command, wall gaps. Several logs are overlaid.

## 10.4 Topics for live inspection

| Topic | Content |
|---|---|
| `/ego_racecar/lap_count`, `/ego_racecar/lap_time` | simulator lap counter and last lap time |
| `/ego_racecar/collision` | simulator collision flag |
| `/local_planner/mode`, `/local_planner/compute_ms` | local-planner mode and time per scan |
