# code

All project software: ROS 2 packages (Python, `ament_python`; messages in `ament_cmake`)
and two folders of offline scripts. Each package has its own README with its interface,
parameters and usage.

The repository is cloned into the workspace's `src/` folder; `colcon` finds the packages in
the subfolders automatically. Setup: [docs/01_setup.md](../docs/01_setup.md).

## Packages, in pipeline order

| Package / folder | Type | Purpose |
|---|---|---|
| [`race_msgs`](race_msgs) | messages | `Waypoint`, `WaypointArray`: a raceline with speed profile |
| [`reactive/gap_follow`](reactive/gap_follow) | node | Follow-the-Gap controller (LiDAR only); drives mapping runs |
| [`reactive/pid_wall`](reactive/pid_wall) | node | PID wall-following controller (LiDAR only); drives mapping runs |
| [`odom_noise`](odom_noise) | node | drifting, repeatable wheel odometry from the simulator's perfect odometry |
| [`localization/cartographer_slam`](localization/cartographer_slam) | nodes + config | Cartographer mapping and localization (stock Cartographer, this car's config) |
| [`mapping_scripts`](mapping_scripts) | offline scripts | map image → centreline + wall widths |
| [`planning_scripts`](planning_scripts) | offline script | centreline → optimised raceline CSV (QP, OSQP) |
| [`global_planner`](global_planner) | node | publishes a raceline CSV as `/global_waypoints` |
| [`localization/particle_filter`](localization/particle_filter) | node | Monte Carlo localization on an occupancy grid |
| [`localization/scan_matching`](localization/scan_matching) | node | PL-ICP frame-to-frame scan matching (map-free) |
| [`local_planner`](local_planner) | node | checks the raceline against the live scan; Frenet / RRT* around problems |
| [`pure_pursuit`](pure_pursuit) | node | path-tracking controller |
| [`race_eval`](race_eval) | node + scripts | lap logger, lap-log plots |
| [`simulation_bringup`](simulation_bringup) | launch + configs + helper nodes | one-command simulation and mapping runs, logging, RViz layouts |

## Dependencies between packages

```
race_msgs ── global_planner ─┐
         ├── local_planner ──┤
         ├── pure_pursuit ───┼── simulation_bringup (starts everything)
         └── race_eval ──────┤
odom_noise ── particle_filter, scan_matching, cartographer_slam ─┤
gap_follow, pid_wall ────────────────────────────────────────────┘
```

## Conventions

- Algorithm code has no ROS imports (e.g. `particle_filter/filter.py`,
  `local_planner/frenet.py`); the node file only converts messages and calls it. The
  algorithms can then be tested offline against simulator ground truth (those tests are
  kept outside this repository).
- Every node's parameters are in its package's `config/*.yaml`. For `simulation_bringup`
  runs, its own copies in `simulation_bringup/config/params/` are used instead.
- Build from the workspace root with `python3 -m colcon build --symlink-install` (venv active).
