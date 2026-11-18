# global_planner

Publishes the global raceline. The raceline itself is computed **offline** by
`code/planning_scripts/optimise_raceline.py` and saved as a CSV; this package only loads
that CSV and publishes it once, latched (nodes that start later still receive it).

- **Build type:** `ament_python`
- **Node:** `raceline_publisher` (executable `raceline_publisher`)
- **Depends on:** `race_msgs`, `nav_msgs`, `geometry_msgs`, `numpy`

## Interface

| Direction | Topic | Type | QoS |
|---|---|---|---|
| publishes | `/global_waypoints` | `race_msgs/WaypointArray` | reliable, transient local, depth 1 |
| publishes | `/global_path` | `nav_msgs/Path` (for RViz) | reliable, transient local, depth 1 |

Both are published once at start-up.

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `csv_path` | `''` (required) | raceline CSV to load |
| `frame_id` | `map` | frame the CSV coordinates are in (e.g. `carto_map` for a Cartographer-map raceline) |
| `waypoints_topic` | `/global_waypoints` | |
| `path_topic` | `/global_path` | |

## Input CSV format

Nine comma-separated columns, `#` comment lines allowed, one row per point in driving
order (the loop is closed implicitly from the last row back to the first):

```
s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, w_left_m, w_right_m
```

This is exactly what `optimise_raceline.py` writes (see `code/planning_scripts/README.md`).
The node refuses a file with a different column count.

## Run

```bash
ros2 run global_planner raceline_publisher --ros-args \
  -p csv_path:=<repo>/data/Spielberg/GT/raceline/Spielberg_GT_mincurv_iter.csv
```

The log line `published N waypoints from ... (length L m, v a-b m/s)` confirms the load.
In normal use this node is started by `simulation_bringup` (the raceline comes from
`stack.raceline` in the scenario file).
