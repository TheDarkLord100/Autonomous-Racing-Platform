# race_msgs

Message definitions for the racing stack. One message type carries a raceline (or any
path with a speed profile) between the global planner, the local planner, the controller
and the evaluation tools.

- **Build type:** `ament_cmake` (rosidl message generation)
- **Used by:** `global_planner`, `local_planner`, `pure_pursuit`, `race_eval`

## Messages

### `race_msgs/Waypoint`

One point of a raceline. Units are in the field names.

| Field | Type | Meaning |
|---|---|---|
| `s_m` | float64 | arc length from the start line [m] |
| `x_m`, `y_m` | float64 | position in the frame of the enclosing `WaypointArray` [m] |
| `psi_rad` | float64 | heading [rad], ROS convention: 0 along +x, counter-clockwise positive |
| `kappa_radpm` | float64 | curvature [rad/m], positive = turning left |
| `vx_mps` | float64 | target speed [m/s] |
| `ax_mps2` | float64 | target longitudinal acceleration [m/s²] |
| `w_left_m`, `w_right_m` | float64 | distance to the left / right wall along the normal [m] |

### `race_msgs/WaypointArray`

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | `frame_id` = frame of every waypoint (e.g. `map`, `carto_map`) |
| `source` | string | where it came from, e.g. `Spielberg_carto_mincurv_iter` (the CSV file stem) |
| `waypoints` | `Waypoint[]` | ordered in the driving direction |

The fields match the columns of the raceline CSVs written by
`code/planning_scripts/optimise_raceline.py`, so a CSV row maps 1:1 onto a `Waypoint`.

## Topics that use it

| Topic | Publisher | QoS | Content |
|---|---|---|---|
| `/global_waypoints` | `global_planner/raceline_publisher` | reliable, transient local, depth 1 | full closed raceline |
| `/local_waypoints` | `local_planner/local_planner_node` | reliable, transient local, depth 1 | 5 m segment ahead of the car, `s_m` from 0 |

Subscribers must use the same transient-local QoS; a volatile subscriber never receives
the latched raceline.

## Build

```bash
cd ~/f1tenth_sim_ws
python3 -m colcon build --packages-select race_msgs
source install/setup.bash
ros2 interface show race_msgs/msg/WaypointArray
```
