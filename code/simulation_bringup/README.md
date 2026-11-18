# simulation_bringup

One command per simulation or mapping run: gym, maps, TF, localization, planning, control,
evaluation, bag and logging, started in order from one config file.

## Racing runs: `sim_bringup_launch.py`

```bash
ros2 launch simulation_bringup sim_bringup_launch.py                                # GT pose (default)
ros2 launch simulation_bringup sim_bringup_launch.py localization:=pf
ros2 launch simulation_bringup sim_bringup_launch.py localization:=carto odom_noise:=true
ros2 launch simulation_bringup sim_bringup_launch.py localization:=scan_matching planner:=rrt
ros2 launch simulation_bringup sim_bringup_launch.py localization:=pf tf_source:=localization
ros2 launch simulation_bringup sim_bringup_launch.py config:=gt_gt.yaml planner:=passthrough
```

| Argument | Values | Default (`bringup.yaml`) |
|---|---|---|
| `config` | a file in `config/` or an absolute path | `bringup.yaml` (Carto stack map) |
| `localization` | `gt`, `pf`, `carto`, `scan_matching` | `gt` |
| `odom_noise` | `true`, `false` (localizers only) | `false` |
| `planner` | `passthrough`, `frenet`, `rrt` | `frenet` |
| `tf_source` | `gym` (truth view), `localization` (belief view) | `gym` |
| `tag` | run-folder label | `auto` = `<config>_<localization>[_noise]_<planner>` |

Maps and racelines are chosen in the config file: `world.map` is what the simulated LiDAR
sees (the truth), `stack.map` / `stack.frame` / `stack.tf` / `stack.raceline` are what the
stack believes. For another raceline or map, copy `bringup.yaml` and edit those lines.

### Localizers

| `localization` | Nodes started | Pose for planner + controller | Ready when |
|---|---|---|---|
| `gt` | none | `/ego_racecar/odom` (perfect) | 2 s after the gym |
| `pf` | `particle_filter` on the stack map | `/pf/odom` | PF logs `initialized at` |
| `carto` | `cartographer_node` (frozen `.pbstream`) + `carto_node` + relay | `/loc/odom` | relay logs `first estimate` |
| `scan_matching` | `scan_matching` (map-free, drifts) + relay | `/loc/odom` | relay logs `first estimate` |

`carto` needs the Cartographer sidecar (yaml with `pbstream:`) next to the stack map; it is
found automatically, or set `stack.carto_state`. With `odom_noise: true` one `odom_noise` node
runs and every odometry consumer (localizer, relay, TF broadcaster) reads its output.

`localization_relay` turns Cartographer's / scan matching's per-scan estimate into Odometry
at odometry rate (estimate at the scan + odometry since then, twist from odometry), the same
thing the PF does for `/pf/odom`.

### Two views (`tf_source`)

The car always drives in the true world and the stack always decides from its estimate;
this only changes what RViz shows and who owns `base_link`.

| `tf_source` | TF tree | RViz | Shows |
|---|---|---|---|
| `gym` | gym: `map -> base_link` = truth | `sim.rviz` (car model) | what really happens |
| `localization` | `map -> carto_map -> odom -> base_link` (REP 105, as on the car) | `sim_belief.rviz` (axes + white GT arrow) | what the stack believes |

Needs a localizer; with `gt` it falls back to `gym`.

## Mapping runs: `mapping_bringup_launch.py`

```bash
ros2 launch simulation_bringup mapping_bringup_launch.py                              # FTG, 3 m/s
ros2 launch simulation_bringup mapping_bringup_launch.py controller:=pid max_speed:=2.5
ros2 launch simulation_bringup mapping_bringup_launch.py odom_noise:=true out_dir:=Spielberg/carto_noisy
```

Arguments: `controller` (`ftg` | `pid`), `max_speed` (m/s; the controller's min speed is
clamped to it), `odom_noise`, `out_dir` (relative to `data_root`, default
`Spielberg/carto_new` so the maps you use are never overwritten), `map_name`, `tag`.
Cartographer starts first; the driver starts once `carto_node` is up. When the map looks
complete, run the two save commands the terminal prints (pbstream + sidecar, then the image).
Then extract the track and optimise racelines as before.

## Logs: `~/f1tenth_simulation_logs/<YYYYmmdd_HHMMSS>_<tag>/`

```
bringup_resolved.yaml   exact config + derived values of this run
sim_resolved.yaml       gym parameters this run used
summary.csv             per lap: time, collisions, local-planner mode %, compute time
<raceline>_<tag>_*.csv  lap_logger (GT pose vs global raceline)
bag/                    rosbag (run.bag_topics; default list depends on the localizer)
ros_logs/               one log file per node process (all severities)
launch_log ->           raw stdout/stderr of every process
```

The terminal shows only the bringup's stage lines and `run_monitor` (one line per lap,
collisions). Live view of everything: `ros2 run rqt_console rqt_console`.

## Layout

```
config/bringup.yaml, gt_gt.yaml   racing scenarios
config/mapping.yaml               mapping run
config/params/                    every node's parameters (edit here)
launch/                           sim_bringup_launch.py, mapping_bringup_launch.py
rviz/                             sim.rviz (truth), sim_belief.rviz (belief), mapping.rviz
simulation_bringup/
  config_resolver.py              config -> derived values (no ROS; testable offline)
  launch_helpers.py               shared launch building blocks
  run_monitor.py                  terminal lap lines + summary.csv
  localization_relay.py           per-scan estimate -> Odometry at odom rate (+ map -> odom)
  odom_tf_broadcaster.py          odom -> base_link TF from odometry (belief view)
```

## Build

```bash
python3 -m colcon build --symlink-install --packages-select f1tenth_gym_ros simulation_bringup
```

## Requires (in f1tenth_gym_ros): `gym_bridge.patch`

1. `rviz_config` launch argument in `gym_bridge_launch.py` (else the gym's default RViz opens)
2. `publish_pose_tf` parameter in `gym_bridge.py` (for `tf_source: localization`)
