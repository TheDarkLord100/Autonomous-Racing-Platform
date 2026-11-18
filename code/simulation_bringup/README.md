# simulation_bringup

One command for a full simulation run: gym, maps, TF, localization, planning, control,
evaluation, bag recording and logging, started in the right order from one scenario file.

```bash
ros2 launch simulation_bringup sim_bringup_launch.py                          # config/bringup.yaml
ros2 launch simulation_bringup sim_bringup_launch.py config:=gt_gt.yaml
ros2 launch simulation_bringup sim_bringup_launch.py planner:=rrt tag:=carto_pf_rrt
```

Launch arguments: `config` (a file in `config/`, or an absolute path), and quick overrides
`planner` (passthrough | frenet | rrt), `localization` (gt | pf), `tag`.

## Layout

```
config/
  bringup.yaml          scenario: Carto map + particle filter + Frenet (default)
  gt_gt.yaml            scenario: GT map + GT pose (baseline)
  params/               every node's parameters; edit here
    sim.yaml              gym bridge (spawn pose, LiDAR, rates)
    pf_params.yaml        particle filter
    odom_noise_params.yaml
    local_planner.yaml
    pure_pursuit.yaml
launch/sim_bringup_launch.py
rviz/sim.rviz           RViz layout (world map, stack map, raceline, local planner, PF pose, ...)
simulation_bringup/
  config_resolver.py    scenario -> derived values (no ROS; testable offline)
  run_monitor.py        the only node printing to the terminal: one line per lap
```

## Scenario file

`world` is what the simulated LiDAR sees (the truth); `stack` is what the stack believes
(the map it localizes on, its frame and the raceline). Paths are relative to `data_root`.

The bringup derives these so they can never disagree with each other (they override
the params files):

| Derived | From |
|---|---|
| pose for local planner + pure pursuit (`/pf/odom` or `/ego_racecar/odom`) | `localization.method` |
| PF odom input (`/ego_racecar/odom_noisy` with noise) | `localization.odom_noise` |
| PF map topic, map frame, **initial pose** | `stack.map`, `stack.frame`, `stack.tf`, spawn in `sim.yaml` |
| second map server for the stack map (on `/<frame>`) | `stack.map` differs from `world.map` |
| static TF `map -> <frame>` | `stack.frame` is not `map` |
| raceline frame | `stack.frame` |

## Startup order

1. Run folder + logging, static TF, gym (bridge, world map, robot model, RViz), stack map
   server + lifecycle manager
2. Particle filter (+ odom_noise), after the stack map server logs `Managed nodes are active`
3. Raceline publisher, local planner, pure pursuit, lap logger, run monitor, bag, after the
   PF logs `initialized at` (with `method: gt`: 2 s after stage 1)

A stage still waiting after `run.stage_timeout` prints a yellow warning naming the node to check.

## Logs: `~/f1tenth_simulation_logs/<YYYYmmdd_HHMMSS>_<tag>/`

```
bringup_resolved.yaml    the exact scenario + derived values of this run
summary.csv              per lap: time, collisions, local-planner mode %, compute time
<raceline>_<tag>_<time>.csv   lap_logger output (GT pose vs global raceline)
bag/                     rosbag (topic list: run.bag_topics)
ros_logs/                one log file per node process (all severities)
launch_log ->            raw stdout/stderr of every process (launch log folder)
```

Live view with filtering by node / severity: `ros2 run rqt_console rqt_console`.
Follow one node: `tail -f ~/f1tenth_simulation_logs/<run>/ros_logs/pf_node_*.log`.

## Build

```bash
python3 -m colcon build --symlink-install --packages-select simulation_bringup
```

`--symlink-install` makes edits to `config/` and `rviz/` take effect without rebuilding.

## Requires (in f1tenth_gym_ros)

A `rviz_config` launch argument in `gym_bridge_launch.py` (see `gym_bridge_rviz_config.patch`).
Without it, the gym opens its own default RViz layout instead of `rviz/sim.rviz`.
