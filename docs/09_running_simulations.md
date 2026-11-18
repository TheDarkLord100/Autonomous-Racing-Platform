# 9. Running simulations

`simulation_bringup` starts a complete run (simulator, maps, TF, localization, planning,
control, evaluation, rosbag) from one scenario file, in the right order, and writes
everything to a run folder. Package reference:
[`code/simulation_bringup/README.md`](../code/simulation_bringup/README.md).

## 9.1 One command

```bash
ros2 launch simulation_bringup sim_bringup_launch.py
```

Defaults (`config/bringup.yaml`): simulator on the ground-truth Spielberg map; stack on the
Cartographer map with its iterative minimum-curvature raceline; ground-truth pose; Frenet
local planner.

Stop with **Ctrl+C**.

## 9.2 What to change, and where

| To change | Use | Values |
|---|---|---|
| localizer | `localization:=` | `gt`, `pf`, `carto`, `scan_matching` |
| odometry noise | `odom_noise:=` | `true`, `false` (localizers only) |
| local planner | `planner:=` | `passthrough`, `frenet`, `rrt` |
| RViz view | `tf_source:=` | `gym` (truth), `localization` (belief) |
| run folder name | `tag:=` | default `auto` = `<scenario>_<localization>[_noise]_<planner>` |
| map / raceline / track | a scenario file, `config:=<file>` | see 9.3 |
| node parameters | `code/simulation_bringup/config/params/*.yaml` | per node |

The bringup uses its **own copies** of every node's parameter file (`config/params/`), so
edit those, not the originals inside each package. Values the bringup derives (topics,
frames, initial poses, odometry input, pose source) are overridden at launch so they always
agree with the scenario.

## 9.3 Scenario files

A scenario file (`code/simulation_bringup/config/*.yaml`) separates the **world** (what the
simulated LiDAR sees: the truth) from the **stack** (what the car believes):

```yaml
world:
  map: ~/f1tenth_sim_ws/f1tenth_racetracks/Spielberg/Spielberg_map      # truth
stack:
  map: Spielberg/carto/Spielberg_map.yaml       # map the localizer uses (relative to data/)
  frame: carto_map                              # its frame
  tf: [0.0, 0.0, -2.879]                        # pose of that frame in the simulator frame
  raceline: Spielberg/carto/raceline/Spielberg_carto_mincurv_iter.csv
localization: {method: gt, odom_noise: false}
planning: {local_planner: frenet}
```

| File | World | Stack |
|---|---|---|
| `bringup.yaml` | GT | Cartographer map + raceline |
| `gt_gt.yaml` | GT | GT map + raceline |

For a new raceline or map, copy a file, edit the `stack` lines and run it with
`config:=<new file>`. If `stack.map` is the same as `world.map`, no second map server or
static transform is started.

## 9.4 Recipes

```bash
B="ros2 launch simulation_bringup sim_bringup_launch.py"

# GT map, GT pose, raceline tracked without local planning
$B config:=gt_gt.yaml planner:=passthrough

# another raceline: a scenario file with a different stack.raceline
$B config:=<scenario file>.yaml planner:=passthrough

# local planners (Cartographer stack map)
$B planner:=passthrough
$B planner:=frenet
$B planner:=rrt

# localizers
$B localization:=pf
$B localization:=carto
$B localization:=scan_matching
$B localization:=pf odom_noise:=true

# belief view
$B localization:=pf tf_source:=localization
```

## 9.5 Start-up order

Nodes start in stages; each stage starts when a node of the previous stage prints a given
log line:

1. run folder and logging; static TF `map → <stack frame>`; simulator (bridge, world map,
   car model, RViz); stack map server and its lifecycle manager
2. localizer (+ `odom_noise`, relay, TF broadcaster), once the stack map server is active
3. raceline publisher, local planner, pure pursuit, lap logger, run monitor, rosbag, once the
   localizer reports a pose (`gt`: 2 s after stage 1)

If a stage is still waiting after 30 s, a yellow line names the node to check.

## 9.6 Terminal and logs

The terminal shows only the bringup's own lines (scenario summary, stages) and the run
monitor: one line per lap and every collision (example output):

```
[bringup] stage 3: pf ready -> starting the stack
[run_monitor]: lap 1: 67.89 s | collisions 0 | modes: pass 71%, frenet 29% | planner 4.1 ms mean, 9.8 ms max
```

Everything else goes to `~/f1tenth_simulation_logs/<YYYYmmdd_HHMMSS>_<tag>/`:

| File | Content |
|---|---|
| `bringup_resolved.yaml` | the exact scenario and derived values (reproduce the run) |
| `sim_resolved.yaml` | the simulator parameters used |
| `summary.csv` | per lap: time, collisions, local-planner mode split, compute time |
| `<raceline>_<tag>_<time>.csv` | lap logger: ground-truth tracking, 100 Hz |
| `bag/` | rosbag (poses, scan, drive, waypoints, planner mode, TF, ...) |
| `ros_logs/` | one log file per node |
| `launch_log` | link to the raw console output of every process |

Live view of all nodes with filtering: `ros2 run rqt_console rqt_console`.

## 9.7 Truth and belief views

```bash
$B localization:=pf                          # truth: car model = real car (default)
$B localization:=pf tf_source:=localization  # belief: TF from the particle filter
```

Only the TF tree and the RViz layout differ. In the belief view the car is drawn as an axes
marker at `base_link` (the estimate) and a white arrow shows `/ego_racecar/odom` (ground
truth) ([2. Architecture](02_architecture.md#truth-and-belief)). With `localization:=gt` the
bringup uses the truth view.

## 9.8 Mapping runs

A separate launch file drives the car with a reactive controller while Cartographer maps:
see [3. Mapping](03_mapping.md).
