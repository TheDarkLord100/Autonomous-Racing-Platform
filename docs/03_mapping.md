# 3. Mapping

A mapping run drives the car around the track with a reactive controller while
Cartographer builds a map: an occupancy grid and a pose graph (`.pbstream`). The saved map
is the input of track extraction, raceline optimisation and localization.

## 3.1 Run

```bash
ros2 launch simulation_bringup mapping_bringup_launch.py                          # FTG at 3 m/s
ros2 launch simulation_bringup mapping_bringup_launch.py controller:=pid max_speed:=2.5
ros2 launch simulation_bringup mapping_bringup_launch.py out_dir:=Spielberg/carto_v2
```

| Argument | Default | Meaning |
|---|---|---|
| `controller` | `ftg` | driver: `ftg` (`gap_follow`) or `pid` (`pid_wall`) |
| `max_speed` | `3.0` | m/s; the controller's minimum speed is clamped to it |
| `odom_noise` | `false` | Cartographer reads `/ego_racecar/odom_noisy` from `odom_noise` |
| `out_dir` | `Spielberg/carto_new` | output folder, relative to `data/` |
| `map_name` | `Spielberg` | output file names |
| `config` | `mapping.yaml` | `code/simulation_bringup/config/mapping.yaml` |

The terminal warns if `out_dir` already contains `<map_name>.pbstream`.

Start-up order: simulator, static TF `map → carto_map` (origin = spawn pose), Cartographer
in mapping mode, then the driver once `carto_node` is up. RViz (`mapping.rviz`) shows the
live map `/carto/map` over the simulator's map.

## 3.2 Save

With the launch still running:

```bash
ros2 service call /carto/save_map std_srvs/srv/Trigger
ros2 run nav2_map_server map_saver_cli -t /carto/map \
  -f ~/f1tenth_sim_ws/src/Autonomous-Racing-Platform/data/Spielberg/carto_new/Spielberg_map
```

`/carto/save_map` finishes the trajectory, waits for Cartographer's final optimisation and
writes the pose graph and sidecar; it replies `success: True` when done. `map_saver_cli`
writes the occupancy grid image. Both commands are printed in the terminal, with the
configured paths, when the driver starts.

Stop with **Ctrl+C**.

## 3.3 Output

```
data/Spielberg/carto_new/
├── Spielberg.pbstream      Cartographer pose graph (used by Cartographer localization)
├── Spielberg.yaml          sidecar: pbstream file name + origin [x, y, yaw] in the simulator frame
├── Spielberg_map.pgm       occupancy grid image
└── Spielberg_map.yaml      image metadata (resolution, origin), frame carto_map
```

Next: [4. Track extraction](04_track_extraction.md). To race on the map, point a scenario
file's `stack.map` at it ([9. Running simulations](09_running_simulations.md)).

## 3.4 Without the bringup

`ros2 launch cartographer_slam carto_mapping_launch.py` starts Cartographer alone
([`cartographer_slam`](../code/localization/cartographer_slam/README.md)); the simulator and
a driver are started separately.
