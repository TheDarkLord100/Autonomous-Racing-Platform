# 8. Control

## 8.1 Pure pursuit

`pure_pursuit` follows `/local_waypoints`: it steers towards the path point one lookahead
distance ahead and takes the speed from the path.

```
lookahead  L = clip(0.5 + 0.10 · v, 0.5, 2.5)  m
curvature  κ = 2 · l_y / l_d²                            (target point in the car frame)
steering   δ = atan(wheelbase · κ) + K · v² · κ          (K = understeer gradient)
speed      path vx_mps at the nearest point, capped at max_speed
```

`K = 0.0028` = (1/C_Sf − 1/C_Sr) / (μ g) with the simulator's tyre stiffnesses. Parameters:
`config/pp_sim.yaml` (`code/simulation_bringup/config/params/pure_pursuit.yaml` in bringup runs).

Reference: [`code/pure_pursuit/README.md`](../code/pure_pursuit/README.md).

## 8.2 Reactive controllers

Two controllers drive from the LiDAR alone (no map, no localization). `simulation_bringup`
uses them in mapping runs (`controller:=ftg|pid max_speed:=<m/s>`, [3. Mapping](03_mapping.md)).

| Controller | Method | Package |
|---|---|---|
| Follow-the-Gap | disparity extension and a safety bubble on the scan, largest gap, steer towards its farthest point; speed from steering angle and gap width | [`gap_follow`](../code/reactive/gap_follow/README.md) |
| PID wall following | distance to one wall from two rays, PID on the distance error with speed-scheduled gains; speed from the free distance ahead | [`pid_wall`](../code/reactive/pid_wall/README.md) |

```bash
ros2 launch gap_follow gap_follow.launch.py      # with the simulator running
ros2 launch pid_wall pid_launch.py
```

## 8.3 `/drive`

One controller publishes `/drive` at a time: `pure_pursuit` in racing runs, a reactive
controller in mapping runs.
