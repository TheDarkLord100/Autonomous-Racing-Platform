# 7. Local planning

`local_planner` checks the next 5 m of the global raceline against the live LiDAR scan
every scan, and publishes either the raceline segment or a local path that keeps the
required clearance and rejoins the raceline. Package reference:
[`code/local_planner/README.md`](../code/local_planner/README.md).

## 7.1 Gate

Every scan (10 Hz):

1. The next 5 m of raceline are moved into the car frame (`base_link`) with the current pose.
2. The scan points are moved into `base_link`.
3. **Gate:** every raceline point must be at least `car_half_width + clearance_margin`
   (0.155 + 0.10 m) from every scan point.

| Gate | Car within `rejoin_tol` (0.15 m) of the raceline | Output |
|---|---|---|
| free | yes, and no plan in the last `hold_cycles` scans | raceline segment (`pass`) |
| free | no | planned path |
| blocked | — | planned path |
| — (planner finds nothing) | — | safest candidate or raceline at `fallback_speed` (`fallback`) |

## 7.2 Planners

| `planner:=` | Method |
|---|---|
| `passthrough` | always the raceline segment |
| `frenet` (default) | quintic lateral offsets from the raceline to 2, 3, 4, 5 m ahead; candidates closer to the scan than the clearance, or tighter than the steering limit, are rejected; the cheapest remaining one is chosen (offset, smoothness, consistency with the last plan, proximity to scan points) |
| `rrt` | RRT* sampled inside the scan-derived track bounds, straight edges with a turning limit; shortcut, cubic spline smoothing, then clearance and curvature re-checked |

Planned paths take the raceline speed, capped by `√(μ·g·grip / |κ|)` on the new path and by
a backward braking pass. Within `near_radius` (0.5 m) of the car the required clearance is
`car_half_width + near_margin` (0.02 m).

## 7.3 Parameters

`code/simulation_bringup/config/params/local_planner.yaml` (bringup runs) or
`code/local_planner/config/local_planner_sim.yaml` (package launch file). Main values:
`horizon` 5.0 m, `clearance_margin` 0.10 m, `rejoin_tol` 0.15 m, `hold_cycles` 3,
`fallback_speed` 1.0 m/s. `clearance_margin` is set below the raceline optimiser's
`--margin` (0.15 m).

## 7.4 Topics

| Topic | Content |
|---|---|
| `/local_waypoints` | the path for `pure_pursuit` (raceline frame, `s_m` from 0) |
| `/local_planner/mode` | `pass`, `frenet`, `rrt`, `rrt_raw`, `fallback` per scan |
| `/local_planner/compute_ms` | time per cycle |
| `/local_planner/markers` | corridor (green clear / red blocked), local path (blue; orange in fallback), candidates or tree (grey) |

## 7.5 RViz markers

The markers are published in `base_link`. RViz places them at the pose TF gives
`base_link`: the ground-truth pose in the truth view, the estimate in the belief view
(`tf_source:=localization`, [2. Architecture](02_architecture.md#truth-and-belief)).
