# 6. Localization

Three localizers estimate the car's pose from the LiDAR and wheel odometry, plus ground
truth as a fourth option. Each produces a `nav_msgs/Odometry` pose for the local planner
and `pure_pursuit`.

## 6.1 Options

| `localization:=` | Method | Map input | Pose topic | Package |
|---|---|---|---|---|
| `gt` (default) | simulator ground truth | none | `/ego_racecar/odom` | — |
| `pf` | particle filter (Monte Carlo localization) | occupancy grid | `/pf/odom` | [`particle_filter`](../code/localization/particle_filter/README.md) |
| `carto` | Cartographer pure localization | `.pbstream` pose graph | `/loc/odom` (via relay) | [`cartographer_slam`](../code/localization/cartographer_slam/README.md) |
| `scan_matching` | PL-ICP, frame-to-frame | none | `/loc/odom` (via relay) | [`scan_matching`](../code/localization/scan_matching/README.md) |

```bash
ros2 launch simulation_bringup sim_bringup_launch.py localization:=pf
ros2 launch simulation_bringup sim_bringup_launch.py localization:=carto
ros2 launch simulation_bringup sim_bringup_launch.py localization:=scan_matching
```

`simulation_bringup` sets each localizer's map, frame, odometry input and initial pose from
the scenario file.

## 6.2 What each one does

**Particle filter.** 2000 pose hypotheses are moved by odometry with noise, weighted by a
likelihood-field scan model on the occupancy grid, and resampled when the effective
sample size drops below a threshold. Each particle carries an odometry scale factor.
Particles are injected around the estimate when the short-term scan fit drops below the
long-term fit (augmented MCL). Between scans the estimate is moved forward with odometry
and published on `/pf/odom` at the odometry rate.

**Cartographer localization.** Stock Cartographer on the frozen pose graph from mapping,
with `carto_localization.lua`. `carto_node` publishes one pose per scan on `/carto/pose`.

**Scan matching (PL-ICP).** Each scan is matched against the previous (or key) scan,
starting from the odometry motion as a guess; the pose is the chain of scan-to-scan motions
from the initial pose. Published per scan on `/scan_match/odom`.

**`localization_relay`** (in `simulation_bringup`), used for `carto` and `scan_matching`:
takes the latest per-scan estimate, adds the odometry motion since that scan's stamp, copies
the odometry twist, and publishes `/loc/odom` on every odometry message.

## 6.3 Odometry noise

`odom_noise` adds a scale error, a drifting heading bias and per-step noise to the
simulator's odometry, with a fixed random seed
([`odom_noise`](../code/odom_noise/README.md)).

```bash
ros2 launch simulation_bringup sim_bringup_launch.py localization:=pf odom_noise:=true
```

With `odom_noise:=true` the bringup starts the node and points the localizer, the relay and
the TF broadcaster at `/ego_racecar/odom_noisy`. Not available with `localization:=gt`.

## 6.4 Frames and initial pose

| Localizer | Frame of the pose | Initial pose |
|---|---|---|
| `pf` | `stack.frame` (e.g. `carto_map`) | spawn pose transformed into `stack.frame` |
| `carto` | `map` (`carto_node` converts from `carto_map` with `stack.tf`) | spawn pose |
| `scan_matching` | `map` | spawn pose |

The spawn pose is `sx, sy, stheta` from `code/simulation_bringup/config/params/sim.yaml`.
The simulator also subscribes to `/initialpose` and moves the car there; bringup runs set
initial poses from parameters instead.

`localization:=carto` needs a Cartographer sidecar (`<name>.yaml` with `pbstream:`) next to
the stack map, or `stack.carto_state` set to it.
