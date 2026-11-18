# Environment changes

The simulator is an external dependency and is not part of this repository. This file
lists the versions used and every change made to them.

| Component | Source | Version used | Modified |
|---|---|---|---|
| `f1tenth_gym` (simulator, Python) | https://github.com/f1tenth/f1tenth_gym | branch `dev-jax`, commit `ec3b1a4` | no |
| `f1tenth_gym_ros` (ROS 2 bridge) | https://github.com/f1tenth/f1tenth_gym_ros | branch `dev-jazzy`, commit `2c01981` | **yes** |
| `f1tenth_racetracks` (track maps) | https://github.com/f1tenth/f1tenth_racetracks | default branch | no |

All changes to `f1tenth_gym_ros` are in one patch against commit `2c01981`:
[`code/simulation_bringup/gym_bridge.patch`](code/simulation_bringup/gym_bridge.patch).

```bash
cd ~/f1tenth_sim_ws/src/f1tenth_gym_ros
git checkout 2c01981
git apply ~/f1tenth_sim_ws/src/Autonomous-Racing-Platform/code/simulation_bringup/gym_bridge.patch
git diff --stat      # config/sim.yaml, f1tenth_gym_ros/gym_bridge.py, launch/gym_bridge_launch.py
```

The defaults of every new parameter and argument reproduce the upstream behaviour, except
`scan_rate_hz` in `sim.yaml` (10 Hz).

---

## 1. Separate publish timers for odometry and LiDAR

**Files:** `f1tenth_gym_ros/gym_bridge.py`, `config/sim.yaml`

**Upstream:** one 250 Hz timer (`timer_callback`, period 0.004 s) publishes scan, odometry,
TF, collision and lap topics.

**Now:** two timers, `odom_timer_callback` and `scan_timer_callback`:

| Parameter | Default in code | `sim.yaml` | Timer publishes |
|---|---|---|---|
| `odom_rate_hz` | 100.0 | 100.0 | odometry, TF (`map → base_link`, laser, wheels), collision, lap count / time |
| `scan_rate_hz` | 40.0 | 10.0 | `LaserScan` |

- `timer_callback` remains and calls both (synchronous mode).
- `_publish_stamp()` returns the message stamp (simulation time with `use_sim_time_bridge`).
- With `use_sim_time_bridge` in asynchronous mode, both timer periods are set to 0, as the
  single timer was upstream.

## 2. `use_rviz` launch argument

**File:** `launch/gym_bridge_launch.py`

**Upstream:** the launch file always starts `foxglove_bridge`, opens Foxglove and prints the
Foxglove lines; there is no RViz node.

**Now:** argument `use_rviz` (default `false`):

| `use_rviz` | Starts |
|---|---|
| `false` | `foxglove_bridge`, browser open, Foxglove log lines (upstream behaviour) |
| `true` | `rviz2` instead; none of the Foxglove actions |

## 3. `rviz_config` launch argument

**File:** `launch/gym_bridge_launch.py`

**Now:** argument `rviz_config` (default: the package's `config/rviz/gym_bridge.rviz`), passed
to `rviz2 -d` when `use_rviz:=true`. `simulation_bringup` passes its own layouts
(`rviz/sim.rviz`, `rviz/sim_belief.rviz`, `rviz/mapping.rviz`).

## 4. `publish_pose_tf` parameter

**Files:** `f1tenth_gym_ros/gym_bridge.py`, `config/sim.yaml`

**Upstream:** the bridge always publishes the ground-truth TF `map → <ns>/base_link`.

**Now:** parameter `publish_pose_tf` (default `true`). With `false`, `_publish_transforms`
returns without publishing that transform; laser and wheel transforms are still published.
`simulation_bringup` sets it from `run.tf_source`:

| `tf_source` | `publish_pose_tf` | `base_link` TF published by |
|---|---|---|
| `gym` (default) | `true` | the bridge (ground truth) |
| `localization` | `false` | `odom_tf_broadcaster` (`odom → base_link`) + the localizer (`map → odom`) |

---

## Local settings (not in the patch)

Machine-specific values in this setup's `config/sim.yaml`. `simulation_bringup` passes its
own bridge parameters (`code/simulation_bringup/config/params/sim.yaml`) and map path, so
these only apply when the bridge is launched directly.

| `config/sim.yaml` key | Upstream | Used here |
|---|---|---|
| `map_path` | `maps/levine` | `<workspace>/f1tenth_racetracks/Spielberg/Spielberg_map` |
| `sx`, `sy`, `stheta` | `-12.0`, `0.0`, `0.0` | `0.0`, `0.0`, `-2.879` |

The Cartographer maps in `data/Spielberg/` were built from this spawn pose; their sidecar
`origin` and the scenario files' `stack.tf` are `[0.0, 0.0, -2.879]`.

## Workspace layout

```
~/f1tenth_sim_ws/
├── .venv/                        python3 -m venv --system-site-packages (gym installed here)
├── f1tenth_gym/                  dev-jax, pip install -e .
├── f1tenth_racetracks/           maps
└── src/
    ├── f1tenth_gym_ros/          dev-jazzy @ 2c01981 + gym_bridge.patch
    └── Autonomous-Racing-Platform/   this repository
```

Full setup: [docs/01_setup.md](docs/01_setup.md).
