# odom_noise

Turns the simulator's perfect ground-truth odometry into drifting, repeatable wheel odometry. Shared by all localization packages so each is tested against the same odometry error.

Why: `f1tenth_gym` publishes the true pose as `/ego_racecar/odom`. A localizer fed that never exercises its motion model and can never drift. Real VESC odometry has scale errors, heading drift and jitter; this node reproduces those.

## Folder structure

```
odom_noise/
├── package.xml                     deps: rclpy, nav_msgs, numpy
├── setup.py                        install rules + executable odom_noise_node
├── setup.cfg
├── resource/odom_noise
├── config/odom_noise_params.yaml   noise parameters
├── launch/odom_noise_launch.py     starts the node (included by other packages' launch files)
├── test/test_noise_model.py        offline tests (no ROS needed)
└── odom_noise/
    ├── noise_model.py              OdometryNoise: the error model (no ROS imports)
    └── odom_noise_node.py          ROS 2 node wrapping it
```

## Model (`noise_model.py`)

Each incoming true pose is turned into a local motion step `(dx, dy, dθ)` relative to the previous one. With trans = |(dx, dy)|:
```
dx' = dx·(1 + trans_scale_error) + N(0, trans_noise·trans)
dy' = dy                          + N(0, trans_noise·trans)
dθ' = dθ·(1 + rot_scale_error) + yaw_drift_per_m·dx
      + N(0, rot_noise_per_rad·|dθ| + rot_noise_per_m·trans)
```
- Corrupted steps are integrated into a pose that starts at the `odom` origin (0, 0, 0) and drifts, like VESC odometry.
- Systematic terms (scale errors, yaw drift) dominate over a lap; white-noise terms add per-step jitter.
- Twist: scaled by the same systematic errors (no white noise).

**Repeatable.** The white noise comes from a generator seeded with `random_seed` (default 42). Exactly 3 random draws per odom message, so the same seed gives the same noise sequence every run. Driving the same trajectory gives the same odometry error, for any localization package. `random_seed: -1` draws a new sequence each run.

**Teleports.** A single step longer than `teleport_threshold` (1 m; a real step at 100 Hz is ≤ ~10 cm) is a simulator reset, e.g. RViz "2D Pose Estimate" moving the car. Real wheel odometry would not see it, so the odom pose is left unchanged and the node logs it.

## Node (`odom_noise_node.py`)

| | Topic | Frame |
|---|---|---|
| in | `/ego_racecar/odom` (ground truth) | `map` |
| out | `/ego_racecar/odom_noisy` | `odom` → `ego_racecar/base_link` |

One output message per input message, same timestamp.

## Parameters (`config/odom_noise_params.yaml`)

- `input_topic`, `output_topic`, `odom_frame`, `base_frame`
- `random_seed` 42 (−1 = random each run)
- Systematic: `trans_scale_error` 0.05, `rot_scale_error` 0.05, `yaw_drift_per_m` 0.005 rad/m
- White noise: `trans_noise` 0.02, `rot_noise_per_rad` 0.02, `rot_noise_per_m` 0.005
- `teleport_threshold` 1.0 m

With the defaults, odometry reports 5% too much distance; a localizer that learns the scale should converge to ≈ 1/1.05 = 0.952.

## Use

Standalone:
```bash
ros2 launch odom_noise odom_noise_launch.py            # args: params_file, use_sim_time
```

From another package's launch file:
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('odom_noise'), 'launch', 'odom_noise_launch.py')),
    launch_arguments={'use_sim_time': 'false'}.items())
```
then point that package's odometry input at `/ego_racecar/odom_noisy` and add `<exec_depend>odom_noise</exec_depend>` to its `package.xml`. `particle_filter` does this with `odom_noise:=true`.

Offline tests: `python3 -m pytest test/ -v` from the package root.