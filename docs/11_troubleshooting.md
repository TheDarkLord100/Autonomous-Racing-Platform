# 11. Troubleshooting

| Symptom | Fix |
|---|---|
| `ModuleNotFoundError` (e.g. `f1tenth_gym`) when launching | activate the venv and rebuild with `python3 -m colcon build` (not plain `colcon build`) |
| FastRTPS / shared-memory port errors at start-up | `rm -rf /dev/shm/fastrtps_port*` |
| A node never receives `/global_waypoints` or `/local_waypoints` | subscribe with reliable + transient-local QoS; check with `ros2 topic info -v <topic>` |
| `pure_pursuit` doesn't move the car | check `ros2 topic hz /local_waypoints` and the pose topic |
| `[bringup] still waiting for "..." from <node>` | read that node's log in `<run folder>/ros_logs/`; the quoted text is the line the bringup waits for |
| Stack map drawn unrotated / offset in RViz | `ros2 param get /<map server> frame_id`; restart the map server with the `frame_id` parameter on one command line |
| RViz "No transform to fixed frame" on markers | stamp markers in `base_link` with time 0 (latest TF) |
| TF reports two parents for `base_link` | with `tf_source: localization`, the bridge must run with `publish_pose_tf: false` (patched bridge, [environment_changes.md](../environment_changes.md)) |
| Belief view: `base_link` not connected to `map` | wait for the localizer to initialise; check its log |
| RViz opens the gym's default layout | apply `gym_bridge.patch` (`rviz_config` argument) |
| `localization:=carto` stops with "needs a Cartographer sidecar" | use a stack map with a sidecar next to it, or set `stack.carto_state` |
| The car jumps when using RViz "2D Pose Estimate" | the simulator also subscribes to `/initialpose`; in bringup runs initial poses come from parameters |
| The whole launch shuts down | the simulator bridge exited; read its log in `ros_logs/` |

```bash
ros2 node list                          # running nodes
ros2 topic hz /scan /ego_racecar/odom   # 10 Hz and 100 Hz
ros2 run tf2_tools view_frames          # TF tree as a PDF
ros2 run rqt_console rqt_console        # all node logs, filterable
```
