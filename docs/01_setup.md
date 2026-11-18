# 1. Setup

Target: **Ubuntu 24.04**, **ROS 2 Jazzy**, native install (no Docker). All project
packages are Python (`ament_python`). The simulator runs from a Python virtual environment
inside the workspace.

## 1.1 ROS 2 and system packages

Install ROS 2 Jazzy Desktop following the official instructions, then:

```bash
sudo apt install python3-venv python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-cartographer-ros ros-jazzy-ackermann-msgs ros-jazzy-foxglove-bridge \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-tf2-tools
```

(`rosdep` in step 1.6 installs anything still missing.)

## 1.2 Workspace and virtual environment

```bash
mkdir -p ~/f1tenth_sim_ws/src
cd ~/f1tenth_sim_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
which pip            # must point into ~/f1tenth_sim_ws/.venv/
```

## 1.3 Simulator (`f1tenth_gym`)

```bash
cd ~/f1tenth_sim_ws
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym && git checkout dev-jax && git checkout ec3b1a4
pip install -e .
```

## 1.4 Race tracks

```bash
cd ~/f1tenth_sim_ws
git clone https://github.com/f1tenth/f1tenth_racetracks
```

The scenario files expect `~/f1tenth_sim_ws/f1tenth_racetracks/Spielberg/Spielberg_map`.

## 1.5 ROS 2 bridge (`f1tenth_gym_ros`) + this project's changes

```bash
cd ~/f1tenth_sim_ws/src
git clone -b dev-jazzy https://github.com/f1tenth/f1tenth_gym_ros
git clone <this-repository-url> Autonomous-Racing-Platform
cd f1tenth_gym_ros && git checkout 2c01981
git apply ../Autonomous-Racing-Platform/code/simulation_bringup/gym_bridge.patch
```

What the patch changes: [environment_changes.md](../environment_changes.md).

## 1.6 Dependencies

```bash
source /opt/ros/jazzy/setup.bash
source ~/f1tenth_sim_ws/.venv/bin/activate
pip install osqp scikit-image pandas      # offline planning / extraction / evaluation scripts
pip install numba                         # optional: ~5x faster scan matching
cd ~/f1tenth_sim_ws
sudo rosdep init 2>/dev/null; rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y
```

`pip install numba --dry-run` shows whether it would change the installed NumPy.

## 1.7 Build

```bash
source /opt/ros/jazzy/setup.bash
source ~/f1tenth_sim_ws/.venv/bin/activate
cd ~/f1tenth_sim_ws
python3 -m colcon build --symlink-install
source install/setup.bash
```

Build with **`python3 -m colcon build`** (with the venv active), not plain `colcon build`.
With `--symlink-install`, edits to Python files, configs and RViz layouts take effect
without rebuilding.

A shell function saves typing the three `source` lines every time:

```bash
# ~/.bashrc
f1sim() { cd ~/f1tenth_sim_ws && source /opt/ros/jazzy/setup.bash && \
          source .venv/bin/activate && source install/setup.bash; }
```

## 1.8 Check

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py use_rviz:=true    # simulator alone
ros2 launch simulation_bringup sim_bringup_launch.py               # full stack, GT pose
```

The second command opens RViz on Spielberg; the car starts driving a few seconds later
and the terminal prints one line per completed lap. If something fails, see
[11_troubleshooting.md](11_troubleshooting.md).

## Data location

Maps, extracted tracks and racelines are in this repository's `data/` folder
([data/README.md](../data/README.md)). Run logs are written to `~/f1tenth_simulation_logs/`.
