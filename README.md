<div align="center">

# Autonomous Racing Platform: Perception, Planning, and Control Algorithm Design for Hardware Realization

**Keywords:** `robotics` `perception` `navigation` `control` `ros2`

</div>

---

## Overview

This project develops and benchmarks a modular autonomy stack for
high-speed autonomous racing on a 1/10th-scale racing vehicle platform.
The work spans the full autonomy pipeline, from reactive control through
localization, planning, and control-layer implementation, evaluated first
in simulation against published benchmark data from other teams on the
same platform, and then progressively deployed on a physical vehicle as
hardware is acquired.

---

## System Overview

> **TODO** — architecture diagram and description, to be added once the
> stack design is finalized.

## Methodology

> **TODO** — perception, planning, and control approach write-up, to be
> added once the stack design is finalized (targeting next week).

### Hardware Platform

| Component  | Details       |
| ---------- | ------------- |
| Platform   | HSP94123 Pro, 1/10-scale 4WD |
| Compute    | NVIDIA Jetson Orin Nano |
| Sensors    | Slamtec C1M1-R2 LiDAR, ICM20947 IMU, VESC-derived (encoderless) odometry |
| Actuation  | VESC-compatible BLDC ESC |
| Power      | 3S 5200mAh LiPo |
| Middleware | ROS 2 Humble |

---

## Results

> **TODO** — to be added once benchmark runs are available.

## Demonstration

> **TODO** — to be added once sim/hardware runs are recorded.

---

## Repository Structure

This repository contains the project's own code, documentation, and
deliverables. It does **not** vendor the simulator or bridge packages the
project depends on — those are external dependencies, cloned separately
into the same ROS 2 workspace. See [Setup](#setup) below.

```text
code/           Project ROS 2 packages
hardware/       BOM, CAD, wiring, PCB design
report/         MTP report and presentation
docs/           Supplementary technical documentation
data/           Sample logs, configs, benchmark reference data
simulation/     Simulation-specific assets/configs (maps, params)
media/          Images, diagrams, videos
```

## Setup
 
Environment: **Ubuntu 22.04**, **ROS 2 Humble**.
 
### 1. Set up the workspace and venv
 
```bash
mkdir -p ~/f1tenth_sim_ws/src
cd ~/f1tenth_sim_ws
sudo apt install python3.10-venv   # if not already present
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
```
 
Confirm it's active before continuing: `which pip` should point inside
`~/f1tenth_sim_ws/.venv/`.
 
### 2. Install f1tenth_gym inside the venv
 
```bash
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym
pip install -e .
```
 
Sanity check:
 
```bash
cd examples
python3 waypoint_follow.py
```
 
### 3. Clone the ROS 2 bridge
 
```bash
cd ~/f1tenth_sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros
```
 
Set the map path in `f1tenth_gym_ros/config/sim.yaml`:
 
```yaml
map_path: '/home/<your_user>/f1tenth_sim_ws/src/f1tenth_gym_ros/maps/levine'
```
 
### 4. Clone this repository's packages into the same `src/`
 
```bash
cd ~/f1tenth_sim_ws/src
git clone <this-repo-url> autonomous_racing_platform
ln -s autonomous_racing_platform/code/* .   # or symlink/copy individual packages
```
 
### 5. Install ROS-side dependencies
 
```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_sim_ws/.venv/bin/activate
cd ~/f1tenth_sim_ws
rosdep install -i --from-path src --rosdistro humble -y
```
 
### 6. Build — using the venv's Python explicitly
 
```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_sim_ws/.venv/bin/activate
cd ~/f1tenth_sim_ws
python3 -m colcon build
```
 
**Important:** use `python3 -m colcon build`, not plain `colcon build`.
`colcon` itself is a system-installed tool, so running it directly ignores
the active venv and generates build scripts pointing at system Python
(which doesn't have `gym` installed). Running it as `python3 -m colcon`
forces it through the venv's interpreter.
 
### 7. Launch
 
```bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
 
RViz should open with the car on the Levine map, `map_server` active, and
no crashed nodes.
---

## TODO

- [ ] Finalize system architecture (methodology, layer boundaries) after
      further literature review — targeted for next week
- [ ] Add system overview diagram
- [ ] Implement reactive baseline package
- [ ] Populate results / benchmark data once available
- [ ] Add demonstration video/link
- [ ] Add thesis report and presentation PDFs to `report/`
- [ ] Add GitHub repository Topics per `PROJECT_GUIDELINES.md` §2

---
## Thesis report and presentation

  | Document        | Link |
  |-----------------|------|
  | Thesis / Report | [report/thesis.pdf](report/thesis.pdf) |
  | Presentation    | [report/presentation.pdf](report/presentation.pdf) |


---  
## Citation
**TODO**
---

## Team

| Role           | Name                |
| ---------------| ------------------- |
| Student        | Vijayant Singh (2025JRB2024, M.Tech Robotics) |
| Supervisor     | Prof. Shubhendu Bhasin |


---