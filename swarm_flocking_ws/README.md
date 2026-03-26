# 🐦 Swarm Flocking with Obstacle Navigation

A ROS 2 Humble multi-robot flocking system using **Reynolds' Boids model** with obstacle avoidance, simulated in Gazebo Classic with TurtleBot3 Burger robots.

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic_11-orange)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04_LTS-purple)

---

## 📋 Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Building](#building)
- [Running the Simulation](#running-the-simulation)
- [Monitoring & Tuning](#monitoring--tuning)
- [Troubleshooting](#troubleshooting)
- [Package Structure](#package-structure)

---

## Overview

Each robot runs an independent **BoidNode** that computes five force components at 10 Hz:

| Force | Weight | Description |
|-------|--------|-------------|
| **Separation** | 1.5 | Avoid colliding with nearby robots |
| **Alignment** | 1.0 | Match flock's average heading |
| **Cohesion** | 1.0 | Steer toward flock centroid |
| **Obstacle** | 2.5 | Repulsive force from LaserScan data |
| **Migration** | 0.3 | Gentle pull toward sequential waypoints |

A singleton **FlockMonitorNode** tracks swarm metrics: centroid, spread, connectivity, and split detection.

---

## Architecture

```
┌─────────────────────────────────────────────────┐
│                   Gazebo World                  │
│  ┌─────────┐ ┌─────────┐       ┌─────────┐     │
│  │ robot_0 │ │ robot_1 │  ...  │ robot_N │     │
│  │ (SDF)   │ │ (SDF)   │       │ (SDF)   │     │
│  └────┬────┘ └────┬────┘       └────┬────┘     │
│       │odom/scan   │                 │          │
└───────┼────────────┼─────────────────┼──────────┘
        │            │                 │
   ┌────▼────┐  ┌────▼────┐     ┌─────▼───┐
   │ boid_0  │  │ boid_1  │ ... │ boid_N  │
   │ (node)  │  │ (node)  │     │ (node)  │
   └────┬────┘  └────┬────┘     └────┬────┘
        │ pose_share  │               │
        └──────┬──────┘───────────────┘
               │
        ┌──────▼──────┐
        │flock_monitor│ → /flock/state, centroid, convex_hull
        └─────────────┘
```

---

## Prerequisites

- **Ubuntu 22.04 LTS** (tested on this version)
- **ROS 2 Humble Hawksbill** (full desktop install)
- **Gazebo Classic 11** (comes with `ros-humble-desktop-full`)

---

## Installation

### 1. Install ROS 2 Humble (if not already installed)

Follow the [official guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), or:

```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop-full
```

### 2. Install TurtleBot3 & Gazebo Packages

```bash
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-description \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-msgs \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-rmw-fastrtps-cpp \
  python3-colcon-common-extensions
```

### 3. Install Build Tools

```bash
sudo apt install -y python3-rosdep python3-vcstool
sudo rosdep init 2>/dev/null || true
rosdep update
```

### 4. Clone the Repository

```bash
cd ~
git clone https://github.com/rohancyriac029/swarm_flocking_ws.git
cd swarm_flocking_ws/swarm_flocking_ws
```

### 5. Set Up Environment Variables

Add these to your `~/.bashrc` for persistence:

```bash
echo '' >> ~/.bashrc
echo '# --- Swarm Flocking Setup ---' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc

# Apply immediately
source ~/.bashrc
```

> **⚠️ Important**: We use **FastDDS** (`rmw_fastrtps_cpp`) instead of the default CycloneDDS because CycloneDDS hits participant limits on loopback-only networks when running 20+ ROS nodes.

---

## Building

```bash
cd ~/swarm_flocking_ws/swarm_flocking_ws

# Install any missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build all three packages
colcon build --symlink-install \
  --packages-select swarm_interfaces swarm_flocking swarm_flocking_gazebo

# Source the workspace
source install/setup.bash
```

---

## Running the Simulation

### Launch the Full Simulation (6 robots)

```bash
# Make sure environment is set
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/swarm_flocking_ws/swarm_flocking_ws/install/setup.bash

# Launch!
ros2 launch swarm_flocking full_sim.launch.py num_robots:=6
```

This starts:
1. **Gazebo** with the obstacle course world (~10s to load)
2. **Robot State Publishers** for TF (at ~28s)
3. **Spawn Entity** for each robot via SDF model (at ~30s, staggered 1.5s apart)
4. **Boid Nodes** for each robot (at ~35s, staggered)
5. **Flock Monitor** for swarm metrics
6. **RViz** for visualization

> **Note**: It takes ~40–50 seconds for everything to start due to intentional delays that prevent race conditions with Gazebo.

### Custom Number of Robots

```bash
# Fewer robots (faster, good for testing)
ros2 launch swarm_flocking full_sim.launch.py num_robots:=3

# More robots (heavier on CPU)
ros2 launch swarm_flocking full_sim.launch.py num_robots:=10
```

---

## Monitoring & Tuning

Open a **second terminal** for monitoring. Always set the same environment:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/swarm_flocking_ws/swarm_flocking_ws/install/setup.bash
```

### Watch Flock Metrics

```bash
ros2 topic echo /flock/state
```

### Check Individual Robot Velocity

```bash
ros2 topic echo /robot_0/cmd_vel
```

### Verify Gazebo is Publishing Sensor Data

```bash
# Should show Publisher count >= 1
ros2 topic info /robot_0/odom -v
```

### List All Active Topics

```bash
ros2 topic list
```

### List All Active Nodes

```bash
ros2 node list
```

### Live-Tune Parameters (While Running)

```bash
# Increase separation (robots spread out more)
ros2 param set /robot_0/boid_0 w_separation 3.0

# Increase cohesion (tighter flock)
ros2 param set /robot_0/boid_0 w_cohesion 2.0

# Increase migration pull (faster goal seeking)
ros2 param set /robot_0/boid_0 w_migration 0.8

# Increase obstacle avoidance
ros2 param set /robot_0/boid_0 w_obstacle 4.0
```

---

## Troubleshooting

### Gazebo crashes (`exit code 255`)

```bash
# Kill leftover processes from previous runs
killall -9 gzserver gzclient 2>/dev/null
pkill -9 -f spawn_entity 2>/dev/null
pkill -9 -f boid_node 2>/dev/null

# Try again
ros2 launch swarm_flocking full_sim.launch.py num_robots:=6
```

### "Failed to find a free participant index"

This is a CycloneDDS limitation. Make sure you're using FastDDS:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Robots spawn but don't move

Check that Gazebo is publishing odometry:

```bash
ros2 topic info /robot_0/odom -v
# Should show Publisher count: 1
```

If Publisher count is 0, the SDF model didn't load Gazebo plugins. Verify:

```bash
ls /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
```

### RViz shows nothing

Make sure your monitoring terminal uses the **same RMW** as the launch terminal:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Topics not visible between terminals

Both terminals must use identical `RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID` values. DDS nodes using different implementations cannot communicate.

---

## Package Structure

```
swarm_flocking_ws/
└── src/
    ├── swarm_interfaces/          # Custom message definitions
    │   └── msg/
    │       └── FlockState.msg     # Swarm metrics message
    │
    ├── swarm_flocking/            # Core flocking logic
    │   ├── config/
    │   │   ├── flocking_params.yaml   # Tunable parameters
    │   │   └── rviz_config.rviz       # RViz display config
    │   ├── launch/
    │   │   ├── full_sim.launch.py     # Master launch (Gazebo + robots + boids)
    │   │   └── spawn_flock.launch.py  # Standalone spawn (Gazebo already running)
    │   ├── swarm_flocking/
    │   │   ├── boid_node.py           # Per-robot Reynolds controller
    │   │   ├── flock_monitor_node.py  # Swarm metrics & visualization
    │   │   └── utils/
    │   │       ├── reynolds.py        # Force computation math
    │   │       └── obstacle_avoidance.py  # LaserScan repulsive force
    │   └── test/                  # Unit tests
    │
    └── swarm_flocking_gazebo/     # Gazebo simulation assets
        ├── launch/
        │   └── gazebo_world.launch.py
        ├── models/
        │   ├── narrow_wall/       # Custom obstacle models
        │   └── goal_marker/
        └── worlds/
            ├── obstacle_course.world   # Main world with bottleneck + maze
            └── open_field.world        # Simple testing world
```

---

## Parameters Reference

All parameters are in `src/swarm_flocking/config/flocking_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_robots` | 6 | Number of robots in the swarm |
| `w_separation` | 1.5 | Separation force weight |
| `w_alignment` | 1.0 | Alignment force weight |
| `w_cohesion` | 1.0 | Cohesion force weight |
| `w_obstacle` | 2.5 | Obstacle avoidance weight |
| `w_migration` | 0.3 | Migration (goal) weight |
| `neighbor_radius` | 3.0 m | Max range to consider a neighbor |
| `separation_radius` | 0.8 m | Below this, separation activates |
| `obstacle_threshold` | 0.6 m | Laser range triggering avoidance |
| `max_linear_vel` | 0.20 m/s | TurtleBot3 Burger max linear speed |
| `max_angular_vel` | 1.5 rad/s | Max angular speed |

---

## License

This project is for educational and research purposes.
