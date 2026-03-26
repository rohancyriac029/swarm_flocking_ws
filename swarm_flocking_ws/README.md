# 🐦 Swarm Flocking with Obstacle Navigation

> **ROS 2 Humble · Gazebo Classic · TurtleBot3 Burger**  
> Reynolds boids (separation + alignment + cohesion) extended with obstacle avoidance and migration forces.  
> 6–10 simulated robots navigate a 30×15 m corridor world — splitting through bottlenecks, merging in open space.

---

## 📖 Project Overview

This project implements **Craig Reynolds' Boids model** on real simulated robots with the critical addition of obstacle avoidance and goal-seeking migration. Each robot runs an independent `boid_node` that:

1. Broadcasts its own pose and velocity to all peers
2. Computes five weighted force vectors every 100 ms
3. Converts the resultant force into `cmd_vel` via a P-controller with low-pass smoothing
4. Publishes heading commands to its Gazebo-simulated TurtleBot3

A singleton `flock_monitor_node` monitors the swarm globally, detects flock splits via BFS graph analysis, computes a convex hull of all robots, and publishes RViz markers.

---

## 🏗️ Architecture Diagram

```
╔══════════════════════════════════════════════════════════════╗
║                     robot_i  (i = 0..N-1)                   ║
║                                                              ║
║  /robot_i/odom ──────► odom_callback                        ║
║  /robot_i/scan ──────► scan_callback                        ║
║                              │                               ║
║            ┌─────────────────▼──────────────────┐           ║
║            │         BoidNode (10 Hz)            │           ║
║            │  ┌──────────────────────────────┐   │           ║
║            │  │ Reynolds math (utils/)        │   │           ║
║            │  │  separation + alignment       │   │           ║
║            │  │  cohesion + obstacle + migr.  │   │           ║
║            │  └──────────────┬───────────────┘   │           ║
║            │     force_to_cmd_vel() + LPF         │           ║
║            └─────────────────┬──────────────────┘           ║
║                              │                               ║
║  /robot_i/cmd_vel ◄──────────┘                              ║
║  /robot_i/pose_share ──────────────────────────────────────►║
║  /robot_i/velocity_share ──────────────────────────────────►║
╚══════════════════════════════════════════════════════════════╝
                    ▼ all /robot_i/pose_share
╔══════════════════════════════════════════════════╗
║          FlockMonitorNode (2 Hz)                 ║
║  • Centroid computation                          ║
║  • Cohesion radius (mean dist from centroid)     ║
║  • BFS split detection → num_subgroups           ║
║  • Graham scan convex hull → /flock/convex_hull  ║
║  • Neighbor links → /flock/neighbor_links        ║
║  • Collision counting                            ║
╚══════════════════════════════════════════════════╝
```

---

## 📡 Topics & Message Flow

| Topic | Type | Publisher | Subscribers |
|-------|------|-----------|-------------|
| `/robot_i/odom` | `nav_msgs/Odometry` | Gazebo | `boid_i` |
| `/robot_i/scan` | `sensor_msgs/LaserScan` | Gazebo | `boid_i` |
| `/robot_i/cmd_vel` | `geometry_msgs/Twist` | `boid_i` | Gazebo diff-drive |
| `/robot_i/pose_share` | `geometry_msgs/PoseStamped` | `boid_i` | all other `boid_j`, `flock_monitor` |
| `/robot_i/velocity_share` | `geometry_msgs/TwistStamped` | `boid_i` | all other `boid_j`, `flock_monitor` |
| `/flock/centroid` | `geometry_msgs/PointStamped` | `flock_monitor` | RViz |
| `/flock/convex_hull` | `visualization_msgs/Marker` | `flock_monitor` | RViz |
| `/flock/neighbor_links` | `visualization_msgs/MarkerArray` | `flock_monitor` | RViz |
| `/flock/state` | `swarm_interfaces/FlockState` | `flock_monitor` | analysis / rosbag |

### Custom Message: `FlockState.msg`
```
std_msgs/Header header
geometry_msgs/Point centroid       # flock center of mass
float32 cohesion_radius            # mean distance from centroid
float32 avg_speed                  # mean linear speed (m/s)
int32   num_active_robots
int32   collision_count            # cumulative near-collision events
bool    is_split                   # true if >1 connected component
int32   num_subgroups
```

---

## 📦 Package Structure

```
swarm_flocking_ws/
└── src/
    ├── swarm_interfaces/                  # Custom message definitions
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   └── msg/FlockState.msg
    │
    ├── swarm_flocking/                    # Python flocking package
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/swarm_flocking
    │   ├── swarm_flocking/
    │   │   ├── __init__.py
    │   │   ├── boid_node.py               ← core per-robot node
    │   │   ├── flock_monitor_node.py      ← singleton monitor
    │   │   └── utils/
    │   │       ├── __init__.py
    │   │       ├── reynolds.py            ← pure math module
    │   │       └── obstacle_avoidance.py  ← LaserScan → repulsion
    │   ├── launch/
    │   │   ├── full_sim.launch.py
    │   │   ├── spawn_flock.launch.py
    │   │   └── flocking.launch.py
    │   ├── config/
    │   │   ├── flocking_params.yaml
    │   │   └── rviz_config.rviz
    │   └── test/
    │       ├── test_reynolds.py
    │       └── test_integration.py
    │
    └── swarm_flocking_gazebo/             # Gazebo worlds & models
        ├── package.xml
        ├── CMakeLists.txt
        ├── worlds/
        │   ├── obstacle_course.world
        │   └── open_field.world
        ├── models/
        │   ├── narrow_wall/
        │   └── goal_marker/
        └── launch/gazebo_world.launch.py
```

---

## 🛠️ Installation

### Prerequisites

```bash
# Ubuntu 22.04 LTS
sudo apt update && sudo apt upgrade -y

# ROS 2 Humble (full desktop)
sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-turtlebot3* -y
sudo apt install ros-humble-robot-state-publisher -y
sudo apt install python3-colcon-common-extensions -y

# Python dependencies
pip3 install pytest
```

### Build

```bash
mkdir -p ~/swarm_flocking_ws/src
cd ~/swarm_flocking_ws

# Copy project files into src/ (or clone)
# Then build:
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select swarm_interfaces swarm_flocking swarm_flocking_gazebo

source install/setup.bash
```

### Set TurtleBot3 Model

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 How to Run

### Option 1 — Full Simulation (recommended)

```bash
# Terminal 1: Launch everything
source ~/swarm_flocking_ws/install/setup.bash
ros2 launch swarm_flocking full_sim.launch.py num_robots:=6
```

### Option 2 — Step by Step

```bash
# Terminal 1: Gazebo world
ros2 launch swarm_flocking_gazebo gazebo_world.launch.py

# Terminal 2: Spawn robots + start boid/monitor nodes
ros2 launch swarm_flocking spawn_flock.launch.py num_robots:=6

# Terminal 3: RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix swarm_flocking)/share/swarm_flocking/config/rviz_config.rviz
```

### Monitoring

```bash
# Watch flock state at 2 Hz
ros2 topic echo /flock/state

# Live-tune a weight (dynamic reconfigure via ROS 2 params)
ros2 param set /robot_0/boid_0 w_separation 2.5

# Record for analysis
ros2 bag record -o flocking_run_1 /flock/state /flock/centroid /robot_0/cmd_vel

# Run tests (pure Python, no ROS needed)
cd ~/swarm_flocking_ws
python3 -m pytest src/swarm_flocking/test/ -v
```

---

## ⚙️ Parameter Tuning Guide

All parameters live in `config/flocking_params.yaml`.

| Parameter | Default | Effect | Tuning Range |
|-----------|---------|--------|-------------|
| `w_separation` | 1.5 | Prevent collisions. **Raise first if robots collide.** | 1.0 – 4.0 |
| `w_alignment` | 1.0 | Synchronize headings. Raise for tighter formation. | 0.5 – 2.0 |
| `w_cohesion` | 1.0 | Keep flock compact. Raise if robots scatter. | 0.5 – 2.5 |
| `w_obstacle` | 2.5 | Obstacle avoidance. Must exceed `w_cohesion` for safety. | 2.0 – 6.0 |
| `w_migration` | 0.3 | Goal pull. Too high → robots ignore obstacles to reach goal. | 0.1 – 0.8 |
| `neighbor_radius` | 3.0 m | Sensing range. Larger = more cooperative but higher CPU. | 2.0 – 5.0 |
| `separation_radius` | 0.8 m | Should be > 2× robot body radius (TurtleBot3: 0.178 m). | 0.5 – 1.2 |
| `obstacle_threshold` | 0.6 m | LaserScan trigger. Smaller = robots react later to walls. | 0.4 – 1.0 |
| `max_linear_vel` | 0.20 m/s | TurtleBot3 Burger hardware limit is 0.22 m/s. | ≤ 0.22 |
| `max_angular_vel` | 1.5 rad/s | Higher → faster turning but more oscillation. | 1.0 – 2.0 |

### Recommended Tuning Workflow

1. **Open field**: Start with `open_field.world`. Verify robots flock without oscillation.
2. **Add obstacle world**: Raise `w_obstacle` until robots safely navigate the bottleneck.
3. **Cohesion vs. split**: If flock splits at bottleneck, lower `w_cohesion` slightly so robots spread out naturally.
4. **Speed vs. stability**: Slow down (`max_linear_vel=0.12`) for narrow maze sections if robots oscillate.

---

## 🧠 Algorithm Details

### Reynolds Forces

Each robot computes five force vectors every 100 ms:

```
v_total = w_sep·v_sep + w_ali·v_ali + w_coh·v_coh + w_obs·v_obs + w_mig·v_mig
```

| Force | Formula | Purpose |
|-------|---------|---------|
| **Separation** | `Σ (self - n) / dist²` for n within r_sep | Avoid collisions |
| **Alignment** | `avg(n_vel) - self_vel`, normalized | Match flock heading |
| **Cohesion** | `centroid(neighbours) - self_pos`, normalized | Stay together |
| **Obstacle** | `Σ -obs_dir × (1-dist/thresh) / dist²` from LaserScan | Avoid walls |
| **Migration** | `waypoint - self_pos`, normalized | Reach goal |

### Velocity Conversion

```
desired_θ = atan2(fy, fx)
error_θ   = normalize_angle(desired_θ - robot_θ)
angular_z = 2.5 × error_θ                      (clamped)
linear_x  = force_mag × cos(error_θ) × max_vel (slow when turning)
```

### Low-Pass Filter

```
output_t = α × raw_t + (1 - α) × output_{t-1}   (α = 0.4)
```

Prevents oscillation from rapid force direction changes.

### Split Detection (BFS)

```
1. Build adjacency graph: edge(i,j) if dist(i,j) < neighbor_radius
2. BFS count connected components
3. is_split = (components > 1)
```

---

## 🌍 World Layout

```
Y=15 ┌──────────────────────────────────────────────────────────┐
     │  START         │◄─1.5m─►│  OPEN FIELD  │■■■MAZE■■■│  G  │
     │  (robots)      │BOTTLNCK │              │          │     │
Y=0  └──────────────────────────────────────────────────────────┘
     X=0             8         15             20         27    30
```

- **Start Zone** (X=0–8): Robots spawn in a 3×2 grid around (2–4, 4–6)
- **Bottleneck** (X=8.5): 1.5 m gap forces flock to split naturally
- **Open Field** (X=9–19): Robots regroup via cohesion force
- **Maze** (X=19–27): 5 staggered wall segments, multiple paths
- **Goal Zone** (X=28, Y=7.5): Green disc, migration waypoint

---

## ⚠️ Known Limitations

1. **Odometry drift** — No EKF or SLAM; accumulated error in long runs. Integrate `robot_localization` for production use.
2. **N² topic subscriptions** — With N robots, each boid subscribes to N-1 peers = O(N²) total. Works up to ~15 robots; above that, consider a shared `/swarm/pose_array` topic.
3. **2D only** — Forces computed in XY plane; ignores Z (fine for ground robots).
4. **No safety stop** — If obstacle avoidance fails, no emergency brake is implemented. Add a watchdog node for physical robots.
5. **Static obstacle assumption** — The planner only reacts to LaserScan; dynamic obstacles (other non-flock objects) may cause issues.

---

## 🎬 Demo Behavior Explanation

### Expected Sequence

| Phase | What You See | Why |
|-------|-------------|-----|
| 0–10s | Robots form loose cluster, align headings | Alignment force syncs velocities |
| 10–25s | Compact group moves toward bottleneck | Migration + cohesion |
| 25–40s | **Flock splits** (is_split=True) at bottleneck | Separation force + physical wall forces single-file flow |
| 40–60s | **Flock merges** in open field | Cohesion pulls sub-groups together; `num_subgroups` drops to 1 |
| 60–90s | Navigate staggered maze, brief splits | Obstacle avoidance dominates |
| 90s+ | Cluster arrives at goal zone | Migration force → zero; robots hover at waypoint |

### Key Metrics to Watch

```bash
ros2 topic echo /flock/state --field cohesion_radius     # should stay < 3m normally
ros2 topic echo /flock/state --field is_split            # watch split/merge events
ros2 topic echo /flock/state --field num_subgroups       # 1 = merged, 2+ = split
```

---

## 📊 Evaluation Metrics

Run a full experiment and record:

```bash
ros2 bag record -o experiment_1 \
  /flock/state /flock/centroid \
  /robot_0/odom /robot_1/odom /robot_2/odom \
  /robot_3/odom /robot_4/odom /robot_5/odom
```

Analyze with:
- **Cohesion over time**: `cohesion_radius` — lower = tighter flock
- **Split events**: count transitions `is_split: false → true`
- **Traverse time**: time from `num_active_robots > 0` to all robots at goal
- **Collision rate**: `collision_count` increments per minute

---

## 🔭 Extension Ideas

- **Predator-prey**: Add a fast "predator" robot; boids apply evasion force when it's nearby
- **GA weight tuning**: Optimize `[w_sep, w_ali, w_coh, w_obs, w_mig]` using genetic algorithm on cohesion + collision metrics
- **Heterogeneous speeds**: Mix burger (0.22 m/s) and waffle (0.26 m/s) robots; observe stragglers
- **Nav2 integration**: Replace migration force with a Nav2 global planner path for complex environments

---

## 📄 License

Apache 2.0 — see individual `package.xml` files.
