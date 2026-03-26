## Project 5: Flocking with Obstacle Navigation (Reynolds + Nav2)

**Difficulty:** Easy | **Tag:** Bio-Inspired, Classic Swarm

### Core Concept
Implement Craig Reynolds' boids model (separation, alignment, cohesion) on real simulated robots to create natural-looking flock movement. The twist: the flock must navigate through an obstacle course with narrow corridors, requiring the flock to dynamically split, navigate, and merge — like birds flowing around buildings.

### Swarm Intelligence Approach
- Reynolds flocking rules (separation, alignment, cohesion)
- Decentralized — each robot applies the three rules locally
- Additional obstacle avoidance rule using LaserScan data

### ROS 2 Architecture

| Component | Detail |
|-----------|--------|
| **Nodes** | `boid_node` (one per robot — computes flocking velocity), `flock_monitor_node` (optional — metrics only) |
| **Topics** | `/robot_i/pose_share` (geometry_msgs/PoseStamped) — broadcast position + heading |
|  | `/robot_i/velocity_share` (geometry_msgs/TwistStamped) — share current velocity |
|  | `/robot_i/cmd_vel` (geometry_msgs/Twist) — output command |
|  | `/robot_i/scan` (sensor_msgs/LaserScan) — for obstacle avoidance |
|  | `/flock/centroid` (geometry_msgs/PointStamped) — computed flock center (for monitoring) |
| **Coordination** | Pure Reynolds rules. Each robot sums: v_sep + v_align + v_coh + v_obstacle + v_goal, normalizes, and publishes as cmd_vel. |

### Simulation Design (Gazebo)
- **Environment:** 30×15m corridor-style world with: wide open start zone → narrow bottleneck → open field → maze section → goal zone
- **Robots:** 6–10 TurtleBot3 Burgers starting in a cluster
- **Obstacles:** Walls forming corridors of varying width (some too narrow for the full flock)
- **Goal:** Waypoint at the far end that provides a gentle global pull (migration urge)

### Visualization (RViz)
- Separation/alignment/cohesion force vectors on each robot (colored ARROW markers)
- Flock centroid and convex hull (LINE_STRIP polygon around all robots)
- Neighbor connections (thin lines between robots within sensing range)
- **Debug:** Per-robot force magnitude text; flock cohesion metric (average inter-robot distance)

### Key Algorithms
- **Separation:** `v_sep = Σ (self - neighbor) / distance²` for neighbors within r_sep
- **Alignment:** `v_align = average(neighbor velocities) - self_velocity`
- **Cohesion:** `v_coh = centroid(neighbors) - self_position`
- **Obstacle avoidance:** Repulsive potential field from LaserScan readings below threshold
- **Migration urge:** Weak constant force toward goal waypoint
- **Weight tuning:** w_sep=1.5, w_align=1.0, w_coh=1.0, w_obs=2.0, w_goal=0.3 (starting values)

### Novelty / Innovation
The flock-splitting-and-merging behavior through narrow passages is visually compelling and non-trivial. Most flocking demos are in open space — adding real obstacle geometry creates emergent bottleneck dynamics.

### Evaluation Metrics
- Flock cohesion over time (mean inter-robot distance)
- Collision count (robot-robot and robot-obstacle)
- Time to traverse full obstacle course
- Split-merge count (how many times the flock fragments and reforms)
- Parameter sensitivity analysis (vary weights, measure flock quality)

### Extension Ideas
- Add predator-prey dynamics (a fast "predator" robot that the flock evades)
- Learn optimal flocking weights via genetic algorithm or Bayesian optimization
- Heterogeneous flock with different robot speeds — test if slow robots get left behind

---