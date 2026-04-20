# Claude Handoff: Swarm Flocking Navigation + Collision/Tumble Debug

## Update (2026-04-20, Open-Field Simplification Applied)

The codebase has now been simplified for a pure swarm coordination baseline:

- Default launch world changed from `obstacle_course.world` to `open_field.world`.
- Internal-obstacle avoidance is now optional and disabled by default (`enable_obstacle_avoidance: false`).
- Flocking loop in `boid_node.py` was simplified by removing adaptive weight scaling from the active control path.
- Goal behavior was simplified: once a robot reaches the final waypoint list entry, it publishes zero `cmd_vel` and holds position.
- Baseline parameters now target cohesive, shared-goal movement in open field.

This means the repository now contains both:
1. The original obstacle-course diagnostic notes below, and
2. A current open-field baseline implementation for "reach goal together" demonstrations.

Date: 2026-04-20
Workspace root: `~/swarm_flocking_ws/swarm_flocking_ws`
Target stack: ROS 2 Humble + Gazebo Classic + TurtleBot3 Burger

## 1) Current Problem Summary

The simulation launches correctly, custom message interfaces are available, and all required topics are present. However, robots still collide/tumble and fail to robustly pass through bottlenecks and reach the final goal.

Observed behavior:
- Robots can reach waypoint 0 (`(8.6, 7.5)`), but then collisions/tumbling occur.
- In some runs they press into walls near bottleneck/maze transitions.
- Average speed reported can be low (`~0.07 m/s`) even with 6 active robots.

Important context:
- This project is mapless by design. It uses waypoint migration + local obstacle avoidance (LaserScan), not Nav2 global planning.

---

## 2) Repository and Package Structure

Main ROS workspace:
- `swarm_flocking_ws/src/swarm_flocking` (Python package)
- `swarm_flocking_ws/src/swarm_flocking_gazebo` (worlds/models/launch)
- `swarm_flocking_ws/src/swarm_interfaces` (custom msg package)

Key files:
- `swarm_flocking_ws/src/swarm_flocking/swarm_flocking/boid_node.py`
- `swarm_flocking_ws/src/swarm_flocking/swarm_flocking/flock_monitor_node.py`
- `swarm_flocking_ws/src/swarm_flocking/swarm_flocking/utils/reynolds.py`
- `swarm_flocking_ws/src/swarm_flocking/swarm_flocking/utils/obstacle_avoidance.py`
- `swarm_flocking_ws/src/swarm_flocking/config/flocking_params.yaml`
- `swarm_flocking_ws/src/swarm_flocking/launch/full_sim.launch.py`
- `swarm_flocking_ws/src/swarm_flocking_gazebo/worlds/obstacle_course.world`
- `swarm_flocking_ws/src/swarm_interfaces/msg/FlockState.msg`

---

## 3) Launch Pipeline and Runtime Composition

### 3.1 Master launch
File: `swarm_flocking_ws/src/swarm_flocking/launch/full_sim.launch.py`

`full_sim.launch.py` starts:
1. Gazebo world via `swarm_flocking_gazebo/launch/gazebo_world.launch.py`
2. Robot state publishers (URDF, TF only)
3. Gazebo spawn of TB3 SDF models (with plugins for odom/scan/cmd_vel)
4. One `boid_node` per robot
5. Single `flock_monitor_node`
6. RViz with provided config

### 3.2 Spawn positions and timing
Spawn grid in `full_sim.launch.py`:
- `start_x=2.0`, `start_y=4.5`, `spacing=0.7`, 3 columns
- For robot `i`: `row=i//3`, `col=i%3`

Spawn positions for `num_robots=6`:
- robot_0: `(2.0, 4.5)`
- robot_1: `(2.7, 4.5)`
- robot_2: `(3.4, 4.5)`
- robot_3: `(2.0, 5.2)`
- robot_4: `(2.7, 5.2)`
- robot_5: `(3.4, 5.2)`

Timing:
- Gazebo ready delay: `30.0s`
- Spawn robot `i` at: `30.0 + 1.5*i`
- Start boid for robot `i` at: `spawn_time + 5.0`

---

## 4) Topics and Interfaces

### 4.1 Required robot topics (per robot i)
- `/robot_i/odom` (nav_msgs/Odometry)
- `/robot_i/scan` (sensor_msgs/LaserScan)
- `/robot_i/cmd_vel` (geometry_msgs/Twist)
- `/robot_i/pose_share` (geometry_msgs/PoseStamped)
- `/robot_i/velocity_share` (geometry_msgs/TwistStamped)

### 4.2 Flock topics
- `/flock/state` (swarm_interfaces/msg/FlockState)
- `/flock/centroid` (geometry_msgs/PointStamped)
- `/flock/convex_hull` (visualization_msgs/Marker)
- `/flock/neighbor_links` (visualization_msgs/MarkerArray)

### 4.3 Custom message contract
File: `swarm_flocking_ws/src/swarm_interfaces/msg/FlockState.msg`

Fields:
- `std_msgs/Header header`
- `geometry_msgs/Point centroid`
- `float32 cohesion_radius`
- `float32 avg_speed`
- `int32 num_active_robots`
- `int32 collision_count`
- `bool is_split`
- `int32 num_subgroups`
- `float32 total_time`
- `float32 collision_rate`
- `float32 mean_cohesion`

---

## 5) Current Parameter Set in Use

File: `swarm_flocking_ws/src/swarm_flocking/config/flocking_params.yaml`

Current values:
- `w_separation: 1.2`
- `w_alignment: 0.8`
- `w_cohesion: 0.9`
- `w_obstacle: 1.8`
- `w_migration: 1.1`
- `neighbor_radius: 2.5`
- `separation_radius: 0.6`
- `obstacle_threshold: 0.55`
- `max_linear_vel: 0.22`
- `max_angular_vel: 1.8`
- adaptive params:
  - `k_sep: 0.4`
  - `min_sep_w: 0.8`
  - `max_sep_w: 2.4`
  - `alpha_coh: 0.15`
  - `min_coh_w: 0.6`
  - `max_coh_w: 1.8`
  - `ideal_separation: 0.8`
  - `threshold_spread: 1.2`
  - `min_obs_w: 0.5`
  - `max_obs_w: 3.0`
  - `laser_epsilon: 0.03`
- frame/lane params:
  - `odom_pose_frame_mode: auto`
  - `waypoint_lane_spread: 0.18`

Current global waypoint list:
- `(8.6, 7.5)`
- `(15.5, 7.5)`
- `(23.0, 9.8)`
- `(27.8, 9.8)`
- `(28.0, 7.5)`

---

## 6) BoidNode Implementation Details

File: `swarm_flocking_ws/src/swarm_flocking/swarm_flocking/boid_node.py`

### 6.1 Core logic per cycle (10 Hz)
Main loop `_flocking_loop()`:
1. Collect valid neighbors (within neighbor radius + not stale)
2. Compute force components:
   - separation: `compute_separation(...)`
   - alignment: `compute_alignment(...)`
   - cohesion: `compute_cohesion(...)`
   - obstacle: `laser_to_repulsive_force(...)`
   - migration: `_get_migration_force(...)`
3. Adaptive scaling:
   - separation weight scales with crowding
   - cohesion weight scales with local spread
   - obstacle weight scales with nearest laser distance
4. Weighted sum:
   - `fx = w_sep*f_sep.x + w_ali*f_ali.x + w_coh*f_coh.x + w_obs*f_obs.x + w_mig*f_mig.x`
   - `fy = w_sep*f_sep.y + w_ali*f_ali.y + w_coh*f_coh.y + w_obs*f_obs.y + w_mig*f_mig.y`
5. Convert resultant force to command via `force_to_cmd_vel(...)`
6. Apply low-pass filter (`LPF_ALPHA=0.4`)
7. Publish `/robot_i/cmd_vel`
8. Advance waypoint when within `WAYPOINT_ARRIVAL_RADIUS`

### 6.2 Important constants and behaviors
- `LPF_ALPHA = 0.4` (adds response lag)
- `WAYPOINT_ARRIVAL_RADIUS = 1.2` (currently large for narrow passages)
- stale timeout: `2.0s`

### 6.3 Odom frame handling
- Declared parameter `odom_pose_frame_mode`: `auto | local | world`
- In `auto`, first odom sample decides whether spawn offset should be applied.
- If local odom: world pose = odom pose + spawn offset.
- If world odom: world pose = odom pose.

### 6.4 Waypoint lane spread
Migration adds lane offset for intermediate waypoints:
- lane index: `(robot_id % 3) - 1` gives `-1, 0, +1`
- adjusted y: `gy += lane_idx * waypoint_lane_spread`
- final waypoint remains shared (no spread)

---

## 7) Reynolds and Obstacle Utilities

### 7.1 Reynolds math
File: `swarm_flocking_ws/src/swarm_flocking/swarm_flocking/utils/reynolds.py`

Provided functions:
- `compute_separation(...)`: inverse-square repulsion for neighbors inside separation radius
- `compute_alignment(...)`: toward average neighbor velocity
- `compute_cohesion(...)`: toward neighbor centroid
- `compute_migration(...)`: normalized vector to target waypoint
- `force_to_cmd_vel(...)`: heading P-controller + cosine alignment speed scaling

Notable detail:
- Linear speed is multiplied by `max(0, cos(heading_error))`, so when heading error is large, robots rotate in place and move slowly.

### 7.2 Obstacle force
File: `swarm_flocking_ws/src/swarm_flocking/swarm_flocking/utils/obstacle_avoidance.py`

Algorithm:
- Process scan rays under threshold
- Compute inverse-square repulsive contribution
- Average over hits
- Rotate force to world frame
- Normalize to unit vector

Notable detail:
- Output is normalized, so only weights determine relative dominance vs other forces.

---

## 8) FlockMonitor Implementation Details

File: `swarm_flocking_ws/src/swarm_flocking/swarm_flocking/flock_monitor_node.py`

Runs at 2 Hz and computes:
- centroid
- cohesion radius
- average speed
- collision_count (new collision pair events)
- split detection via connected components
- convex hull marker
- neighbor link markers
- publishes `FlockState`

Completion condition:
- If all active robots are within `1.2m` of final waypoint, monitor declares completion, sends zero cmd_vel to all robots, logs CSV, and exits.

---

## 9) Obstacle Course Geometry (Exact)

File: `swarm_flocking_ws/src/swarm_flocking_gazebo/worlds/obstacle_course.world`

World envelope:
- Arena approx 30 x 15 meters
- Outer walls:
  - north: pose `(15, 15, 0.5)`, size `(30.4, 0.2, 1.0)`
  - south: pose `(15, 0, 0.5)`, size `(30.4, 0.2, 1.0)`
  - west: pose `(0, 7.5, 0.5)`, size `(0.2, 15.4, 1.0)`
  - east: pose `(30, 7.5, 0.5)`, size `(0.2, 15.4, 1.0)`

Bottleneck at x=8.5:
- lower segment (`bottleneck_south`): pose `(8.5, 3.375, 0.5)`, size `(0.25, 6.75, 1.0)`
- upper segment (`bottleneck_north`): pose `(8.5, 11.625, 0.5)`, size `(0.25, 6.75, 1.0)`
- gap approximately `y in [6.75, 8.25]` (width 1.5m)

Open field divider:
- `divider_south`: pose `(14, 2.5, 0.5)`, size `(0.25, 5.0, 1.0)`

Maze section (x=19..27):
- `maze_h1`: pose `(21, 4.0, 0.5)`, size `(4.0, 0.25, 1.0)`
- `maze_h2`: pose `(23, 11.0, 0.5)`, size `(4.0, 0.25, 1.0)`
- `maze_v1`: pose `(24, 7.5, 0.5)`, size `(0.25, 4.0, 1.0)`
- `maze_v2`: pose `(26.5, 5.0, 0.5)`, size `(0.25, 5.0, 1.0)`
- `maze_h3`: pose `(26, 10.5, 0.5)`, size `(3.0, 0.25, 1.0)`

Goal marker:
- pose `(28, 7.5, 0.01)`, cylinder radius `1.5`

---

## 10) Reproduced Diagnostics from User Session

Confirmed healthy:
- `swarm_interfaces/msg/FlockState` type is valid and visible.
- All required `/robot_i/...` topics and `/flock/...` topics are present.
- Odom/scan/cmd_vel pub-sub wiring is correct (`ros2 topic info -v`).
- Requested params loaded at runtime:
  - `w_migration=1.4`
  - `w_obstacle=1.2`
  - `max_linear_vel=0.22`
  - waypoints list includes `(23.0, 9.8)` and `(27.8, 9.8)`

Still failing behavior:
- Logs show robots reach waypoint 0, then collisions/tumbling happen.
- Example monitor sample from user screenshot:
  - `avg_speed ~ 0.070`
  - `num_active_robots = 6`
  - `collision_count = 0` (at that exact sample)
  - `is_split = false`
  - `num_subgroups = 1`

---

## 11) Likely Root Causes

1. Waypoint arrival radius too large for narrow transitions
- `WAYPOINT_ARRIVAL_RADIUS = 1.2` allows waypoint advancement before robustly clearing bottleneck geometry.
- This can cause corner-cutting and wall contacts immediately after "reached waypoint".

2. No global corridor planner
- Current approach is waypoint vector + local repulsion.
- In tight passages this can produce local minima, oscillation, or wall-pressing.

3. High compression at bottleneck entry
- Six robots spawn in compact cluster and may approach similar lane.
- Even with lane spread, crowding can produce unstable contact dynamics.

4. Dynamic parameter update missing
- `BoidNode` reads parameters once at startup.
- There is no runtime parameter callback (`add_on_set_parameters_callback`), so `ros2 param set` does not actually change behavior immediately.

5. Contact dynamics and heading-speed coupling
- If heading error is high, linear speed drops via cosine factor.
- In clutter, robots can rotate and scrape while making little forward progress.

---

## 12) Requested Implementation Work for Claude (Priority)

### P0: Safety and determinism
1. Parameterize waypoint arrival radius
- Add parameter `waypoint_arrival_radius` in `boid_node.py`.
- Replace hardcoded `WAYPOINT_ARRIVAL_RADIUS` check with this parameter.
- Set default to `0.35` to avoid premature switching in bottleneck.

2. Add runtime parameter callback
- Implement `add_on_set_parameters_callback` in `BoidNode`.
- Support live updates for all key weights and radii.
- Return proper `SetParametersResult`.

3. Add near-obstacle speed limiter
- If nearest valid scan distance `< obstacle_threshold`, scale down linear command:
  - `scale = clamp((d - d_stop)/(obstacle_threshold - d_stop), 0, 1)`
  - `cmd.linear.x *= scale`
- This reduces impact energy and tumble risk.

### P1: Navigation robustness
4. Add progress-gated waypoint transitions
- For each waypoint, optionally require directional progress before increment.
- Example for bottleneck waypoint: robot must satisfy both
  - distance < radius
  - and `x > x_gate` (or crossing test relative to previous waypoint)

5. Add waypoint graph per corridor stage
- Define explicit pre-gap, gap-center, post-gap checkpoints:
  - e.g. `(6.8, 6.9) -> (8.55, 7.5) -> (11.2, 7.5) -> ...`
- Keep lane spread modest and remove spread near very narrow gates.

6. Add deadlock escape behavior
- If speed below threshold for N seconds and obstacle force high:
  - temporary bias along corridor tangent
  - or micro-random heading perturbation

### P2: Debug visibility
7. Publish force component debug markers per robot
- Show sep/aln/coh/obs/mig vectors with color coding.
- Add bool parameter `publish_force_debug`.

8. Monitor split/collision diagnostics
- Extend monitor logs every 2-5s with speed, collision_rate, cohesion, subgroup count.

---

## 13) Concrete File-Level Change Targets

1. `swarm_flocking_ws/src/swarm_flocking/swarm_flocking/boid_node.py`
- Add parameter callback.
- Add configurable waypoint arrival radius.
- Add obstacle-proximity speed clipping.
- Add gated waypoint transition logic.
- Add optional deadlock recovery mode.

2. `swarm_flocking_ws/src/swarm_flocking/config/flocking_params.yaml`
- Add new params:
  - `waypoint_arrival_radius`
  - `near_obstacle_stop_distance`
  - `deadlock_speed_threshold`
  - `deadlock_timeout_s`
  - `publish_force_debug`

3. `swarm_flocking_ws/src/swarm_flocking/swarm_flocking/flock_monitor_node.py`
- Optional periodic status logs.
- Optional explicit monitor rate parameter.

4. `swarm_flocking_ws/src/swarm_interfaces/msg/FlockState.msg` (optional)
- Add fields only if needed for new debug state.
- If changed, rebuild interface package and source overlay.

---

## 14) Validation Plan for Claude

After implementation, run these checks:

1. Interface and params
- `ros2 interface show swarm_interfaces/msg/FlockState`
- `ros2 param list /robot_0/boid_0`
- `ros2 param set /robot_0/boid_0 w_migration 1.6` should take effect immediately

2. Topic graph
- Verify all `/robot_i/{odom,scan,cmd_vel,pose_share,velocity_share}` exist
- Verify `/flock/{state,centroid,convex_hull,neighbor_links}`

3. Behavioral acceptance
- Robots pass bottleneck without repeated wall impact.
- Fewer tumbles/collisions at first wall and maze entry.
- `avg_speed` increases while keeping collisions manageable.
- At least one run reaches final goal zone near `(28, 7.5)`.

4. Repeatability
- Run 3 trials with same params and report:
  - completion time
  - collision_count
  - mean_cohesion
  - split events

---

## 15) Build/Run Notes

- Build interfaces + app when message/schema changes:
  - `colcon build --symlink-install --packages-select swarm_interfaces swarm_flocking swarm_flocking_gazebo`
- Always source same workspace overlay in each terminal:
  - `source /opt/ros/humble/setup.bash`
  - `source ~/swarm_flocking_ws/swarm_flocking_ws/install/setup.bash`
- Keep consistent DDS env:
  - `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
  - `export ROS_DOMAIN_ID=0`

---

## 16) Final Ask to Claude

Please implement a robust anti-wall/tumble navigation update with these explicit goals:
1. No premature waypoint switching in bottlenecks.
2. Safe slowdown near walls/obstacles.
3. Runtime parameter updates without restart.
4. Improved corridor traversal reliability in `obstacle_course.world`.
5. Clear debug outputs to diagnose force balance and deadlocks.
