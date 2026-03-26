# Swarm Robots — Full Implementation Plans

**Top 2 Projects selected for depth:**
1. **Flocking with Obstacle Navigation** (Easy — start here)
2. **Ant Colony Foraging & Pheromone Trails** (Medium — build on flocking skills)

---
---

# PROJECT A: Flocking with Obstacle Navigation

## 1. Package Structure

```
swarm_flocking_ws/
└── src/
    ├── swarm_flocking/                    # Main Python package
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/
    │   │   └── swarm_flocking
    │   ├── swarm_flocking/
    │   │   ├── __init__.py
    │   │   ├── boid_node.py              # Core: one instance per robot
    │   │   ├── flock_monitor_node.py     # Metrics + centroid publisher
    │   │   └── utils/
    │   │       ├── __init__.py
    │   │       ├── reynolds.py           # Separation, alignment, cohesion math
    │   │       └── obstacle_avoidance.py # LaserScan → repulsive force
    │   ├── launch/
    │   │   ├── spawn_flock.launch.py     # Spawn N robots in Gazebo
    │   │   ├── flocking.launch.py        # Start all boid nodes
    │   │   └── full_sim.launch.py        # Gazebo + RViz + spawn + flocking
    │   ├── config/
    │   │   ├── flocking_params.yaml      # Tunable weights and thresholds
    │   │   └── rviz_config.rviz          # Pre-configured RViz layout
    │   └── test/
    │       ├── test_reynolds.py          # Unit tests for force calculations
    │       └── test_integration.py       # Full 3-robot smoke test
    │
    ├── swarm_flocking_gazebo/             # Gazebo world + models
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── worlds/
    │   │   ├── obstacle_course.world     # Main corridor world
    │   │   └── open_field.world          # Simple test world
    │   ├── models/
    │   │   ├── narrow_wall/              # Bottleneck wall segments
    │   │   │   ├── model.config
    │   │   │   └── model.sdf
    │   │   └── goal_marker/              # Visual goal zone
    │   │       ├── model.config
    │   │       └── model.sdf
    │   └── launch/
    │       └── gazebo_world.launch.py
    │
    └── swarm_interfaces/                  # Custom message definitions
        ├── package.xml
        ├── CMakeLists.txt
        └── msg/
            └── FlockState.msg            # Aggregated flock metrics
```

## 2. Configuration File — `flocking_params.yaml`

```yaml
# config/flocking_params.yaml
flocking:
  # Number of robots to spawn
  num_robots: 6

  # === Reynolds Weights ===
  w_separation: 1.5       # Highest priority — avoid collisions
  w_alignment: 1.0        # Match neighbor heading
  w_cohesion: 1.0         # Stay near flock center
  w_obstacle: 2.5         # Reactive obstacle dodge
  w_migration: 0.3        # Gentle pull toward goal

  # === Sensing Ranges (meters) ===
  neighbor_radius: 3.0    # Max distance to consider another robot a "neighbor"
  separation_radius: 0.8  # Distance below which separation force activates
  obstacle_threshold: 0.6 # LaserScan range that triggers avoidance

  # === Motion Limits ===
  max_linear_vel: 0.20    # m/s (TurtleBot3 burger max is 0.22)
  max_angular_vel: 1.5    # rad/s
  
  # === Goal Waypoints (sequential) ===
  waypoints:
    - [12.0, 1.0]         # Enter corridor
    - [12.0, 7.0]         # Through bottleneck
    - [12.0, 13.0]        # Final goal zone
```

## 3. Custom Message — `FlockState.msg`

```
# swarm_interfaces/msg/FlockState.msg
std_msgs/Header header
geometry_msgs/Point centroid           # Flock center of mass
float32 cohesion_radius                # Average distance from centroid
float32 avg_speed                      # Mean linear velocity
int32 num_active_robots                # Robots currently flocking
int32 collision_count                  # Cumulative collisions detected
bool is_split                          # True if flock fragmented into subgroups
```

## 4. Core Node Pseudocode — `boid_node.py`

```python
#!/usr/bin/env python3
"""
boid_node.py — One instance per robot.
Subscribes to all neighbors' poses/velocities,
computes Reynolds forces + obstacle avoidance,
publishes cmd_vel.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class BoidNode(Node):
    def __init__(self):
        super().__init__('boid_node')
        
        # --- Parameters ---
        self.declare_parameters_from_yaml('flocking')   # Load from YAML
        self.robot_id      = self.get_parameter('robot_id').value
        self.num_robots    = self.get_parameter('flocking.num_robots').value
        self.w_sep         = self.get_parameter('flocking.w_separation').value
        self.w_ali         = self.get_parameter('flocking.w_alignment').value
        self.w_coh         = self.get_parameter('flocking.w_cohesion').value
        self.w_obs         = self.get_parameter('flocking.w_obstacle').value
        self.w_mig         = self.get_parameter('flocking.w_migration').value
        self.neighbor_r    = self.get_parameter('flocking.neighbor_radius').value
        self.sep_r         = self.get_parameter('flocking.separation_radius').value
        self.obs_thresh    = self.get_parameter('flocking.obstacle_threshold').value
        self.max_lin       = self.get_parameter('flocking.max_linear_vel').value
        self.max_ang       = self.get_parameter('flocking.max_angular_vel').value
        self.waypoints     = self.get_parameter('flocking.waypoints').value
        self.current_wp    = 0

        # --- State ---
        self.my_pose = None          # (x, y, theta)
        self.my_velocity = None      # (vx, vy)
        self.neighbor_poses = {}     # {robot_id: (x, y, theta)}
        self.neighbor_vels  = {}     # {robot_id: (vx, vy)}
        self.latest_scan = None      # LaserScan data

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(
            Twist, f'/robot_{self.robot_id}/cmd_vel', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, f'/robot_{self.robot_id}/pose_share', 10)
        self.vel_pub = self.create_publisher(
            TwistStamped, f'/robot_{self.robot_id}/velocity_share', 10)

        # --- Subscribers ---
        # Own odometry (convert to pose)
        self.create_subscription(
            PoseStamped, f'/robot_{self.robot_id}/odom_pose',
            self.odom_callback, 10)

        # LaserScan for obstacles
        self.create_subscription(
            LaserScan, f'/robot_{self.robot_id}/scan',
            self.scan_callback, 10)

        # Subscribe to ALL other robots' poses and velocities
        for i in range(self.num_robots):
            if i != self.robot_id:
                self.create_subscription(
                    PoseStamped, f'/robot_{i}/pose_share',
                    lambda msg, rid=i: self.neighbor_pose_callback(rid, msg), 10)
                self.create_subscription(
                    TwistStamped, f'/robot_{i}/velocity_share',
                    lambda msg, rid=i: self.neighbor_vel_callback(rid, msg), 10)

        # --- Main Loop at 10 Hz ---
        self.timer = self.create_timer(0.1, self.flocking_loop)

    # =============================================
    #  CALLBACKS
    # =============================================

    def odom_callback(self, msg):
        """Store own position and heading from odometry."""
        self.my_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            yaw_from_quaternion(msg.pose.orientation)
        )

    def scan_callback(self, msg):
        """Store latest laser scan."""
        self.latest_scan = msg

    def neighbor_pose_callback(self, robot_id, msg):
        """Store neighbor's latest position."""
        self.neighbor_poses[robot_id] = (
            msg.pose.position.x,
            msg.pose.position.y,
            yaw_from_quaternion(msg.pose.orientation)
        )

    def neighbor_vel_callback(self, robot_id, msg):
        """Store neighbor's latest velocity."""
        self.neighbor_vels[robot_id] = (
            msg.twist.linear.x,
            msg.twist.linear.y
        )

    # =============================================
    #  MAIN FLOCKING LOOP (runs at 10 Hz)
    # =============================================

    def flocking_loop(self):
        if self.my_pose is None:
            return

        # Step 1: Broadcast own pose and velocity to neighbors
        self.publish_own_state()

        # Step 2: Identify neighbors within sensing range
        neighbors = self.get_neighbors_in_range()

        # Step 3: Compute each Reynolds force component
        f_sep = self.compute_separation(neighbors)
        f_ali = self.compute_alignment(neighbors)
        f_coh = self.compute_cohesion(neighbors)
        f_obs = self.compute_obstacle_force()
        f_mig = self.compute_migration_force()

        # Step 4: Weighted sum of all forces
        fx = (self.w_sep * f_sep[0] +
              self.w_ali * f_ali[0] +
              self.w_coh * f_coh[0] +
              self.w_obs * f_obs[0] +
              self.w_mig * f_mig[0])

        fy = (self.w_sep * f_sep[1] +
              self.w_ali * f_ali[1] +
              self.w_coh * f_coh[1] +
              self.w_obs * f_obs[1] +
              self.w_mig * f_mig[1])

        # Step 5: Convert (fx, fy) world-frame force → (linear_vel, angular_vel)
        cmd = self.force_to_cmd_vel(fx, fy)

        # Step 6: Clamp and publish
        cmd.linear.x  = clamp(cmd.linear.x, -self.max_lin, self.max_lin)
        cmd.angular.z  = clamp(cmd.angular.z, -self.max_ang, self.max_ang)
        self.cmd_pub.publish(cmd)

        # Step 7: Check waypoint advancement
        self.check_waypoint_progress()

    # =============================================
    #  REYNOLDS FORCE CALCULATIONS
    # =============================================

    def get_neighbors_in_range(self):
        """Return list of (id, x, y, vx, vy) for robots within neighbor_radius."""
        neighbors = []
        mx, my, _ = self.my_pose
        for rid, (nx, ny, _) in self.neighbor_poses.items():
            dist = math.hypot(nx - mx, ny - my)
            if dist < self.neighbor_r and dist > 0.01:
                vx, vy = self.neighbor_vels.get(rid, (0.0, 0.0))
                neighbors.append((rid, nx, ny, vx, vy, dist))
        return neighbors

    def compute_separation(self, neighbors):
        """
        SEPARATION: Steer away from neighbors that are too close.
        
        For each neighbor within separation_radius:
            vector = my_position - neighbor_position
            weight = 1 / distance^2  (stronger when closer)
            accumulate weighted vectors
        Normalize result.
        """
        fx, fy = 0.0, 0.0
        mx, my, _ = self.my_pose
        count = 0

        for (rid, nx, ny, _, _, dist) in neighbors:
            if dist < self.sep_r:
                # Vector pointing AWAY from neighbor
                dx = mx - nx
                dy = my - ny
                # Inverse-square weighting: closer = stronger push
                fx += dx / (dist * dist)
                fy += dy / (dist * dist)
                count += 1

        if count > 0:
            fx /= count
            fy /= count
            fx, fy = normalize(fx, fy)

        return (fx, fy)

    def compute_alignment(self, neighbors):
        """
        ALIGNMENT: Steer toward average heading of neighbors.
        
        Average all neighbor velocities.
        Subtract own velocity.
        Result is the "correction" to match the flock's direction.
        """
        if not neighbors:
            return (0.0, 0.0)

        avg_vx = sum(vx for (_, _, _, vx, _, _) in neighbors) / len(neighbors)
        avg_vy = sum(vy for (_, _, _, _, vy, _) in neighbors) / len(neighbors)

        # Difference between flock average and my velocity
        my_vx = self.my_velocity[0] if self.my_velocity else 0.0
        my_vy = self.my_velocity[1] if self.my_velocity else 0.0

        fx = avg_vx - my_vx
        fy = avg_vy - my_vy
        fx, fy = normalize(fx, fy)
        return (fx, fy)

    def compute_cohesion(self, neighbors):
        """
        COHESION: Steer toward the centroid of nearby neighbors.
        
        Compute centroid of all neighbor positions.
        Return vector from my position toward centroid.
        """
        if not neighbors:
            return (0.0, 0.0)

        cx = sum(nx for (_, nx, _, _, _, _) in neighbors) / len(neighbors)
        cy = sum(ny for (_, _, ny, _, _, _) in neighbors) / len(neighbors)

        mx, my, _ = self.my_pose
        fx = cx - mx
        fy = cy - my
        fx, fy = normalize(fx, fy)
        return (fx, fy)

    def compute_obstacle_force(self):
        """
        OBSTACLE AVOIDANCE: Repulsive force from LaserScan.
        
        For each laser ray below obstacle_threshold:
            Compute ray's (x, y) endpoint in robot frame
            Add repulsive vector pointing AWAY from that point
            Weight by inverse distance (closer = stronger)
        Transform to world frame and return.
        """
        if self.latest_scan is None:
            return (0.0, 0.0)

        fx, fy = 0.0, 0.0
        scan = self.latest_scan

        for i, distance in enumerate(scan.ranges):
            if distance < self.obs_thresh and distance > scan.range_min:
                # Angle of this laser ray
                angle = scan.angle_min + i * scan.angle_increment

                # Point in robot frame where obstacle is
                obs_x = distance * math.cos(angle)
                obs_y = distance * math.sin(angle)

                # Repulsive force: AWAY from obstacle, inverse distance
                repulsion = 1.0 / (distance * distance)
                fx -= obs_x * repulsion
                fy -= obs_y * repulsion

        # Transform from robot frame to world frame
        _, _, theta = self.my_pose
        world_fx = fx * math.cos(theta) - fy * math.sin(theta)
        world_fy = fx * math.sin(theta) + fy * math.cos(theta)

        world_fx, world_fy = normalize(world_fx, world_fy)
        return (world_fx, world_fy)

    def compute_migration_force(self):
        """
        MIGRATION: Gentle constant pull toward current waypoint.
        
        Vector from my position toward waypoint, normalized.
        This gives the flock a shared goal direction.
        """
        if self.current_wp >= len(self.waypoints):
            return (0.0, 0.0)

        gx, gy = self.waypoints[self.current_wp]
        mx, my, _ = self.my_pose
        fx = gx - mx
        fy = gy - my
        fx, fy = normalize(fx, fy)
        return (fx, fy)

    # =============================================
    #  VELOCITY CONVERSION
    # =============================================

    def force_to_cmd_vel(self, fx, fy):
        """
        Convert world-frame force vector (fx, fy) to
        differential-drive commands (linear.x, angular.z).
        
        1. Compute desired heading: atan2(fy, fx)
        2. Heading error = desired_heading - current_heading
        3. angular.z = proportional to heading error
        4. linear.x  = proportional to force magnitude,
                        reduced when heading error is large
                        (slow down while turning)
        """
        cmd = Twist()
        _, _, theta = self.my_pose

        desired_heading = math.atan2(fy, fx)
        heading_error = normalize_angle(desired_heading - theta)
        force_magnitude = math.hypot(fx, fy)

        # Proportional angular control
        cmd.angular.z = 2.0 * heading_error

        # Linear speed: full speed when aligned, slow when turning
        alignment_factor = max(0.0, math.cos(heading_error))
        cmd.linear.x = force_magnitude * alignment_factor * self.max_lin

        return cmd


# =============================================
#  UTILITY FUNCTIONS
# =============================================

def normalize(x, y, epsilon=1e-6):
    mag = math.hypot(x, y)
    if mag < epsilon:
        return (0.0, 0.0)
    return (x / mag, y / mag)

def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))

def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def yaw_from_quaternion(q):
    """Extract yaw from geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = BoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 5. Flock Monitor Node — `flock_monitor_node.py`

```python
"""
flock_monitor_node.py — Subscribes to all robot poses,
computes flock-wide metrics, publishes for RViz.
"""

class FlockMonitorNode(Node):
    def __init__(self):
        super().__init__('flock_monitor')
        self.num_robots = ...  # from params
        self.robot_poses = {}
        
        # Subscribe to all robots
        for i in range(self.num_robots):
            self.create_subscription(
                PoseStamped, f'/robot_{i}/pose_share',
                lambda msg, rid=i: self.pose_cb(rid, msg), 10)
        
        # Publishers for RViz visualization
        self.centroid_pub   = self.create_publisher(PointStamped, '/flock/centroid', 10)
        self.hull_pub       = self.create_publisher(Marker, '/flock/convex_hull', 10)
        self.links_pub      = self.create_publisher(MarkerArray, '/flock/neighbor_links', 10)
        self.state_pub      = self.create_publisher(FlockState, '/flock/state', 10)
        
        self.timer = self.create_timer(0.5, self.compute_metrics)  # 2 Hz
    
    def compute_metrics(self):
        """
        PSEUDOCODE:
        
        1. Collect all known robot positions
        2. Compute centroid = mean(all positions)
        3. Compute cohesion_radius = mean(distance from each robot to centroid)
        4. Compute average speed from velocity topics
        5. Detect flock splits:
             - Build adjacency graph (edge if dist < neighbor_radius)
             - Count connected components
             - is_split = (components > 1)
        6. Publish:
             - Centroid as PointStamped
             - Convex hull as LINE_STRIP marker
             - Neighbor links as thin line MarkerArray
             - FlockState message with all metrics
        """
        pass  # Implementation follows pseudocode above
```

## 6. Launch File — `spawn_flock.launch.py`

```python
"""
spawn_flock.launch.py — Spawns N TurtleBot3 robots in Gazebo
with unique namespaces and initial positions.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import math

def generate_robot_spawns(context):
    num_robots = int(context.launch_configurations['num_robots'])
    actions = []
    
    # Arrange robots in a cluster at the start zone
    start_x, start_y = 2.0, 1.0
    spacing = 0.6  # meters between robots
    
    for i in range(num_robots):
        # Grid layout: 3 columns
        row = i // 3
        col = i % 3
        x = start_x + col * spacing
        y = start_y + row * spacing
        
        # Spawn robot model in Gazebo
        spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', f'robot_{i}',
                '-topic', f'/robot_{i}/robot_description',
                '-x', str(x),
                '-y', str(y),
                '-z', '0.01',
                '-robot_namespace', f'/robot_{i}'
            ],
            output='screen'
        )
        
        # Start boid node for this robot
        boid = Node(
            package='swarm_flocking',
            executable='boid_node',
            namespace=f'robot_{i}',
            name=f'boid_{i}',
            parameters=[{
                'robot_id': i,
            }, 'config/flocking_params.yaml'],
            output='screen'
        )
        
        actions.extend([spawn, boid])
    
    # Start monitor node (one instance)
    monitor = Node(
        package='swarm_flocking',
        executable='flock_monitor_node',
        name='flock_monitor',
        parameters=['config/flocking_params.yaml'],
        output='screen'
    )
    actions.append(monitor)
    
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='6'),
        OpaqueFunction(function=generate_robot_spawns),
    ])
```

## 7. Gazebo World — `obstacle_course.world` (structure)

```xml
<!-- worlds/obstacle_course.world -->
<!-- Layout:
     
     START ZONE (open)    BOTTLENECK    OPEN FIELD    MAZE     GOAL
     [  robots  ]  →  | narrow gap |  →  [open]  →  |walls|  → [G]
     
     0m          6m    7m        9m   10m       18m  19m  27m  28m 30m
-->
<sdf version='1.7'>
  <world name='obstacle_course'>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>

    <!-- Outer boundary walls (30m x 15m) -->
    <model name='wall_north'>
      <static>true</static>
      <pose>15 15 0.5 0 0 0</pose>
      <link name='link'>
        <collision name='c'><geometry><box><size>30 0.2 1.0</size></box></geometry></collision>
        <visual name='v'><geometry><box><size>30 0.2 1.0</size></box></geometry></visual>
      </link>
    </model>
    <!-- ... south, east, west walls similarly ... -->

    <!-- Bottleneck: two walls with 1.5m gap at y=7.5 -->
    <model name='bottleneck_left'>
      <static>true</static>
      <pose>8 5.0 0.5 0 0 0</pose>
      <link name='link'>
        <collision name='c'><geometry><box><size>0.2 10.0 1.0</size></box></geometry></collision>
        <visual name='v'><geometry><box><size>0.2 10.0 1.0</size></box></geometry></visual>
      </link>
    </model>
    <model name='bottleneck_right'>
      <static>true</static>
      <pose>8 11.5 0.5 0 0 0</pose>
      <link name='link'>
        <collision name='c'><geometry><box><size>0.2 7.0 1.0</size></box></geometry></collision>
        <visual name='v'><geometry><box><size>0.2 7.0 1.0</size></box></geometry></visual>
      </link>
    </model>

    <!-- Maze section: scattered box obstacles -->
    <model name='maze_box_1'>
      <static>true</static>
      <pose>22 4 0.5 0 0 0</pose>
      <link name='link'>
        <collision name='c'><geometry><box><size>2.0 0.2 1.0</size></box></geometry></collision>
        <visual name='v'><geometry><box><size>2.0 0.2 1.0</size></box></geometry></visual>
      </link>
    </model>
    <!-- ... more maze walls ... -->

    <!-- Goal zone visual marker -->
    <model name='goal_marker'>
      <static>true</static>
      <pose>28 7.5 0.01 0 0 0</pose>
      <link name='link'>
        <visual name='v'>
          <geometry><cylinder><radius>1.5</radius><length>0.02</length></cylinder></geometry>
          <material><ambient>0 1 0 0.5</ambient></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```



| Week | Tasks |
|------|-------|
| **1** | Set up workspace. Spawn multi-robot in Gazebo. Verify namespaced topics (`/robot_0/cmd_vel`, etc.). Build Gazebo world. |
| **2** | Implement `reynolds.py` (separation + alignment + cohesion). Test in open field. Tune weights. |
| **3** | Add obstacle avoidance from LaserScan. Add migration force. Test in obstacle_course world. Implement flock_monitor for RViz. |
| **4** | Parameter tuning. Record rosbag data. Generate evaluation plots. Document results. |

## 9. Key Commands to Run

```bash
# Terminal 1 — Launch everything
ros2 launch swarm_flocking full_sim.launch.py num_robots:=6

# Terminal 2 — Monitor flock state
ros2 topic echo /flock/state

# Terminal 3 — Live-tune a weight (dynamic reconfigure)
ros2 param set /robot_0/boid_0 flocking.w_separation 2.0

# Terminal 4 — Record data for analysis
ros2 bag record /flock/state /flock/centroid -o flocking_experiment_1
```

---
---

