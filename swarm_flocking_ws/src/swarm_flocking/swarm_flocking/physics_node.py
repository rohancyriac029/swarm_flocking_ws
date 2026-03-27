#!/usr/bin/env python3
# FILE: swarm_flocking/physics_node.py
"""
PhysicsNode — Lightweight 2D Kinematics Engine.

Responsibilities:
  • Replaces Gazebo by implementing a strict zero-order hold decoupled simulation.
  • Subscribes to all /robot_i/cmd_vel streams.
  • Integrates dt=0.1s kinematic bounds securely.
  • Synthesizes and publishes strict /robot_i/odom arrays and blank /robot_i/scan.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from swarm_flocking.utils.reynolds import normalize_angle, clamp

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

class PhysicsNode(Node):
    def __init__(self):
        super().__init__('physics_node')
        
        self.declare_parameter('num_robots', 6)
        self.declare_parameter('max_linear_vel', 0.22)
        self.declare_parameter('max_angular_vel', 2.8)
        
        self.num_robots = int(self.get_parameter('num_robots').value)
        self.max_lin = float(self.get_parameter('max_linear_vel').value)
        self.max_ang = float(self.get_parameter('max_angular_vel').value)
        
        # Internal kinematic state: x, y, theta, vx, omega
        self.state = {
            i: {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'vx': 0.0, 'omega': 0.0}
            for i in range(self.num_robots)
        }
        
        self.odom_pubs = {}
        self.scan_pubs = {}
        
        for i in range(self.num_robots):
            self.create_subscription(
                Twist, 
                f'/robot_{i}/cmd_vel', 
                lambda msg, rid=i: self._cmd_cb(rid, msg), 
                STATE_QOS
            )
            self.odom_pubs[i] = self.create_publisher(Odometry, f'/robot_{i}/odom', SENSOR_QOS)
            self.scan_pubs[i] = self.create_publisher(LaserScan, f'/robot_{i}/scan', SENSOR_QOS)

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self._physics_step)
        
        self.get_logger().info(f"PhysicsNode booted for {self.num_robots} robots using headless 2D kinematics.")

    def _cmd_cb(self, rid: int, msg: Twist) -> None:
        """Zero-order hold caching of incoming commands."""
        # Safety clamping at the physics layer
        self.state[rid]['vx'] = clamp(msg.linear.x, -self.max_lin, self.max_lin)
        self.state[rid]['omega'] = clamp(msg.angular.z, -self.max_ang, self.max_ang)

    def _physics_step(self) -> None:
        """Synchronous deterministic integration loop."""
        stamp = self.get_clock().now().to_msg()
        
        for i in range(self.num_robots):
            s = self.state[i]
            
            # 1. Closed-loop Kinematic Expansion
            s['x'] += s['vx'] * math.cos(s['theta']) * self.dt
            s['y'] += s['vx'] * math.sin(s['theta']) * self.dt
            s['theta'] = normalize_angle(s['theta'] + s['omega'] * self.dt)
            
            # 2. Publish Standard /odom
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = f'odom_{i}'
            odom.child_frame_id = f'base_footprint_{i}'
            
            odom.pose.pose.position.x = float(s['x'])
            odom.pose.pose.position.y = float(s['y'])
            
            cy = math.cos(s['theta'] * 0.5)
            sy = math.sin(s['theta'] * 0.5)
            odom.pose.pose.orientation.w = float(cy)
            odom.pose.pose.orientation.z = float(sy)
            
            odom.twist.twist.linear.x = float(s['vx'])
            odom.twist.twist.angular.z = float(s['omega'])
            
            self.odom_pubs[i].publish(odom)
            
            # 3. Publish Mock /scan (Infinite Bounds)
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = f'base_scan_{i}'
            scan.angle_min = 0.0
            scan.angle_max = 6.28
            scan.angle_increment = 0.0174
            scan.range_min = 0.12
            scan.range_max = 3.5
            scan.ranges = [float('inf')] * 360
            self.scan_pubs[i].publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PhysicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
