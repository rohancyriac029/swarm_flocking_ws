import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    num_robots = 6
    waypoints = [12.0, 1.0, 12.0, 7.0, 12.0, 13.0]

    # Spawn configuration mimicking Gazebo offsets.
    # physics_node starts at (0,0) locally, boid_node adds the offset to get world frame.
    spawn_coords = [
        (-2.0, -0.5), (-2.0,  0.5),
        (-3.0, -0.5), (-3.0,  0.5),
        (-4.0, -0.5), (-4.0,  0.5)
    ]

    # 1. Physics Engine (Headless)
    physics_node = Node(
        package='swarm_flocking',
        executable='physics_node',
        name='physics_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'num_robots': num_robots,
        }]
    )
    ld.add_action(physics_node)

    # 2. Monitor Node (Experiment Manager)
    monitor_node = Node(
        package='swarm_flocking',
        executable='flock_monitor_node',
        name='flock_monitor_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'num_robots': num_robots,
            'waypoints': waypoints,
        }]
    )
    ld.add_action(monitor_node)

    # 3. Decentralized Boid Nodes
    for i in range(num_robots):
        sx, sy = spawn_coords[i]
        boid_node = Node(
            package='swarm_flocking',
            executable='boid_node',
            name=f'boid_node_{i}',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_id': i,
                'num_robots': num_robots,
                'spawn_x': sx,
                'spawn_y': sy,
                'waypoints': waypoints,
                # Adaptive logic leverages defaults specified in boid_node.py
            }]
        )
        ld.add_action(boid_node)

    return ld
