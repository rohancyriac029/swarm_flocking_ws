#!/usr/bin/env python3
"""
coordination_sim.launch.py

Explicit coordination-only launch profile:
- Uses coordination_empty.world (no internal obstacles, no boundary walls)
- Reuses full_sim.launch.py pipeline for robot spawning, boid nodes, monitor, and RViz

Usage:
  ros2 launch swarm_flocking coordination_sim.launch.py num_robots:=6
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_flocking = get_package_share_directory('swarm_flocking')
    pkg_gazebo = get_package_share_directory('swarm_flocking_gazebo')

    world_file = os.path.join(pkg_gazebo, 'worlds', 'coordination_empty.world')
    full_sim = os.path.join(pkg_flocking, 'launch', 'full_sim.launch.py')

    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='6',
        description='Number of TurtleBot3 robots to spawn')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) time')

    turtlebot3_model_arg = DeclareLaunchArgument(
        'turtlebot3_model',
        default_value='burger',
        description='TurtleBot3 model type')

    include_full_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(full_sim),
        launch_arguments={
            'num_robots': LaunchConfiguration('num_robots'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'turtlebot3_model': LaunchConfiguration('turtlebot3_model'),
            'world': world_file,
        }.items(),
    )

    return LaunchDescription([
        num_robots_arg,
        use_sim_time_arg,
        turtlebot3_model_arg,
        include_full_sim,
    ])
