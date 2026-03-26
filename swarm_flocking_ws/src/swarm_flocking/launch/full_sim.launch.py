#!/usr/bin/env python3
# FILE: launch/full_sim.launch.py
"""
full_sim.launch.py — Master launch file.

Starts:
  1. Gazebo with obstacle_course.world
  2. N TurtleBot3 robots (robot_state_publisher + Gazebo spawn)
  3. N boid_node instances (one per robot)
  4. flock_monitor_node (singleton)
  5. RViz with pre-configured layout

Usage:
    ros2 launch swarm_flocking full_sim.launch.py num_robots:=6
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package share directories
    pkg_flocking = get_package_share_directory('swarm_flocking')
    pkg_gazebo   = get_package_share_directory('swarm_flocking_gazebo')
    pkg_tb3_desc = get_package_share_directory('turtlebot3_description')

    # Shared filepaths
    world_file   = os.path.join(pkg_gazebo, 'worlds', 'obstacle_course.world')
    params_file  = os.path.join(pkg_flocking, 'config', 'flocking_params.yaml')
    rviz_cfg     = os.path.join(pkg_flocking, 'config', 'rviz_config.rviz')

    # Declare arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots', default_value='6',
        description='Number of TurtleBot3 robots to spawn')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) time')

    tb3_model_arg = DeclareLaunchArgument(
        'turtlebot3_model', default_value='burger',
        description='TurtleBot3 model type')

    # Gazebo environment variables
    gazebo_models_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(pkg_gazebo, 'models') + ':' +
        os.environ.get('GAZEBO_MODEL_PATH', ''))

    # Launch Gazebo world (via swarm_flocking_gazebo launch)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo_world.launch.py')),
        launch_arguments={'world': world_file}.items())

    # Spawn robots + boid nodes via OpaqueFunction
    spawn_and_boids = OpaqueFunction(function=_spawn_all)

    # Flock monitor (singleton)
    monitor_node = Node(
        package='swarm_flocking',
        executable='flock_monitor_node',
        name='flock_monitor',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    return LaunchDescription([
        gazebo_models_path,
        num_robots_arg,
        use_sim_time_arg,
        tb3_model_arg,
        gazebo_launch,
        spawn_and_boids,
        monitor_node,
        rviz_node,
    ])


def _spawn_all(context, *args, **kwargs):
    """OpaqueFunction that spawns N robots and their boid nodes."""
    from launch_ros.actions import Node as RosNode
    from launch.actions import TimerAction

    pkg_flocking = get_package_share_directory('swarm_flocking')
    pkg_tb3_desc = get_package_share_directory('turtlebot3_description')
    params_file  = os.path.join(pkg_flocking, 'config', 'flocking_params.yaml')

    num_robots = int(context.launch_configurations.get('num_robots', '6'))
    use_sim_time = context.launch_configurations.get('use_sim_time', 'true')
    tb3_model    = context.launch_configurations.get('turtlebot3_model', 'burger')

    urdf_path = os.path.join(
        pkg_tb3_desc, 'urdf', f'turtlebot3_{tb3_model}.urdf')

    # Grid spawn: 3 columns
    start_x, start_y = 2.0, 4.5
    spacing = 0.7

    actions = []
    for i in range(num_robots):
        row = i // 3
        col = i % 3
        x = start_x + col * spacing
        y = start_y + row * spacing
        ns = f'robot_{i}'

        # robot_state_publisher per robot (unique namespace)
        rsp = RosNode(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            parameters=[{
                'robot_description': _read_urdf(urdf_path),
                'use_sim_time': use_sim_time == 'true',
                'frame_prefix': f'{ns}/',
            }],
            output='screen',
        )

        # Spawn entity in Gazebo
        spawn = RosNode(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', ns,
                '-topic', f'/{ns}/robot_description',
                '-x', str(x),
                '-y', str(y),
                '-z', '0.01',
                '-robot_namespace', f'/{ns}',
            ],
            output='screen',
        )

        # boid_node for this robot (delayed to let Gazebo stabilise)
        boid = TimerAction(
            period=float(5 + i * 0.5),  # stagger launches
            actions=[
                RosNode(
                    package='swarm_flocking',
                    executable='boid_node',
                    name=f'boid_{i}',
                    namespace=ns,
                    parameters=[
                        params_file,
                        {
                            'robot_id': i,
                            'num_robots': num_robots,
                            'use_sim_time': use_sim_time == 'true',
                        },
                    ],
                    remappings=[
                        ('odom', f'/{ns}/odom'),
                        ('scan', f'/{ns}/scan'),
                    ],
                    output='screen',
                ),
            ],
        )

        actions.extend([rsp, spawn, boid])

    return actions


def _read_urdf(urdf_path: str) -> str:
    """Read URDF file content as string (required by robot_state_publisher)."""
    try:
        with open(urdf_path, 'r') as f:
            return f.read()
    except FileNotFoundError:
        # Fallback: return a minimal valid URDF so the node doesn't crash
        return (
            '<?xml version="1.0"?>'
            '<robot name="turtlebot3_burger">'
            '<link name="base_link"/>'
            '</robot>'
        )
