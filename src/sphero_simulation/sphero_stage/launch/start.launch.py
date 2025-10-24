#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 launch file for sphero stage simulation.
Replicates the functionality of the original ROS 1 start.py script.
"""

import os
import sys
import math
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def distribute_circle(k, n, center_x=0, center_y=0, radius=1):
    """Distribute robots in a circle formation."""
    x = radius * math.cos(k / n * 2 * math.pi) + center_x
    y = radius * math.sin(k / n * 2 * math.pi) + center_y
    return x, y


def distribute_line(k, n, center_x=0, center_y=0, separation=0.5, direction='horizontal'):
    """Distribute robots in a line formation."""
    if direction == 'horizontal':
        x = center_x + (k - (n - 1) / 2) * separation
        y = center_y
    else:
        x = center_x
        y = center_y + (k - (n - 1) / 2) * separation
    return x, y


def create_world_file(context, *args, **kwargs):
    """Create world file from template and configuration."""
    # Get package share directory
    package_share = FindPackageShare('sphero_stage').perform(context)

    # Load configuration
    config_path = os.path.join(package_share, 'launch', 'launch_params.yaml')
    with open(config_path, 'r') as stream:
        config = yaml.safe_load(stream)

    map_name = config['map_name']
    num_of_robots = config['num_of_robots']
    distribution = config['distribution']
    params = config['distribution_list'][distribution]

    # File paths
    template_file = os.path.join(package_share, 'config', 'world_templates', f'{map_name}.temp')
    worlds_dir = os.path.join(package_share, 'config', 'worlds')
    new_map_file = os.path.join(worlds_dir, f'{map_name}_{num_of_robots}.world')

    # Create worlds directory if it doesn't exist
    os.makedirs(worlds_dir, exist_ok=True)

    if not os.path.isfile(template_file):
        print(f"\033[31mUnable to create files - missing template: {template_file}\033[0m")
        return []

    proto_line = "robot ( pose [ {x:.3f} {y:.3f} 0 0.000 ] name \"robot_{k}\" color \"{color}\")\n"

    with open(new_map_file, 'w') as output, open(template_file, 'r') as temp:
        output.write(temp.read())

        for k in range(num_of_robots):
            if distribution == 'circle':
                x, y = distribute_circle(k, num_of_robots, **params)
            elif distribution == 'line':
                x, y = distribute_line(k, num_of_robots, **params)
            args = {'x': x, 'y': y, 'k': k, 'color': 'blue'}
            output.write(proto_line.format(**args))

    print(f"\033[36mCreated world file with {num_of_robots} robots and map '{map_name}'.\033[0m")

    # Set launch configurations for use by other nodes
    return [
        SetLaunchConfiguration('num_of_robots', str(num_of_robots)),
        SetLaunchConfiguration('map_name', map_name),
        SetLaunchConfiguration('map_world', new_map_file),
        SetLaunchConfiguration('map_yaml', os.path.join(package_share, 'config', 'maps', map_name, f'{map_name}.yaml'))
    ]


def launch_setup(context, *args, **kwargs):
    """Setup launch configuration and return nodes."""

    # Get launch configurations
    map_world = LaunchConfiguration('map_world')
    map_yaml = LaunchConfiguration('map_yaml')
    start_rviz = LaunchConfiguration('start_rviz')

    # Package paths
    sphero_stage_share = FindPackageShare('sphero_stage')
    sphero_description_share = FindPackageShare('sphero_description')

    nodes = []

    # Map server node (ROS 2 equivalent)
    nodes.append(
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': True
            }],
            output='screen'
        )
    )

    # Map server lifecycle manager
    nodes.append(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            parameters=[{
                'node_names': ['map_server'],
                'use_sim_time': True,
                'autostart': True
            }],
            output='screen'
        )
    )

    # Stage simulator node
    nodes.append(
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='simulator',
            parameters=[
                {'world_file': map_world},
                {'enforce_prefixes': False},
                {'one_tf_tree': True},
            ],
            output='screen'
        )
    )

    # Static TF broadcasters for all robots
    num_robots_int = int(context.launch_configurations['num_of_robots'])
    for i in range(num_robots_int):
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_publisher_robot_{i}',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--yaw', '0', '--pitch', '0', '--roll', '0',
                    '--frame-id', 'map', '--child-frame-id', f'robot_{i}/odom']
            )
        )

    # Robot state publisher (for URDF)
    # We'll use Command to read the URDF file content
    urdf_file_path = PathJoinSubstitution([sphero_description_share, 'urdf', 'simple_ball.urdf'])
    robot_description = Command(['cat ', urdf_file_path])

    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            condition=IfCondition(start_rviz),
            output='screen'
        )
    )

    # RViz node
    rviz_config = PathJoinSubstitution([sphero_stage_share, 'launch', 'sphero_sim.rviz'])

    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(start_rviz),
            output='screen'
        )
    )

    return nodes


def generate_launch_description():
    """Generate the launch description."""

    # Check for --rviz argument
    start_rviz_default = 'true' if '--rviz' in sys.argv else 'false'

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'start_rviz',
            default_value=start_rviz_default,
            description='Whether to start RViz'
        ),

        # Create world file and set configurations
        OpaqueFunction(function=create_world_file),

        # Launch nodes
        OpaqueFunction(function=launch_setup)
    ])