#!/usr/bin/env python3
"""
Launch SLAM Toolbox for mapping with Go2 robot in Isaac Sim
This launch file starts:
- SLAM Toolbox for mapping
- Pointcloud to LaserScan converter
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    
    # Get package directories
    go2_nav_dir = get_package_share_directory('go2_navigation')
    
    # Configuration paths
    slam_params_file = os.path.join(go2_nav_dir, 'config', 'slam_toolbox_params.yaml')
    
    # Create base_frame with robot namespace
    base_frame = f"{robot_name}/base_link"
    
    # Pointcloud to LaserScan conversion node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', f'/{robot_name}/point_cloud2'),
            ('scan', f'/{robot_name}/scan')
        ],
        parameters=[{
            'target_frame': '',  # Empty = pass through, no transform at all
            'transform_tolerance': 0.01,
            'min_height': -0.5,
            'max_height': 2.0,
            'angle_min': -3.14159,  # -180 degrees
            'angle_max': 3.14159,   # 180 degrees
            'angle_increment': 0.0087,  # 0.5 degree
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }]
    )
    
    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time == 'true',
                'base_frame': base_frame,  # e.g., robot0/base_link
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': f'/{robot_name}/scan',
                'use_scan_matching': True,
                'use_scan_barycenter': False,
                'minimum_time_interval': 0.1,
                'transform_publish_period': 0.05,
            }
        ],
        remappings=[
            ('scan', f'/{robot_name}/scan')
        ],
        output='screen'
    )
    
    return [pointcloud_to_laserscan_node, slam_toolbox_node]


def generate_launch_description():
    # Launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='robot0',
        description='Robot namespace (e.g., robot0, robot1)')
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    
    # Add opaque function to set up nodes
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld
