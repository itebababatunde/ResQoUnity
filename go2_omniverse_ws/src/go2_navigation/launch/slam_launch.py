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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    go2_nav_dir = get_package_share_directory('go2_navigation')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Configuration paths
    slam_params_file = os.path.join(go2_nav_dir, 'config', 'slam_toolbox_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_name = LaunchConfiguration('robot_name', default='robot0')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='robot0',
        description='Robot namespace (e.g., robot0, robot1)')
    
    # Pointcloud to LaserScan conversion node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', [robot_name, '/point_cloud2']),
            ('scan', [robot_name, '/scan'])
        ],
        parameters=[{
            'target_frame': 'base_link',
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
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan', [robot_name, '/scan'])
        ],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    
    # Add nodes
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(slam_toolbox_node)
    
    return ld

