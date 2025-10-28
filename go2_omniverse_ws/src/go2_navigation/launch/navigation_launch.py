#!/usr/bin/env python3
"""
Launch Nav2 navigation stack for Go2 robot in Isaac Sim
This launch file starts the full Nav2 stack for autonomous navigation
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    map_yaml_file = LaunchConfiguration('map').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)
    
    # Get package directories
    go2_nav_dir = get_package_share_directory('go2_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration paths
    nav2_params_file = os.path.join(go2_nav_dir, 'config', 'nav2_params.yaml')
    
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
            'target_frame': base_frame,  # Transform to robot base frame
            'transform_tolerance': 0.5,
            'min_height': -0.5,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }]
    )
    
    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
        }.items()
    )
    
    return [pointcloud_to_laserscan_node, nav2_bringup]


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
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # Add opaque function to set up nodes
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld
