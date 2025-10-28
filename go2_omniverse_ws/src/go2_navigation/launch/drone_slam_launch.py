#!/usr/bin/env python3
"""
Launch SLAM Toolbox for mapping with Drone in Isaac Sim
This launch file starts:
- SLAM Toolbox for mapping
- Pointcloud to LaserScan converter (optimized for aerial SLAM)
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
    flight_altitude = float(LaunchConfiguration('flight_altitude').perform(context))
    slice_thickness = float(LaunchConfiguration('slice_thickness').perform(context))
    
    # Get package directories
    go2_nav_dir = get_package_share_directory('go2_navigation')
    
    # Configuration paths
    slam_params_file = os.path.join(go2_nav_dir, 'config', 'slam_toolbox_params.yaml')
    
    # Create base_frame with robot namespace
    base_frame = f"{robot_name}/base_link"
    
    # Calculate height slice for drone at altitude
    # We create a horizontal slice at the drone's altitude for 2D mapping
    min_height = -flight_altitude - slice_thickness/2
    max_height = -flight_altitude + slice_thickness/2
    
    print(f"[Drone SLAM] Configuring for flight altitude: {flight_altitude}m")
    print(f"[Drone SLAM] LaserScan height slice: {min_height:.2f}m to {max_height:.2f}m")
    
    # Pointcloud to LaserScan conversion node (drone-optimized)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', f'/{robot_name}/point_cloud2'),
            ('scan', f'/{robot_name}/scan')
        ],
        parameters=[{
            'target_frame': '',  # Empty = pass through, no transform
            'transform_tolerance': 0.01,
            # Height filtering for drone - create horizontal slice at flight altitude
            'min_height': min_height,
            'max_height': max_height,
            # Full 360-degree coverage
            'angle_min': -3.14159,  # -180 degrees
            'angle_max': 3.14159,   # 180 degrees
            'angle_increment': 0.00872,  # ~0.5 degree resolution
            'scan_time': 0.1,
            'range_min': 0.2,  # Ignore very close points
            'range_max': 25.0,  # Longer range for aerial view
            'use_inf': True,
            'inf_epsilon': 1.0,
            # Concurrency for better performance
            'concurrency_level': 1,
        }],
        output='screen'
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
                # Drone-specific tuning
                'use_scan_matching': True,
                'use_scan_barycenter': True,  # Better for sparse aerial data
                'minimum_time_interval': 0.2,  # Slower updates for flying
                'transform_publish_period': 0.02,
                'minimum_travel_distance': 0.3,  # Larger for drone movement
                'minimum_travel_heading': 0.3,
                # Increased ranges for aerial mapping
                'max_laser_range': 25.0,
                'loop_search_maximum_distance': 5.0,
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
    
    declare_flight_altitude_cmd = DeclareLaunchArgument(
        'flight_altitude',
        default_value='2.0',
        description='Drone flight altitude in meters (for height slice calculation)')
    
    declare_slice_thickness_cmd = DeclareLaunchArgument(
        'slice_thickness',
        default_value='1.0',
        description='Thickness of horizontal slice for 2D mapping (meters)')
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_flight_altitude_cmd)
    ld.add_action(declare_slice_thickness_cmd)
    
    # Add opaque function to set up nodes
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld

