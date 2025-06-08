#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    
    # Get URDF file path - use the complete robot description file
    urdf_file = PathJoinSubstitution([
        FindPackageShare('custom_crx5ia_description'),
        'urdf',
        'crx5ia_robot_mapping.xacro'
    ])
    
    # Process URDF to get robot_description
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/livox/pointcloud',
        description='Input point cloud topic from Livox Mid-70'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic', 
        default_value='/livox/pointcloud_filtered',
        description='Output filtered point cloud topic'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_self_filter'),
            'params',
            'example.yaml'
        ]),
        description='Path to self filter configuration file'
    )
    
    sensor_frame_arg = DeclareLaunchArgument(
        'sensor_frame',
        default_value='livox_frame',
        description='Sensor frame for the LiDAR'
    )

    # Self filter node with proper parameters
    self_filter_node = Node(
        package='robot_self_filter',
        executable='self_filter',
        name='self_filter',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'robot_description': robot_description_content,
                'sensor_frame': LaunchConfiguration('sensor_frame'),
                'in_pointcloud_topic': LaunchConfiguration('input_topic'),
                'lidar_sensor_type': 0,  # XYZ sensor type for Livox (intensity field different)
                'use_rgb': False,
                'max_queue_size': 10
            }
        ],
        remappings=[
            ('cloud_out', LaunchConfiguration('output_topic')),
        ],
        output='screen'
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg, 
        config_file_arg,
        sensor_frame_arg,
        self_filter_node
    ]) 