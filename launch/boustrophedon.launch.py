#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directory
    pkg_clearo = get_package_share_directory('clearo')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    target_distance = LaunchConfiguration('target_distance', default='1.0')
    offset_distance = LaunchConfiguration('offset_distance', default='0.2')
    safety_margin = LaunchConfiguration('safety_margin', default='0.15')
    
    # Launch argument declarations
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    
    declare_target_distance = DeclareLaunchArgument(
        'target_distance',
        default_value='1.0',
        description='Forward movement distance in meters'
    )
    
    declare_offset_distance = DeclareLaunchArgument(
        'offset_distance',
        default_value='0.2',
        description='Offset distance between columns in meters'
    )
    
    declare_safety_margin = DeclareLaunchArgument(
        'safety_margin',
        default_value='0.15',
        description='Safety margin from walls in meters'
    )
    
    # Modular Boustrophedon Controller Node
    boustrophedon_controller = Node(
        package='clearo',
        executable='boustrophedon_controller.py',
        name='boustrophedon_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_distance': target_distance,
            'offset_distance': offset_distance,
            'safety_margin': safety_margin,
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/scan', '/scan'),
            ('/imu', '/imu'),
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_target_distance,
        declare_offset_distance,
        declare_safety_margin,
        boustrophedon_controller,
    ])