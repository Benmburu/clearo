#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    distance_arg = DeclareLaunchArgument(
        'distance',
        default_value='2.0',
        description='Distance to travel in meters'
    )
    
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='0.2',
        description='Forward speed in m/s'
    )
    
    kp_heading_arg = DeclareLaunchArgument(
        'kp_heading',
        default_value='1.0',
        description='PID gain for heading correction'
    )

    # PID Straight Line Controller Node
    pid_controller_node = Node(
        package='clearo',
        executable='pid_straight_line_controller.py',
        name='pid_straight_line_controller',
        output='screen',
        parameters=[{
            'target_distance': LaunchConfiguration('distance'),
            'forward_speed': LaunchConfiguration('speed'),
            'kp_heading': LaunchConfiguration('kp_heading')
        }]
    )

    return LaunchDescription([
        distance_arg,
        speed_arg,
        kp_heading_arg,
        pid_controller_node
    ])