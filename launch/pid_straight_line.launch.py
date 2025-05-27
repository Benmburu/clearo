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
        default_value='1.0',
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
    
    target_heading_arg = DeclareLaunchArgument(
        'target_heading_deg',
        default_value='-999.0',
        description='Target heading in degrees (use -999 to use current heading, or specify 0-360)'
    )
    
    turn_speed_arg = DeclareLaunchArgument(
        'turn_speed',
        default_value='0.3',
        description='Angular velocity for 90-degree turns (rad/s)'
    )
    
    offset_distance_arg = DeclareLaunchArgument(
        'offset_distance',
        default_value='0.2',
        description='Offset distance for boustrophedon pattern (meters)'
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
            'kp_heading': LaunchConfiguration('kp_heading'),
            'target_heading_deg': LaunchConfiguration('target_heading_deg'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'offset_distance': LaunchConfiguration('offset_distance')
        }]
    )

    return LaunchDescription([
        distance_arg,
        speed_arg,
        kp_heading_arg,
        target_heading_arg,
        turn_speed_arg,
        offset_distance_arg,
        pid_controller_node
    ])