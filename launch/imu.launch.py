#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Get launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # IMU Node
    imu_node = Node(
        package='clearo',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/imu/data', '/imu/data'),
        ]
    )
    
    # Static transform publisher for IMU
    imu_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_publisher',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        imu_node,
        imu_transform,
    ])