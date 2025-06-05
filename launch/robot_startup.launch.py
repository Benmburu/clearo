#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the package directory
    clearo_pkg = FindPackageShare('clearo')
    
    # Define the launch files to include
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([clearo_pkg, 'launch', 'launch_robot.launch.py'])
        ])
    )
    
    # Add a small delay before starting RPLidar to ensure robot core is ready
    rplidar = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([clearo_pkg, 'launch', 'rplidar.launch.py'])
                ])
            )
        ]
    )
    
    # Add another delay for IMU
    imu = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([clearo_pkg, 'launch', 'imu.launch.py'])
                ])
            )
        ]
    )
    
    # Add delay for joystick
    joystick = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([clearo_pkg, 'launch', 'joystick.launch.py'])
                ])
            )
        ]
    )
    
    # Start web server after all other services are running
    web_server = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='clearo',
                executable='robot_web_server.py',
                name='robot_web_interface',
                output='screen',
                parameters=[{
                    'port': 5000,
                    'host': '0.0.0.0'
                }]
            )
        ]
    )
    
    return LaunchDescription([
        launch_robot,
        rplidar,
        imu,
        joystick,
        web_server,
    ])