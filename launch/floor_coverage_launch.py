#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    start_x_arg = DeclareLaunchArgument(
        'start_x',
        default_value='0.0',
        description='Starting X coordinate for coverage area'
    )
    
    start_y_arg = DeclareLaunchArgument(
        'start_y', 
        default_value='0.0',
        description='Starting Y coordinate for coverage area'
    )
    
    area_width_arg = DeclareLaunchArgument(
        'area_width',
        default_value='3.0',
        description='Width of area to cover in meters'
    )
    
    area_height_arg = DeclareLaunchArgument(
        'area_height',
        default_value='2.0', 
        description='Height of area to cover in meters'
    )
    
    row_spacing_arg = DeclareLaunchArgument(
        'row_spacing',
        default_value='0.25',
        description='Spacing between cleaning rows in meters'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for waypoints'
    )
    
    # Coverage controller node
    coverage_node = Node(
        package='clearo',
        executable='floor_coverage_controller.py',
        name='floor_coverage_controller',
        output='screen',
        parameters=[{
            'start_x': LaunchConfiguration('start_x'),
            'start_y': LaunchConfiguration('start_y'),
            'area_width': LaunchConfiguration('area_width'),
            'area_height': LaunchConfiguration('area_height'),
            'row_spacing': LaunchConfiguration('row_spacing'),
            'frame_id': LaunchConfiguration('frame_id'),
        }]
    )
    
    return LaunchDescription([
        start_x_arg,
        start_y_arg,
        area_width_arg,
        area_height_arg,
        row_spacing_arg,
        frame_id_arg,
        coverage_node
    ])