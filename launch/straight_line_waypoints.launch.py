from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    row_length_arg = DeclareLaunchArgument(
        'row_length',
        default_value='2.0',
        description='Length of each row in meters'
    )
    
    lane_width_arg = DeclareLaunchArgument(
        'lane_width',
        default_value='0.3',
        description='Distance between rows in meters'
    )
    
    num_rows_arg = DeclareLaunchArgument(
        'num_rows',
        default_value='5',
        description='Number of rows to cover'
    )
    
    overlap_arg = DeclareLaunchArgument(
        'overlap',
        default_value='0.05',
        description='Overlap between passes in meters'
    )
    
    points_per_row_arg = DeclareLaunchArgument(
        'points_per_row',
        default_value='10',
        description='Number of waypoints per row'
    )
    
    # Create node with parameters from launch arguments
    waypoint_follower_node = Node(
        package='clearo',
        executable='straight_line_waypoints.py',  # Keep the filename the same unless you rename it
        name='boustrophedon_waypoint_follower',
        output='screen',
        parameters=[{
            'row_length': LaunchConfiguration('row_length'),
            'lane_width': LaunchConfiguration('lane_width'),
            'num_rows': LaunchConfiguration('num_rows'),
            'overlap': LaunchConfiguration('overlap'),
            'points_per_row': LaunchConfiguration('points_per_row')
        }]
    )
    
    return LaunchDescription([
        row_length_arg,
        lane_width_arg,
        num_rows_arg,
        overlap_arg,
        points_per_row_arg,
        waypoint_follower_node
    ])