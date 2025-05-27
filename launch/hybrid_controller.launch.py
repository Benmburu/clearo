from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Existing parameters
        DeclareLaunchArgument('forward_speed', default_value='0.2'),
        DeclareLaunchArgument('kp_heading', default_value='1.0'),
        DeclareLaunchArgument('turn_speed', default_value='0.1'),
        
        # New coverage parameters
        DeclareLaunchArgument(
            'grid_length',
            default_value='2.0',
            description='Length of the area to cover in meters'
        ),
        DeclareLaunchArgument(
            'grid_width',
            default_value='2.0',
            description='Width of the area to cover in meters'
        ),
        DeclareLaunchArgument(
            'grid_columns',
            default_value='4',
            description='Number of columns in the coverage pattern'
        ),
        DeclareLaunchArgument(
            'line_spacing',
            default_value='0.5',
            description='Spacing between parallel lines in meters'
        ),
        
        Node(
            package='clearo',
            executable='hybrid_controller.py',
            name='hybrid_controller',
            output='screen',
            parameters=[{
                'forward_speed': LaunchConfiguration('forward_speed'),
                'kp_heading': LaunchConfiguration('kp_heading'),
                'turn_speed': LaunchConfiguration('turn_speed'),
                'grid_length': LaunchConfiguration('grid_length'),
                'grid_width': LaunchConfiguration('grid_width'),
                'grid_columns': LaunchConfiguration('grid_columns'),
                'line_spacing': LaunchConfiguration('line_spacing'),
            }]
        )
    ])