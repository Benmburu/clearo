# launch/nmpc_controller.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clearo',
            executable='nmpc_path_planner.py',
            name='nmpc_controller',
            output='screen'
        )
    ])