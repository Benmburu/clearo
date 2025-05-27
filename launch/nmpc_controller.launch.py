# launch/nmpc_controller.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clearo',
            executable='nmpc_controller.py',
            name='nmpc_controller',
            output='screen'
        )
    ])