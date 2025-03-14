# type: ignore

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package.
    package_name='roomba'
    
    # Get package directory
    pkg_path = os.path.join(get_package_share_directory(package_name))
    
    # Default world path
    default_world_path = os.path.join(pkg_path, 'worlds', 'empty.world')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=default_world_path)
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Path to the world file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Include the robot state publisher
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': 'true'}.items()
    )
    
    # Launch Gazebo with the specified world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )
    
    # Give Gazebo some time to start up before spawning the robot
    delay = ExecuteProcess(
        cmd=['sleep', '5'],
        output='screen'
    )
    
    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'roomba',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Launch the controllers
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', 'joint_broad'],
        output='screen',
    )
    
    # Return the launch description
    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        rsp_launch,
        gazebo_launch,
        delay,
        spawn_entity,
        controller_spawner
    ])