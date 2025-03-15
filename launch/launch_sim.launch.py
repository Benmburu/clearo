import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Define package name
    package_name = 'clearo'
    print(f"[DEBUG] Package Name: {package_name}")

        # Include the robot state publisher
    rsp_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    print("[DEBUG] Included rsp.launch.py")

    # Get package directory
    pkg_path = os.path.join(get_package_share_directory(package_name))
    print(f"[DEBUG] Package Path: {pkg_path}")

    # Default world path
    # default_world_path = os.path.join(pkg_path, 'worlds', 'empty.world')
    # print(f"[DEBUG] Default World Path: {default_world_path}")

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    default_world = os.path.join(
            get_package_share_directory(package_name),
            'worlds',
            'empty.world'
            )    
        
    world = LaunchConfiguration('world')

    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to the world file'
    )
    print("[DEBUG] Declared world launch argument")

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    print("[DEBUG] Declared use_sim_time launch argument")

    # Launch Gazebo with the specified world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': '-r -v4', 'on_exit_shutdown': 'true'}.items()

    )
    print("[DEBUG] Included gz_sim.launch.py")

    # Give Gazebo some time to start up before spawning the robot
    delay = ExecuteProcess(
        cmd=['sleep', '5'],
        output='screen'
    )
    print("[DEBUG] Added delay before spawning entity")

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-z', '0.1'
        ],
        output='screen'
    )
    print("[DEBUG] Spawn entity node created")
    spawn_delay = TimerAction(period=5.0, actions=[spawn_entity])

    # Launch the controllers
    # controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['diff_cont', 'joint_broad'],
    #     output='screen',
    # )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    print("[DEBUG] Launched the controllers")

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )


    # Return the launch description
    print("[DEBUG] Launch description created, returning...")
    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        rsp_launch,
        gazebo_launch,
        delay,
        # spawn_delay,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
        # spawn_entity,
        # controller_spawner
    ])
