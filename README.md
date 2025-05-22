Clearo
A ROS2 package for controlling a floor-cleaning robot to follow a boustrophedon path, forked from Josh Newans' articubot_one project and modified for the Robotics Dojo 2024.
Overview
Clearo is a ROS2 package designed to control a floor-cleaning robot that navigates in a boustrophedon pattern (back-and-forth rows) for efficient cleaning. This project is based on Josh Newans' articubot_one, with modifications to support the Clearo robot's cleaning functionality. See Josh's YouTube playlist for foundational tutorials.
Quick Links

Sources
Hardware Configuration
Software Dependencies
Modified Parameters
Note


Usage
Prerequisites
Simulation
Real Robot
Cleaning Path


Misc.
Future

Sources

diffDriveArduino from RedstoneGithub's fork of Josh's diffdrive_arduino.
sllidar and rplidar.
joy_tester, serial, serial_motor_demo, and ros_arduino_bridge from Josh Newans.

Our technical design paper and technical presentation slides may provide additional details.
Hardware Configuration

Raspberry Pi running Ubuntu 22.04, dev machine running Ubuntu 22.04.
RPLidar A1 M8 connected to Pi via USB.
Arduino Mega running ROSArduinoBridge.ino, connected to Pi USB.
200 RPM motors with built-in encoders, connected as shown in encoder_driver.h.
L298N motor driver, connected as shown in motor_driver.h.

Images of the robot: here, here.
Software Dependencies

Ubuntu 22.04 with ROS2 Humble. Docker on Windows is not recommended due to network issues; native Ubuntu is preferred for performance.
Required packages: twist-mux, ros-humble-teleop-twist-keyboard, ros-humble-teleop-twist-joy, slam-toolbox, ros-humble-navigation2, ros-humble-nav2-bringup, ros-humble-gazebo-ros-pkgs, ros-humble-xacro, ros2-control.
tmux (or similar) is recommended for managing multiple terminals.

Modified Parameters
src/clearo/description
robot_core.xacro and robot.urdf.xacro
Adapted from Josh's code with some elements removed or modified.
gazebo_control.xacro
<wheel_separation>0.275</wheel_separation>
<wheel_diameter>0.085</wheel_diameter>

ros2_control.xacro
<!-- Arduino UNO -->
<param name="device">/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_558343235333516082E0-if00</param>
<param name="baud_rate">57600</param>
<param name="enc_counts_per_rev">1974</param>

src/clearo/config/
mapper_params_online_async.yaml
resolution: 0.025 # (default 0.05)
loop_search_maximum_distance: 6.0 # (default 3.0, prevents "teleportation" during mapping)

nav2_params.yaml

initial_pose: Added to save time during localization. Reset odom, launch online_async_launch.py, then set initial pose as (0, 0, 0) in RViz.
controller_server: Uses MPPI controller (default: DWB).
local_costmap & global_costmap: resolution: 0.025 (default 0.05), robot_radius: 0.025 (default 0.05).
amcl: laser_max_range: 12.0, min_range: 0.3.
velocity_smoother: feedback: "CLOSED_LOOP", velocities: [0.35, 0.0, -0], accelerations: [12.5, 0.0, 8.2].

my_controllers.yaml
wheel_separation: 0.275
wheel_radius: 0.0425

src/clearo/launch/
rplidar.launch.py
"serial_port": "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"

all files in src/sllidar_ros2/launch and src/rplidar_ros/launch
serial_port = LaunchConfiguration('serial_port', default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0') # rplidar

Note

See Josh's [video](https://www.youtube.com/watch?v=4VVrTCnxvSw&list=PLunhqkrRNRhYAffV8JDi cambiato per il progetto Clearo.
online_async_launch.py, localization_launch.py, navigation_launch.py, mapper_params_online_async.yaml, nav2_params.yaml were copied from slam-toolbox and nav2_bringup directories (/opt/ros/humble/share/) and modified (e.g., use get_package_share_directory('clearo') instead of nav2_bringup).
Serial ports use /dev/serial/by-id for reliability.

Usage
Preliminary

Install ROS2 Humble.
Add source /opt/ros/humble/setup.bash to ~/.bashrc.
Install Nav2, SLAM Toolbox, colcon, and optionally set RMW_IMPLEMENTATION to cyclonedds:sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp


Install additional packages:sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-twist-mux


Disable brltty service to free serial ports:systemctl stop brltty-udev.service
sudo systemctl mask brltty-udev.service
systemctl stop brltty.service
systemctl disable brltty.service


Enable serial port access:sudo usermod -a -G dialout $USER

Reboot after this step.
Clone the repository on both Pi and dev machine.
Edit src/clearo/description/ros2_control.xacro line 11 to match your microcontroller’s /dev/serial/by-id (check with ls /dev/serial/by-id).
Build the workspace:colcon build --symlink-install

Ignore initial errors and run again if needed.
Source the workspace:source install/setup.bash



Simulation
Run on PC:
ros2 launch clearo launch_sim.launch.py use_sim_time:=true world:=./src/clearo/worlds/dojo2024

Set use_sim_time:=true in all relevant launch files.
Real Robot

On PC, run RViz:rviz2 -d ./src/comp.rviz


On Pi, launch the robot:ros2 launch clearo launch_robot.launch.py


On Pi, launch RPLidar:ros2 launch clearo rplidar.launch.py

Alternatively, use:ros2 launch rplidar_ros rplidar_a1_launch.py

orros2 launch sllidar_ros2 sllidar_a1_launch.py



For Mapping Phase

On PC (preferred), run SLAM:ros2 launch clearo online_async_launch.py slam_params_file:=./src/clearo/config/mapper_params_online_async.yaml use_sim_time:=false


On PC or Pi, run teleop:ros2 run teleop_twist_keyboard teleop_twist_keyboard

Or configure a gamepad using joy_tester and joystick.yaml (see Josh’s video), then:ros2 launch clearo joystick.launch.py


Drive the robot using u i o j k l m , . (keyboard) or gamepad until a satisfactory map appears in RViz.
In RViz, use the SLAM Toolbox panel to save the map (four files will be saved).

For Navigation Phase

For localization, edit mapper_params_online_async.yaml to set mode: localization and specify map_file_name (path to .data or .posegraph file). Then:ros2 launch clearo online_async_launch.py slam_params_file:=./src/clearo/config/mapper_params_online_async.yaml

Or use:ros2 launch clearo localization_launch.py map:=path_to_map.yaml use_sim_time:=false


On PC (recommended), launch Nav2:ros2 launch clearo navigation_launch.py map_subscribe_transient_local:=true params_file:=./src/clearo/config/nav2_params.yaml use_sim_time:=false


Set waypoints in RViz for navigation.

Cleaning Path
To run the Clearo robot in a boustrophedon cleaning path:
ros2 launch clearo straight_line_waypoints.launch.py row_length:=3.0 num_rows:=8

Adjustable Parameters

row_length: Length of each cleaning pass (default: 2.0m)
lane_width: Width between passes (default: 0.3m, robot’s width)
overlap: Overlap between passes (default: 0.05m or 5cm)
num_rows: Number of rows to clean
points_per_row: Waypoint density (more points = smoother path)

Example:
ros2 launch clearo straight_line_waypoints.launch.py row_length:=3.0 num_rows:=8

Misc.

If RPLidar fails to launch, unplug/replug USB connections or try a different Type-C cable/power supply.
Save RViz config with Ctrl+S.
For simulations, use a powerful PC or split tasks across two PCs on the same network.
Use floats in YAML files to avoid launch failures.
Set ROS_DOMAIN_ID to avoid interference:export ROS_DOMAIN_ID=<unique-number>


Use cyclonedds for better Nav2 compatibility:export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp



Future

Automate the boustrophedon path planning for dynamic environments.
Optimize weaving in straight paths by adjusting angular velocity or controller settings.
Reduce robot size (e.g., wheel separation) for better maneuverability.

Explore high-resolution laser scans and costmaps.
Test alternative controllers (RPP, TEB, Graceful) and planners (SMAC, ThetaStar).
Incorporate IMU or depth camera for enhanced perception.

License
This project is licensed under the same license as Josh Newans' articubot_one. See the LICENSE file for details.
