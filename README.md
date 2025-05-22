# Clearo

A ROS2 package for controlling a floor-cleaning robot to follow a boustrophedon path, forked from Josh Newans' [articubot_one project](https://github.com/joshnewans/articubot_one) and modified for the [Robotics Dojo 2024](https://roboticsdojo.github.io/competition2024.html).

## Overview

Clearo is a ROS2 package designed to control a floor-cleaning robot that navigates in a boustrophedon pattern (back-and-forth rows) for efficient cleaning. This project is based on Josh Newans' articubot_one, with modifications to support the Clearo robot's cleaning functionality. See Josh's [YouTube playlist](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT) for foundational tutorials.

## Quick Links
- [Sources](#sources)
- [Hardware Configuration](#hardware-configuration)
- [Software Dependencies](#software-dependencies)
- [Modified Parameters](#modified-parameters)
  - [Note](#note)
- [Usage](#usage)
  - [Prerequisites](#preliminary)
  - [Simulation](#simulation)
  - [Real Robot](#real-robot)
  - [Cleaning Path](#cleaning-path)
- [Misc.](#misc)
- [Future](#future)

# Sources
- diffDriveArduino from [RedstoneGithub's fork](https://github.com/RedstoneGithub/diffdrive_arduino) of Josh's diffdrive_arduino.
- [sllidar](https://github.com/Slamtec/sllidar_ros2) and [rplidar](https://docs.ros.org/en/ros2_packages/humble/api/rplidar_ros/).
- [joy_tester](https://github.com/joshnewans/joy_tester), [serial](https://github.com/joshnewans/serial), [serial_motor_demo](https://github.com/joshnewans/serial_motor_demo), and [ros_arduino_bridge](https://github.com/joshnewans/ros_arduino_bridge) from Josh Newans.

# Hardware Configuration
- Raspberry Pi running Ubuntu 22.04, dev machine running Ubuntu 22.04.
- RPLidar A1 M8 connected to Pi via USB.
- Arduino Mega running [ROSArduinoBridge.ino](arduino-code/ROSArduinoBridge/ROSArduinoBridge.ino), connected to Pi USB.
- 200 RPM motors with built-in encoders, connected as shown in [encoder_driver.h](arduino-code/ROSArduinoBridge/encoder_driver.h).
- L298N motor driver, connected as shown in [motor_driver.h](arduino-code/ROSArduinoBridge/motor_driver.h).

Images of the robot: [here](our-robot1.png), [here](our-robot2.png).

# Software Dependencies
- Ubuntu 22.04 with ROS2 Humble. Docker on Windows is not recommended due to network issues; native Ubuntu is preferred for performance.
- Required packages: `twist-mux`, `ros-humble-teleop-twist-keyboard`, `ros-humble-teleop-twist-joy`, `slam-toolbox`, `ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-gazebo-ros-pkgs`, `ros-humble-xacro`, `ros2-control`.
- `tmux` (or similar) is recommended for managing multiple terminals.

# Modified Parameters
## src/clearo/description
### robot_core.xacro and robot.urdf.xacro
Adapted from Josh's code with some elements removed or modified.

### gazebo_control.xacro
```xml
<wheel_separation>0.275</wheel_separation>
<wheel_diameter>0.085</wheel_diameter>
```

### ros2_control.xacro
```xml
<!-- Arduino Mega -->
<param name="device">/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Mega_2560_12148509806232150650-if00</param>
<param name="baud_rate">57600</param>
<param name="enc_counts_per_rev">988</param>
```

## src/clearo/config/
### mapper_params_online_async.yaml
```yaml
resolution: 0.025 # (default 0.05)
loop_search_maximum_distance: 6.0 # (default 3.0, prevents "teleportation" during mapping)
```

### nav2_params.yaml
- **initial_pose**: Added to save time during localization. Reset odom, launch `online_async_launch.py`, then set initial pose as (0, 0, 0) in RViz.
- **controller_server**: Uses [MPPI controller](https://docs.nav2.org/configuration/packages/configuring-mppic.html) (default: DWB).
- **local_costmap & global_costmap**: `resolution: 0.025` (default 0.05), `robot_radius: 0.025` (default 0.05).
- **amcl**: `laser_max_range: 12.0`, `min_range: 0.3`.
- **velocity_smoother**: `feedback: "CLOSED_LOOP"`, velocities: `[0.35, 0.0, -0]`, accelerations: `[12.5, 0.0, 8.2]`.

### my_controllers.yaml
```yaml
wheel_separation: 0.275
wheel_radius: 0.0425
```

## src/clearo/launch/
### rplidar.launch.py
```python
"serial_port": "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
```

## all files in src/sllidar_ros2/launch and src/rplidar_ros/launch
```python
serial_port = LaunchConfiguration('serial_port', default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0') # rplidar
```

## Note
- See Josh's [video](https://www.youtube.com/watch?v=4VVrTCnxvSw&list=PLunhqkrRNRhYAffV8JDi cambiato per il progetto Clearo.
- **online_async_launch.py**, **localization_launch.py**, **navigation_launch.py**, **mapper_params_online_async.yaml**, **nav2_params.yaml** were copied from `slam-toolbox` and `nav2_bringup` directories (`/opt/ros/humble/share/`) and modified (e.g., use `get_package_share_directory('clearo')` instead of `nav2_bringup`).
- Serial ports use `/dev/serial/by-id` for reliability.

# Usage
## Preliminary
1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
2. Add `source /opt/ros/humble/setup.bash` to `~/.bashrc`.
3. Install [Nav2, SLAM Toolbox](https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/), `colcon`, and optionally set `RMW_IMPLEMENTATION` to `cyclonedds`:
   ```bash
   sudo apt install ros-humble-rmw-cyclonedds-cpp
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
4. Install additional packages:
   ```bash
   sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-twist-mux
   ```
5. Disable `brltty` service to free serial ports:
   ```bash
   systemctl stop brltty-udev.service
   sudo systemctl mask brltty-udev.service
   systemctl stop brltty.service
   systemctl disable brltty.service
   ```
6. Enable serial port access:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   Reboot after this step.
7. Clone the repository on both Pi and dev machine.
8. Edit `src/clearo/description/ros2_control.xacro` line 11 to match your microcontroller’s `/dev/serial/by-id` (check with `ls /dev/serial/by-id`).
9. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```
   Ignore initial errors and run again if needed.
10. Source the workspace:
    ```bash
    source install/setup.bash
    ```

## Simulation
Run on PC:
```bash
ros2 launch clearo launch_sim.launch.py use_sim_time:=true world:=./src/clearo/worlds/dojo2024
```
Set `use_sim_time:=true` in all relevant launch files.

## Real Robot
1. On PC, run RViz:
   ```bash
   rviz2 -d ./src/comp.rviz
   ```
2. On Pi, launch the robot:
   ```bash
   ros2 launch clearo launch_robot.launch.py
   ```
3. On Pi, launch RPLidar:
   ```bash
   ros2 launch clearo rplidar.launch.py
   ```
   Alternatively, use:
   ```bash
   ros2 launch rplidar_ros rplidar_a1_launch.py
   ```
   or
   ```bash
   ros2 launch sllidar_ros2 sllidar_a1_launch.py
   ```

### For Mapping Phase
1. On PC (preferred), run SLAM:
   ```bash
   ros2 launch clearo online_async_launch.py slam_params_file:=./src/clearo/config/mapper_params_online_async.yaml use_sim_time:=false
   ```
2. On PC or Pi, run teleop:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Or configure a gamepad using `joy_tester` and `joystick.yaml` (see Josh’s [video](https://www.youtube.com/watch?v=F5XlNiCKbrY&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=17)), then:
   ```bash
   ros2 launch clearo joystick.launch.py
   ```
3. Drive the robot using `u i o j k l m , .` (keyboard) or gamepad until a satisfactory map appears in RViz.
4. In RViz, use the SLAM Toolbox panel to save the map (four files will be saved).

### For Navigation Phase
1. For localization, edit `mapper_params_online_async.yaml` to set `mode: localization` and specify `map_file_name` (path to `.data` or `.posegraph` file). Then:
   ```bash
   ros2 launch clearo online_async_launch.py slam_params_file:=./src/clearo/config/mapper_params_online_async.yaml
   ```
   Or use:
   ```bash
   ros2 launch clearo localization_launch.py map:=path_to_map.yaml use_sim_time:=false
   ```
2. On PC (recommended), launch Nav2:
   ```bash
   ros2 launch clearo navigation_launch.py map_subscribe_transient_local:=true params_file:=./src/clearo/config/nav2_params.yaml use_sim_time:=false
   ```
3. Set waypoints in RViz for navigation.

## Cleaning Path
To run the Clearo robot in a boustrophedon cleaning path:
```bash
ros2 launch clearo straight_line_waypoints.launch.py row_length:=3.0 num_rows:=8
```

### Adjustable Parameters
- **row_length**: Length of each cleaning pass (default: 2.0m)
- **lane_width**: Width between passes (default: 0.3m, robot’s width)
- **overlap**: Overlap between passes (default: 0.05m or 5cm)
- **num_rows**: Number of rows to clean
- **points_per_row**: Waypoint density (more points = smoother path)

Example:
```bash
ros2 launch clearo straight_line_waypoints.launch.py row_length:=3.0 num_rows:=8
```

# Misc.
- If RPLidar fails to launch, unplug/replug USB connections or try a different Type-C cable/power supply.
- Save RViz config with `Ctrl+S`.
- For simulations, use a powerful PC or split tasks across two PCs on the same network.
- Use floats in YAML files to avoid launch failures.
- Set `ROS_DOMAIN_ID` to avoid interference:
  ```bash
  export ROS_DOMAIN_ID=<unique-number>
  ```
- Use `cyclonedds` for better Nav2 compatibility:
  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```

# Future
- Automate the boustrophedon path planning for dynamic environments.
- Optimize weaving in straight paths by adjusting angular velocity or controller settings.
- Reduce robot size (e.g., wheel separation) for better maneuverability.
- Improve power delivery to RPLidar motor (e.g., modify USB cable for 5V line).
- Explore high-resolution laser scans and costmaps.
- Test alternative controllers ([RPP](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html), [TEB](https://docs.nav2.org/configuration/packages/configuring-teb-controller.html), [Graceful](https://docs.nav2.org/configuration/packages/configuring-graceful-motion-controller.html)) and planners ([SMAC](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html), [ThetaStar](https://docs.nav2.org/configuration/packages/configuring-thetastar.html)).
- Incorporate IMU or depth camera for enhanced perception.
- Develop a simple BMS with MOSFET switch for battery safety.

## License
This project is licensed under the same license as Josh Newans' articubot_one. See the LICENSE file for details.