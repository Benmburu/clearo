#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import time
from enum import Enum
import casadi as ca
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class ControlMode(Enum):
    PID = 1
    NMPC = 2

class HybridController(Node):
    def __init__(self):
        super().__init__('hybrid_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Add path publisher
        self.path_pub = self.create_publisher(Path, '/hybrid_controller/planned_path', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # PID Parameters - Reduced speed for better accuracy
        self.declare_parameter('forward_speed', 0.15)  # Reduced from 0.2
        self.declare_parameter('kp_heading', 1.5)      # Increased for better heading control
        self.declare_parameter('turn_speed', 0.08)     # Reduced from 0.1
        
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp_heading = self.get_parameter('kp_heading').value
        self.turn_speed = self.get_parameter('turn_speed').value
        
        # NMPC Parameters
        self.obstacle_threshold = 1.2  # Meters
        self.safe_distance = 0.4  # Minimum distance to consider path safe
        
        # Coverage parameters
        self.declare_parameter('grid_length', 2.0)
        self.declare_parameter('grid_width', 2.0)
        self.declare_parameter('grid_columns', 4)
        self.declare_parameter('line_spacing', 0.5)
        
        self.grid_length = self.get_parameter('grid_length').value
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_columns = self.get_parameter('grid_columns').value
        self.line_spacing = self.get_parameter('line_spacing').value
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.obstacles = []
        self.control_mode = ControlMode.PID
        self.odom_received = False
        self.scan_received = False
        
        self.get_logger().info('Hybrid Controller initialized')

    def publish_path(self, waypoints, frame_id='base_link'):
        """Publish path for visualization in RViz"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id
        
        for wp_x, wp_y in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = frame_id
            
            if frame_id == 'base_link':
                # Transform to base_link frame
                dx = wp_x - self.current_x
                dy = wp_y - self.current_y
                
                # Transform to robot's local frame
                local_x = dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
                local_y = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)
                
                pose_stamped.pose.position.x = local_x
                pose_stamped.pose.position.y = local_y
            else:
                # Use global coordinates
                pose_stamped.pose.position.x = wp_x
                pose_stamped.pose.position.y = wp_y
            
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(waypoints)} waypoints in {frame_id} frame')


    def scan_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Convert scan to obstacles in robot frame
        self.obstacles = []
        for r, a in zip(ranges, angles):
            if np.isfinite(r) and 0.1 < r < self.obstacle_threshold:
                x = r * np.cos(a)
                y = r * np.sin(a)
                self.obstacles.append((x, y))
        
        self.scan_received = True

    def is_path_clear(self, target_x, target_y):
        """Check if path to target is clear of obstacles"""
        if not self.obstacles:
            return True
            
        # Create path line from current position to target
        path_points = np.linspace(
            [self.current_x, self.current_y],
            [target_x, target_y],
            num=20
        )
        
        # Check each point along path
        for point in path_points:
            for obs_x, obs_y in self.obstacles:
                # Transform obstacle to map frame
                obs_map_x = self.current_x + obs_x * np.cos(self.current_yaw) - obs_y * np.sin(self.current_yaw)
                obs_map_y = self.current_y + obs_x * np.sin(self.current_yaw) + obs_y * np.cos(self.current_yaw)
                
                # Check distance to obstacle
                dist = np.sqrt((point[0] - obs_map_x)**2 + (point[1] - obs_map_y)**2)
                if dist < self.safe_distance:
                    return False
        
        return True

    def find_safe_intermediate_goal(self, target_x, target_y):
        """Find a safe intermediate goal when obstacle detected"""
        # Search for safe point along path
        path_points = np.linspace(
            [self.current_x, self.current_y],
            [target_x, target_y],
            num=20
        )
        
        for point in path_points:
            if self.is_path_clear(point[0], point[1]):
                return point[0], point[1]
        
        return None, None

    # def move_forward_distance(self, distance):
    #     """Move forward/backward by a specific distance in meters"""
    #     direction = "forward" if distance > 0 else "backward"
    #     self.get_logger().info(f'Moving {direction} {abs(distance):.2f} meters')
        
    #     # Wait for odometry data
    #     if not self.odom_received:
    #         self.get_logger().info('Waiting for odometry data...')
    #         while not self.odom_received and rclpy.ok():
    #             rclpy.spin_once(self, timeout_sec=0.1)

    #     # Record starting position
    #     start_x = self.current_x
    #     start_y = self.current_y
    #     start_yaw = self.current_yaw

    #     twist = Twist()
    #     last_log_time = time.time()

    #     # Set minimum speed to overcome motor stiction
    #     min_speed = 0.08  # Minimum speed to ensure movement
    #     base_speed = 0.15  # Normal movement speed

    #     while rclpy.ok():
    #         # Calculate distance traveled
    #         dx = self.current_x - start_x
    #         dy = self.current_y - start_y
    #         distance_traveled = math.sqrt(dx**2 + dy**2)

    #         # Check if we've reached the target distance
    #         if distance_traveled >= abs(distance):
    #             twist.linear.x = 0.0
    #             twist.angular.z = 0.0
    #             self.cmd_vel_pub.publish(twist)
    #             self.get_logger().info(f'Target distance reached: {distance_traveled:.3f}m')
    #             return True

    #         # Set speed with direction
    #         if abs(distance) - distance_traveled < 0.05:  # Last 5cm
    #             speed = min_speed
    #         else:
    #             speed = base_speed
                
    #         # Apply direction
    #         twist.linear.x = speed if distance > 0 else -speed

    #         # PID heading correction
    #         heading_error = start_yaw - self.current_yaw
    #         heading_error = self.normalize_angle(heading_error)
    #         twist.angular.z = self.kp_heading * heading_error

    #         self.cmd_vel_pub.publish(twist)

    #         # Log progress
    #         current_time = time.time()
    #         if current_time - last_log_time > 0.5:
    #             self.get_logger().info(
    #                 f'Progress: {distance_traveled:.3f}m / {abs(distance):.3f}m '
    #                 f'({100*distance_traveled/abs(distance):.1f}%)'
    #             )
    #             last_log_time = current_time

    #         rclpy.spin_once(self, timeout_sec=0.02)

    def move_forward_distance(self, distance):
        """Move forward/backward by a specific distance with obstacle avoidance"""
        direction = "forward" if distance > 0 else "backward"
        self.get_logger().info(f'Moving {direction} {abs(distance):.2f} meters')
        
        # Wait for odometry and scan data
        if not self.odom_received:
            self.get_logger().info('Waiting for odometry data...')
            while not self.odom_received and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        
        # Wait for scan data
        while not self.scan_received and rclpy.ok():
            self.get_logger().info('Waiting for laser scan data...')
            rclpy.spin_once(self, timeout_sec=0.1)

        # Record starting position
        start_x = self.current_x
        start_y = self.current_y
        start_yaw = self.current_yaw
        
        # Calculate target position
        target_x = start_x + distance * math.cos(start_yaw)
        target_y = start_y + distance * math.sin(start_yaw)

        twist = Twist()
        last_log_time = time.time()
        min_speed = 0.08
        base_speed = 0.15
        obstacle_check_interval = 0.1  # Check for obstacles every 100ms
        last_obstacle_check = 0

        while rclpy.ok():
            current_time = time.time()
            
            # Check for obstacles periodically
            if current_time - last_obstacle_check > obstacle_check_interval:
                if distance > 0 and self.detect_obstacle_ahead():  # Only check when moving forward
                    self.get_logger().info('Obstacle detected! Switching to avoidance mode...')
                    
                    # Stop current movement
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
                    
                    # Calculate remaining distance
                    dx = self.current_x - start_x
                    dy = self.current_y - start_y
                    distance_traveled = math.sqrt(dx**2 + dy**2)
                    remaining_distance = abs(distance) - distance_traveled
                    
                    if remaining_distance > 0.1:  # If significant distance remains
                        # Use obstacle avoidance to reach target
                        if self.navigate_around_obstacle(target_x, target_y):
                            self.get_logger().info('Successfully navigated around obstacle')
                            return True
                        else:
                            self.get_logger().error('Failed to navigate around obstacle')
                            return False
                    else:
                        # Close enough to target
                        self.get_logger().info('Close enough to target, stopping')
                        return True
                        
                last_obstacle_check = current_time

            # Calculate distance traveled
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            distance_traveled = math.sqrt(dx**2 + dy**2)

            # Check if we've reached the target distance
            if distance_traveled >= abs(distance):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f'Target distance reached: {distance_traveled:.3f}m')
                return True

            # Set speed with direction
            if abs(distance) - distance_traveled < 0.05:
                speed = min_speed
            else:
                speed = base_speed
                
            twist.linear.x = speed if distance > 0 else -speed

            # PID heading correction
            heading_error = start_yaw - self.current_yaw
            heading_error = self.normalize_angle(heading_error)
            twist.angular.z = self.kp_heading * heading_error

            self.cmd_vel_pub.publish(twist)

            # Log progress
            if current_time - last_log_time > 0.5:
                self.get_logger().info(
                    f'Progress: {distance_traveled:.3f}m / {abs(distance):.3f}m '
                    f'({100*distance_traveled/abs(distance):.1f}%)'
                )
                last_log_time = current_time

            rclpy.spin_once(self, timeout_sec=0.02)

    def detect_obstacle_ahead(self):
        """Detect if there's an obstacle directly ahead within safe distance"""
        if not self.obstacles:
            return False
        
        # Check for obstacles in front of robot (within a cone)
        for obs_x, obs_y in self.obstacles:
            # Only consider obstacles in front of the robot
            if obs_x > 0 and abs(obs_y) < 0.3:  # 30cm wide detection cone
                distance = math.sqrt(obs_x**2 + obs_y**2)
                if distance < self.safe_distance:
                    return True
        return False

    def generate_avoidance_path(self, target_x, target_y):
        """Generate a path around obstacles using simple geometric planning"""
        # Find the closest obstacle
        closest_obs = None
        min_dist = float('inf')
        
        for obs_x, obs_y in self.obstacles:
            # Transform to map frame
            obs_map_x = self.current_x + obs_x * np.cos(self.current_yaw) - obs_y * np.sin(self.current_yaw)
            obs_map_y = self.current_y + obs_x * np.sin(self.current_yaw) + obs_y * np.cos(self.current_yaw)
            
            dist = math.sqrt((obs_map_x - self.current_x)**2 + (obs_map_y - self.current_y)**2)
            if dist < min_dist and obs_x > 0:  # Only consider obstacles ahead
                min_dist = dist
                closest_obs = (obs_map_x, obs_map_y)
        
        if closest_obs is None:
            return [(target_x, target_y)]
        
        obs_x, obs_y = closest_obs
        
        # Calculate avoidance waypoints
        # Vector from robot to obstacle
        to_obs_x = obs_x - self.current_x
        to_obs_y = obs_y - self.current_y
        
        # Perpendicular vector (for side-stepping)
        perp_x = -to_obs_y
        perp_y = to_obs_x
        perp_length = math.sqrt(perp_x**2 + perp_y**2)
        
        if perp_length > 0:
            perp_x /= perp_length
            perp_y /= perp_length
        
        # Avoidance distance
        avoidance_dist = 0.6  # 60cm to the side
        
        # Generate waypoints: side-step, forward, side-step back
        waypoints = []
        
        # Side-step around obstacle
        side_x = self.current_x + perp_x * avoidance_dist
        side_y = self.current_y + perp_y * avoidance_dist
        waypoints.append((side_x, side_y))
        
        # Move forward past obstacle
        forward_x = side_x + to_obs_x * 1.2  # Go a bit past the obstacle
        forward_y = side_y + to_obs_y * 1.2
        waypoints.append((forward_x, forward_y))
        
        # Return to original path line
        return_x = forward_x - perp_x * avoidance_dist
        return_y = forward_y - perp_y * avoidance_dist
        waypoints.append((return_x, return_y))
        
        # Final target
        waypoints.append((target_x, target_y))

        # Publish the generated path for visualization
        self.publish_path(waypoints, 'base_link')
        
        return waypoints

    def navigate_around_obstacle(self, target_x, target_y):
        """Navigate around obstacle using generated waypoints"""
        self.get_logger().info('Generating avoidance path...')
        
        # Generate avoidance waypoints
        waypoints = self.generate_avoidance_path(target_x, target_y)

        # Publish path for visualization
        self.publish_path(waypoints, 'base_link')
        
        self.get_logger().info(f'Generated {len(waypoints)} waypoints for avoidance')
        
        # Follow each waypoint using PID control
        for i, (wp_x, wp_y) in enumerate(waypoints):
            self.get_logger().info(f'Moving to waypoint {i+1}/{len(waypoints)}: ({wp_x:.2f}, {wp_y:.2f})')
            
            if not self.move_to_goal(wp_x, wp_y):
                self.get_logger().error(f'Failed to reach waypoint {i+1}')
                return False
            
            # Brief pause between waypoints
            time.sleep(0.5)
        
        self.get_logger().info('Obstacle avoidance complete')
        return True

    def move_to_goal(self, target_x, target_y):
        """Move to goal using PID control"""
        self.get_logger().info(f'Moving to goal: ({target_x:.2f}, {target_y:.2f})')
        
        # Use PID for movement
        if not self.execute_pid_movement(target_x, target_y):
            return False
        
        self.get_logger().info('Goal reached!')
        return True

    def execute_pid_movement(self, target_x, target_y):
        """Execute PID-controlled movement"""
        # Wait for odometry data
        if not self.odom_received:
            self.get_logger().info('Waiting for odometry data...')
            while not self.odom_received and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

        # Record starting position
        start_x = self.current_x
        start_y = self.current_y
        start_yaw = self.current_yaw

        # Calculate distance in robot's local frame
        dx = target_x - start_x
        dy = target_y - start_y
        
        # Transform goal to robot's local frame
        local_dx = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
        local_dy = -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)
        
        # Use local distance for movement
        distance = math.sqrt(local_dx**2 + local_dy**2)

        self.get_logger().info(f'Starting straight line movement:')
        self.get_logger().info(f'  From: ({start_x:.2f}, {start_y:.2f})')
        self.get_logger().info(f'  Local distance to travel: {distance:.2f}m')
        self.get_logger().info(f'  Initial heading: {math.degrees(start_yaw):.1f}°')

        twist = Twist()
        start_time = time.time()

        while rclpy.ok():
            # Calculate current distance traveled in robot's local frame
            current_dx = self.current_x - start_x
            current_dy = self.current_y - start_y
            
            # Transform to robot's local frame
            local_current_dx = current_dx * math.cos(start_yaw) + current_dy * math.sin(start_yaw)
            local_current_dy = -current_dx * math.sin(start_yaw) + current_dy * math.cos(start_yaw)
            
            distance_traveled = math.sqrt(local_current_dx**2 + local_current_dy**2)

            # Check if we've reached the target
            if distance_traveled >= distance:
                # Stop the robot
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f'Target distance reached: {distance_traveled:.3f}m')
                return True

            # Set forward velocity
            twist.linear.x = self.forward_speed

            # PID heading correction to maintain straight line
            heading_error = start_yaw - self.current_yaw
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            twist.angular.z = self.kp_heading * heading_error

            # Publish command
            self.cmd_vel_pub.publish(twist)

            # Log progress periodically
            current_time = time.time()
            if current_time - start_time > 1.0:
                self.get_logger().info(
                    f'Distance: {distance_traveled:.3f}m / {distance:.3f}m, '
                    f'Heading error: {math.degrees(heading_error):.1f}°'
                )
                start_time = current_time

            rclpy.spin_once(self, timeout_sec=0.05)

        return False

    def execute_nmpc_movement(self, target_x, target_y):
        """Execute NMPC-controlled movement"""
        # NMPC parameters
        N = 10  # Prediction horizon
        dt = 0.1  # Time step
        max_speed = self.forward_speed
        max_angular_speed = self.turn_speed
        
        # Initialize optimization problem
        opti = ca.Opti()
        
        # Decision variables
        x = opti.variable(3, N+1)  # state trajectory [x, y, theta]
        u = opti.variable(2, N)    # control inputs [v, omega]
        
        # Parameters
        x0 = opti.parameter(3)     # current state
        xref = opti.parameter(3)   # target state
        
        # Set current state
        current_state = [self.current_x, self.current_y, self.current_yaw]
        target_state = [target_x, target_y, self.current_yaw]  # maintain current heading
        
        # Objective function
        cost = 0
        Q = np.diag([1.0, 1.0, 0.1])  # State cost
        R = np.diag([0.1, 0.05])      # Control cost
        
        for k in range(N):
            # State error cost
            state_error = x[:, k] - xref
            cost += ca.mtimes([state_error.T, Q @ state_error])
            
            # Control cost
            control_cost = ca.mtimes([u[:, k].T, R @ u[:, k]])
            cost += control_cost
            
            # Obstacle avoidance cost
            for obs_x, obs_y in self.obstacles:
                dist = ca.sqrt((x[0, k] - obs_x)**2 + (x[1, k] - obs_y)**2)
                cost += 100.0 / (dist + 0.1)  # Repulsive potential
    
        opti.minimize(cost)
        
        # System dynamics constraints
        for k in range(N):
            # Unicycle model
            opti.subject_to(x[:, k+1] == x[:, k] + dt * ca.vertcat(
                u[0, k] * ca.cos(x[2, k]),
                u[0, k] * ca.sin(x[2, k]),
                u[1, k]
            ))
        
        # Input constraints
        opti.subject_to(opti.bounded(-max_speed, u[0, :], max_speed))
        opti.subject_to(opti.bounded(-max_angular_speed, u[1, :], max_angular_speed))
        
        # Initial state constraint
        opti.subject_to(x[:, 0] == x0)
        
        # Create solver
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        opti.solver('ipopt', opts)
        
        # Control loop
        start_time = time.time()
        while rclpy.ok():
            try:
                # Set current state and reference
                opti.set_value(x0, current_state)
                opti.set_value(xref, target_state)
                
                # Solve optimization problem
                sol = opti.solve()
                
                # Extract first control input
                v = sol.value(u[0, 0])
                omega = sol.value(u[1, 0])
                
                # Apply control
                twist = Twist()
                twist.linear.x = v
                twist.angular.z = omega
                self.cmd_vel_pub.publish(twist)
                
                # Check if goal reached
                dist_to_goal = math.sqrt(
                    (self.current_x - target_x)**2 + 
                    (self.current_y - target_y)**2
                )
                if dist_to_goal < 0.05:  # 5cm tolerance
                    # Stop the robot
                    twist = Twist()
                    self.cmd_vel_pub.publish(twist)
                    return True
                    
                # Log progress periodically
                if time.time() - start_time > 1.0:
                    self.get_logger().info(f'Distance to target: {dist_to_goal:.2f}m')
                    start_time = time.time()
                    
                # Update current state
                current_state = [self.current_x, self.current_y, self.current_yaw]
                
            except Exception as e:
                self.get_logger().error(f'NMPC optimization failed: {str(e)}')
                return False
                
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return False

    def generate_coverage_path(self):
        """Generate waypoints for grid coverage pattern"""
        waypoints = []
        
        # Calculate number of rows based on width and spacing
        num_rows = max(2, int(self.grid_width / self.line_spacing))
        
        # Generate boustrophedon pattern
        for i in range(num_rows):
            y = i * self.line_spacing
            
            if i % 2 == 0:
                # Forward pass
                waypoints.append((0.0, y))
                waypoints.append((self.grid_length, y))
            else:
                # Reverse pass
                waypoints.append((self.grid_length, y))
                waypoints.append((0.0, y))
        
        return waypoints

    def odom_callback(self, msg):
        """Process odometry data to update robot's position and orientation"""
        # Get position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Get orientation (yaw) from quaternion
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('Received first odometry message')

    def turn_right_90_degrees(self):
        """Turn exactly 90 degrees to the right"""
        self.get_logger().info('Starting 90-degree right turn...')
        
        # Calculate target heading (90 degrees clockwise from current)
        start_heading = self.current_yaw
        target_heading = self.normalize_angle(start_heading - math.pi*105/180)  # Exact 90 degrees
        
        self.get_logger().info(f'Turn start heading: {math.degrees(start_heading):.1f}°')
        self.get_logger().info(f'Turn target heading: {math.degrees(target_heading):.1f}°')
        
        twist = Twist()
        last_log_time = time.time()
        
        while rclpy.ok():
            # Calculate heading error
            heading_error = target_heading - self.current_yaw
            heading_error = self.normalize_angle(heading_error)
            
            # Check if turn is complete
            if abs(heading_error) < math.radians(2.0):  # Increased tolerance
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
                total_turn = self.normalize_angle(self.current_yaw - start_heading)
                self.get_logger().info(f'Turn completed! Turned {math.degrees(total_turn):.1f}°')
                break
            
            # Use constant turning speed with minimum threshold
            turn_speed = 0.3  # Increased base turning speed
            twist.angular.z = -turn_speed if heading_error < 0 else turn_speed
            self.cmd_vel_pub.publish(twist)
            
            # Log progress periodically
            current_time = time.time()
            if current_time - last_log_time > 0.5:
                current_turn = self.normalize_angle(self.current_yaw - start_heading)
                self.get_logger().info(
                    f'Turning... Current: {math.degrees(self.current_yaw):.1f}°, '
                    f'Turned: {math.degrees(current_turn):.1f}°'
                )
                last_log_time = current_time
            
            rclpy.spin_once(self, timeout_sec=0.02)

    def turn_left_90_degrees(self):
        """Turn exactly 90 degrees to the left"""
        self.get_logger().info('Starting 90-degree left turn...')
        
        # Calculate target heading (90 degrees counterclockwise from current)
        start_heading = self.current_yaw
        target_heading = self.normalize_angle(start_heading + math.pi/2)  # +90 degrees
        
        self.get_logger().info(f'Turn start heading: {math.degrees(start_heading):.1f}°')
        self.get_logger().info(f'Turn target heading: {math.degrees(target_heading):.1f}°')
        
        twist = Twist()
        last_log_time = time.time()
        
        while rclpy.ok():
            # Calculate heading error
            heading_error = target_heading - self.current_yaw
            heading_error = self.normalize_angle(heading_error)
            
            # Check if turn is complete
            if abs(heading_error) < math.radians(2.0):
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
                total_turn = self.normalize_angle(self.current_yaw - start_heading)
                self.get_logger().info(f'Turn completed! Turned {math.degrees(total_turn):.1f}°')
                break
            
            # Use constant turning speed
            turn_speed = 0.3
            twist.angular.z = turn_speed if heading_error > 0 else -turn_speed
            self.cmd_vel_pub.publish(twist)
            
            # Log progress periodically
            current_time = time.time()
            if current_time - last_log_time > 0.5:
                current_turn = self.normalize_angle(self.current_yaw - start_heading)
                self.get_logger().info(
                    f'Turning... Current: {math.degrees(self.current_yaw):.1f}°, '
                    f'Turned: {math.degrees(current_turn):.1f}°'
                )
                last_log_time = current_time
            
            rclpy.spin_once(self, timeout_sec=0.02)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    controller = HybridController()
    
    try:
        # Wait for odometry
        while not controller.odom_received and rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.1)
            
        controller.get_logger().info('=== Starting Boustrophedon Pattern ===')
        
        # Number of rows to cover
        num_rows = 3  # Adjust this for coverage area
        current_row = 0
        
        while current_row < num_rows and rclpy.ok():
            # Move forward 1 meter
            controller.get_logger().info(f'Row {current_row + 1}: Forward movement (1.0m)')
            if not controller.move_forward_distance(1.0):
                controller.get_logger().error('Failed to complete forward movement')
                break
                
            time.sleep(1.0)
            
            # Skip turning on last row
            if current_row >= num_rows - 1:
                break
                
            # First turn (alternate between right and left)
            if current_row % 2 == 0:
                controller.get_logger().info('Turning right...')
                controller.turn_right_90_degrees()
            else:
                controller.get_logger().info('Turning left...')
                controller.turn_left_90_degrees()
                
            time.sleep(1.0)
            
            # Move backward 10cm for overlap
            controller.get_logger().info('Backward movement for overlap (0.1m)')
            if not controller.move_forward_distance(-0.1):
                controller.get_logger().error('Failed to complete backward movement')
                break
                
            time.sleep(1.0)
            
            # Second turn (same direction as first)
            if current_row % 2 == 0:
                controller.get_logger().info('Turning right...')
                controller.turn_right_90_degrees()
            else:
                controller.get_logger().info('Turning left...')
                controller.turn_left_90_degrees()
                
            time.sleep(1.0)
            current_row += 1
        
        controller.get_logger().info('=== Boustrophedon Pattern Complete ===')
        
    except KeyboardInterrupt:
        controller.get_logger().info('Navigation interrupted by user')
    finally:
        # Stop the robot
        twist = Twist()
        controller.cmd_vel_pub.publish(twist)
        # Clean shutdown
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()