#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time
import numpy as np


class PIDStraightLineController(Node):
    def __init__(self):
        super().__init__('pid_straight_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Parameters
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('kp_heading', 1.0)
        self.declare_parameter('target_heading_deg', -999.0)
        self.declare_parameter('turn_speed', 0.1)  # Angular velocity for turns
        self.declare_parameter('offset_distance', 0.2)  # 20cm offset
        
        # Obstacle avoidance parameters
        self.declare_parameter('obstacle_threshold', 0.5)  # Distance to consider as obstacle
        self.declare_parameter('safety_margin', 0.15)  # Safety margin from obstacles
        self.declare_parameter('triangle_side_distance', 0.4)  # Distance for triangular detour
        self.declare_parameter('scan_angle_range', 30.0)  # Degrees to scan ahead
        
        self.target_distance = self.get_parameter('target_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp_heading = self.get_parameter('kp_heading').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.offset_distance = self.get_parameter('offset_distance').value
        
        # Obstacle avoidance parameters
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.triangle_side_distance = self.get_parameter('triangle_side_distance').value
        self.scan_angle_range = math.radians(self.get_parameter('scan_angle_range').value)
        
        # Handle target heading parameter
        target_heading_param = self.get_parameter('target_heading_deg').value
        if isinstance(target_heading_param, str) and target_heading_param.strip() == '':
            self.target_heading_deg = None
        elif target_heading_param == -999.0:
            self.target_heading_deg = None
        else:
            try:
                self.target_heading_deg = float(target_heading_param)
            except (ValueError, TypeError):
                self.get_logger().warn(f'Invalid target_heading_deg: {target_heading_param}, using current heading')
                self.target_heading_deg = None
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.start_x = None
        self.start_y = None
        self.target_yaw = None
        self.odom_received = False
        self.last_log_time = 0.0
        
        # Laser scan data
        self.scan_data = None
        self.scan_received = False
        
        self.get_logger().info(f'Boustrophedon Controller with Obstacle Avoidance initialized:')
        self.get_logger().info(f'  Target distance: {self.target_distance}m')
        self.get_logger().info(f'  Forward speed: {self.forward_speed}m/s')
        self.get_logger().info(f'  Turn speed: {self.turn_speed}rad/s')
        self.get_logger().info(f'  Offset distance: {self.offset_distance}m')
        self.get_logger().info(f'  Heading PID gain: {self.kp_heading}')
        self.get_logger().info(f'  Obstacle threshold: {self.obstacle_threshold}m')
        self.get_logger().info(f'  Safety margin: {self.safety_margin}m')
        self.get_logger().info(f'  Triangle side distance: {self.triangle_side_distance}m')
        
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        
        if not self.odom_received:
            self.odom_received = True
            self.start_x = self.current_x
            self.start_y = self.current_y
            
            # Set target heading
            if self.target_heading_deg is not None:
                self.target_yaw = math.radians(self.target_heading_deg)
                self.get_logger().info(f'Using specified target heading: {self.target_heading_deg:.1f} degrees')
            else:
                self.target_yaw = self.current_yaw
                self.get_logger().info(f'Using current heading as target: {math.degrees(self.target_yaw):.1f} degrees')
            
            self.get_logger().info(f'Starting position: ({self.start_x:.2f}, {self.start_y:.2f})')
            self.get_logger().info(f'Current heading: {math.degrees(self.current_yaw):.1f} degrees')
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        self.scan_data = msg
        self.scan_received = True
    
    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def check_path_clear(self, distance, heading=None):
        """
        Check if the path ahead is clear of obstacles
        Args:
            distance: Distance to check ahead (meters)
            heading: Direction to check (radians), defaults to current heading
        Returns:
            tuple: (is_clear, obstacle_distance)
        """
        if not self.scan_received or self.scan_data is None:
            self.get_logger().warn('No scan data available')
            return True, float('inf')
        
        if heading is None:
            heading = self.current_yaw
        
        # Convert heading to scan frame (assuming scan frame aligns with robot base)
        target_angle = self.normalize_angle(heading - self.current_yaw)
        
        # Find the corresponding scan indices
        angle_min = self.scan_data.angle_min
        angle_max = self.scan_data.angle_max
        angle_increment = self.scan_data.angle_increment
        
        # Check if target angle is within scan range
        if target_angle < angle_min or target_angle > angle_max:
            self.get_logger().warn(f'Target angle {math.degrees(target_angle):.1f}° outside scan range')
            return True, float('inf')
        
        # Get range of indices to check
        center_index = int((target_angle - angle_min) / angle_increment)
        angle_range_indices = int(self.scan_angle_range / (2 * angle_increment))
        
        start_idx = max(0, center_index - angle_range_indices)
        end_idx = min(len(self.scan_data.ranges), center_index + angle_range_indices + 1)
        
        min_distance = float('inf')
        
        for i in range(start_idx, end_idx):
            range_val = self.scan_data.ranges[i]
            
            # Skip invalid readings
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            
            if range_val < min_distance:
                min_distance = range_val
        
        is_clear = min_distance > distance + self.safety_margin
        
        self.get_logger().info(f'Path check: distance={distance:.2f}m, min_obstacle={min_distance:.2f}m, clear={is_clear}')
        
        return is_clear, min_distance
    
    def find_safe_detour_point(self, obstacle_distance, original_heading):
        """
        Find a safe point for the triangular detour
        Args:
            obstacle_distance: Distance to the obstacle
            original_heading: Original heading before detour
        Returns:
            tuple: (safe_x, safe_y, angle_to_point) or None if no safe point found
        """
        # Calculate position just before obstacle
        safe_forward_dist = max(0.1, obstacle_distance - self.safety_margin - 0.1)
        obstacle_x = self.current_x + safe_forward_dist * math.cos(original_heading)
        obstacle_y = self.current_y + safe_forward_dist * math.sin(original_heading)
        
        # Try detour angles (prefer left side, then right side)
        detour_angles = [math.pi/3, math.pi/4, math.pi/6, -math.pi/3, -math.pi/4, -math.pi/6]  # 60°, 45°, 30° left then right
        
        for detour_angle in detour_angles:
            # Calculate detour point
            detour_heading = self.normalize_angle(original_heading + detour_angle)
            detour_x = obstacle_x + self.triangle_side_distance * math.cos(detour_heading)
            detour_y = obstacle_y + self.triangle_side_distance * math.sin(detour_heading)
            
            # Check if this point is safe
            angle_to_detour = math.atan2(detour_y - self.current_y, detour_x - self.current_x)
            distance_to_detour = math.sqrt((detour_x - self.current_x)**2 + (detour_y - self.current_y)**2)
            
            is_clear, _ = self.check_path_clear(distance_to_detour, angle_to_detour)
            
            if is_clear:
                self.get_logger().info(f'Found safe detour point at ({detour_x:.2f}, {detour_y:.2f}), angle: {math.degrees(detour_angle):.1f}°')
                return detour_x, detour_y, angle_to_detour
        
        self.get_logger().warn('No safe detour point found!')
        return None
    
    def navigate_to_point(self, target_x, target_y, description=""):
        """
        Navigate to a specific point using PID control
        Args:
            target_x, target_y: Target coordinates
            description: Description for logging
        """
        if description:
            self.get_logger().info(f'Navigating to point: {description} ({target_x:.2f}, {target_y:.2f})')
        
        twist = Twist()
        
        while rclpy.ok():
            # Process callbacks for fresh data
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Calculate distance and angle to target
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance_to_target = math.sqrt(dx**2 + dy**2)
            angle_to_target = math.atan2(dy, dx)
            
            # Check if we've reached the target
            if distance_to_target < 0.05:  # 5cm tolerance
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f'Reached target point: {distance_to_target:.3f}m away')
                break
            
            # Calculate heading error
            heading_error = angle_to_target - self.current_yaw
            heading_error = self.normalize_angle(heading_error)
            
            # If we need to turn significantly, turn first
            if abs(heading_error) > math.pi/6:  # 30 degrees
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed * (1 if heading_error > 0 else -1)
            else:
                # Move forward with heading correction
                twist.linear.x = min(self.forward_speed, distance_to_target * 2)  # Slow down as we approach
                twist.angular.z = self.kp_heading * heading_error
            
            # Publish command
            self.cmd_vel_pub.publish(twist)
            
            # Log progress
            current_time = time.time()
            if current_time - self.last_log_time > 1.0:
                self.get_logger().info(f'Distance to target: {distance_to_target:.3f}m, heading error: {math.degrees(heading_error):.1f}°')
                self.last_log_time = current_time
            
            time.sleep(0.05)
    
    def execute_triangular_detour(self, obstacle_distance, original_heading):
        """
        Execute triangular obstacle avoidance maneuver
        Args:
            obstacle_distance: Distance to the detected obstacle
            original_heading: Original heading before obstacle detection
        Returns:
            tuple: (return_x, return_y) - point where robot should return to original path
        """
        self.get_logger().info('=== Starting Triangular Obstacle Avoidance ===')
        
        # Find safe detour point
        detour_result = self.find_safe_detour_point(obstacle_distance, original_heading)
        if detour_result is None:
            self.get_logger().error('Cannot find safe detour path! Stopping.')
            return None, None
        
        detour_x, detour_y, _ = detour_result
        
        # Step 1: Navigate to detour point
        self.get_logger().info('DETOUR STEP 1: Moving to detour point')
        self.navigate_to_point(detour_x, detour_y, "detour point")
        
        time.sleep(0.5)
        
        # Step 2: Calculate return point on original path
        # Move past the obstacle on the original path
        safe_return_distance = obstacle_distance + self.triangle_side_distance + self.safety_margin
        return_x = self.start_x + safe_return_distance * math.cos(original_heading)
        return_y = self.start_y + safe_return_distance * math.sin(original_heading)
        
        # Check if return path is clear
        angle_to_return = math.atan2(return_y - self.current_y, return_x - self.current_x)
        distance_to_return = math.sqrt((return_x - self.current_x)**2 + (return_y - self.current_y)**2)
        
        is_clear, _ = self.check_path_clear(distance_to_return, angle_to_return)
        
        if not is_clear:
            # Adjust return point further ahead
            safe_return_distance += 0.3
            return_x = self.start_x + safe_return_distance * math.cos(original_heading)
            return_y = self.start_y + safe_return_distance * math.sin(original_heading)
            self.get_logger().info('Adjusted return point due to obstacles')
        
        # Step 3: Navigate to return point
        self.get_logger().info('DETOUR STEP 2: Returning to original path')
        self.navigate_to_point(return_x, return_y, "return to original path")
        
        self.get_logger().info('=== Triangular Obstacle Avoidance Complete ===')
        
        return return_x, return_y
    
    def turn_right_90_degrees(self):
        """Turn exactly 100 degrees to the right"""
        self.get_logger().info('Starting 100-degree right turn...')
        
        # Calculate target heading (100 degrees clockwise from current)
        start_heading = self.current_yaw
        target_heading = self.normalize_angle(start_heading - math.pi*105/180)  # -100 degrees
        
        self.get_logger().info(f'Turn start heading: {math.degrees(start_heading):.1f}°')
        self.get_logger().info(f'Turn target heading: {math.degrees(target_heading):.1f}°')
        
        twist = Twist()
        
        while rclpy.ok():
            # Process callbacks for fresh odometry
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Calculate heading error
            heading_error = target_heading - self.current_yaw
            heading_error = self.normalize_angle(heading_error)
            
            # Check if turn is complete (within 2 degrees tolerance)
            if abs(heading_error) < math.radians(0.5):
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
                total_turn = self.normalize_angle(self.current_yaw - start_heading)
                self.get_logger().info(f'Turn completed! Turned {math.degrees(total_turn):.1f}°')
                self.get_logger().info(f'Final heading: {math.degrees(self.current_yaw):.1f}°')
                break
            
            # Apply turning velocity (negative for right turn)
            if heading_error > 0:
                twist.angular.z = self.turn_speed
            else:
                twist.angular.z = -self.turn_speed
            
            twist.linear.x = 0.0  # No forward movement during turn
            self.cmd_vel_pub.publish(twist)
            
            # Log progress
            current_time = time.time()
            if current_time - self.last_log_time > 0.5:
                current_turn = self.normalize_angle(self.current_yaw - start_heading)
                self.get_logger().info(f'Turning... Current: {math.degrees(self.current_yaw):.1f}°, '
                                     f'Turned: {math.degrees(current_turn):.1f}°, '
                                     f'Error: {math.degrees(heading_error):.1f}°')
                self.last_log_time = current_time
            
            time.sleep(0.05)
    
    def move_distance_with_obstacle_avoidance(self, distance, description=""):
        """
        Move forward for a specific distance with obstacle avoidance
        """
        if description:
            self.get_logger().info(f'Starting {description}: {distance:.2f}m (with obstacle avoidance)')
        else:
            self.get_logger().info(f'Moving forward: {distance:.2f}m (with obstacle avoidance)')
        
        # Wait for scan data
        self.get_logger().info('Waiting for scan data...')
        while not self.scan_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not rclpy.ok():
            return
        
        # Record starting position and heading
        start_x = self.current_x
        start_y = self.current_y
        original_heading = self.current_yaw
        target_heading = original_heading  # Maintain current heading
        
        # First check for obstacles in the full path
        is_clear, obstacle_distance = self.check_path_clear(distance)
        
        if not is_clear:
            self.get_logger().info(f'Obstacle detected at {obstacle_distance:.2f}m! Executing detour...')
            
            # Execute triangular detour
            return_x, return_y = self.execute_triangular_detour(obstacle_distance, original_heading)
            
            if return_x is None or return_y is None:
                self.get_logger().error('Detour failed! Stopping movement.')
                return
            
            # Update our position reference
            remaining_distance = distance - math.sqrt((return_x - start_x)**2 + (return_y - start_y)**2)
            
            if remaining_distance > 0.1:  # If we still need to travel more
                self.get_logger().info(f'Continuing original movement for remaining {remaining_distance:.2f}m')
                self.move_distance(remaining_distance, "remaining distance after detour")
            else:
                self.get_logger().info('Target distance achieved through detour')
            
            return
        
        # No obstacles detected, proceed with normal movement
        self.get_logger().info('Path clear, proceeding with normal movement')
        self.move_distance(distance, description)
    
    def move_distance(self, distance, description=""):
        """Move forward for a specific distance (original method, no obstacle avoidance)"""
        if description:
            self.get_logger().info(f'Starting {description}: {distance:.2f}m')
        else:
            self.get_logger().info(f'Moving forward: {distance:.2f}m')
        
        # Record starting position
        start_x = self.current_x
        start_y = self.current_y
        target_heading = self.current_yaw  # Maintain current heading
        
        twist = Twist()
        
        while rclpy.ok():
            # Process callbacks for fresh odometry
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Calculate distance traveled
            distance_traveled = math.sqrt(
                (self.current_x - start_x)**2 + 
                (self.current_y - start_y)**2
            )
            
            # Check if target distance reached
            if distance_traveled >= distance:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f'Distance completed: {distance_traveled:.3f}m')
                break
            
            # Set forward velocity
            twist.linear.x = self.forward_speed
            
            # Heading correction to maintain straight line
            heading_error = target_heading - self.current_yaw
            heading_error = self.normalize_angle(heading_error)
            twist.angular.z = self.kp_heading * heading_error
            
            # Publish command
            self.cmd_vel_pub.publish(twist)
            
            # Log progress
            current_time = time.time()
            if current_time - self.last_log_time > 1.0:
                self.get_logger().info(f'Distance: {distance_traveled:.3f}m / {distance:.3f}m')
                self.last_log_time = current_time
            
            time.sleep(0.05)
    
    def execute_boustrophedon_pattern(self):
        """Execute the complete boustrophedon pattern with obstacle avoidance"""
        # Wait for first odometry message
        self.get_logger().info('Waiting for odometry data...')
        while not self.odom_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not rclpy.ok():
            return
        
        self.get_logger().info('=== Starting Boustrophedon Pattern with Obstacle Avoidance ===')
        
        # Step 1: Move forward the target distance with obstacle avoidance
        self.get_logger().info('STEP 1: Forward movement with obstacle avoidance')
        self.move_distance_with_obstacle_avoidance(self.target_distance, "main forward sweep")
        
        time.sleep(0.5)  # Brief pause between steps
        
        # Step 2: First 90-degree right turn
        self.get_logger().info('STEP 2: First right turn')
        self.turn_right_90_degrees()
        
        time.sleep(0.5)
        
        # Step 3: Move offset distance (20cm) - use normal movement for short distances
        self.get_logger().info('STEP 3: Offset movement')
        self.move_distance(self.offset_distance, "offset movement")
        
        time.sleep(0.5)
        
        # Step 4: Second 90-degree right turn
        self.get_logger().info('STEP 4: Second right turn')
        self.turn_right_90_degrees()
        
        self.get_logger().info('=== Boustrophedon Pattern Complete ===')
        self.get_logger().info('Robot should now be facing opposite direction, offset by 20cm')


def main(args=None):
    rclpy.init(args=args)
    
    controller = PIDStraightLineController()
    
    try:
        controller.execute_boustrophedon_pattern()
        controller.get_logger().info('Pattern completed successfully!')
        
    except KeyboardInterrupt:
        controller.get_logger().info('Pattern interrupted by user')
        
    finally:
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        try:
            controller.cmd_vel_pub.publish(twist)
        except:
            pass
        
        # Clean shutdown
        try:
            controller.destroy_node()
        except:
            pass
            
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()