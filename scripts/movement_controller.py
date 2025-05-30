#!/usr/bin/env python3
"""
Movement Controller - Handles forward movement with heading correction
"""
import rclpy
from geometry_msgs.msg import Twist
import math
import time

# Change these relative imports to absolute imports
import sys
import os

# Add the scripts directory to Python path
scripts_dir = os.path.dirname(os.path.abspath(__file__))
if scripts_dir not in sys.path:
    sys.path.append(scripts_dir)

from base_robot_controller import BaseRobotController


class MovementController(BaseRobotController):
    """Controller for precise forward movement with heading correction"""
    
    def __init__(self, node_name='movement_controller'):
        super().__init__(node_name)
        self.get_logger().info('Movement Controller initialized')
    
    def move_distance(self, distance, description="", target_heading=None):
        """
        Move forward for a specific distance using IMU for heading correction
        
        Args:
            distance (float): Distance to move in meters
            description (str): Description for logging
            target_heading (float): Specific heading to maintain (if None, uses current heading)
        
        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        if description:
            self.get_logger().info(f'Starting {description}: {distance:.2f}m')
        else:
            self.get_logger().info(f'Moving forward: {distance:.2f}m')
        
        # Log initial state before movement
        self.log_heading_state("MOVE_START")
        
        # Record starting position and heading
        start_x = self.current_x
        start_y = self.current_y
        
        # Use provided target heading or current heading
        if target_heading is not None:
            heading_to_maintain = target_heading
            self.get_logger().info(f'Using provided target heading: {math.degrees(heading_to_maintain):.2f}°')
        else:
            heading_to_maintain = self.get_current_heading()
            self.get_logger().info(f'Using current heading as target: {math.degrees(heading_to_maintain):.2f}°')
        
        heading_source = "IMU" if self.imu_received else "odometry"
        self.get_logger().info(f'Using {heading_source} for heading correction during movement')
        
        twist = Twist()
        max_heading_error = 0.0
        movement_start_time = time.time()
        
        try:
            while rclpy.ok():
                # Process callbacks for fresh sensor data
                rclpy.spin_once(self, timeout_sec=0.01)
                
                # Calculate distance traveled
                distance_traveled = math.sqrt(
                    (self.current_x - start_x)**2 + 
                    (self.current_y - start_y)**2
                )
                
                # Check if target distance reached
                if distance_traveled >= distance:
                    self.stop_robot()
                    
                    movement_time = time.time() - movement_start_time
                    avg_speed = distance_traveled / movement_time if movement_time > 0 else 0
                    
                    self.get_logger().info(f'Distance completed: {distance_traveled:.3f}m in {movement_time:.1f}s')
                    self.get_logger().info(f'Average speed: {avg_speed:.3f}m/s')
                    self.get_logger().info(f'Maximum heading error during movement: {math.degrees(max_heading_error):.2f}°')
                    
                    # Log final state after movement
                    self.log_heading_state("MOVE_COMPLETE")
                    return True
                
                # Set forward velocity
                twist.linear.x = self.forward_speed
                
                # Heading correction to maintain straight line
                current_heading = self.get_current_heading()
                heading_error = heading_to_maintain - current_heading
                heading_error = self.normalize_angle(heading_error)
                
                # Track maximum heading error
                max_heading_error = max(max_heading_error, abs(heading_error))
                
                # Calculate angular correction with PID control
                angular_correction = self.kp_heading * heading_error
                
                # Limit angular velocity to prevent oscillation
                max_angular_vel = 0.5  # rad/s
                angular_correction = max(-max_angular_vel, min(max_angular_vel, angular_correction))
                
                twist.angular.z = angular_correction
                
                # Publish command
                self.cmd_vel_pub.publish(twist)
                
                # Enhanced logging with heading information
                current_time = time.time()
                if current_time - self.last_log_time > 1.0:
                    progress_percent = min(100, (distance_traveled / distance) * 100)
                    
                    log_msg = f'Distance: {distance_traveled:.3f}m / {distance:.3f}m ({progress_percent:.0f}%)'
                    log_msg += f' | Heading: Current={math.degrees(current_heading):.2f}°'
                    log_msg += f', Target={math.degrees(heading_to_maintain):.2f}°'
                    log_msg += f', Error={math.degrees(heading_error):.2f}°'
                    log_msg += f', Angular_Cmd={angular_correction:.3f}rad/s'
                    log_msg += f' | Max_Error={math.degrees(max_heading_error):.2f}°'
                    
                    self.get_logger().info(log_msg)
                    self.last_log_time = current_time
                
                time.sleep(0.05)
                
        except Exception as e:
            self.get_logger().error(f'Error during movement: {str(e)}')
            self.stop_robot()
            return False
    
    def move_to_position(self, target_x, target_y, description=""):
        """
        Move to a specific position (x, y) coordinate
        
        Args:
            target_x (float): Target X coordinate
            target_y (float): Target Y coordinate
            description (str): Description for logging
        
        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        if description:
            self.get_logger().info(f'Starting {description}: target ({target_x:.2f}, {target_y:.2f})')
        else:
            self.get_logger().info(f'Moving to position: ({target_x:.2f}, {target_y:.2f})')
        
        # Calculate distance and direction to target
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_heading = math.atan2(dy, dx)
        
        self.get_logger().info(f'Distance to target: {distance:.3f}m')
        self.get_logger().info(f'Required heading: {math.degrees(target_heading):.2f}°')
        
        # First, turn to face the target (this would require the turn controller)
        # For now, we'll just move with heading correction
        return self.move_distance(distance, description, target_heading)
    
    def check_obstacle_ahead(self, min_distance=0.3):
        """
        Check if there's an obstacle ahead using LIDAR
        
        Args:
            min_distance (float): Minimum safe distance in meters
        
        Returns:
            bool: True if obstacle detected, False otherwise
        """
        if self.latest_scan is None:
            self.get_logger().warn('No LIDAR data available for obstacle detection')
            return False
        
        ranges = self.latest_scan.ranges
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        # Check front sector (±30 degrees from front)
        front_angle_range = math.radians(30)
        num_readings = len(ranges)
        
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            
            # Check if angle is in front sector
            if abs(angle) <= front_angle_range:
                if (distance >= self.latest_scan.range_min and 
                    distance <= self.latest_scan.range_max and 
                    not math.isnan(distance) and 
                    not math.isinf(distance)):
                    
                    if distance < min_distance:
                        self.get_logger().warn(f'Obstacle detected at {distance:.2f}m ahead')
                        return True
        
        return False