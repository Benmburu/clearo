#!/usr/bin/env python3
"""
Turn Controller - Handles left and right turns with precise angle control
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


class TurnController(BaseRobotController):
    """Controller for precise turning movements"""
    
    def __init__(self, node_name='turn_controller'):
        super().__init__(node_name)
        
        # Turn-specific parameters
        self.declare_parameter('turn_tolerance_deg', 1.0)
        self.declare_parameter('turn_completion_threshold_deg', 89.0)
        
        self.turn_tolerance_deg = self.get_parameter('turn_tolerance_deg').value
        self.turn_completion_threshold_deg = self.get_parameter('turn_completion_threshold_deg').value
        
        self.get_logger().info('Turn Controller initialized')
        self.get_logger().info(f'Turn tolerance: {self.turn_tolerance_deg}°')
        self.get_logger().info(f'Turn completion threshold: {self.turn_completion_threshold_deg}°')
    
    def turn_to_heading(self, target_heading_deg, description=""):
        """
        Turn to a specific absolute heading
        
        Args:
            target_heading_deg (float): Target heading in degrees
            description (str): Description for logging
        
        Returns:
            bool: True if turn completed successfully, False otherwise
        """
        if description:
            self.get_logger().info(f'Starting {description}: target {target_heading_deg:.1f}°')
        else:
            self.get_logger().info(f'Turning to heading: {target_heading_deg:.1f}°')
        
        # Log initial state
        self.log_heading_state("TURN_START")
        
        start_heading = self.get_current_heading()
        target_heading_rad = math.radians(target_heading_deg)
        
        heading_source = "IMU" if self.imu_received else "odometry"
        self.get_logger().info(f'Turn start heading ({heading_source}): {math.degrees(start_heading):.1f}°')
        self.get_logger().info(f'Turn target heading: {target_heading_deg:.1f}°')
        
        return self._execute_turn(start_heading, target_heading_rad, description)
    
    def turn_relative(self, angle_deg, description=""):
        """
        Turn by a relative angle from current heading
        
        Args:
            angle_deg (float): Angle to turn in degrees (positive = left, negative = right)
            description (str): Description for logging
        
        Returns:
            bool: True if turn completed successfully, False otherwise
        """
        if description:
            self.get_logger().info(f'Starting {description}: {angle_deg:.1f}° relative turn')
        else:
            self.get_logger().info(f'Turning relative: {angle_deg:.1f}°')
        
        # Log initial state
        self.log_heading_state("TURN_START")
        
        start_heading = self.get_current_heading()
        target_heading_rad = self.normalize_angle(start_heading + math.radians(angle_deg))
        
        heading_source = "IMU" if self.imu_received else "odometry"
        self.get_logger().info(f'Turn start heading ({heading_source}): {math.degrees(start_heading):.1f}°')
        self.get_logger().info(f'Turn target heading: {math.degrees(target_heading_rad):.1f}°')
        
        return self._execute_turn(start_heading, target_heading_rad, description)
    
    def turn_right_90_degrees(self, description=""):
        """Turn exactly 90 degrees to the right"""
        if not description:
            description = "90-degree right turn"
        return self.turn_relative(-90.0, description)
    
    def turn_left_90_degrees(self, description=""):
        """Turn exactly 90 degrees to the left"""
        if not description:
            description = "90-degree left turn"
        return self.turn_relative(90.0, description)
    
    def turn_around(self, description=""):
        """Turn 180 degrees (turn around)"""
        if not description:
            description = "180-degree turn around"
        return self.turn_relative(180.0, description)
    
    def _execute_turn(self, start_heading, target_heading_rad, description=""):
        """
        Execute the actual turn movement
        
        Args:
            start_heading (float): Starting heading in radians
            target_heading_rad (float): Target heading in radians
            description (str): Description for logging
        
        Returns:
            bool: True if turn completed successfully, False otherwise
        """
        twist = Twist()
        turn_start_time = time.time()
        max_angular_error = 0.0
        
        try:
            while rclpy.ok():
                # Process callbacks for fresh sensor data
                rclpy.spin_once(self, timeout_sec=0.01)
                
                # Calculate heading error using IMU if available
                current_heading = self.get_current_heading()
                heading_error = target_heading_rad - current_heading
                heading_error = self.normalize_angle(heading_error)
                
                # Track maximum angular error
                max_angular_error = max(max_angular_error, abs(heading_error))
                
                # Check if turn is complete using angle tolerance
                if abs(heading_error) <= math.radians(self.turn_tolerance_deg):
                    self.stop_robot()
                    
                    # Calculate actual turn made
                    total_turn = self.normalize_angle(current_heading - start_heading)
                    turn_time = time.time() - turn_start_time
                    
                    heading_source = "IMU" if self.imu_received else "odometry"
                    self.get_logger().info(f'Turn completed! Turned {math.degrees(total_turn):.1f}° in {turn_time:.1f}s')
                    self.get_logger().info(f'Final heading ({heading_source}): {math.degrees(current_heading):.1f}°')
                    self.get_logger().info(f'Final heading error: {math.degrees(heading_error):.2f}°')
                    self.get_logger().info(f'Maximum angular error during turn: {math.degrees(max_angular_error):.2f}°')
                    
                    # Log final state
                    self.log_heading_state("TURN_COMPLETE")
                    return True
                
                # Calculate angular velocity using bang-bang control (like original)
                if heading_error > 0:
                    angular_velocity = self.turn_speed  # Turn left
                else:
                    angular_velocity = -self.turn_speed  # Turn right
                
                # Apply angular velocity
                twist.linear.x = 0.0  # No forward movement during turn
                twist.angular.z = angular_velocity
                self.cmd_vel_pub.publish(twist)
                
                # Log progress
                current_time = time.time()
                if current_time - self.last_log_time > 0.5:
                    current_turn = self.normalize_angle(current_heading - start_heading)
                    turn_progress = abs(math.degrees(current_turn))
                    
                    log_msg = f'Turning... Current: {math.degrees(current_heading):.1f}°'
                    log_msg += f', Turned: {math.degrees(current_turn):.1f}°'
                    log_msg += f', Error: {math.degrees(heading_error):.1f}°'
                    log_msg += f', Angular_Cmd: {angular_velocity:.3f}rad/s'
                    
                    self.get_logger().info(log_msg)
                    self.last_log_time = current_time
                
                time.sleep(0.05)
                
        except Exception as e:
            self.get_logger().error(f'Error during turn: {str(e)}')
            self.stop_robot()
            return False
        
        # If we get here, something went wrong
        self.stop_robot()
        return False
    
    def _execute_turn_with_pid(self, start_heading, target_heading_rad, description=""):
        """
        Execute turn with PID control instead of bang-bang
        (Alternative implementation for smoother turns)
        
        Args:
            start_heading (float): Starting heading in radians
            target_heading_rad (float): Target heading in radians
            description (str): Description for logging
        
        Returns:
            bool: True if turn completed successfully, False otherwise
        """
        # PID parameters for turning
        kp_turn = 2.0
        ki_turn = 0.1
        kd_turn = 0.05
        
        integral_error = 0.0
        previous_error = 0.0
        twist = Twist()
        turn_start_time = time.time()
        dt = 0.05
        
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                
                current_heading = self.get_current_heading()
                heading_error = target_heading_rad - current_heading
                heading_error = self.normalize_angle(heading_error)
                
                # Check if turn is complete
                if abs(heading_error) <= math.radians(self.turn_tolerance_deg):
                    self.stop_robot()
                    
                    total_turn = self.normalize_angle(current_heading - start_heading)
                    turn_time = time.time() - turn_start_time
                    
                    self.get_logger().info(f'PID Turn completed! Turned {math.degrees(total_turn):.1f}° in {turn_time:.1f}s')
                    self.log_heading_state("TURN_COMPLETE")
                    return True
                
                # PID control
                integral_error += heading_error * dt
                derivative_error = (heading_error - previous_error) / dt
                
                # Anti-windup for integral term
                max_integral = 0.5
                integral_error = max(-max_integral, min(max_integral, integral_error))
                
                angular_velocity = (kp_turn * heading_error + 
                                  ki_turn * integral_error + 
                                  kd_turn * derivative_error)
                
                # Limit angular velocity
                max_turn_speed = self.turn_speed * 2  # Allow faster turns with PID
                angular_velocity = max(-max_turn_speed, min(max_turn_speed, angular_velocity))
                
                twist.linear.x = 0.0
                twist.angular.z = angular_velocity
                self.cmd_vel_pub.publish(twist)
                
                previous_error = heading_error
                
                # Log progress
                current_time = time.time()
                if current_time - self.last_log_time > 0.5:
                    current_turn = self.normalize_angle(current_heading - start_heading)
                    
                    log_msg = f'PID Turning... Current: {math.degrees(current_heading):.1f}°'
                    log_msg += f', Turned: {math.degrees(current_turn):.1f}°'
                    log_msg += f', Error: {math.degrees(heading_error):.1f}°'
                    log_msg += f', Angular_Cmd: {angular_velocity:.3f}rad/s'
                    log_msg += f', P: {kp_turn * heading_error:.3f}'
                    log_msg += f', I: {ki_turn * integral_error:.3f}'
                    log_msg += f', D: {kd_turn * derivative_error:.3f}'
                    
                    self.get_logger().info(log_msg)
                    self.last_log_time = current_time
                
                time.sleep(dt)
                
        except Exception as e:
            self.get_logger().error(f'Error during PID turn: {str(e)}')
            self.stop_robot()
            return False
        
        # If we get here, something went wrong
        self.stop_robot()
        return False