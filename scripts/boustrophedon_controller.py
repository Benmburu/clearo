#!/usr/bin/env python3
"""
Modular Boustrophedon Controller - Single node approach with separate controller classes
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time
import numpy as np

# Import modules
import sys
import os

# Add the scripts directory to Python path
scripts_dir = os.path.dirname(os.path.abspath(__file__))
if scripts_dir not in sys.path:
    sys.path.append(scripts_dir)

# Import the controller classes (we'll use them as helper classes, not separate nodes)
from base_robot_controller import BaseRobotController


class ModularBoustrophedonController(BaseRobotController):
    """
    Main controller that orchestrates movement and turning using separate controller logic
    """
    
    def __init__(self, node_name='modular_boustrophedon_controller'):
        super().__init__(node_name)
        
        # Pattern-specific parameters (only declare if not already declared)
        try:
            self.declare_parameter('target_distance', 1.0)
        except:
            pass
        try:
            self.declare_parameter('offset_distance', 0.2)
        except:
            pass
        try:
            self.declare_parameter('safety_margin', 0.15)
        except:
            pass
        
        self.target_distance = self.get_parameter('target_distance').value
        self.offset_distance = self.get_parameter('offset_distance').value
        self.safety_margin = self.get_parameter('safety_margin').value
        
        # LIDAR subscription for room analysis
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.latest_scan = None
        self.scan_received = False
        
        # Room analysis variables
        self.room_width_left = None
        self.room_width_right = None
        self.room_width_total = None
        self.columns_needed = None
        self.current_column = 0
        self.moving_right = True  # Direction flag for boustrophedon pattern
        
        self.get_logger().info('Modular Boustrophedon Controller initialized')
        self.get_logger().info(f'Target distance: {self.target_distance}m')
        self.get_logger().info(f'Offset distance: {self.offset_distance}m')
        self.get_logger().info(f'Safety margin: {self.safety_margin}m')
    
    def scan_callback(self, msg):
        """Process LIDAR scan data"""
        self.latest_scan = msg
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info('LIDAR scan data received')
    
    def calculate_room_width(self):
        """Calculate room width using LIDAR data"""
        if self.latest_scan is None:
            self.get_logger().warn('No LIDAR data available for width calculation')
            return False
        
        # Get scan parameters
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        ranges = np.array(self.latest_scan.ranges)
        
        # Filter out invalid readings
        valid_ranges = ranges[(ranges >= self.latest_scan.range_min) & 
                             (ranges <= self.latest_scan.range_max) & 
                             (~np.isnan(ranges)) & 
                             (~np.isinf(ranges))]
        
        if len(valid_ranges) == 0:
            self.get_logger().warn('No valid LIDAR readings found')
            return False
        
        # Calculate angles for each reading
        angles = np.array([angle_min + i * angle_increment for i in range(len(ranges))])
        
        # Find left and right wall distances (perpendicular to robot)
        left_angle_target = math.pi / 2  # 90 degrees
        right_angle_target = -math.pi / 2  # -90 degrees
        
        # Find closest angle indices to left and right
        left_idx = np.argmin(np.abs(angles - left_angle_target))
        right_idx = np.argmin(np.abs(angles - right_angle_target))
        
        # Get average of nearby readings for more stable measurement
        window_size = 5  # Average over 5 readings
        
        # Left wall distance
        left_start = max(0, left_idx - window_size // 2)
        left_end = min(len(ranges), left_idx + window_size // 2 + 1)
        left_readings = ranges[left_start:left_end]
        left_valid = left_readings[(left_readings >= self.latest_scan.range_min) & 
                                  (left_readings <= self.latest_scan.range_max) & 
                                  (~np.isnan(left_readings)) & 
                                  (~np.isinf(left_readings))]
        
        # Right wall distance
        right_start = max(0, right_idx - window_size // 2)
        right_end = min(len(ranges), right_idx + window_size // 2 + 1)
        right_readings = ranges[right_start:right_end]
        right_valid = right_readings[(right_readings >= self.latest_scan.range_min) & 
                                    (right_readings <= self.latest_scan.range_max) & 
                                    (~np.isnan(right_readings)) & 
                                    (~np.isinf(right_readings))]
        
        if len(left_valid) == 0 or len(right_valid) == 0:
            self.get_logger().warn('Could not get valid wall distance measurements')
            return False
        
        self.room_width_left = np.mean(left_valid)
        self.room_width_right = np.mean(right_valid)
        self.room_width_total = self.room_width_left + self.room_width_right
        
        # Calculate number of columns needed
        effective_width = self.room_width_total - (2 * self.safety_margin)
        self.columns_needed = max(1, int(math.ceil(effective_width / self.offset_distance)))
        
        self.get_logger().info(f'Room width calculation:')
        self.get_logger().info(f'  Left wall distance: {self.room_width_left:.2f}m')
        self.get_logger().info(f'  Right wall distance: {self.room_width_right:.2f}m')
        self.get_logger().info(f'  Total room width: {self.room_width_total:.2f}m')
        self.get_logger().info(f'  Effective cleaning width: {effective_width:.2f}m')
        self.get_logger().info(f'  Columns needed: {self.columns_needed}')
        
        return True
    
    def wait_for_sensors(self):
        """Wait for all required sensors to be ready"""
        self.get_logger().info('Waiting for sensor data...')
        
        # Wait for basic sensors
        while (not self.odom_received or not self.scan_received) and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Wait a bit more for IMU if available
        imu_wait_time = 0
        while not self.imu_received and imu_wait_time < 20 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            imu_wait_time += 1
        
        if self.imu_received:
            self.get_logger().info('IMU data available - using IMU for precise heading control')
        else:
            self.get_logger().warn('IMU data not available - falling back to odometry for heading')
        
        return rclpy.ok()
    
    def execute_simple_boustrophedon_pattern(self):
        """Execute the original simple boustrophedon pattern"""
        if not self.wait_for_sensors():
            return False
        
        self.get_logger().info('=== Starting Simple Boustrophedon Pattern ===')
        
        # Log initial robot state
        self.log_heading_state("PATTERN_START")
        
        try:
            # Step 1: Move forward the target distance
            self.get_logger().info('STEP 1: Forward movement')
            if not self.move_distance(self.target_distance, "main forward sweep"):
                self.get_logger().error('Failed to complete forward movement')
                return False
            
            time.sleep(0.5)  # Brief pause between steps
            
            # Step 2: First 90-degree right turn
            self.get_logger().info('STEP 2: First right turn')
            if not self.turn_right_90_degrees("first right turn"):
                self.get_logger().error('Failed to complete first right turn')
                return False
            
            time.sleep(0.5)
            
            # Step 3: Move offset distance (20cm)
            self.get_logger().info('STEP 3: Offset movement')
            if not self.move_distance(self.offset_distance, "offset movement"):
                self.get_logger().error('Failed to complete offset movement')
                return False
            
            time.sleep(0.5)
            
            # Step 4: Second 90-degree right turn
            self.get_logger().info('STEP 4: Second right turn')
            if not self.turn_right_90_degrees("second right turn"):
                self.get_logger().error('Failed to complete second right turn')
                return False
            
            # Log final robot state
            self.log_heading_state("PATTERN_COMPLETE")
            
            self.get_logger().info('=== Simple Boustrophedon Pattern Complete ===')
            self.get_logger().info('Robot should now be facing opposite direction, offset by 20cm')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error during pattern execution: {str(e)}')
            self.stop_robot()
            return False
    
    def execute_complete_boustrophedon_pattern(self):
        """Execute the complete room coverage boustrophedon pattern"""
        if not self.wait_for_sensors():
            return False
        
        # Calculate room dimensions
        if not self.calculate_room_width():
            self.get_logger().error('Failed to calculate room width, using default pattern')
            self.columns_needed = 3  # Default fallback
        
        self.get_logger().info('=== Starting Complete Boustrophedon Coverage Pattern ===')
        self.get_logger().info(f'Will execute {self.columns_needed} columns')
        
        # Log initial robot state
        self.log_heading_state("COMPLETE_PATTERN_START")
        
        try:
            for column in range(self.columns_needed):
                self.current_column = column
                self.get_logger().info(f'=== COLUMN {column + 1}/{self.columns_needed} ===')
                
                # Log state at start of each column
                self.log_heading_state(f"COLUMN_{column + 1}_START")
                
                # Step 1: Move forward the target distance
                self.get_logger().info(f'COLUMN {column + 1}: Forward sweep')
                if not self.move_distance(
                    self.target_distance, 
                    f"column {column + 1} forward sweep"
                ):
                    self.get_logger().error(f'Failed to complete forward sweep for column {column + 1}')
                    return False
                
                time.sleep(0.5)
                
                # Check if this is the last column
                if column == self.columns_needed - 1:
                    self.get_logger().info('=== Final column completed ===')
                    break
                
                # Execute turn sequence based on pattern direction
                if not self._execute_turn_sequence(column):
                    self.get_logger().error(f'Failed to complete turn sequence for column {column + 1}')
                    return False
                
                # Toggle direction for next column
                self.moving_right = not self.moving_right
                
                # Log state at end of each column
                self.log_heading_state(f"COLUMN_{column + 1}_COMPLETE")
                self.get_logger().info(f'Column {column + 1} completed, now facing opposite direction')
            
            # Log final robot state
            self.log_heading_state("COMPLETE_PATTERN_FINISHED")
            
            self.get_logger().info('=== Complete Boustrophedon Pattern Finished ===')
            self.get_logger().info(f'Successfully cleaned {self.columns_needed} columns')
            self.get_logger().info(f'Total area coverage: ~{self.columns_needed * self.offset_distance:.2f}m width × {self.target_distance:.2f}m length')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error during complete pattern execution: {str(e)}')
            self.stop_robot()
            return False
    
    def _execute_turn_sequence(self, column):
        """Execute the turn sequence for transitioning between columns"""
        try:
            if self.moving_right:
                # Moving right: turn right, move offset, turn right
                self.get_logger().info('Turn sequence: right-offset-right')
                
                if not self.turn_right_90_degrees("transition right turn 1"):
                    return False
                time.sleep(0.5)
                
                if not self.move_distance(self.offset_distance, "transition offset movement"):
                    return False
                time.sleep(0.5)
                
                if not self.turn_right_90_degrees("transition right turn 2"):
                    return False
                time.sleep(0.5)
                
            else:
                # Moving left: turn left, move offset, turn left
                self.get_logger().info('Turn sequence: left-offset-left')
                
                if not self.turn_left_90_degrees("transition left turn 1"):
                    return False
                time.sleep(0.5)
                
                if not self.move_distance(self.offset_distance, "transition offset movement"):
                    return False
                time.sleep(0.5)
                
                if not self.turn_left_90_degrees("transition left turn 2"):
                    return False
                time.sleep(0.5)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error during turn sequence: {str(e)}')
            return False
    
    def debug_movement_only(self):
        """Test only forward movement for debugging"""
        if not self.wait_for_sensors():
            return False
            
        self.get_logger().info('=== DEBUG: Testing Forward Movement Only ===')
        return self.move_distance(self.target_distance, "debug forward movement")
    
    def debug_turn_only(self, direction='right'):
        """Test only turning for debugging"""
        if not self.wait_for_sensors():
            return False
            
        self.get_logger().info(f'=== DEBUG: Testing {direction.capitalize()} Turn Only ===')
        
        if direction.lower() == 'right':
            return self.turn_right_90_degrees("debug right turn")
        elif direction.lower() == 'left':
            return self.turn_left_90_degrees("debug left turn")
        else:
            self.get_logger().error(f'Invalid turn direction: {direction}')
            return False
    
    def debug_single_sequence(self):
        """Test a single forward-turn-offset-turn sequence for debugging"""
        if not self.wait_for_sensors():
            return False
            
        self.get_logger().info('=== DEBUG: Testing Single Sequence ===')
        
        try:
            # Forward
            if not self.move_distance(self.target_distance, "debug forward"):
                return False
            time.sleep(0.5)
            
            # Turn right
            if not self.turn_right_90_degrees("debug right turn 1"):
                return False
            time.sleep(0.5)
            
            # Offset
            if not self.move_distance(self.offset_distance, "debug offset"):
                return False
            time.sleep(0.5)
            
            # Turn right again
            if not self.turn_right_90_degrees("debug right turn 2"):
                return False
            
            self.get_logger().info('=== DEBUG: Single Sequence Complete ===')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error during debug sequence: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    controller = ModularBoustrophedonController()
    
    try:
        # Choose which pattern/test to execute:
        
        # For debugging individual components:
        # controller.debug_movement_only()
        # controller.debug_turn_only('right')  # or 'left'
        # controller.debug_single_sequence()
        
        # For full patterns:
        # controller.execute_simple_boustrophedon_pattern()  # Simple pattern
        controller.execute_complete_boustrophedon_pattern()  # Full room coverage
        
        controller.get_logger().info('Pattern completed successfully!')
        
    except KeyboardInterrupt:
        controller.get_logger().info('Pattern interrupted by user')
        
    finally:
        # Stop the robot
        controller.stop_robot()
        
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