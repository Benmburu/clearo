#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
import math
import time
import numpy as np


class PIDStraightLineController(Node):
    def __init__(self):
        super().__init__('pid_straight_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        self.MINIMUM_SAFE_CLEARANCE = 0.15  # 15cm - your main tuning parameter
        self.CLEARANCE_TOLERANCE = 0.02     # ±2cm tolerance
        self.TARGET_APPROACH_CLEARANCE = self.MINIMUM_SAFE_CLEARANCE + 0.05  # 20cm for approach

        self.ROBOT_LENGTH = 0.304

        # Parameters
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('kp_heading', 12.0)
        self.declare_parameter('target_heading_deg', -999.0)
        self.declare_parameter('turn_speed', 0.1)  # Angular velocity for turns
        self.declare_parameter('offset_distance', 0.05)  # 20cm offset
        self.declare_parameter('safety_margin', 0.15)  # 15cm safety margin from walls

        # NEW: Simple obstacle detection variables
        self.obstacle_ahead = False
        self.obstacle_distance = float('inf')

        self.in_avoidance_maneuver = False
        self.avoidance_original_heading = None
        
        self.target_distance = self.get_parameter('target_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp_heading = self.get_parameter('kp_heading').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.offset_distance = self.get_parameter('offset_distance').value
        self.safety_margin = self.get_parameter('safety_margin').value
        
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
        self.current_yaw_imu = 0.0  # IMU-based yaw
        self.start_x = None
        self.start_y = None
        self.target_yaw = None
        self.odom_received = False
        self.scan_received = False
        self.imu_received = False
        self.last_log_time = 0.0
        
        # LIDAR data for room analysis
        self.latest_scan = None
        self.room_width_left = None
        self.room_width_right = None
        self.room_width_total = None
        self.columns_needed = None
        self.current_column = 0
        self.moving_right = True  # Direction flag for boustrophedon pattern
        
        self.get_logger().info(f'Enhanced Boustrophedon Controller with IMU initialized:')
        self.get_logger().info(f'  Target distance: {self.target_distance}m')
        self.get_logger().info(f'  Forward speed: {self.forward_speed}m/s')
        self.get_logger().info(f'  Turn speed: {self.turn_speed}rad/s')
        self.get_logger().info(f'  Offset distance: {self.offset_distance}m')
        self.get_logger().info(f'  Safety margin: {self.safety_margin}m')
        self.get_logger().info(f'  Heading PID gain: {self.kp_heading}')
        
    def imu_callback(self, msg):
        """Process IMU data for heading information"""
        # Extract yaw from quaternion
        orientation = msg.orientation
        self.current_yaw_imu = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        
        if not self.imu_received:
            self.imu_received = True
            self.get_logger().info('IMU data received')
            self.get_logger().info(f'Initial IMU heading: {math.degrees(self.current_yaw_imu):.1f} degrees')
    
    def get_current_heading(self):
        """Get current heading from IMU if available, otherwise odometry"""
        if self.imu_received:
            return self.current_yaw_imu
        else:
            return self.current_yaw
    
    def scan_callback(self, msg):
        """Process LIDAR scan data"""
        self.latest_scan = msg
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info('LIDAR scan data received')
        # NEW: Just store the scan for obstacle checking
        self.check_path_obstacles()

    def test_lidar_orientation(self):
        """Test function to understand LIDAR orientation"""
        if self.latest_scan is None:
            self.get_logger().warn('No LIDAR data for orientation test')
            return
        
        ranges = np.array(self.latest_scan.ranges)
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        self.get_logger().info('=== LIDAR ORIENTATION TEST ===')
        self.get_logger().info(f'Total LIDAR beams: {len(ranges)}')
        self.get_logger().info(f'Angle range: {math.degrees(angle_min):.1f}° to {math.degrees(angle_min + len(ranges) * angle_increment):.1f}°')
        
        # Find closest obstacle in each direction
        directions = {
            'FRONT (0°)': (-math.pi/12, math.pi/12),
            'RIGHT (-90°)': (-math.pi/2 - math.pi/12, -math.pi/2 + math.pi/12),
            'BACK (±180°)': (math.pi - math.pi/12, math.pi),
            'LEFT (90°)': (math.pi/2 - math.pi/12, math.pi/2 + math.pi/12)
        }
        
        for direction_name, (min_angle, max_angle) in directions.items():
            sector_distances = []
            for i, range_val in enumerate(ranges):
                angle = angle_min + i * angle_increment
                angle = self.normalize_angle(angle)
                
                if (min_angle <= angle <= max_angle and 
                    self.latest_scan.range_min <= range_val <= 3.0):
                    sector_distances.append(range_val)
            
            if sector_distances:
                closest = min(sector_distances)
                self.get_logger().info(f'{direction_name}: closest obstacle at {closest:.2f}m')
            else:
                self.get_logger().info(f'{direction_name}: no obstacles detected')

    def log_lidar_debug(self):
        """Log LIDAR data for debugging obstacle detection"""
        if self.latest_scan is None:
            return
        
        ranges = np.array(self.latest_scan.ranges)
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        # Log readings in different sectors
        sectors = {
            'FRONT': (-math.pi/12, math.pi/12),      # ±15 degrees
            'LEFT': (math.pi/3, 2*math.pi/3),       # 60-120 degrees  
            'BACK': (2*math.pi/3, math.pi),         # 120-180 degrees
            'RIGHT': (-2*math.pi/3, -math.pi/3)     # -120 to -60 degrees
        }
        
        for sector_name, (min_angle, max_angle) in sectors.items():
            sector_ranges = []
            for i, range_val in enumerate(ranges):
                angle = angle_min + i * angle_increment
                angle = self.normalize_angle(angle)
                
                if (min_angle <= angle <= max_angle and 
                    self.latest_scan.range_min <= range_val <= self.latest_scan.range_max):
                    sector_ranges.append(range_val)
            
            if sector_ranges:
                min_dist = min(sector_ranges)
                avg_dist = sum(sector_ranges) / len(sector_ranges)
                self.get_logger().info(f'{sector_name}: min={min_dist:.2f}m, avg={avg_dist:.2f}m, readings={len(sector_ranges)}')

    def log_raw_lidar_data(self):
        """Log raw LIDAR data around front sector for debugging"""
        if self.latest_scan is None:
            return
        
        ranges = np.array(self.latest_scan.ranges)
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        self.get_logger().info('=== RAW LIDAR DATA DUMP (Front ±45°) ===')
        self.get_logger().info(f'angle_min: {math.degrees(angle_min):.1f}°, angle_increment: {math.degrees(angle_increment):.1f}°')
        
        # Look at beams around the front (after rotation compensation)
        front_beams = []
        for i, range_val in enumerate(ranges):
            # Apply rotation compensation
            angle = angle_min + i * angle_increment + math.pi
            angle = self.normalize_angle(angle)
            angle_deg = math.degrees(angle)
            
            # Collect beams within ±45° of front
            if -45 <= angle_deg <= 45:
                front_beams.append({
                    'beam': i,
                    'raw_angle_deg': math.degrees(angle_min + i * angle_increment),
                    'corrected_angle_deg': angle_deg,
                    'range': range_val,
                    'valid': self.latest_scan.range_min <= range_val <= 3.0
                })
        
        # Sort by corrected angle
        front_beams.sort(key=lambda x: x['corrected_angle_deg'])
        
        self.get_logger().info(f'Found {len(front_beams)} beams in front ±45° sector:')
        for beam in front_beams:
            status = "VALID" if beam['valid'] else "INVALID"
            self.get_logger().info(
                f'  Beam {beam["beam"]:3d}: {beam["range"]:.3f}m at {beam["corrected_angle_deg"]:6.1f}° '
                f'(raw: {beam["raw_angle_deg"]:6.1f}°) [{status}]'
            )
        
        self.get_logger().info('=' * 50)
    
    # def move_distance(self, distance, description="", skip_obstacle_avoidance=False):
    #     """Move forward for a specific distance with continuous obstacle monitoring"""
        
    #     # Determine if this is a backward movement
    #     is_backward_movement = distance < 0
        
    #     # Pre-movement obstacle check (only for forward movements, not during obstacle avoidance)
    #     if not skip_obstacle_avoidance and not is_backward_movement:
    #         self.get_logger().info(f'=== PRE-MOVEMENT OBSTACLE CHECK ===')
    #         self.check_path_obstacles()
            
    #         if hasattr(self, 'obstacle_ahead') and self.obstacle_ahead:
    #             current_clearance = getattr(self, 'obstacle_distance', float('inf'))
    #             self.get_logger().info(f'OBSTACLE STATUS: obstacle_ahead={self.obstacle_ahead}')
    #             self.get_logger().info(f'OBSTACLE DISTANCE: {current_clearance:.3f}m')
    #             self.get_logger().info(f'PLANNED MOVEMENT: {distance:.3f}m')
    #             self.get_logger().info(f'MINIMUM_SAFE_CLEARANCE: {self.MINIMUM_SAFE_CLEARANCE:.3f}m (±{self.CLEARANCE_TOLERANCE:.3f}m)')
                
    #             # NEW: Check if we need to backup first
    #             if current_clearance < (self.MINIMUM_SAFE_CLEARANCE - self.CLEARANCE_TOLERANCE):
    #                 backup_distance = (self.TARGET_APPROACH_CLEARANCE - current_clearance) + 0.05
    #                 self.get_logger().warn(f'TOO CLOSE: {current_clearance:.3f}m < {self.MINIMUM_SAFE_CLEARANCE:.3f}m')
    #                 self.get_logger().warn(f'BACKING UP: {backup_distance:.3f}m to create safe clearance')
                    
    #                 # Execute backup
    #                 twist = Twist()
    #                 twist.linear.x = 0.0
    #                 twist.angular.z = 0.0
    #                 self.cmd_vel_pub.publish(twist)
    #                 time.sleep(0.1)
                    
    #                 self.move_distance(-backup_distance, "emergency clearance backup", skip_obstacle_avoidance=True)
    #                 time.sleep(0.2)  # Brief pause for sensor update
                
    #             if distance > 0.1:  # For movements > 10cm
    #                 self.get_logger().info(f'DECISION: Starting obstacle avoidance')
    #                 self.navigate_around_obstacle(distance)
    #                 return
    #             else:
    #                 self.get_logger().info(f'DECISION: Small distance ({distance:.3f}m), proceeding with caution')

    #     else:
    #         if is_backward_movement:
    #             self.get_logger().info(f'=== BACKWARD MOVEMENT (skipping obstacle checks) ===')
    #         else:
    #             self.get_logger().info(f'=== OBSTACLE AVOIDANCE MOVEMENT (skipping pre-check) ===')

    #     if description:
    #         direction = "backward" if is_backward_movement else "forward"
    #         self.get_logger().info(f'Starting {description} ({direction}): {abs(distance):.2f}m')
    #     else:
    #         direction = "backward" if is_backward_movement else "forward"
    #         self.get_logger().info(f'Moving {direction}: {abs(distance):.2f}m')
        
    #     # Log initial state
    #     self.log_heading_state("MOVE_START")
        
    #     # Record starting position and heading
    #     start_x = self.current_x
    #     start_y = self.current_y
    #     target_heading = self.get_current_heading()

    #     self.get_logger().info(f'TARGET HEADING SET TO: {math.degrees(target_heading):.2f}°')
        
    #     twist = Twist()
    #     max_heading_error = 0.0
    #     loop_count = 0
    #     last_obstacle_check = 0
        
    #     # NEW: Variables for continuous monitoring (only for forward movement)
    #     previous_distances = []  # Track last few distance readings
    #     distance_trend_threshold = 5  # Number of readings to analyze
    #     early_warning_distance = 0.35  # Start slowing down at 35cm
    #     emergency_stop_distance = 0.15  # Emergency stop at 15cm
        
    #     while rclpy.ok():
    #         loop_count += 1

    #         # Simple stuck detection (only for forward movement)
    #         if not is_backward_movement and loop_count > 50:
    #             distance_traveled = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
    #             if distance_traveled < 0.05:  # 50 loops with less than 5cm progress
    #                 if self.obstacle_ahead:
    #                     current_clearance = getattr(self, 'obstacle_distance', float('inf'))
    #                     if current_clearance < self.TARGET_APPROACH_CLEARANCE:
    #                         self.get_logger().warn(f'ROBOT APPEARS STUCK: Backing up to create space')
    #                         twist.linear.x = 0.0
    #                         twist.angular.z = 0.0
    #                         self.cmd_vel_pub.publish(twist)
                            
    #                         backup_distance = max(0.10, (self.TARGET_APPROACH_CLEARANCE - current_clearance) + 0.05)
    #                         self.move_distance(-backup_distance, "stuck recovery backup", skip_obstacle_avoidance=True)
                            
    #                         remaining_distance = distance - distance_traveled
    #                         if remaining_distance > 0.05:
    #                             self.navigate_around_obstacle(remaining_distance)
    #                         return
        
    #         current_time = time.time()
            
    #         # Process callbacks for fresh sensor data
    #         rclpy.spin_once(self, timeout_sec=0.01)
            
    #         # Calculate distance traveled
    #         distance_traveled = math.sqrt(
    #             (self.current_x - start_x)**2 + 
    #             (self.current_y - start_y)**2
    #         )
            
    #         # ENHANCED OBSTACLE MONITORING - Only for forward movements
    #         if not skip_obstacle_avoidance and not is_backward_movement:
    #             self.check_path_obstacles()
            
    #             # NEW: Track obstacle distance trend for predictive stopping
    #             if hasattr(self, 'obstacle_distance') and self.obstacle_distance != float('inf'):
    #                 previous_distances.append(self.obstacle_distance)
    #                 if len(previous_distances) > distance_trend_threshold:
    #                     previous_distances.pop(0)  # Keep only recent readings
                    
    #                 # Calculate trend (if distance is decreasing rapidly)
    #                 if len(previous_distances) >= 3:
    #                     recent_change = previous_distances[-1] - previous_distances[-3]
    #                     if recent_change < -0.05:  # Distance decreasing by 5cm over 3 readings
    #                         self.get_logger().warn(f'RAPID APPROACH DETECTED: Distance decreased by {-recent_change*100:.1f}cm in 3 readings')
            
    #         # NEW: Multi-level obstacle response (only for forward movement)
    #         if not is_backward_movement and not skip_obstacle_avoidance and self.obstacle_ahead:
    #             current_clearance = getattr(self, 'obstacle_distance', float('inf'))
                
    #             self.get_logger().info(f'=== OBSTACLE DETECTED DURING MOVEMENT ===')
    #             self.get_logger().info(f'Loop: {loop_count}, Distance traveled: {distance_traveled:.3f}m / {distance:.3f}m')
    #             self.get_logger().info(f'Obstacle clearance: {current_clearance:.3f}m')
                
    #             # EMERGENCY STOP - Immediate halt with backward movement if too close
    #             if current_clearance < (self.MINIMUM_SAFE_CLEARANCE - self.CLEARANCE_TOLERANCE):
    #                 self.get_logger().error(f'EMERGENCY STOP! Clearance {current_clearance:.3f}m < {self.MINIMUM_SAFE_CLEARANCE:.3f}m')
    #                 twist.linear.x = 0.0
    #                 twist.angular.z = 0.0
    #                 self.cmd_vel_pub.publish(twist)
    #                 time.sleep(0.1)  # Brief pause to ensure stop
                    
    #                 # Calculate backup distance to achieve target clearance
    #                 backup_distance = (self.TARGET_APPROACH_CLEARANCE - current_clearance) + 0.05
    #                 self.get_logger().warn(f'EMERGENCY BACKUP: {backup_distance:.3f}m to achieve {self.TARGET_APPROACH_CLEARANCE:.3f}m clearance')
                    
    #                 # Move backwards (skip_obstacle_avoidance=True to prevent recursion)
    #                 self.move_distance(-backup_distance, "emergency backup from obstacle", skip_obstacle_avoidance=True)

    #                 # Start obstacle avoidance with remaining distance
    #                 remaining_distance = distance - distance_traveled
    #                 if remaining_distance > 0.05:
    #                     self.navigate_around_obstacle(remaining_distance)
    #                 return
                
    #             # EARLY WARNING - Slow down but continue
    #             elif current_clearance < early_warning_distance:
    #                 speed_factor = (current_clearance - emergency_stop_distance) / (early_warning_distance - emergency_stop_distance)
    #                 speed_factor = max(0.2, min(1.0, speed_factor))  # Between 20% and 100% speed
                    
    #                 self.get_logger().warn(f'EARLY WARNING: Slowing to {speed_factor*100:.0f}% speed (clearance: {current_clearance:.3f}m)')
    #                 twist.linear.x = self.forward_speed * speed_factor
                    
    #                 # If we're moving very slowly and still detecting obstacles, start avoidance
    #                 if speed_factor < 0.4 and loop_count > 20:  # Give it some time
    #                     self.get_logger().info(f'SLOW SPEED AVOIDANCE: Starting avoidance due to slow progress')
    #                     twist.linear.x = 0.0
    #                     twist.angular.z = 0.0
    #                     self.cmd_vel_pub.publish(twist)
                        
    #                     remaining_distance = distance - distance_traveled
    #                     if remaining_distance > 0.05:
    #                         self.navigate_around_obstacle(remaining_distance)
    #                     return
                
    #             # NORMAL OBSTACLE - Continue but monitor closely
    #             else:
    #                 self.get_logger().info(f'OBSTACLE MONITORING: Clearance {current_clearance:.3f}m - continuing')
    #                 twist.linear.x = self.forward_speed
    #         else:
    #             # No obstacle or backward movement - set appropriate speed
    #             if is_backward_movement:
    #                 twist.linear.x = -self.forward_speed  # Negative speed for backward
    #             else:
    #                 twist.linear.x = self.forward_speed   # Positive speed for forward
            
    #         # Check if target distance reached
    #         if abs(distance_traveled) >= abs(distance):
    #             twist.linear.x = 0.0
    #             twist.angular.z = 0.0
    #             self.cmd_vel_pub.publish(twist)
    #             self.get_logger().info(f'Distance completed: {distance_traveled:.3f}m in {loop_count} loops')
    #             self.get_logger().info(f'Maximum heading error: {math.degrees(max_heading_error):.2f}°')
    #             self.log_heading_state("MOVE_COMPLETE")
    #             break
            
    #         # Heading correction
    #         current_heading = self.get_current_heading()
    #         heading_error = target_heading - current_heading
    #         heading_error = self.normalize_angle(heading_error)
    #         max_heading_error = max(max_heading_error, abs(heading_error))
            
    #         angular_correction = -self.kp_heading * heading_error
    #         twist.angular.z = angular_correction
            
    #         # Publish movement command
    #         self.cmd_vel_pub.publish(twist)
            
    #         # Enhanced logging
    #         if current_time - self.last_log_time > 1.0:
    #             progress_percent = min(100, (distance_traveled / abs(distance)) * 100)
                
    #             direction = "backward" if is_backward_movement else "forward"
    #             log_msg = f'{direction.capitalize()} Progress: {distance_traveled:.3f}m / {abs(distance):.3f}m ({progress_percent:.0f}%)'
    #             log_msg += f' | Heading: {math.degrees(current_heading):.2f}° (error: {math.degrees(heading_error):.2f}°)'
                
    #             # Add obstacle status to regular logs (only for forward movement)
    #             if not is_backward_movement and hasattr(self, 'obstacle_ahead'):
    #                 obstacle_status = "OBSTACLE" if self.obstacle_ahead else "CLEAR"
    #                 if hasattr(self, 'obstacle_distance') and self.obstacle_distance != float('inf'):
    #                     log_msg += f' | {obstacle_status} ({self.obstacle_distance:.3f}m)'
    #                 else:
    #                     log_msg += f' | {obstacle_status}'
                
    #             log_msg += f' | Loops: {loop_count}'
    #             self.get_logger().info(log_msg)
    #             self.last_log_time = current_time

    #         # Reduced sleep for more responsive obstacle detection
    #         time.sleep(0.02)  # 50Hz instead of 20Hz for better responsiveness
    
    def move_distance(self, distance, description="", skip_obstacle_avoidance=False):
        """Move forward for a specific distance with continuous obstacle monitoring"""
        
        # Determine if this is a backward movement
        is_backward_movement = distance < 0
        
        # Pre-movement obstacle check (only for forward movements, not during obstacle avoidance)
        if not skip_obstacle_avoidance and not is_backward_movement:
            self.get_logger().info(f'=== PRE-MOVEMENT OBSTACLE CHECK ===')
            self.check_path_obstacles()
            
            if hasattr(self, 'obstacle_ahead') and self.obstacle_ahead:
                current_clearance = getattr(self, 'obstacle_distance', float('inf'))
                self.get_logger().info(f'OBSTACLE STATUS: obstacle_ahead={self.obstacle_ahead}')
                self.get_logger().info(f'OBSTACLE DISTANCE: {current_clearance:.3f}m')
                self.get_logger().info(f'PLANNED MOVEMENT: {distance:.3f}m')
                self.get_logger().info(f'MINIMUM_SAFE_CLEARANCE: {self.MINIMUM_SAFE_CLEARANCE:.3f}m (±{self.CLEARANCE_TOLERANCE:.3f}m)')
                
                # NEW: Check if we need to backup first
                if current_clearance < (self.MINIMUM_SAFE_CLEARANCE - self.CLEARANCE_TOLERANCE):
                    backup_distance = (self.TARGET_APPROACH_CLEARANCE - current_clearance) + 0.05
                    self.get_logger().warn(f'TOO CLOSE: {current_clearance:.3f}m < {self.MINIMUM_SAFE_CLEARANCE:.3f}m')
                    self.get_logger().warn(f'BACKING UP: {backup_distance:.3f}m to create safe clearance')
                    
                    # Execute backup
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(0.1)
                    
                    self.move_distance(-backup_distance, "emergency clearance backup", skip_obstacle_avoidance=True)
                    time.sleep(0.2)  # Brief pause for sensor update
                
                if distance > 0.1:  # For movements > 10cm
                    self.get_logger().info(f'DECISION: Starting obstacle avoidance')
                    self.navigate_around_obstacle(distance)
                    return
                else:
                    self.get_logger().info(f'DECISION: Small distance ({distance:.3f}m), proceeding with caution')

        else:
            if is_backward_movement:
                self.get_logger().info(f'=== BACKWARD MOVEMENT (skipping obstacle checks) ===')
            else:
                self.get_logger().info(f'=== OBSTACLE AVOIDANCE MOVEMENT (skipping pre-check) ===')

        if description:
            direction = "backward" if is_backward_movement else "forward"
            self.get_logger().info(f'Starting {description} ({direction}): {abs(distance):.2f}m')
        else:
            direction = "backward" if is_backward_movement else "forward"
            self.get_logger().info(f'Moving {direction}: {abs(distance):.2f}m')
        
        # Log initial state
        self.log_heading_state("MOVE_START")
        
        # Record starting position and heading
        start_x = self.current_x
        start_y = self.current_y
        target_heading = self.get_current_heading()

        self.get_logger().info(f'TARGET HEADING SET TO: {math.degrees(target_heading):.2f}°')
        
        twist = Twist()
        max_heading_error = 0.0
        loop_count = 0
        last_obstacle_check = 0
        
        # NEW: Variables for continuous monitoring (only for forward movement)
        previous_distances = []  # Track last few distance readings
        distance_trend_threshold = 5  # Number of readings to analyze
        early_warning_distance = 0.35  # Start slowing down at 35cm
        emergency_stop_distance = 0.15  # Emergency stop at 15cm
        
        while rclpy.ok():
            loop_count += 1

            # Simple stuck detection (only for forward movement)
            if not is_backward_movement and loop_count > 50:
                distance_traveled = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
                if distance_traveled < 0.05:  # 50 loops with less than 5cm progress
                    if self.obstacle_ahead:
                        current_clearance = getattr(self, 'obstacle_distance', float('inf'))
                        if current_clearance < self.TARGET_APPROACH_CLEARANCE:
                            self.get_logger().warn(f'ROBOT APPEARS STUCK: Backing up to create space')
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.cmd_vel_pub.publish(twist)
                            
                            backup_distance = max(0.10, (self.TARGET_APPROACH_CLEARANCE - current_clearance) + 0.05)
                            self.move_distance(-backup_distance, "stuck recovery backup", skip_obstacle_avoidance=True)
                            
                            remaining_distance = distance - distance_traveled
                            if remaining_distance > 0.05:
                                self.navigate_around_obstacle(remaining_distance)
                            return
        
            current_time = time.time()
            
            # Process callbacks for fresh sensor data
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Calculate distance traveled
            distance_traveled = math.sqrt(
                (self.current_x - start_x)**2 + 
                (self.current_y - start_y)**2
            )
            
            # ENHANCED OBSTACLE MONITORING - Only for forward movements
            if not skip_obstacle_avoidance and not is_backward_movement:
                self.check_path_obstacles()
            
                # NEW: Track obstacle distance trend for predictive stopping
                if hasattr(self, 'obstacle_distance') and self.obstacle_distance != float('inf'):
                    previous_distances.append(self.obstacle_distance)
                    if len(previous_distances) > distance_trend_threshold:
                        previous_distances.pop(0)  # Keep only recent readings
                    
                    # Calculate trend (if distance is decreasing rapidly)
                    if len(previous_distances) >= 3:
                        recent_change = previous_distances[-1] - previous_distances[-3]
                        if recent_change < -0.05:  # Distance decreasing by 5cm over 3 readings
                            self.get_logger().warn(f'RAPID APPROACH DETECTED: Distance decreased by {-recent_change*100:.1f}cm in 3 readings')
            
            # NEW: Multi-level obstacle response (only for forward movement)
            if not is_backward_movement and not skip_obstacle_avoidance and self.obstacle_ahead:
                current_clearance = getattr(self, 'obstacle_distance', float('inf'))
                
                self.get_logger().info(f'=== OBSTACLE DETECTED DURING MOVEMENT ===')
                self.get_logger().info(f'Loop: {loop_count}, Distance traveled: {distance_traveled:.3f}m / {distance:.3f}m')
                self.get_logger().info(f'Obstacle clearance: {current_clearance:.3f}m')
                
                # CHECK: Are we in an avoidance maneuver and possibly detecting a wall?
                if hasattr(self, 'in_avoidance_maneuver') and self.in_avoidance_maneuver:
                    self.get_logger().info('AVOIDANCE MODE: Checking if this is wall detection during return path')
                    
                    # If we're close to completing the movement and clearance isn't critically dangerous
                    progress_ratio = distance_traveled / abs(distance) if distance != 0 else 1.0
                    if (progress_ratio > 0.7 and current_clearance > 0.08):  # 70% progress and not critically close
                        self.get_logger().info(f'WALL DETECTION: {progress_ratio*100:.0f}% progress, {current_clearance:.2f}m clearance - STOPPING avoidance here')
                        
                        # Stop movement
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(twist)
                        
                        # Clear avoidance state - robot is now at a safe position to continue normally
                        self.in_avoidance_maneuver = False
                        if hasattr(self, 'avoidance_original_heading'):
                            self.avoidance_original_heading = None
                        
                        self.get_logger().info('AVOIDANCE COMPLETE - robot at safe distance from original obstacle')
                        self.get_logger().info('Exiting movement - robot can continue normally from this position')
                        return  # Exit the movement, robot is at a safe position
                
                # EMERGENCY STOP - Immediate halt with backward movement if too close
                if current_clearance < (self.MINIMUM_SAFE_CLEARANCE - self.CLEARANCE_TOLERANCE):
                    self.get_logger().error(f'EMERGENCY STOP! Clearance {current_clearance:.3f}m < {self.MINIMUM_SAFE_CLEARANCE:.3f}m')
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(0.1)  # Brief pause to ensure stop
                    
                    # Calculate backup distance to achieve target clearance
                    backup_distance = (self.TARGET_APPROACH_CLEARANCE - current_clearance) + 0.05
                    self.get_logger().warn(f'EMERGENCY BACKUP: {backup_distance:.3f}m to achieve {self.TARGET_APPROACH_CLEARANCE:.3f}m clearance')
                    
                    # Move backwards (skip_obstacle_avoidance=True to prevent recursion)
                    self.move_distance(-backup_distance, "emergency backup from obstacle", skip_obstacle_avoidance=True)

                    # Start obstacle avoidance with remaining distance (only if not already in avoidance)
                    remaining_distance = distance - distance_traveled
                    if remaining_distance > 0.05 and not (hasattr(self, 'in_avoidance_maneuver') and self.in_avoidance_maneuver):
                        self.navigate_around_obstacle(remaining_distance)
                    return
                
                # EARLY WARNING - Slow down but continue
                elif current_clearance < early_warning_distance:
                    speed_factor = (current_clearance - emergency_stop_distance) / (early_warning_distance - emergency_stop_distance)
                    speed_factor = max(0.2, min(1.0, speed_factor))  # Between 20% and 100% speed
                    
                    self.get_logger().warn(f'EARLY WARNING: Slowing to {speed_factor*100:.0f}% speed (clearance: {current_clearance:.3f}m)')
                    twist.linear.x = self.forward_speed * speed_factor
                    
                    # If we're moving very slowly and still detecting obstacles, start avoidance
                    if speed_factor < 0.4 and loop_count > 20:  # Give it some time
                        self.get_logger().info(f'SLOW SPEED AVOIDANCE: Starting avoidance due to slow progress')
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(twist)
                        
                        remaining_distance = distance - distance_traveled
                        if remaining_distance > 0.05:
                            self.navigate_around_obstacle(remaining_distance)
                        return
                
                # NORMAL OBSTACLE - Continue but monitor closely
                else:
                    self.get_logger().info(f'OBSTACLE MONITORING: Clearance {current_clearance:.3f}m - continuing')
                    twist.linear.x = self.forward_speed
            else:
                # No obstacle or backward movement - set appropriate speed
                if is_backward_movement:
                    twist.linear.x = -self.forward_speed  # Negative speed for backward
                else:
                    twist.linear.x = self.forward_speed   # Positive speed for forward
            
            # Check if target distance reached
            if abs(distance_traveled) >= abs(distance):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f'Distance completed: {distance_traveled:.3f}m in {loop_count} loops')
                self.get_logger().info(f'Maximum heading error: {math.degrees(max_heading_error):.2f}°')
                self.log_heading_state("MOVE_COMPLETE")
                break
            
            # Heading correction
            current_heading = self.get_current_heading()
            heading_error = target_heading - current_heading
            heading_error = self.normalize_angle(heading_error)
            max_heading_error = max(max_heading_error, abs(heading_error))
            
            angular_correction = -self.kp_heading * heading_error
            twist.angular.z = angular_correction
            
            # Publish movement command
            self.cmd_vel_pub.publish(twist)
            
            # Enhanced logging
            if current_time - self.last_log_time > 1.0:
                progress_percent = min(100, (distance_traveled / abs(distance)) * 100)
                
                direction = "backward" if is_backward_movement else "forward"
                log_msg = f'{direction.capitalize()} Progress: {distance_traveled:.3f}m / {abs(distance):.3f}m ({progress_percent:.0f}%)'
                log_msg += f' | Heading: {math.degrees(current_heading):.2f}° (error: {math.degrees(heading_error):.2f}°)'
                
                # Add obstacle status to regular logs (only for forward movement)
                if not is_backward_movement and hasattr(self, 'obstacle_ahead'):
                    obstacle_status = "OBSTACLE" if self.obstacle_ahead else "CLEAR"
                    if hasattr(self, 'obstacle_distance') and self.obstacle_distance != float('inf'):
                        log_msg += f' | {obstacle_status} ({self.obstacle_distance:.3f}m)'
                    else:
                        log_msg += f' | {obstacle_status}'
                
                log_msg += f' | Loops: {loop_count}'
                self.get_logger().info(log_msg)
                self.last_log_time = current_time

            # Reduced sleep for more responsive obstacle detection
            time.sleep(0.02)  # 50Hz instead of 20Hz for better responsiveness
    
    def check_path_obstacles(self):
        """Optimized obstacle checking with faster processing"""
        if self.latest_scan is None:
            self.obstacle_ahead = False
            return
        
        ranges = np.array(self.latest_scan.ranges)
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        # Robot dimensions
        ROBOT_LENGTH = 0.304
        LIDAR_OFFSET_FROM_FRONT = ROBOT_LENGTH / 2
        
        # Focus only on front sector for speed during movement
        front_sector = (-math.pi/12, math.pi/12)  # ±15° around 0°
        
        # Quick front obstacle check
        front_ranges = []
        for i, range_val in enumerate(ranges):
            angle = angle_min + i * angle_increment + math.pi
            angle = self.normalize_angle(angle)
            
            if (front_sector[0] <= angle <= front_sector[1] and 
                self.latest_scan.range_min <= range_val <= 3.0):
                front_ranges.append(range_val)
        
        if front_ranges:
            raw_distance = min(front_ranges)
            self.obstacle_distance = raw_distance - LIDAR_OFFSET_FROM_FRONT
            
            # Use a slightly more conservative threshold for real-time detection
            detection_threshold = self.MINIMUM_SAFE_CLEARANCE + 0.10  # 10cm buffer for detection
            self.obstacle_ahead = self.obstacle_distance < detection_threshold

            # Only log detailed info periodically to avoid spam
            current_time = time.time()
            if not hasattr(self, 'last_detailed_log') or current_time - self.last_detailed_log > 2.0:
                self.get_logger().info(f'FRONT OBSTACLE: Raw={raw_distance:.3f}m, Corrected={self.obstacle_distance:.3f}m, Detected={self.obstacle_ahead}')
                self.last_detailed_log = current_time
        else:
            self.obstacle_ahead = False
            self.obstacle_distance = float('inf')

    def check_side_clearances(self):
        """Check clearance on both left and right sides for optimal avoidance direction"""
        if self.latest_scan is None:
            return float('inf'), float('inf')
        
        ranges = np.array(self.latest_scan.ranges)
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        # Define side sectors (90° left and right from robot front)
        # Adjusted for the coordinate transformation (+π)
        left_sector = (math.pi/4, math.pi/2 + math.pi/4)    # 45° to 135° (left side)
        right_sector = (-math.pi/2 - math.pi/4, -math.pi/4)  # -135° to -45° (right side)
        
        left_distances = []
        right_distances = []
        
        # Check scan range for debugging
        total_angle_range = len(ranges) * angle_increment
        self.get_logger().info(f'Scan info: {len(ranges)} points, angle_min={angle_min:.2f}, increment={angle_increment:.4f}, total_range={total_angle_range:.2f}')
        
        for i, range_val in enumerate(ranges):
            angle = angle_min + i * angle_increment + math.pi
            angle = self.normalize_angle(angle)
            
            # Only consider valid ranges
            if self.latest_scan.range_min <= range_val <= 3.0:
                # Check left side
                if left_sector[0] <= angle <= left_sector[1]:
                    left_distances.append(range_val)
                
                # Check right side  
                elif right_sector[0] <= angle <= right_sector[1]:
                    right_distances.append(range_val)
        
        # Calculate minimum distances (closest obstacles on each side)
        left_clearance = min(left_distances) if left_distances else float('inf')
        right_clearance = min(right_distances) if right_distances else float('inf')
        
        # Apply LIDAR offset correction for more accurate distance
        ROBOT_WIDTH = 0.178  # Typical robot width
        LIDAR_OFFSET_FROM_SIDE = ROBOT_WIDTH / 2
        
        if left_clearance != float('inf'):
            left_clearance = left_clearance - LIDAR_OFFSET_FROM_SIDE
        if right_clearance != float('inf'):
            right_clearance = right_clearance - LIDAR_OFFSET_FROM_SIDE
        
        self.get_logger().info(f'Side clearances - Left: {left_clearance:.2f}m, Right: {right_clearance:.2f}m')
        self.get_logger().info(f'Left samples: {len(left_distances)}, Right samples: {len(right_distances)}')
        
        return left_clearance, right_clearance

    def choose_avoidance_direction(self):
        """Determine optimal direction for obstacle avoidance"""
        left_clearance, right_clearance = self.check_side_clearances()
        
        # Minimum required clearance for safe navigation
        MINIMUM_SIDE_CLEARANCE = 0.4  # 40cm minimum clearance needed
        
        # Decision logic
        if left_clearance >= MINIMUM_SIDE_CLEARANCE and right_clearance >= MINIMUM_SIDE_CLEARANCE:
            # Both sides clear - choose the side with more clearance
            if left_clearance > right_clearance:
                direction = "left"
                clearance = left_clearance
                self.get_logger().info(f'Both sides clear - choosing LEFT (better clearance: {left_clearance:.2f}m vs {right_clearance:.2f}m)')
            else:
                direction = "right" 
                clearance = right_clearance
                self.get_logger().info(f'Both sides clear - choosing RIGHT (better clearance: {right_clearance:.2f}m vs {left_clearance:.2f}m)')
        
        elif left_clearance >= MINIMUM_SIDE_CLEARANCE:
            # Only left side clear
            direction = "left"
            clearance = left_clearance
            self.get_logger().info(f'Only LEFT side clear (clearance: {left_clearance:.2f}m, right blocked: {right_clearance:.2f}m)')
        
        elif right_clearance >= MINIMUM_SIDE_CLEARANCE:
            # Only right side clear
            direction = "right"
            clearance = right_clearance
            self.get_logger().info(f'Only RIGHT side clear (clearance: {right_clearance:.2f}m, left blocked: {left_clearance:.2f}m)')
        
        else:
            # Both sides blocked - choose the less blocked side or default to left
            if left_clearance > right_clearance:
                direction = "left"
                clearance = left_clearance
                self.get_logger().warn(f'BOTH SIDES BLOCKED - choosing LEFT as less blocked (L:{left_clearance:.2f}m vs R:{right_clearance:.2f}m)')
            else:
                direction = "right"
                clearance = right_clearance  
                self.get_logger().warn(f'BOTH SIDES BLOCKED - choosing RIGHT as less blocked (R:{right_clearance:.2f}m vs L:{left_clearance:.2f}m)')
        
        return direction, clearance

    # ADDITIONAL HELPER METHOD for even more responsive detection
    def emergency_obstacle_check(self):
        """Ultra-fast obstacle check for emergency situations"""
        if self.latest_scan is None:
            return False, float('inf')
        
        ranges = np.array(self.latest_scan.ranges)
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        # Check only the most critical front beams
        center_beam_count = min(10, len(ranges) // 8)  # Check center 10 beams or 1/8 of total
        center_start = len(ranges) // 2 - center_beam_count // 2
        
        min_distance = float('inf')
        for i in range(center_start, center_start + center_beam_count):
            if i < len(ranges):
                range_val = ranges[i]
                if self.latest_scan.range_min <= range_val <= 3.0:
                    min_distance = min(min_distance, range_val)
        
        if min_distance != float('inf'):
            corrected_distance = min_distance - 0.152  # Quick correction
            return corrected_distance < 0.12, corrected_distance  # 12cm emergency threshold
        
        return False, float('inf')

    def measure_obstacle_width_from_front(self):
        """Measure obstacle width when approaching from the front"""
        if self.latest_scan is None:
            return 0.3  # Default for unknown obstacles
        
        ranges = np.array(self.latest_scan.ranges)
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        # Wider front sector for width measurement
        front_sector = (-math.pi/4, math.pi/4)  # ±45° for better width detection
        
        obstacle_angles = []
        obstacle_distances = []
        
        for i, range_val in enumerate(ranges):
            angle = angle_min + i * angle_increment + math.pi
            angle = self.normalize_angle(angle)
            
            if front_sector[0] <= angle <= front_sector[1]:
                if self.latest_scan.range_min <= range_val <= 2.0:
                    corrected_distance = range_val - (self.ROBOT_LENGTH / 2)
                    if corrected_distance < (self.MINIMUM_SAFE_CLEARANCE + 0.3):  # Obstacle detection threshold
                        obstacle_angles.append(angle)
                        obstacle_distances.append(corrected_distance)
        
        if not obstacle_angles or len(obstacle_angles) < 3:  # Need at least 3 points for reliable measurement
            self.get_logger().info('Insufficient points for width measurement, using default')
            return 0.3
        
        # Find the angular span of the obstacle
        angular_span = max(obstacle_angles) - min(obstacle_angles)
        avg_distance = sum(obstacle_distances) / len(obstacle_distances)
        
        # Calculate width using arc length formula: width = distance × angular_span
        estimated_width = avg_distance * angular_span
        
        # For cylindrical objects, we might see them as wider due to scanning angle
        # Apply correction factor for typical cylindrical obstacles
        corrected_width = estimated_width * 0.8  # Empirical correction
        
        self.get_logger().info(f'Width measurement from front:')
        self.get_logger().info(f'  - Angular span: {math.degrees(angular_span):.1f}°')
        self.get_logger().info(f'  - Average distance: {avg_distance:.2f}m')
        self.get_logger().info(f'  - Raw width estimate: {estimated_width:.2f}m')
        self.get_logger().info(f'  - Corrected width: {corrected_width:.2f}m')
        self.get_logger().info(f'  - Scan points used: {len(obstacle_angles)}')
        
        return max(0.15, min(corrected_width, 0.8))  # Clamp between 15cm and 80cm
    
    def calculate_sideways_clearance(self, obstacle_width):
        """Calculate required sideways clearance for differential drive robot"""
        # For differential drive robot rotating on center axis:
        # Need: robot_radius + obstacle_radius + safety_margin
        
        ROBOT_RADIUS = 0.178 / 2  # Half of robot width (turning radius)
        obstacle_radius = obstacle_width / 2
        SAFETY_MARGIN = 0.05  # 5cm safety margin
        
        required_clearance = ROBOT_RADIUS + obstacle_radius + SAFETY_MARGIN
        
        self.get_logger().info(f'Sideways clearance calculation:')
        self.get_logger().info(f'  - Robot turning radius: {ROBOT_RADIUS:.2f}m')
        self.get_logger().info(f'  - Obstacle radius: {obstacle_radius:.2f}m')
        self.get_logger().info(f'  - Safety margin: {SAFETY_MARGIN:.2f}m')
        self.get_logger().info(f'  - Total required clearance: {required_clearance:.2f}m')
        
        return max(0.2, min(required_clearance, 0.6))  # Clamp between 20cm and 60cm

    def measure_obstacle_length_from_side(self):
        """Measure obstacle length after turning 90° (now viewing from the side)"""
        if self.latest_scan is None:
            return 0.5
        
        ranges = np.array(self.latest_scan.ranges)
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        # After turning 90°, the obstacle is now to our side
        # Check the side where obstacle should be (±45° from perpendicular)
        side_sector = (-math.pi/4, math.pi/4)  # ±45° from current forward direction
        
        obstacle_distances = []
        obstacle_angles = []
        
        for i, range_val in enumerate(ranges):
            angle = angle_min + i * angle_increment + math.pi
            angle = self.normalize_angle(angle)
            
            if side_sector[0] <= angle <= side_sector[1]:
                if self.latest_scan.range_min <= range_val <= 1.5:  # Closer range for side measurement
                    obstacle_distances.append(range_val)
                    obstacle_angles.append(angle)
        
        if not obstacle_distances: # or len(obstacle_distances) < 3:
            self.get_logger().info('Insufficient side scan data, using default length')
            return 0.5
        
        # Find the span of obstacle detection from the side
        min_distance = min(obstacle_distances)
        # filtered_distances = [d for d in obstacle_distances if d <= min_distance + 0.3]
        filtered_distances = [d for d in obstacle_distances if d <= min_distance + 0.15]
        filtered_angles = [obstacle_angles[i] for i, d in enumerate(obstacle_distances) if d <= min_distance + 0.3]

        # Use only the angular span of the filtered (close) points
        angular_span = max(filtered_angles) - min(filtered_angles) if len(filtered_angles) > 1 else 0.1
        avg_distance = sum(filtered_distances) / len(filtered_distances)

        self.get_logger().info(f'  - Min distance: {min_distance:.2f}m, Max filtered: {max(filtered_distances):.2f}m')
        self.get_logger().info(f'  - Filtered angular span: {math.degrees(angular_span):.1f}°')

        # Calculate length: for cylindrical object, length ≈ distance × angular_span
        estimated_length = avg_distance * angular_span        
        
        # Add safety margins: obstacle length + robot length + buffer
        safe_forward_distance = estimated_length + 0.304 + 0.2  # robot length + buffer
        
        self.get_logger().info(f'Length measurement from side:')
        self.get_logger().info(f'  - Side angular span: {math.degrees(angular_span):.1f}°')
        self.get_logger().info(f'  - Average side distance: {avg_distance:.2f}m')
        self.get_logger().info(f'  - Estimated length: {estimated_length:.2f}m')
        self.get_logger().info(f'  - Safe forward distance: {safe_forward_distance:.2f}m')
        self.get_logger().info(f'  - Total scan points: {len(obstacle_distances)}, Filtered: {len(filtered_distances)}')        
        return max(0.4, min(safe_forward_distance, 1.2))  # Clamp between 40cm and 1.2m

    def navigate_around_obstacle_left(self, original_distance, approach_distance):
        """Navigate around obstacle using LEFT side avoidance with smart measurements"""
        self.get_logger().info('=== LEFT SIDE AVOIDANCE SEQUENCE ===')
        
        # Step 1: Measure obstacle width from front (before turning)
        obstacle_width = self.measure_obstacle_width_from_front()
        sideways_distance = self.calculate_sideways_clearance(obstacle_width)
        
        # Step 2: Turn 90° left
        self.get_logger().info('Step 2: Turning left 90°')
        self.turn_left_90_degrees()
        
        # Step 3: Move sideways to clear obstacle
        self.get_logger().info(f'Step 3: Moving sideways {sideways_distance:.2f}m (calculated from width)')
        self.move_distance(sideways_distance, "moving sideways around obstacle")
        
        # Step 4: Turn 90° right (back to original heading) 
        self.get_logger().info('Step 4: Turning right 90° (back to original heading)')
        self.turn_right_90_degrees()
        
        # # Step 5: Measure obstacle length from side (after turning)
        # forward_distance = self.measure_obstacle_length_from_side()
        # self.get_logger().info(f'Step 5: Moving past obstacle {forward_distance:.2f}m (calculated from length)')
        # self.move_distance(forward_distance, "moving past obstacle")
        
        # Step 5: Move past obstacle with dynamic length detection
        self.get_logger().info('Step 5: Moving past obstacle with dynamic detection')
        forward_distance = self.move_past_obstacle_dynamically()

        # Step 6: Turn 90° right (toward original path)
        self.get_logger().info('Step 6: Turning right 90° (toward original path)')
        self.turn_right_90_degrees()
        
        # Step 7: Return to original path
        return_distance = sideways_distance
        self.get_logger().info(f'Step 7: Returning to path {return_distance:.2f}m')  
        self.move_distance(return_distance, "returning to original path")
        
        # Step 8: Turn 90° left (back to original heading)
        self.get_logger().info('Step 8: Turning left 90° (back to original heading)')
        self.turn_left_90_degrees()
        
        return forward_distance

    def navigate_around_obstacle_right(self, original_distance, approach_distance):
        """Navigate around obstacle using RIGHT side avoidance with smart measurements"""
        self.get_logger().info('=== RIGHT SIDE AVOIDANCE SEQUENCE ===')
        
        # Step 1: Measure obstacle width from front (before turning)
        obstacle_width = self.measure_obstacle_width_from_front()
        sideways_distance = self.calculate_sideways_clearance(obstacle_width)
        
        # Step 2: Turn 90° right
        self.get_logger().info('Step 2: Turning right 90°')
        self.turn_right_90_degrees()
        
        # Step 3: Move sideways to clear obstacle
        self.get_logger().info(f'Step 3: Moving sideways {sideways_distance:.2f}m (calculated from width)')
        self.move_distance(sideways_distance, "moving sideways around obstacle")
        
        # Step 4: Turn 90° left (back to original heading) 
        self.get_logger().info('Step 4: Turning left 90° (back to original heading)')
        self.turn_left_90_degrees()
        
        # # Step 5: Measure obstacle length from side (after turning)
        # forward_distance = self.measure_obstacle_length_from_side()
        # self.get_logger().info(f'Step 5: Moving past obstacle {forward_distance:.2f}m (calculated from length)')
        # self.move_distance(forward_distance, "moving past obstacle")
        
        # Step 5: Move past obstacle with dynamic length detection
        self.get_logger().info('Step 5: Moving past obstacle with dynamic detection')
        forward_distance = self.move_past_obstacle_dynamically()
        
        # Step 6: Turn 90° left (toward original path)
        self.get_logger().info('Step 6: Turning left 90° (toward original path)')
        self.turn_left_90_degrees()
        
        # Step 7: Return to original path
        return_distance = sideways_distance
        self.get_logger().info(f'Step 7: Returning to path {return_distance:.2f}m')  
        self.move_distance(return_distance, "returning to original path")
        
        # Step 8: Turn 90° right (back to original heading)
        self.get_logger().info('Step 8: Turning right 90° (back to original heading)')
        self.turn_right_90_degrees()
        
        return forward_distance
    
    def navigate_around_obstacle(self, original_distance):
        """Navigate around obstacle using intelligent side selection"""
        # Mark that we're starting an avoidance maneuver
        self.in_avoidance_maneuver = True
        self.avoidance_original_heading = self.get_current_heading()
        
        self.get_logger().info(f'Obstacle detected - Robot front clearance: {self.obstacle_distance:.2f}m')
        self.get_logger().info('=== OBSTACLE AVOIDANCE TRIGGERED ===')
        self.get_logger().info(f'Actual robot front clearance: {self.obstacle_distance:.2f}m')
        self.get_logger().info(f'Original planned distance: {original_distance:.2f}m')
        self.get_logger().info(f'Current position: ({self.current_x:.2f}, {self.current_y:.2f})')

        # Step 1: Approach obstacle closer (stop 10cm before robot front hits obstacle)
        target_clearance = 0.10  # Want 10cm clearance from robot front to obstacle
        current_clearance = self.obstacle_distance  # This is already corrected distance from robot front
        approach_distance = 0.0

        # Check if clearance is within acceptable range (target ± tolerance)
        if current_clearance > (self.TARGET_APPROACH_CLEARANCE + self.CLEARANCE_TOLERANCE):
            # Too far, approach closer
            approach_distance = current_clearance - self.TARGET_APPROACH_CLEARANCE
            self.get_logger().info(f'Step 1: Approaching obstacle - moving {approach_distance:.2f}m (current: {current_clearance:.2f}m, target: {self.TARGET_APPROACH_CLEARANCE:.2f}m)')
            self.move_distance(approach_distance, "approaching obstacle", skip_obstacle_avoidance=True)
        elif current_clearance < (self.TARGET_APPROACH_CLEARANCE - self.CLEARANCE_TOLERANCE):
            # Too close, back up to safe distance
            backup_distance = (self.TARGET_APPROACH_CLEARANCE - current_clearance) + 0.05
            approach_distance = -backup_distance  # Negative for backing up
            self.get_logger().info(f'Step 1: Too close for safe avoidance - backing up {backup_distance:.2f}m')
            self.move_distance(-backup_distance, "avoidance safety backup", skip_obstacle_avoidance=True)
        else:
            self.get_logger().info(f'Step 1: Clearance acceptable ({current_clearance:.2f}m ± {self.CLEARANCE_TOLERANCE:.2f}m), proceeding')

        # NEW: Determine best avoidance direction
        avoidance_direction, side_clearance = self.choose_avoidance_direction()
        
        # Execute appropriate avoidance maneuver
        if avoidance_direction == "left":
            forward_distance = self.navigate_around_obstacle_left(original_distance, approach_distance)
        else:  # right
            forward_distance = self.navigate_around_obstacle_right(original_distance, approach_distance)
        
        # Step 9: Continue with remaining distance
        total_avoidance_forward = max(0, approach_distance) + forward_distance
        remaining_distance = original_distance - total_avoidance_forward
        if remaining_distance > 0.05:
            self.get_logger().info(f'Step 9: Completing remaining distance {remaining_distance:.2f}m')
            self.move_distance(remaining_distance, "completing original distance")
        else:
            self.get_logger().info('Step 9: No remaining distance to cover')
        
        self.get_logger().info('=== OBSTACLE AVOIDANCE COMPLETED ===')

        # Clear avoidance state
        self.in_avoidance_maneuver = False
        self.avoidance_original_heading = None

    def move_past_obstacle_dynamically(self, max_distance=1.2):
        """Move forward while dynamically detecting when obstacle ends"""
        self.get_logger().info(f'Starting dynamic obstacle detection, max distance: {max_distance:.2f}m')
        
        start_position = self.get_current_position()
        obstacle_still_detected = True
        distance_moved = 0.0
        
        while obstacle_still_detected and distance_moved < max_distance:
            # Move forward in small increments
            increment = 0.1  # 10cm steps
            self.move_distance(increment, "dynamic obstacle detection")
            distance_moved += increment
            
            # Check if obstacle is still detected to our side
            obstacle_still_detected = self.is_obstacle_still_beside_us()
            
            self.get_logger().info(f'Moved {distance_moved:.2f}m, obstacle still detected: {obstacle_still_detected}')
            
            if not obstacle_still_detected:
                # Move a bit more for safety clearance
                safety_distance = 0.2
                self.move_distance(safety_distance, "safety clearance after obstacle")
                distance_moved += safety_distance
                self.get_logger().info(f'Obstacle cleared! Total distance: {distance_moved:.2f}m')
                break
        
        if distance_moved >= max_distance:
            self.get_logger().info(f'Reached max distance {max_distance:.2f}m, assuming obstacle cleared')
        
        return distance_moved

    def is_obstacle_still_beside_us(self):
        """Check if obstacle is still detected to our side (perpendicular direction)"""
        if self.latest_scan is None:
            return True  # Conservative: assume obstacle still there
        
        ranges = np.array(self.latest_scan.ranges)
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        # Look perpendicular to our movement (90° to the side)
        perpendicular_angles = [math.pi/2, -math.pi/2]  # Right and left sides
        detection_threshold = 1.0  # 1m detection range
        
        for target_angle in perpendicular_angles:
            for i, range_val in enumerate(ranges):
                angle = angle_min + i * angle_increment + math.pi
                angle = self.normalize_angle(angle)
                
                # Check if we're looking perpendicular (±15° tolerance)
                if abs(angle - target_angle) <= math.pi/12:
                    if self.latest_scan.range_min <= range_val <= detection_threshold:
                        return True  # Obstacle still detected
        
        return False  # No obstacle detected to the side
    
    def get_current_position(self):
        """Get current robot position"""
        if hasattr(self, 'current_position'):
            return self.current_position
        return (0.0, 0.0)  # Fallback

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
            
            # Set target heading - prefer IMU if available
            if self.target_heading_deg is not None:
                self.target_yaw = math.radians(self.target_heading_deg)
                self.get_logger().info(f'Using specified target heading: {self.target_heading_deg:.1f} degrees')
            else:
                current_heading = self.get_current_heading()
                self.target_yaw = current_heading
                heading_source = "IMU" if self.imu_received else "odometry"
                self.get_logger().info(f'Using current {heading_source} heading as target: {math.degrees(self.target_yaw):.1f} degrees')
            
            self.get_logger().info(f'Starting position: ({self.start_x:.2f}, {self.start_y:.2f})')
            self.get_logger().info(f'Current odometry heading: {math.degrees(self.current_yaw):.1f} degrees')
            if self.imu_received:
                self.get_logger().info(f'Current IMU heading: {math.degrees(self.current_yaw_imu):.1f} degrees')
    
    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def log_heading_state(self, context=""):
        """Log current heading state from all sources"""
        heading_source = "IMU" if self.imu_received else "odometry"
        current_heading = self.get_current_heading()
        
        log_msg = f'[{context}] Heading State:'
        log_msg += f' Current({heading_source}): {math.degrees(current_heading):.2f}°'
        
        if self.imu_received:
            log_msg += f', IMU: {math.degrees(self.current_yaw_imu):.2f}°'
        
        log_msg += f', Odom: {math.degrees(self.current_yaw):.2f}°'
        log_msg += f', Position: ({self.current_x:.3f}, {self.current_y:.3f})'
        
        self.get_logger().info(log_msg)
    
    def turn_right_90_degrees(self):
        """Turn exactly 90 degrees to the right using original bang-bang approach with IMU"""
        self.get_logger().info('Starting 90-degree right turn (using original approach with IMU)...')
        
        # Log initial state
        self.log_heading_state("TURN_START")
        
        # Calculate target heading (90 degrees clockwise from current)
        start_heading = self.get_current_heading()
        target_heading = self.normalize_angle(start_heading - math.pi/2)  # -90 degrees
        
        heading_source = "IMU" if self.imu_received else "odometry"
        self.get_logger().info(f'Turn start heading ({heading_source}): {math.degrees(start_heading):.1f}°')
        self.get_logger().info(f'Turn target heading: {math.degrees(target_heading):.1f}°')
        
        twist = Twist()
        
        while rclpy.ok():
            # Process callbacks for fresh sensor data
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Calculate heading error using IMU if available
            current_heading = self.get_current_heading()
            heading_error = target_heading - current_heading
            heading_error = self.normalize_angle(heading_error)
            
            # Check if turn is complete (within 0.5 degrees tolerance - same as original)
            total_turn = abs(self.normalize_angle(current_heading - start_heading))

            if total_turn >= math.radians(89.0):
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
                total_turn = self.normalize_angle(current_heading - start_heading)
                self.get_logger().info(f'Turn completed! Turned {math.degrees(total_turn):.1f}°')
                self.get_logger().info(f'Final heading ({heading_source}): {math.degrees(current_heading):.1f}°')
                
                # Log final state
                self.log_heading_state("TURN_COMPLETE")
                break
            
            # Apply turning velocity (negative for right turn) - same bang-bang as original
            if heading_error > 0:
                twist.angular.z = self.turn_speed
            else:
                twist.angular.z = -self.turn_speed
            
            twist.linear.x = 0.0  # No forward movement during turn
            self.cmd_vel_pub.publish(twist)
            
            # Log progress
            current_time = time.time()
            if current_time - self.last_log_time > 0.5:
                current_turn = self.normalize_angle(current_heading - start_heading)
                self.get_logger().info(f'Turning... Current: {math.degrees(current_heading):.1f}°, '
                                     f'Turned: {math.degrees(current_turn):.1f}°, '
                                     f'Error: {math.degrees(heading_error):.1f}°')
                self.last_log_time = current_time
            
            time.sleep(0.05)
    
    def turn_left_90_degrees(self):
        """Turn exactly 90 degrees to the left using original bang-bang approach with IMU"""
        self.get_logger().info('Starting 90-degree left turn (using original approach with IMU)...')
        
        # Log initial state
        self.log_heading_state("TURN_START")
        
        # Calculate target heading (90 degrees counter-clockwise from current)
        start_heading = self.get_current_heading()
        target_heading = self.normalize_angle(start_heading + math.pi/2)  # +90 degrees
        
        heading_source = "IMU" if self.imu_received else "odometry"
        self.get_logger().info(f'Turn start heading ({heading_source}): {math.degrees(start_heading):.1f}°')
        self.get_logger().info(f'Turn target heading: {math.degrees(target_heading):.1f}°')
        
        twist = Twist()
        
        while rclpy.ok():
            # Process callbacks for fresh sensor data
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Calculate heading error using IMU if available
            current_heading = self.get_current_heading()
            heading_error = target_heading - current_heading
            heading_error = self.normalize_angle(heading_error)
            
            # Check if turn is complete (within 0.5 degrees tolerance - same as original)
            total_turn = abs(self.normalize_angle(current_heading - start_heading))

            if total_turn >= math.radians(89.0):
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
                total_turn = self.normalize_angle(current_heading - start_heading)
                self.get_logger().info(f'Turn completed! Turned {math.degrees(total_turn):.1f}°')
                self.get_logger().info(f'Final heading ({heading_source}): {math.degrees(current_heading):.1f}°')
                
                # Log final state
                self.log_heading_state("TURN_COMPLETE")
                break
            
            # Apply turning velocity (positive for left turn) - same bang-bang as original
            if heading_error > 0:
                twist.angular.z = self.turn_speed
            else:
                twist.angular.z = -self.turn_speed
            
            twist.linear.x = 0.0  # No forward movement during turn
            self.cmd_vel_pub.publish(twist)
            
            # Log progress
            current_time = time.time()
            if current_time - self.last_log_time > 0.5:
                current_turn = self.normalize_angle(current_heading - start_heading)
                self.get_logger().info(f'Turning... Current: {math.degrees(current_heading):.1f}°, '
                                     f'Turned: {math.degrees(current_turn):.1f}°, '
                                     f'Error: {math.degrees(heading_error):.1f}°')
                self.last_log_time = current_time
            
            time.sleep(0.05)
    
    
    def execute_boustrophedon_pattern(self):
        """Execute the original simple boustrophedon pattern with IMU enhancement"""
        # Wait for first odometry message
        self.get_logger().info('Waiting for sensor data...')
        while not self.odom_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Wait a bit for IMU if available
        imu_wait_time = 0
        while not self.imu_received and imu_wait_time < 20 and rclpy.ok():  # Wait up to 2 seconds for IMU
            rclpy.spin_once(self, timeout_sec=0.1)
            imu_wait_time += 1
        
        if self.imu_received:
            self.get_logger().info('IMU data available - using IMU for precise heading control')
        else:
            self.get_logger().warn('IMU data not available - falling back to odometry for heading')
        
        if not rclpy.ok():
            return
        
        self.get_logger().info('=== Starting Original Boustrophedon Pattern with IMU Enhancement ===')
        
        # Log initial robot state
        self.log_heading_state("PATTERN_START")
        
        # Step 1: Move forward the target distance
        self.get_logger().info('STEP 1: Forward movement')
        self.move_distance(self.target_distance, "main forward sweep")
        
        time.sleep(0.5)  # Brief pause between steps
        
        # Step 2: First 90-degree right turn
        self.get_logger().info('STEP 2: First right turn')
        self.turn_right_90_degrees()
        
        time.sleep(0.5)
        
        # Step 3: Move offset distance (20cm)
        self.get_logger().info('STEP 3: Offset movement')
        self.move_distance(self.offset_distance, "offset movement")
        
        time.sleep(0.5)
        
        # Step 4: Second 90-degree right turn
        self.get_logger().info('STEP 4: Second right turn')
        self.turn_right_90_degrees()
        
        # Log final robot state
        self.log_heading_state("PATTERN_COMPLETE")
        
        self.get_logger().info('=== Original Boustrophedon Pattern Complete ===')
        self.get_logger().info('Robot should now be facing opposite direction, offset by 20cm')
    
    def execute_complete_boustrophedon_pattern(self):
        """Execute the complete room coverage boustrophedon pattern with IMU enhancement"""
        # Wait for sensors
        self.get_logger().info('Waiting for sensor data...')
        while (not self.odom_received or not self.scan_received) and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Wait a bit more for IMU if available
        imu_wait_time = 0
        while not self.imu_received and imu_wait_time < 20 and rclpy.ok():  # Wait up to 2 seconds for IMU
            rclpy.spin_once(self, timeout_sec=0.1)
            imu_wait_time += 1
        
        if self.imu_received:
            self.get_logger().info('IMU data available - using IMU for precise heading control')
        else:
            self.get_logger().warn('IMU data not available - falling back to odometry for heading')
        
        if not rclpy.ok():
            return
        
        # Calculate room dimensions
        if not self.calculate_room_width():
            self.get_logger().error('Failed to calculate room width, using default pattern')
            self.columns_needed = 3  # Default fallback
        
        self.get_logger().info('=== Starting Complete Boustrophedon Coverage Pattern with IMU ===')
        self.get_logger().info(f'Will execute {self.columns_needed} columns')
        
        # Log initial robot state
        self.log_heading_state("COMPLETE_PATTERN_START")
        
        for column in range(self.columns_needed):
            self.current_column = column
            self.get_logger().info(f'=== COLUMN {column + 1}/{self.columns_needed} ===')
            
            # Log state at start of each column
            self.log_heading_state(f"COLUMN_{column + 1}_START")
            
            # Step 1: Move forward the target distance
            self.get_logger().info(f'STEP 1: Forward sweep (Column {column + 1})')
            self.move_distance(self.target_distance, f"column {column + 1} forward sweep")
            time.sleep(0.5)
            
            # Check if this is the last column
            if column == self.columns_needed - 1:
                self.get_logger().info('=== Final column completed ===')
                break
            
            # Determine turn direction based on current pattern direction
            if self.moving_right:
                # Moving right: turn right, move offset, turn right
                self.get_logger().info('STEP 2: Right turn (moving right pattern)')
                self.turn_right_90_degrees()
                time.sleep(0.5)
                
                # self.get_logger().info('STEP 3: Offset movement')
                # self.move_distance(self.offset_distance, "offset movement")
                # time.sleep(0.5)
                
                self.get_logger().info('STEP 4: Right turn to face opposite direction')
                self.turn_right_90_degrees()
                time.sleep(0.5)
            else:
                # Moving left: turn left, move offset, turn left
                self.get_logger().info('STEP 2: Left turn (moving left pattern)')
                self.turn_left_90_degrees()
                time.sleep(0.5)
                
                # self.get_logger().info('STEP 3: Offset movement')
                # self.move_distance(self.offset_distance, "offset movement")
                # time.sleep(0.5)
                
                self.get_logger().info('STEP 4: Left turn to face opposite direction')
                self.turn_left_90_degrees()
                time.sleep(0.5)
            
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


def main(args=None):
    rclpy.init(args=args)
    
    controller = PIDStraightLineController()

    controller.test_lidar_orientation()
    time.sleep(2)  # Give time to read the logs
    
    try:
        # You can choose which pattern to execute:
        # controller.execute_boustrophedon_pattern()  # Original simple pattern
        controller.execute_complete_boustrophedon_pattern()  # Full room coverage
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