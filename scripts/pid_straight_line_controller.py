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
        
        # Parameters
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('kp_heading', 12.0)
        self.declare_parameter('target_heading_deg', -999.0)
        self.declare_parameter('turn_speed', 0.1)  # Angular velocity for turns
        self.declare_parameter('offset_distance', 0.2)  # 20cm offset
        self.declare_parameter('safety_margin', 0.15)  # 15cm safety margin from walls

        # NEW: Simple obstacle detection variables
        self.obstacle_ahead = False
        self.obstacle_distance = float('inf')
        
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

    # def check_path_obstacles(self):
    #     """Simple obstacle detection in forward path with detailed debugging"""
    #     if self.latest_scan is None:
    #         self.obstacle_ahead = False
    #         return
        
    #     ranges = np.array(self.latest_scan.ranges)
    #     angle_min = self.latest_scan.angle_min
    #     angle_increment = self.latest_scan.angle_increment
        
    #     # Check forward sector (±15 degrees) - angle 0 should be robot's front
    #     forward_angles = []
    #     forward_ranges = []
    #     forward_indices = []  # Track which LIDAR beam indices
        
    #     for i, range_val in enumerate(ranges):
    #         angle = angle_min + i * angle_increment + math.pi  # ADD 180° rotation back
    #         angle = self.normalize_angle(angle)
            
    #         if (-math.pi/12 <= angle <= math.pi/12 and  # ±15 degrees
    #             self.latest_scan.range_min <= range_val <= 2.0):
    #             forward_angles.append(angle)
    #             forward_ranges.append(range_val)
    #             forward_indices.append(i)  # Store beam index
        
    #     if forward_ranges:
    #         self.obstacle_distance = min(forward_ranges)
            
    #         # NEW: Find the index of the closest reading
    #         min_distance_idx = forward_ranges.index(self.obstacle_distance)
    #         closest_beam_idx = forward_indices[min_distance_idx]
    #         closest_angle = forward_angles[min_distance_idx]
            
    #         self.obstacle_ahead = self.obstacle_distance < 0.4
            
    #         # Detailed logging
    #         self.get_logger().info(f'OBSTACLE DETECTED:')
    #         self.get_logger().info(f'  Distance: {self.obstacle_distance:.2f}m')
    #         self.get_logger().info(f'  LIDAR beam index: {closest_beam_idx} (out of {len(ranges)} total beams)')
    #         self.get_logger().info(f'  Beam angle: {math.degrees(closest_angle):.1f}° (0° = robot front)')
    #         self.get_logger().info(f'  Total forward readings: {len(forward_ranges)}')
            
    #         # Also check what's at the back (180°) for comparison
    #         back_ranges = []
    #         for i, range_val in enumerate(ranges):
    #             angle = angle_min + i * angle_increment
    #             angle = self.normalize_angle(angle)
    #             # Check back sector (±15 degrees around 180°)
    #             if ((math.pi - math.pi/12) <= abs(angle) <= math.pi and  
    #                 self.latest_scan.range_min <= range_val <= 2.0):
    #                 back_ranges.append(range_val)
            
    #         if back_ranges:
    #             back_distance = min(back_ranges)
    #             self.get_logger().info(f'  For comparison - Back distance: {back_distance:.2f}m')
            
    #     else:
    #         self.obstacle_ahead = False      

    def check_path_obstacles(self):
        """Check obstacles in all directions with LIDAR mounting offset correction"""
        if self.latest_scan is None:
            self.obstacle_ahead = False
            return
        
        ranges = np.array(self.latest_scan.ranges)
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        # Robot dimensions and LIDAR mounting position
        ROBOT_LENGTH = 0.404  # 30.4cm in meters
        LIDAR_OFFSET_FROM_FRONT = ROBOT_LENGTH / 2  # 15.2cm - LIDAR is center-mounted
        LIDAR_OFFSET_FROM_BACK = ROBOT_LENGTH / 2   # 15.2cm - same distance to back
        LIDAR_OFFSET_FROM_SIDES = 0.202  # Assuming robot width is also 30.4cm and LIDAR is centered
        
        # Define sectors for each direction (±15 degrees each)
        sectors = {
            'FRONT': (-math.pi/12, math.pi/12),           # ±15° around 0°
            'RIGHT': (-math.pi/2 - math.pi/12, -math.pi/2 + math.pi/12),  # ±15° around -90°
            'BACK': (math.pi - math.pi/12, math.pi + math.pi/12),          # ±15° around 180°
            'LEFT': (math.pi/2 - math.pi/12, math.pi/2 + math.pi/12)       # ±15° around 90°
        }
        
        # Check each direction and apply appropriate offset correction
        direction_results = {}
        
        for direction_name, (min_angle, max_angle) in sectors.items():
            sector_ranges = []
            sector_angles = []
            sector_indices = []
            
            for i, range_val in enumerate(ranges):
                # Apply your LIDAR rotation (because it's mounted backwards)
                angle = angle_min + i * angle_increment + math.pi
                angle = self.normalize_angle(angle)
                
                # Check if angle is in this sector and range is valid
                if (min_angle <= angle <= max_angle and 
                    self.latest_scan.range_min <= range_val <= 3.0):
                    sector_ranges.append(range_val)
                    sector_angles.append(angle)
                    sector_indices.append(i)
            
            # Store results for this direction with offset correction
            if sector_ranges:
                raw_distance = min(sector_ranges)
                min_idx = sector_ranges.index(raw_distance)
                
                # Apply offset correction based on direction
                if direction_name == 'FRONT':
                    corrected_distance = raw_distance - LIDAR_OFFSET_FROM_FRONT
                elif direction_name == 'BACK':
                    corrected_distance = raw_distance - LIDAR_OFFSET_FROM_BACK
                elif direction_name in ['LEFT', 'RIGHT']:
                    corrected_distance = raw_distance - LIDAR_OFFSET_FROM_SIDES
                
                direction_results[direction_name] = {
                    'raw_distance': raw_distance,
                    'corrected_distance': corrected_distance,
                    'angle': sector_angles[min_idx],
                    'beam_index': sector_indices[min_idx],
                    'readings_count': len(sector_ranges)
                }
            else:
                direction_results[direction_name] = None
        
        # Log results for all directions with both raw and corrected distances
        self.get_logger().info('=== LIDAR SCAN WITH OFFSET CORRECTION ===')
        self.get_logger().info(f'Robot length: {ROBOT_LENGTH*100:.1f}cm, LIDAR offset from front: {LIDAR_OFFSET_FROM_FRONT*100:.1f}cm')
        
        for direction in ['FRONT', 'RIGHT', 'BACK', 'LEFT']:
            if direction_results[direction]:
                result = direction_results[direction]
                self.get_logger().info(
                    f'{direction:5}: Raw={result["raw_distance"]:.2f}m → Corrected={result["corrected_distance"]:.2f}m '
                    f'| beam {result["beam_index"]:3d} (angle: {math.degrees(result["angle"]):6.1f}°)'
                )
            else:
                self.get_logger().info(f'{direction:5}: NO OBSTACLES detected')
        
        # Set obstacle detection based on CORRECTED front distance
        if direction_results['FRONT']:
            self.obstacle_distance = direction_results['FRONT']['corrected_distance']
            # Use corrected distance for obstacle detection
            self.obstacle_ahead = self.obstacle_distance < 0.25  # Now this should be actual clearance from robot edge
            
            self.get_logger().info(f'OBSTACLE DETECTION:')
            self.get_logger().info(f'  Raw LIDAR reading: {direction_results["FRONT"]["raw_distance"]:.2f}m')
            self.get_logger().info(f'  Actual robot clearance: {self.obstacle_distance:.2f}m')
            self.get_logger().info(f'  Obstacle ahead: {self.obstacle_ahead}')
        else:
            self.obstacle_ahead = False
            self.obstacle_distance = float('inf')
            self.get_logger().info('NO OBSTACLE AHEAD detected')
        
        # Add separator for readability
        self.get_logger().info('=' * 70)

    def navigate_around_obstacle(self, original_distance):
        """Navigate around obstacle using corrected distances"""
        self.get_logger().info(f'Obstacle detected - Robot front clearance: {self.obstacle_distance:.2f}m')
        self.get_logger().info('=== OBSTACLE AVOIDANCE TRIGGERED ===')
        self.get_logger().info(f'Actual robot front clearance: {self.obstacle_distance:.2f}m')
        self.get_logger().info(f'Original planned distance: {original_distance:.2f}m')
        self.get_logger().info(f'Current position: ({self.current_x:.2f}, {self.current_y:.2f})')

        # Step 1: Approach obstacle closer (stop 10cm before robot front hits obstacle)
        # Since self.obstacle_distance is already corrected to robot edge, we can use it directly
        target_clearance = 0.20  # Want 10cm clearance from robot front to obstacle
        current_clearance = self.obstacle_distance  # This is already corrected distance from robot front

        if current_clearance > target_clearance + 0.05:  # Only approach if we're more than 15cm away
            approach_distance = current_clearance - target_clearance
            self.get_logger().info(f'Step 1: Approaching obstacle - moving {approach_distance:.2f}m (current clearance: {current_clearance:.2f}m)')
            self.move_distance(approach_distance, "approaching obstacle")
        else:
            self.get_logger().info(f'Step 1: Already close enough to obstacle (clearance: {current_clearance:.2f}m), skipping approach')

        
        # Step 2: Turn 90° left
        self.get_logger().info('Step 2: Turning left 90°')
        self.turn_left_90_degrees()
        
        # Step 3: Move sideways to clear obstacle
        sideways_distance = 0.3  # 50cm sideways
        self.get_logger().info(f'Step 3: Moving sideways {sideways_distance:.2f}m')
        self.move_distance(sideways_distance, "moving sideways around obstacle")
        
        # Step 4: Turn 90° right (back to original heading) 
        self.get_logger().info('Step 4: Turning right 90° (back to original heading)')
        self.turn_right_90_degrees()
        
        # Step 5: Move forward past obstacle
        forward_distance = 0.5  # 80cm forward past obstacle
        self.get_logger().info(f'Step 5: Moving past obstacle {forward_distance:.2f}m')
        self.move_distance(forward_distance, "moving past obstacle")
        
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
        
        # Step 9: Continue with remaining distance
        total_avoidance_forward = approach_distance + forward_distance
        remaining_distance = original_distance - total_avoidance_forward
        if remaining_distance > 0.05:
            self.get_logger().info(f'Step 9: Completing remaining distance {remaining_distance:.2f}m')
            self.move_distance(remaining_distance, "completing original distance")
        else:
            self.get_logger().info('Step 9: No remaining distance to cover')
        
        self.get_logger().info('=== OBSTACLE AVOIDANCE COMPLETED ===')
   
    def move_distance(self, distance, description=""):
        """Move forward for a specific distance using IMU for heading correction"""

        # Safety check: Don't move forward if we're too close to an obstacle
        if hasattr(self, 'obstacle_ahead') and self.obstacle_ahead and distance > 0:
            if hasattr(self, 'obstacle_distance') and self.obstacle_distance < 0.08:  # Less than 8cm clearance
                self.get_logger().warn(f'SAFETY: Refusing to move forward - too close to obstacle ({self.obstacle_distance:.3f}m clearance)')
                return

        # # NEW: Check for obstacles first
        # if hasattr(self, 'obstacle_ahead') and self.obstacle_ahead and distance > 0.5:
        #     self.get_logger().info('=== OBSTACLE AVOIDANCE DEBUG ===')
        #     self.log_lidar_debug()  # NEW: Log LIDAR sectors
        #     self.navigate_around_obstacle(distance)
        #     return
    
        if description:
            self.get_logger().info(f'Starting {description}: {distance:.2f}m')
        else:
            self.get_logger().info(f'Moving forward: {distance:.2f}m')
        
        # Log initial state before movement
        self.log_heading_state("MOVE_START")
        
        # Record starting position and heading (use IMU if available)
        start_x = self.current_x
        start_y = self.current_y
        target_heading = self.get_current_heading()  # Maintain current heading using IMU

        self.get_logger().info(f'TARGET HEADING SET TO: {math.degrees(target_heading):.2f}° (from current heading)')
        self.get_logger().info(f'This heading was captured at position: ({self.current_x:.3f}, {self.current_y:.3f})')

        
        heading_source = "IMU" if self.imu_received else "odometry"
        self.get_logger().info(f'Using {heading_source} for heading correction during movement')
        self.get_logger().info(f'Target heading for straight line: {math.degrees(target_heading):.2f}°')
        
        twist = Twist()
        max_heading_error = 0.0  # Track maximum heading deviation
        
        while rclpy.ok():
            # Process callbacks for fresh odometry and IMU
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Calculate distance traveled
            distance_traveled = math.sqrt(
                (self.current_x - start_x)**2 + 
                (self.current_y - start_y)**2
            )
            
            # CHECK FOR OBSTACLES DURING MOVEMENT (ADD THIS BLOCK)
            self.check_path_obstacles()
            if self.obstacle_ahead:
                self.get_logger().info(f'OBSTACLE DETECTED during movement at {distance_traveled:.3f}m!')
                self.get_logger().info(f'Obstacle clearance: {self.obstacle_distance:.3f}m')
                
                # Stop the robot immediately
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
                # Calculate remaining distance
                remaining_distance = distance - distance_traveled
                self.get_logger().info(f'Remaining distance: {remaining_distance:.3f}m')
                
                # Start obstacle avoidance with remaining distance
                if remaining_distance > 0.05:
                    self.navigate_around_obstacle(remaining_distance)
                else:
                    self.get_logger().info('Remaining distance too small, stopping here')
                return
            
            # Check if target distance reached
            if distance_traveled >= distance:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f'Distance completed: {distance_traveled:.3f}m')
                self.get_logger().info(f'Maximum heading error during movement: {math.degrees(max_heading_error):.2f}°')
                
                # Log final state after movement
                self.log_heading_state("MOVE_COMPLETE")
                break
            
            # Set forward velocity
            twist.linear.x = self.forward_speed
            
            # Heading correction to maintain straight line using IMU if available
            current_heading = self.get_current_heading()
            heading_error = target_heading - current_heading
            heading_error = self.normalize_angle(heading_error)
            
            # Track maximum heading error
            max_heading_error = max(max_heading_error, abs(heading_error))
            
            # Calculate angular correction
            # IMU is mounted upside down - flip sign to correct for inverted yaw axis

            angular_correction = -self.kp_heading * heading_error
            twist.angular.z = angular_correction
            
            # Publish command
            self.cmd_vel_pub.publish(twist)
            
            # Enhanced logging with heading information
            current_time = time.time()
            if current_time - self.last_log_time > 1.0:
                progress_percent = min(100, (distance_traveled / distance) * 100)
                
                # Log progress with detailed heading info
                log_msg = f'Distance: {distance_traveled:.3f}m / {distance:.3f}m ({progress_percent:.0f}%)'
                log_msg += f' | Heading: Current={math.degrees(current_heading):.2f}°'
                log_msg += f', Target={math.degrees(target_heading):.2f}°'
                log_msg += f', Error={math.degrees(heading_error):.2f}°'
                log_msg += f', Angular_Cmd={angular_correction:.3f}rad/s'
                log_msg += f' | Max_Error={math.degrees(max_heading_error):.2f}°'
                
                self.get_logger().info(log_msg)
                self.last_log_time = current_time

            self.get_logger().info(f'PID Debug: Error={math.degrees(heading_error):.2f}°, '
                    f'Correction={angular_correction:.3f}rad/s, '
                    f'Expected_turn_direction={"LEFT" if angular_correction > 0 else "RIGHT"}')

            
            time.sleep(0.05)
    
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
                
                self.get_logger().info('STEP 3: Offset movement')
                self.move_distance(self.offset_distance, "offset movement")
                time.sleep(0.5)
                
                self.get_logger().info('STEP 4: Right turn to face opposite direction')
                self.turn_right_90_degrees()
                time.sleep(0.5)
            else:
                # Moving left: turn left, move offset, turn left
                self.get_logger().info('STEP 2: Left turn (moving left pattern)')
                self.turn_left_90_degrees()
                time.sleep(0.5)
                
                self.get_logger().info('STEP 3: Offset movement')
                self.move_distance(self.offset_distance, "offset movement")
                time.sleep(0.5)
                
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