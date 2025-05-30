#!/usr/bin/env python3
"""
Base Robot Controller - Shared functionality for robot control
Handles sensor data, basic state management, and common utilities
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
import math
import time
import numpy as np


class BaseRobotController(Node):
    """Base controller class with shared sensor handling and utilities"""
    
    def __init__(self, node_name='base_robot_controller'):
        super().__init__(node_name)
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # Declare common parameters
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.1)
        self.declare_parameter('kp_heading', 12.0)
        self.declare_parameter('safety_margin', 0.15)
        
        # Get parameters
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.kp_heading = self.get_parameter('kp_heading').value
        self.safety_margin = self.get_parameter('safety_margin').value
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_yaw_imu = 0.0
        self.start_x = None
        self.start_y = None
        
        # Sensor status flags
        self.odom_received = False
        self.scan_received = False
        self.imu_received = False
        
        # LIDAR data
        self.latest_scan = None
        
        # Logging
        self.last_log_time = 0.0
        
        self.get_logger().info(f'Base Robot Controller initialized with node name: {node_name}')
        
    def imu_callback(self, msg):
        """Process IMU data for heading information"""
        orientation = msg.orientation
        self.current_yaw_imu = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        
        if not self.imu_received:
            self.imu_received = True
            self.get_logger().info('IMU data received')
            self.get_logger().info(f'Initial IMU heading: {math.degrees(self.current_yaw_imu):.1f} degrees')
    
    def scan_callback(self, msg):
        """Process LIDAR scan data"""
        self.latest_scan = msg
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info('LIDAR scan data received')
    
    def odom_callback(self, msg):
        """Process odometry data"""
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
            self.get_logger().info(f'Starting position: ({self.start_x:.2f}, {self.start_y:.2f})')
            self.get_logger().info(f'Initial odometry heading: {math.degrees(self.current_yaw):.1f} degrees')
    
    def get_current_heading(self):
        """Get current heading from IMU if available, otherwise odometry"""
        if self.imu_received:
            return self.current_yaw_imu
        else:
            return self.current_yaw
    
    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle"""
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
    
    def stop_robot(self):
        """Stop the robot immediately"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def wait_for_sensors(self, timeout_sec=10.0):
        """Wait for essential sensors to be ready"""
        self.get_logger().info('Waiting for sensor data...')
        
        start_time = time.time()
        while (not self.odom_received) and rclpy.ok():
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('Timeout waiting for odometry data')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Wait a bit for IMU if available (non-critical)
        imu_wait_time = 0
        while not self.imu_received and imu_wait_time < 20 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            imu_wait_time += 1
        
        if self.imu_received:
            self.get_logger().info('IMU data available - using IMU for precise heading control')
        else:
            self.get_logger().warn('IMU data not available - falling back to odometry for heading')
        
        return True
    
    def calculate_room_width(self):
        """Calculate room width using LIDAR data"""
        if self.latest_scan is None:
            self.get_logger().warn('No LIDAR data available for width calculation')
            return None, None, None
        
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
            return None, None, None
        
        # Calculate angles for each reading
        angles = np.array([angle_min + i * angle_increment for i in range(len(ranges))])
        
        # Find left and right wall distances (perpendicular to robot)
        left_angle_target = math.pi / 2  # 90 degrees
        right_angle_target = -math.pi / 2  # -90 degrees
        
        # Find closest angle indices to left and right
        left_idx = np.argmin(np.abs(angles - left_angle_target))
        right_idx = np.argmin(np.abs(angles - right_angle_target))
        
        # Get average of nearby readings for more stable measurement
        window_size = 5
        
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
            return None, None, None
        
        room_width_left = np.mean(left_valid)
        room_width_right = np.mean(right_valid)
        room_width_total = room_width_left + room_width_right
        
        self.get_logger().info(f'Room width calculation:')
        self.get_logger().info(f'  Left wall distance: {room_width_left:.2f}m')
        self.get_logger().info(f'  Right wall distance: {room_width_right:.2f}m')
        self.get_logger().info(f'  Total room width: {room_width_total:.2f}m')
        
        return room_width_left, room_width_right, room_width_total