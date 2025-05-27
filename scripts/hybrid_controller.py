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

class ControlMode(Enum):
    PID = 1
    NMPC = 2

class HybridController(Node):
    def __init__(self):
        super().__init__('hybrid_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # PID Parameters
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('kp_heading', 1.0)
        self.declare_parameter('turn_speed', 0.1)
        
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp_heading = self.get_parameter('kp_heading').value
        self.turn_speed = self.get_parameter('turn_speed').value
        
        # NMPC Parameters
        self.obstacle_threshold = 0.7  # Meters
        self.safe_distance = 0.4  # Minimum distance to consider path safe
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.obstacles = []
        self.control_mode = ControlMode.PID
        self.odom_received = False
        self.scan_received = False
        
        self.get_logger().info('Hybrid Controller initialized')

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

    def move_to_goal(self, target_x, target_y):
        """Move to goal using hybrid control"""
        self.get_logger().info(f'Moving to goal: ({target_x:.2f}, {target_y:.2f})')
        
        while rclpy.ok():
            # Check if path is clear
            if not self.is_path_clear(target_x, target_y):
                self.get_logger().info('Obstacle detected, finding safe intermediate goal')
                safe_x, safe_y = self.find_safe_intermediate_goal(target_x, target_y)
                
                if safe_x is not None:
                    self.get_logger().info(f'Found safe intermediate goal: ({safe_x:.2f}, {safe_y:.2f})')
                    self.control_mode = ControlMode.NMPC
                    if not self.execute_nmpc_movement(safe_x, safe_y):
                        return False
                    continue
                else:
                    self.get_logger().error('No safe path found')
                    return False
            
            # Use PID for normal movement
            self.control_mode = ControlMode.PID
            if not self.execute_pid_movement(target_x, target_y):
                return False
            
            # Check if goal reached
            dist_to_goal = math.sqrt(
                (self.current_x - target_x)**2 + 
                (self.current_y - target_y)**2
            )
            if dist_to_goal < 0.05:  # 5cm tolerance
                self.get_logger().info('Goal reached!')
                return True
            
            rclpy.spin_once(self, timeout_sec=0.1)

    # Add PID and NMPC movement methods here
    def execute_pid_movement(self, target_x, target_y):
        """Execute PID-controlled movement"""
        # Implementation from pid_straight_line_controller.py
        # ... (add PID control logic)
        pass

    def execute_nmpc_movement(self, target_x, target_y):
        """Execute NMPC-controlled movement"""
        # Implementation from nmpc_controller.py
        # ... (add NMPC control logic)
        pass

def main(args=None):
    rclpy.init(args=args)
    controller = HybridController()
    
    try:
        # Example movement sequence
        goals = [
            (1.0, 0.0),   # Move forward 1m
            (1.0, 1.0),   # Move sideways 1m
            (0.0, 1.0),   # Move back 1m
            (0.0, 0.0)    # Return to start
        ]
        
        for goal_x, goal_y in goals:
            if not controller.move_to_goal(goal_x, goal_y):
                controller.get_logger().error('Failed to reach goal')
                break
                
    except KeyboardInterrupt:
        controller.get_logger().info('Navigation interrupted by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
