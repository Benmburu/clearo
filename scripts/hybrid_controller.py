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

    def execute_pid_movement(self, target_x, target_y):
        """Execute PID-controlled movement"""
        # Calculate target heading and distance
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        target_heading = math.atan2(dy, dx)
        distance = math.sqrt(dx*dx + dy*dy)
        
        self.get_logger().info(f'PID movement to ({target_x:.2f}, {target_y:.2f})')
        self.get_logger().info(f'Distance: {distance:.2f}m, Heading: {math.degrees(target_heading):.1f}°')
        
        # First, turn to face the target
        while rclpy.ok():
            # Calculate heading error
            heading_error = target_heading - self.current_yaw
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            
            # Check if we're facing the right direction
            if abs(heading_error) < math.radians(2.0):  # 2-degree tolerance
                break
                
            # Create and publish turn command
            twist = Twist()
            twist.angular.z = self.kp_heading * heading_error
            twist.angular.z = max(min(twist.angular.z, self.turn_speed), -self.turn_speed)
            self.cmd_vel_pub.publish(twist)
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Then move forward while maintaining heading
        start_time = time.time()
        while rclpy.ok():
            # Calculate current distance to target
            current_dx = target_x - self.current_x
            current_dy = target_y - self.current_y
            current_distance = math.sqrt(current_dx*current_dx + current_dy*current_dy)
            
            # Check if we've reached the target
            if current_distance < 0.05:  # 5cm tolerance
                # Stop the robot
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                return True
                
            # Calculate heading error
            heading_error = target_heading - self.current_yaw
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            
            # Create movement command
            twist = Twist()
            twist.linear.x = self.forward_speed
            twist.angular.z = self.kp_heading * heading_error
            
            # Publish command
            self.cmd_vel_pub.publish(twist)
            
            # Log progress periodically
            if time.time() - start_time > 1.0:
                self.get_logger().info(f'Distance to target: {current_distance:.2f}m')
                start_time = time.time()
                
            rclpy.spin_once(self, timeout_sec=0.1)
        
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
