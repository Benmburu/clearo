#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class PIDStraightLineController(Node):
    def __init__(self):
        super().__init__('pid_straight_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Declare and get parameters
        self.declare_parameter('target_distance', 2.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('kp_heading', 1.0)
        
        self.target_distance = self.get_parameter('target_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp_heading = self.get_parameter('kp_heading').value
        
        # Initialize position and orientation variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.target_yaw = None
        
        # Flag to track if we've received odometry data
        self.odom_received = False
        
        self.get_logger().info(f'PID Controller initialized:')
        self.get_logger().info(f'  Target distance: {self.target_distance}m')
        self.get_logger().info(f'  Forward speed: {self.forward_speed}m/s')
        self.get_logger().info(f'  Heading PID gain: {self.kp_heading}')
        
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion using built-in math
        orientation = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('Odometry data received, ready to move')
    
    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle (rotation around z-axis)"""
        # Formula for yaw from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def move_straight(self):
        # Wait for odometry data
        while not self.odom_received and rclpy.ok():
            self.get_logger().info('Waiting for odometry data...')
            rclpy.spin_once(self, timeout_sec=1.0)
        
        if not rclpy.ok():
            return
            
        # Set target heading (current heading when starting)
        self.target_yaw = self.current_yaw
        start_x = self.current_x
        start_y = self.current_y
        
        self.get_logger().info(f'Starting movement from ({start_x:.2f}, {start_y:.2f})')
        self.get_logger().info(f'Target heading: {math.degrees(self.target_yaw):.1f} degrees')
        
        twist = Twist()
        rate = self.create_rate(10)  # 10 Hz
        
        while rclpy.ok():
            # Calculate distance traveled
            distance_traveled = math.sqrt(
                (self.current_x - start_x)**2 + 
                (self.current_y - start_y)**2
            )
            
            if distance_traveled >= self.target_distance:
                # Stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f'Target reached! Distance traveled: {distance_traveled:.2f}m')
                break
            
            # Forward velocity
            twist.linear.x = self.forward_speed
            
            # Heading correction
            heading_error = self.target_yaw - self.current_yaw
            # Normalize angle to [-pi, pi]
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            twist.angular.z = self.kp_heading * heading_error
            
            self.cmd_vel_pub.publish(twist)
            
            # Log progress every 2 seconds
            if int(distance_traveled * 5) % 10 == 0:  # Every 0.2m
                self.get_logger().info(f'Distance: {distance_traveled:.2f}m / {self.target_distance:.2f}m')
            
            rclpy.spin_once(self)
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    
    controller = PIDStraightLineController()
    
    try:
        # Start the movement
        controller.move_straight()
        
        # Keep the node alive for a bit after completion
        controller.get_logger().info('Movement completed. Shutting down...')
        
    except KeyboardInterrupt:
        controller.get_logger().info('Movement interrupted by user')
    finally:
        # Make sure robot stops
        twist = Twist()
        controller.cmd_vel_pub.publish(twist)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()