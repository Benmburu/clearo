#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class PIDStraightLineController(Node):
    def __init__(self):
        super().__init__('pid_straight_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        
        # Parameters
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('kp_heading', 1.0)
        self.declare_parameter('target_heading_deg', -999.0)  # Use -999 as "not set" indicator
        
        self.target_distance = self.get_parameter('target_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp_heading = self.get_parameter('kp_heading').value
        
        # Handle target heading parameter
        target_heading_param = self.get_parameter('target_heading_deg').value
        if isinstance(target_heading_param, str) and target_heading_param.strip() == '':
            self.target_heading_deg = None  # Empty string means use current heading
        elif target_heading_param == -999.0:
            self.target_heading_deg = None  # Default value means use current heading
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
        
        self.get_logger().info(f'PID Controller initialized:')
        self.get_logger().info(f'  Target distance: {self.target_distance}m')
        self.get_logger().info(f'  Forward speed: {self.forward_speed}m/s')
        self.get_logger().info(f'  Heading PID gain: {self.kp_heading}')
        
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
    
    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def move_straight(self):
        # Wait for first odometry message
        self.get_logger().info('Waiting for odometry data...')
        while not self.odom_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not rclpy.ok():
            return
            
        self.get_logger().info('Starting movement...')
        
        twist = Twist()
        
        while rclpy.ok():
            # Process any pending callbacks to get fresh odometry
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Calculate distance traveled
            if self.start_x is not None and self.start_y is not None:
                distance_traveled = math.sqrt(
                    (self.current_x - self.start_x)**2 + 
                    (self.current_y - self.start_y)**2
                )
            else:
                distance_traveled = 0.0
            
            # Check if target reached
            if distance_traveled >= self.target_distance:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f'Target reached! Distance: {distance_traveled:.3f}m')
                break
            
            # Set forward velocity
            twist.linear.x = self.forward_speed
            
            # Heading correction
            if self.target_yaw is not None:
                heading_error = self.target_yaw - self.current_yaw
                # Normalize to [-pi, pi]
                heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
                twist.angular.z = self.kp_heading * heading_error
            else:
                twist.angular.z = 0.0
            
            # Publish command
            self.cmd_vel_pub.publish(twist)
            
            # Log progress every 2 seconds
            current_time = time.time()
            if current_time - self.last_log_time > 2.0:
                self.get_logger().info(f'Distance: {distance_traveled:.3f}m / {self.target_distance:.3f}m')
                self.get_logger().info(f'Position: ({self.current_x:.3f}, {self.current_y:.3f})')
                self.get_logger().info(f'Heading error: {math.degrees(heading_error):.1f}°')
                self.last_log_time = current_time
            
            # Small sleep to prevent excessive CPU usage
            time.sleep(0.05)  # 20Hz update rate


def main(args=None):
    rclpy.init(args=args)
    
    controller = PIDStraightLineController()
    
    try:
        controller.move_straight()
        controller.get_logger().info('Movement completed successfully!')
        
    except KeyboardInterrupt:
        controller.get_logger().info('Movement interrupted by user')
        
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