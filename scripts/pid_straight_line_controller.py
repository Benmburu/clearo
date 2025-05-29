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
        self.declare_parameter('target_heading_deg', -999.0)
        self.declare_parameter('turn_speed', 0.1)  # Angular velocity for turns
        self.declare_parameter('offset_distance', 0.2)  # 20cm offset
        
        self.target_distance = self.get_parameter('target_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp_heading = self.get_parameter('kp_heading').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.offset_distance = self.get_parameter('offset_distance').value
        
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
        self.start_x = None
        self.start_y = None
        self.target_yaw = None
        self.odom_received = False
        self.last_log_time = 0.0
        
        self.get_logger().info(f'Boustrophedon Controller initialized:')
        self.get_logger().info(f'  Target distance: {self.target_distance}m')
        self.get_logger().info(f'  Forward speed: {self.forward_speed}m/s')
        self.get_logger().info(f'  Turn speed: {self.turn_speed}rad/s')
        self.get_logger().info(f'  Offset distance: {self.offset_distance}m')
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
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def turn_right_90_degrees(self):
        """Turn exactly 100 degrees to the right"""
        self.get_logger().info('Starting 100-degree right turn...')
        
        # Calculate target heading (100 degrees clockwise from current)
        start_heading = self.current_yaw
        target_heading = self.normalize_angle(start_heading - math.pi*105/180)  # -100 degrees
        
        self.get_logger().info(f'Turn start heading: {math.degrees(start_heading):.1f}°')
        self.get_logger().info(f'Turn target heading: {math.degrees(target_heading):.1f}°')
        
        twist = Twist()
        
        while rclpy.ok():
            # Process callbacks for fresh odometry
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Calculate heading error
            heading_error = target_heading - self.current_yaw
            heading_error = self.normalize_angle(heading_error)
            
            # Check if turn is complete (within 2 degrees tolerance)
            if abs(heading_error) < math.radians(0.5):
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
                total_turn = self.normalize_angle(self.current_yaw - start_heading)
                self.get_logger().info(f'Turn completed! Turned {math.degrees(total_turn):.1f}°')
                self.get_logger().info(f'Final heading: {math.degrees(self.current_yaw):.1f}°')
                break
            
            # Apply turning velocity (negative for right turn)
            if heading_error > 0:
                twist.angular.z = self.turn_speed
            else:
                twist.angular.z = -self.turn_speed
            
            twist.linear.x = 0.0  # No forward movement during turn
            self.cmd_vel_pub.publish(twist)
            
            # Log progress
            current_time = time.time()
            if current_time - self.last_log_time > 0.5:
                current_turn = self.normalize_angle(self.current_yaw - start_heading)
                self.get_logger().info(f'Turning... Current: {math.degrees(self.current_yaw):.1f}°, '
                                     f'Turned: {math.degrees(current_turn):.1f}°, '
                                     f'Error: {math.degrees(heading_error):.1f}°')
                self.last_log_time = current_time
            
            time.sleep(0.05)
    
    # def turn_right_90_degrees(self):
    #     """Turn exactly 90 degrees to the right using PID control"""
    #     self.get_logger().info('Starting 90-degree right turn with PID control...')
        
    #     # Calculate target heading (90 degrees clockwise from current)
    #     start_heading = self.current_yaw
    #     # Adjust the target angle to match the real-world 90 degrees
    #     # Based on your logs, a 145-degree change in the odometry reading corresponds to a 90-degree real turn
    #     # So we'll use that ratio to calculate our target
    #     real_world_adjustment = 145.0/90.0  # Adjustment factor
    #     target_heading = self.normalize_angle(start_heading - (math.pi/2) * real_world_adjustment)
        
    #     self.get_logger().info(f'Turn start heading: {math.degrees(start_heading):.1f}°')
    #     self.get_logger().info(f'Turn target heading: {math.degrees(target_heading):.1f}°')
        
    #     twist = Twist()
        
    #     # PID parameters for turning - adjusted to prevent oscillation
    #     kp = 0.3  # Reduced from 0.5 to decrease overshoot
    #     ki = 0.01  # Reduced from 0.05 to reduce oscillation
    #     kd = 0.3  # Increased from 0.2 to dampen oscillations
        
    #     integral = 0.0
    #     last_error = 0.0
    #     max_integral = 0.5  # Reduced anti-windup limit
        
    #     # Rate limiter
    #     max_angular_velocity = self.turn_speed
    #     min_angular_velocity = 0.05  # Minimum velocity to overcome friction
        
    #     # Add timeout mechanism to prevent infinite loops
    #     start_time = time.time()
    #     timeout = 15.0  # 15 seconds max for a turn
        
    #     # Add settling time counter to ensure stability before completing
    #     settled_count = 0
    #     required_settled_readings = 5  # Need this many consecutive stable readings
        
    #     while rclpy.ok():
    #         # Process callbacks for fresh odometry
    #         rclpy.spin_once(self, timeout_sec=0.01)
            
    #         # Calculate heading error
    #         heading_error = target_heading - self.current_yaw
    #         heading_error = self.normalize_angle(heading_error)
            
    #         # PID calculation
    #         integral += heading_error * 0.05  # dt = 0.05
    #         integral = max(min(integral, max_integral), -max_integral)  # Anti-windup
            
    #         derivative = (heading_error - last_error) / 0.05
    #         last_error = heading_error
            
    #         # PID output
    #         output = kp * heading_error + ki * integral + kd * derivative
            
    #         # Rate limiting
    #         if abs(output) < min_angular_velocity and abs(heading_error) > math.radians(0.5):
    #             output = math.copysign(min_angular_velocity, output)
            
    #         output = max(min(output, max_angular_velocity), -max_angular_velocity)
            
    #         # Check if turn is complete (within tolerance)
    #         if abs(heading_error) < math.radians(1.0):
    #             settled_count += 1
    #             if settled_count >= required_settled_readings:
    #                 twist.angular.z = 0.0
    #                 self.cmd_vel_pub.publish(twist)
                    
    #                 total_turn = self.normalize_angle(self.current_yaw - start_heading)
    #                 self.get_logger().info(f'Turn completed! Turned {math.degrees(total_turn):.1f}°')
    #                 self.get_logger().info(f'Final heading: {math.degrees(self.current_yaw):.1f}°')
    #                 break
    #         else:
    #             settled_count = 0  # Reset if we're not within tolerance
            
    #         # Check for timeout
    #         if time.time() - start_time > timeout:
    #             twist.angular.z = 0.0
    #             self.cmd_vel_pub.publish(twist)
    #             self.get_logger().warn(f'Turn timeout! Best effort completed. Current heading: {math.degrees(self.current_yaw):.1f}°')
    #             break
            
    #         # Apply turning velocity
    #         twist.angular.z = -output  # Negative for right turn
    #         twist.linear.x = 0.0  # No forward movement during turn
    #         self.cmd_vel_pub.publish(twist)
            
    #         # Log progress
    #         current_time = time.time()
    #         if current_time - self.last_log_time > 0.5:
    #             current_turn = self.normalize_angle(self.current_yaw - start_heading)
    #             self.get_logger().info(f'Turning... Current: {math.degrees(self.current_yaw):.1f}°, '
    #                                 f'Turned: {math.degrees(current_turn):.1f}°, '
    #                                 f'Error: {math.degrees(heading_error):.1f}°')
    #             self.last_log_time = current_time
            
    #         time.sleep(0.05)
    
    def move_distance(self, distance, description=""):
        """Move forward for a specific distance"""
        if description:
            self.get_logger().info(f'Starting {description}: {distance:.2f}m')
        else:
            self.get_logger().info(f'Moving forward: {distance:.2f}m')
        
        # Record starting position
        start_x = self.current_x
        start_y = self.current_y
        target_heading = self.current_yaw  # Maintain current heading
        
        twist = Twist()
        
        while rclpy.ok():
            # Process callbacks for fresh odometry
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Calculate distance traveled
            distance_traveled = math.sqrt(
                (self.current_x - start_x)**2 + 
                (self.current_y - start_y)**2
            )
            
            # Check if target distance reached
            if distance_traveled >= distance:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f'Distance completed: {distance_traveled:.3f}m')
                break
            
            # Set forward velocity
            twist.linear.x = self.forward_speed
            
            # Heading correction to maintain straight line
            heading_error = target_heading - self.current_yaw
            heading_error = self.normalize_angle(heading_error)
            twist.angular.z = self.kp_heading * heading_error
            
            # Publish command
            self.cmd_vel_pub.publish(twist)
            
            # Log progress
            current_time = time.time()
            if current_time - self.last_log_time > 1.0:
                self.get_logger().info(f'Distance: {distance_traveled:.3f}m / {distance:.3f}m')
                self.last_log_time = current_time
            
            time.sleep(0.05)
    
    def execute_boustrophedon_pattern(self):
        """Execute the complete boustrophedon pattern"""
        # Wait for first odometry message
        self.get_logger().info('Waiting for odometry data...')
        while not self.odom_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not rclpy.ok():
            return
        
        self.get_logger().info('=== Starting Boustrophedon Pattern ===')
        
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
        
        self.get_logger().info('=== Boustrophedon Pattern Complete ===')
        self.get_logger().info('Robot should now be facing opposite direction, offset by 20cm')


def main(args=None):
    rclpy.init(args=args)
    
    controller = PIDStraightLineController()
    
    try:
        controller.execute_boustrophedon_pattern()
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