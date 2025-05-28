#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import math
import time


class PIDStraightLineController(Node):
    def __init__(self):
        super().__init__('pid_straight_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        
        # Visualization publishers
        self.planned_path_pub = self.create_publisher(Marker, '/pid_planned_path', 10)
        self.actual_path_pub = self.create_publisher(Path, '/pid_actual_trajectory', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/pid_target_pose', 10)
        self.path_markers_pub = self.create_publisher(MarkerArray, '/pid_path_markers', 10)
        
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
        
        # Visualization variables
        self.actual_path = Path()
        self.actual_path.header.frame_id = "odom"
        self.planned_waypoints = []
        self.current_target_pose = None
        self.path_recording_active = False
        
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
        
        # Record actual trajectory if active
        if self.path_recording_active:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.header.frame_id = "odom"
            pose_stamped.pose = msg.pose.pose
            self.actual_path.poses.append(pose_stamped)
            self.actual_path.header.stamp = msg.header.stamp
            self.actual_path_pub.publish(self.actual_path)
        
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
            
            # Generate and publish planned path
            self.generate_planned_path()
    
    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def generate_planned_path(self):
        """Generate and publish the planned boustrophedon path"""
        if not self.odom_received:
            return
            
        self.get_logger().info('Generating planned path visualization...')
        
        # Calculate waypoints based on current position and heading
        waypoints = []
        
        # Starting point
        current_x, current_y, current_heading = self.start_x, self.start_y, self.target_yaw
        waypoints.append((current_x, current_y, "Start"))
        
        # Point 1: After first forward movement
        x1 = current_x + self.target_distance * math.cos(current_heading)
        y1 = current_y + self.target_distance * math.sin(current_heading)
        waypoints.append((x1, y1, "End of first sweep"))
        
        # Point 2: After first turn (90° right)
        heading_after_turn1 = self.normalize_angle(current_heading - math.pi/2)
        waypoints.append((x1, y1, "After first turn"))
        
        # Point 3: After offset movement
        x2 = x1 + self.offset_distance * math.cos(heading_after_turn1)
        y2 = y1 + self.offset_distance * math.sin(heading_after_turn1)
        waypoints.append((x2, y2, "After offset"))
        
        # Point 4: After second turn (another 90° right)
        heading_after_turn2 = self.normalize_angle(heading_after_turn1 - math.pi/2)
        waypoints.append((x2, y2, "Final position"))
        
        self.planned_waypoints = waypoints
        
        # Publish planned path as line markers
        self.publish_planned_path_markers()
        
        # Publish waypoint markers
        self.publish_waypoint_markers()
    
    def publish_planned_path_markers(self):
        """Publish the planned path as line strip markers"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pid_planned_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set scale (line width)
        marker.scale.x = 0.05
        
        # Set color (blue for planned path)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        # Add points to create the path
        if len(self.planned_waypoints) >= 2:
            # Forward path
            start_point = Point()
            start_point.x = self.planned_waypoints[0][0]
            start_point.y = self.planned_waypoints[0][1]
            start_point.z = 0.1
            marker.points.append(start_point)
            
            end_point = Point()
            end_point.x = self.planned_waypoints[1][0]
            end_point.y = self.planned_waypoints[1][1]
            end_point.z = 0.1
            marker.points.append(end_point)
            
            # Offset path
            if len(self.planned_waypoints) >= 4:
                offset_point = Point()
                offset_point.x = self.planned_waypoints[3][0]
                offset_point.y = self.planned_waypoints[3][1]
                offset_point.z = 0.1
                marker.points.append(offset_point)
        
        self.planned_path_pub.publish(marker)
    
    def publish_waypoint_markers(self):
        """Publish waypoint markers"""
        marker_array = MarkerArray()
        
        for i, (x, y, description) in enumerate(self.planned_waypoints):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            
            # Scale
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Color (green for waypoints)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
            # Add text marker for description
            text_marker = Marker()
            text_marker.header.frame_id = "odom"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "waypoint_labels"
            text_marker.id = i + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.2
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.1
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"{i}: {description}"
            marker_array.markers.append(text_marker)
        
        self.path_markers_pub.publish(marker_array)
    
    def publish_target_marker(self, x, y, description="Target"):
        """Publish current target position marker"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "current_target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0
        
        # Scale
        marker.scale.x = 0.2
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Color (red for current target)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        
        self.target_marker_pub.publish(marker)
        
        self.get_logger().info(f'Target marker published: {description} at ({x:.2f}, {y:.2f})')
    
    def start_trajectory_recording(self):
        """Start recording the actual trajectory"""
        self.path_recording_active = True
        self.actual_path.poses.clear()
        self.get_logger().info('Started trajectory recording')
    
    def turn_right_90_degrees(self):
        """Turn exactly 100 degrees to the right"""
        self.get_logger().info('Starting 90-degree right turn...')
        
        # Calculate target heading (100 degrees clockwise from current)
        start_heading = self.current_yaw
        target_heading = self.normalize_angle(start_heading - math.pi*105/180)  # -105 degrees for real 90°
        
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
        
        # Publish target position marker
        target_x = start_x + distance * math.cos(target_heading)
        target_y = start_y + distance * math.sin(target_heading)
        self.publish_target_marker(target_x, target_y, description if description else "Forward target")
        
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
        
        # Start trajectory recording
        self.start_trajectory_recording()
        
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
        
        # Stop trajectory recording
        self.path_recording_active = False
        
        self.get_logger().info('=== Boustrophedon Pattern Complete ===')
        self.get_logger().info('Robot should now be facing opposite direction, offset by 20cm')
        self.get_logger().info('Check RViz for path visualization!')


def main(args=None):
    rclpy.init(args=args)
    
    controller = PIDStraightLineController()
    
    try:
        controller.execute_boustrophedon_pattern()
        controller.get_logger().info('Pattern completed successfully!')
        
        # Keep node alive to continue publishing visualization
        controller.get_logger().info('Keeping visualization active... Press Ctrl+C to exit')
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=1.0)
        
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