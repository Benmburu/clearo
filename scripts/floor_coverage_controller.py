#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler
import math
import time

class FloorCoverageController(Node):
    def __init__(self):
        super().__init__('floor_coverage_controller')
        
        # Action client for nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Coverage parameters (adjust these for your robot and area)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('area_width', 3.0)  # meters
        self.declare_parameter('area_height', 2.0)  # meters
        self.declare_parameter('row_spacing', 0.25)  # spacing between rows (should be < robot width)
        self.declare_parameter('frame_id', 'map')
        
        # Get parameters
        self.start_x = self.get_parameter('start_x').get_parameter_value().double_value
        self.start_y = self.get_parameter('start_y').get_parameter_value().double_value
        self.area_width = self.get_parameter('area_width').get_parameter_value().double_value
        self.area_height = self.get_parameter('area_height').get_parameter_value().double_value
        self.row_spacing = self.get_parameter('row_spacing').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # State variables
        self.current_goal_index = 0
        self.waypoints = []
        self.is_executing = False
        
        # Wait for action server
        self.get_logger().info('Waiting for NavigateToPose action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('NavigateToPose action server available!')
        
        # Generate waypoints
        self.generate_waypoints()
        
        # Timer to check if we should start (gives you time to position robot)
        self.timer = self.create_timer(2.0, self.timer_callback)
        
    def generate_waypoints(self):
        """Generate boustrophedon (back-and-forth) waypoints"""
        self.waypoints = []
        
        # Calculate number of rows
        num_rows = int(self.area_height / self.row_spacing) + 1
        
        self.get_logger().info(f'Generating coverage path with {num_rows} rows')
        self.get_logger().info(f'Area: {self.area_width}m x {self.area_height}m')
        self.get_logger().info(f'Row spacing: {self.row_spacing}m')
        
        for row in range(num_rows):
            y = self.start_y + (row * self.row_spacing)
            
            # Alternate direction for each row (boustrophedon pattern)
            if row % 2 == 0:
                # Left to right
                start_x = self.start_x
                end_x = self.start_x + self.area_width
                direction = 0.0  # facing right
            else:
                # Right to left
                start_x = self.start_x + self.area_width
                end_x = self.start_x
                direction = math.pi  # facing left
            
            # Add waypoint at start of row
            waypoint = self.create_pose_stamped(start_x, y, direction)
            self.waypoints.append(waypoint)
            
            # Add waypoint at end of row
            waypoint = self.create_pose_stamped(end_x, y, direction)
            self.waypoints.append(waypoint)
        
        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints')
        
        # Print waypoints for debugging
        for i, wp in enumerate(self.waypoints):
            x = wp.pose.position.x
            y = wp.pose.position.y
            self.get_logger().info(f'Waypoint {i}: ({x:.2f}, {y:.2f})')
    
    def create_pose_stamped(self, x, y, yaw):
        """Create a PoseStamped message from x, y, yaw"""
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return pose
    
    def timer_callback(self):
        """Main execution loop"""
        if not self.is_executing and len(self.waypoints) > 0:
            self.get_logger().info('Starting floor coverage mission!')
            self.is_executing = True
            self.send_next_goal()
    
    def send_next_goal(self):
        """Send the next waypoint to nav2"""
        if self.current_goal_index >= len(self.waypoints):
            self.get_logger().info('Floor coverage mission completed!')
            self.is_executing = False
            return
        
        goal_pose = self.waypoints[self.current_goal_index]
        
        self.get_logger().info(f'Sending goal {self.current_goal_index + 1}/{len(self.waypoints)}: '
                              f'({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')
        
        # Create action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Send goal
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle goal completion"""
        result = future.result().result
        
        if result:
            self.get_logger().info(f'Goal {self.current_goal_index + 1} completed successfully!')
        else:
            self.get_logger().warn(f'Goal {self.current_goal_index + 1} failed!')
        
        # Move to next waypoint
        self.current_goal_index += 1
        
        # Wait a bit before sending next goal (gives robot time to settle)
        time.sleep(1.0)
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    
    controller = FloorCoverageController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Coverage mission interrupted by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()