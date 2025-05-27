#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import math

class StraightLineWaypointFollower(Node):
    def __init__(self):
        super().__init__('straight_line_waypoint_follower')
        
        # Create an action client for the FollowWaypoints action
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # Simple parameters - just what we need
        self.declare_parameter('distance', 1.0)  # Distance to travel in meters
        self.declare_parameter('num_waypoints', 5)  # Number of waypoints along the path
        
        self.distance = self.get_parameter('distance').get_parameter_value().double_value
        self.num_waypoints = self.get_parameter('num_waypoints').get_parameter_value().integer_value
        
        self.get_logger().info(f'Straight line controller initialized - Distance: {self.distance}m, Waypoints: {self.num_waypoints}')
        
    def create_pose(self, x, y, yaw=0.0):
        """Create a PoseStamped message with given coordinates."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion (simplified for z-axis rotation only)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        
        return pose
        
    def generate_straight_line_waypoints(self):
        """Generate waypoints for a straight line path."""
        poses = []
        step_size = self.distance / self.num_waypoints
        
        # Create waypoints from current position (0,0) to target distance
        for i in range(self.num_waypoints + 1):  # +1 to include the final point
            x = i * step_size
            y = 0.0  # Keep y constant - straight line
            yaw = 0.0  # Keep facing forward (positive x direction)
            poses.append(self.create_pose(x, y, yaw))
        
        return poses

    def send_waypoints(self):
        """Send the straight line waypoints to the robot."""
        # Wait for the navigation action server to be available
        self.get_logger().info('Waiting for follow_waypoints action server...')
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return
        
        # Generate the waypoints
        poses = self.generate_straight_line_waypoints()
        
        # Create the goal message
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses
        
        self.get_logger().info(f'Sending {len(poses)} waypoints for straight line path')
        for i, pose in enumerate(poses):
            self.get_logger().info(f'Waypoint {i}: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}')
        
        # Send the goal to the action server
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server')
            return
            
        self.get_logger().info('Goal accepted! Robot is moving...')
        
        # Wait for the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        """Handle the final result."""
        result = future.result().result
        self.get_logger().info('Robot finished moving in straight line!')
        

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create the node
    node = StraightLineWaypointFollower()
    
    # Send the waypoints
    node.send_waypoints()
    
    try:
        # Keep the node running until interrupted
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()