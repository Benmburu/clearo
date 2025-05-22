#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import math

class BoustrophedonWaypointFollower(Node):
    def __init__(self):
        super().__init__('boustrophedon_waypoint_follower')
        
        # Create an action client for the FollowWaypoints action
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # Declare and get parameters
        self.declare_parameter('row_length', 2.0)  # Length of each row in meters
        self.declare_parameter('lane_width', 0.3)  # Distance between rows in meters
        self.declare_parameter('num_rows', 5)  # Number of rows to cover
        self.declare_parameter('overlap', 0.05)  # Overlap between passes in meters
        self.declare_parameter('points_per_row', 10)  # Number of waypoints per row
        
        self.row_length = self.get_parameter('row_length').get_parameter_value().double_value
        self.lane_width = self.get_parameter('lane_width').get_parameter_value().double_value
        self.num_rows = self.get_parameter('num_rows').get_parameter_value().integer_value
        self.overlap = self.get_parameter('overlap').get_parameter_value().double_value
        self.points_per_row = self.get_parameter('points_per_row').get_parameter_value().integer_value
        
        # Effective lane width (considering overlap)
        self.effective_width = self.lane_width - self.overlap
        
        # Log initialization
        self.get_logger().info('Boustrophedon waypoint follower initialized')
        self.get_logger().info(f'Row length: {self.row_length}m, Lane width: {self.lane_width}m')
        self.get_logger().info(f'Number of rows: {self.num_rows}, Overlap: {self.overlap}m')
        
    def create_pose(self, x, y, yaw=0.0):
        """Create a PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(0.0)  # roll = 0
        sr = math.sin(0.0)
        cp = math.cos(0.0)  # pitch = 0
        sp = math.sin(0.0)
        
        pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        return pose
        
    def generate_boustrophedon_waypoints(self):
        """Generate waypoints for a boustrophedon pattern."""
        poses = []
        step_size = self.row_length / self.points_per_row
        
        # Add starting point
        poses.append(self.create_pose(x=0.0, y=0.0, yaw=0.0))
        
        for i in range(self.num_rows):
            # Even rows move in the positive x direction
            if i % 2 == 0:
                # Add waypoints along the row
                for j in range(1, self.points_per_row + 1):
                    x = j * step_size
                    y = i * self.effective_width
                    # At the end of the row, point in the direction of the next row
                    yaw = 0.0 if j < self.points_per_row else math.pi/2
                    poses.append(self.create_pose(x=x, y=y, yaw=yaw))
                
                # Add waypoint to move to the next row (if not the last row)
                if i < self.num_rows - 1:
                    poses.append(self.create_pose(
                        x=self.row_length,
                        y=(i + 1) * self.effective_width,
                        yaw=math.pi  # Facing negative x direction
                    ))
            
            # Odd rows move in the negative x direction
            else:
                # Add waypoints along the row
                for j in range(1, self.points_per_row + 1):
                    x = self.row_length - (j * step_size)
                    y = i * self.effective_width
                    # At the end of the row, point in the direction of the next row
                    yaw = math.pi if j < self.points_per_row else math.pi/2
                    poses.append(self.create_pose(x=x, y=y, yaw=yaw))
                
                # Add waypoint to move to the next row (if not the last row)
                if i < self.num_rows - 1:
                    poses.append(self.create_pose(
                        x=0.0,
                        y=(i + 1) * self.effective_width,
                        yaw=0.0  # Facing positive x direction
                    ))
        
        return poses
        
    def send_waypoints(self):
        """Send boustrophedon pattern waypoints."""
        # Wait for action server
        self.get_logger().info('Waiting for follow_waypoints action server...')
        self.follow_waypoints_client.wait_for_server()
        
        # Generate boustrophedon waypoints
        poses = self.generate_boustrophedon_waypoints()
            
        # Create goal message
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses
        
        self.get_logger().info(f'Sending {len(poses)} waypoints for a boustrophedon path')
        
        # Send goal
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server')
            return
            
        self.get_logger().info('Goal accepted by the action server')
        
        # Get the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        status = future.result().result
        self.get_logger().info('Waypoint following completed successfully')
        

def main():
    # Initialize ROS
    rclpy.init()
    
    # Create node
    node = BoustrophedonWaypointFollower()
    
    # Send waypoints
    node.send_waypoints()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()