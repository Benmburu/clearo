#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
import math
from tf_transformations import quaternion_from_euler
from tf2_ros import Buffer, TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class Nav2StraightLineGoal(Node):
    def __init__(self):
        super().__init__('nav2_straight_line_goal')
        
        # Use callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        self._action_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Add costmap subscription
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10,
            callback_group=self.callback_group
        )
        self.costmap = None
        self.costmap_received = False
        self.init_complete = False

        # Parameters
        self.declare_parameter('distance', 1.0)
        self.distance = self.get_parameter('distance').value
        self.frame_id = 'base_link'
        
        # Create timer for initialization check
        self.create_timer(1.0, self.check_ready, callback_group=self.callback_group)
        
        self.get_logger().info(f'Initialized with distance: {self.distance}m, frame_id: {self.frame_id}')

    def check_ready(self):
        """Check if all required components are ready"""
        if not self.init_complete and self.costmap_received:
            try:
                # Check if transform is available
                self.tf_buffer.lookup_transform(
                    self.costmap.header.frame_id,
                    self.frame_id,
                    rclpy.time.Time()
                )
                self.init_complete = True
                self.get_logger().info('All components ready, sending goal...')
                self.send_goal()
            except Exception as e:
                self.get_logger().info('Waiting for transforms...')

    def costmap_callback(self, msg):
        if not self.costmap_received:
            self.get_logger().info('Costmap received')
        self.costmap = msg
        self.costmap_received = True

    def is_position_safe(self, x, y):
        if not self.costmap_received:
            self.get_logger().warn('No costmap data available yet')
            return True  # Changed to True to allow navigation if costmap isn't ready
            
        try:
            # Transform point from base_link to costmap frame
            transform = self.tf_buffer.lookup_transform(
                self.costmap.header.frame_id,
                self.frame_id,
                rclpy.time.Time())
            
            # Apply transform to coordinates
            costmap_x = x + transform.transform.translation.x
            costmap_y = y + transform.transform.translation.y
            
            # Convert to grid coordinates
            grid_x = int((costmap_x - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
            grid_y = int((costmap_y - self.costmap.info.origin.position.y) / self.costmap.info.resolution)
            
            # Check bounds
            if (grid_x < 0 or grid_x >= self.costmap.info.width or 
                grid_y < 0 or grid_y >= self.costmap.info.height):
                self.get_logger().warn('Goal position outside costmap bounds')
                return True  # Allow navigation even if outside bounds
                
            # Check occupancy
            index = grid_y * self.costmap.info.width + grid_x
            cost = self.costmap.data[index]
            
            return cost < 50 or cost == -1  # Allow unknown space
            
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {str(e)}')
            return True  # Allow navigation if transform fails

    def send_goal(self):
        # Wait for the action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set goal position
        goal_msg.pose.pose.position.x = self.distance
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.position.z = 0.0

        # Set orientation to current orientation (no rotation)
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.get_logger().info(f'Sending goal: Move forward {self.distance}m')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current distance to goal: {feedback.distance_remaining:.2f}m')

    def result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().warn('Failed to reach the goal!')

        # Shutdown the node after completion
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    node = Nav2StraightLineGoal()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()