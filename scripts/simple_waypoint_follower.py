import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # List of (x, y) tuples as waypoints
        self.waypoints = [(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]  # You can customize this
        self.current_index = 0

        self.position = None
        self.yaw = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.position = (pose.position.x, pose.position.y)

        # Extract yaw from quaternion
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.position is None or self.current_index >= len(self.waypoints):
            return

        target = self.waypoints[self.current_index]
        dx = target[0] - self.position[0]
        dy = target[1] - self.position[1]
        distance = math.hypot(dx, dy)

        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - self.yaw)

        cmd = Twist()

        # If angle is way off, rotate in place
        if abs(angle_diff) > 0.1:
            cmd.angular.z = 0.5 * angle_diff
        else:
            cmd.linear.x = 0.2 * distance
            cmd.angular.z = 0.5 * angle_diff

        if distance < 0.2:
            self.get_logger().info(f"Reached waypoint {self.current_index + 1}")
            self.current_index += 1

        self.cmd_vel_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
