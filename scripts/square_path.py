#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)

        self.current_yaw = None
        self.start_yaw = None
        self.state = 'IDLE'

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw

    def move_forward(self, distance=1.0, speed=0.2):
        twist = Twist()
        twist.linear.x = speed

        duration = distance / speed
        self.cmd_vel_pub.publish(twist)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=duration))
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

    def rotate_90_degrees(self, speed=0.3):
        self.start_yaw = self.current_yaw
        target_angle = math.radians(90)
        twist = Twist()
        twist.angular.z = speed

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.current_yaw is None or self.start_yaw is None:
                continue
            delta_yaw = self.normalize_angle(self.current_yaw - self.start_yaw)
            if abs(delta_yaw) >= target_angle:
                break
            self.cmd_vel_pub.publish(twist)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def run(self):
        while self.current_yaw is None:
            rclpy.spin_once(self)
            self.get_logger().info("Waiting for odom...")

        for i in range(4):
            self.get_logger().info(f"Leg {i+1}: Moving forward")
            self.move_forward()
            time.sleep(0.5)
            self.get_logger().info("Rotating 90 degrees")
            self.rotate_90_degrees()
            time.sleep(0.5)

        self.get_logger().info("Finished square path")


def main(args=None):
    rclpy.init(args=args)
    mover = SquareMover()
    mover.run()
    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
