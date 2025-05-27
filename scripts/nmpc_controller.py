#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import casadi as ca

class NMPCController(Node):
    def __init__(self):  # Fixed: was _init_
        super().__init__('nmpc_controller')  # Fixed: was super()._init_

        self.x = np.zeros(3)
        self.goal = np.array([2.0, 0.0])
        self.obstacles = []

        self.N = 10
        self.dt = 0.1
        self.max_speed = 0.2
        self.max_angular_speed = 0.8
        self.obstacle_threshold = 0.7
        self.safe_distance = 0.1
        self.lambda_obs = 50.0

        # Subscribe to your robot's topics (adjust topic names as needed)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # Fixed topic name
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.nmpc_traj_pub = self.create_publisher(Float64MultiArray, '/nmpc_trajectory', 10)
        self.control_pub = self.create_publisher(Float64MultiArray, '/control_action', 10)

        self.setup_nmpc()
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('NMPC Controller initialized')
        self.get_logger().info(f'Goal: ({self.goal[0]}, {self.goal[1]})')

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        self.obstacles = [(r * np.cos(a), r * np.sin(a)) for r, a in zip(ranges, angles)
                          if np.isfinite(r) and 0.1 < r < self.obstacle_threshold]

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # Fixed: was ori.y*2, should be ori.y**2
        theta = np.arctan2(2.0 * (ori.w * ori.z + ori.x * ori.y),
                           1.0 - 2.0 * (ori.y**2 + ori.z**2))
        self.x = np.array([pos.x, pos.y, theta])

    def setup_nmpc(self):
        self.opti = ca.Opti()
        x = self.opti.variable(3, self.N + 1)
        self.u = self.opti.variable(2, self.N)

        self.x0 = self.opti.parameter(3)
        self.x_ref = self.opti.parameter(3, self.N + 1)
        self.obstacles_param = self.opti.parameter(2, 10)

        cost = 0
        for t in range(self.N):
            cost += ca.sumsqr(x[:2, t+1] - self.x_ref[:2, t+1]) + 0.1 * ca.sumsqr(self.u[:, t])
            for j in range(10):
                d_obs = ca.norm_2(x[:2, t+1] - self.obstacles_param[:, j])
                repulsion = self.lambda_obs * (1 / (d_obs + 0.1)**2 + 10 * ca.exp(-d_obs))
                cost += repulsion
                self.opti.subject_to(d_obs >= self.safe_distance)

        self.opti.minimize(cost)

        for t in range(self.N):
            self.opti.subject_to(x[:, t+1] == x[:, t] + self.dt * ca.vertcat(
                self.u[0, t] * ca.cos(x[2, t]),
                self.u[0, t] * ca.sin(x[2, t]),
                self.u[1, t]
            ))

        self.opti.subject_to(self.opti.bounded(-self.max_speed, self.u[0, :], self.max_speed))
        self.opti.subject_to(self.opti.bounded(-self.max_angular_speed, self.u[1, :], self.max_angular_speed))
        self.opti.subject_to(x[:, 0] == self.x0)
        
        # Configure solver with better options
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        self.opti.solver('ipopt', opts)

    def compute_control_action(self):
        x_dm = ca.DM(self.x)
        goal_traj = np.array([
            np.linspace(self.x[0], self.goal[0], self.N + 1),
            np.linspace(self.x[1], self.goal[1], self.N + 1),
            np.zeros(self.N + 1)
        ])
        obs_array = np.full((2, 10), 1000.0)
        if self.obstacles:
            obs_arr = np.array(self.obstacles).T
            obs_arr = obs_arr[:, :10] if obs_arr.shape[1] > 10 else np.pad(obs_arr, ((0, 0), (0, 10 - obs_arr.shape[1])), constant_values=1000.0)
            obs_array = obs_arr

        self.opti.set_value(self.x0, x_dm)
        self.opti.set_value(self.x_ref, goal_traj)
        self.opti.set_value(self.obstacles_param, obs_array)

        try:
            sol = self.opti.solve()
            u0 = sol.value(self.u[:, 0])

            # Publish control action
            control_msg = Float64MultiArray()
            control_msg.data = [float(u0[0]), float(u0[1])]
            self.control_pub.publish(control_msg)

            # Publish NMPC trajectory (next point)
            traj_msg = Float64MultiArray()
            traj_msg.data = [float(goal_traj[0, 1]), float(goal_traj[1, 1])]
            self.nmpc_traj_pub.publish(traj_msg)

            return u0
        except RuntimeError as e:
            self.get_logger().error(f"NMPC Solver Failed: {e}")
            return np.array([0.0, 0.0])

    def control_loop(self):
        # Check if goal is reached
        distance_to_goal = np.linalg.norm(self.x[:2] - self.goal)
        if distance_to_goal < 0.1:
            self.get_logger().info("Goal reached!")
            cmd = Twist()  # Stop the robot
            self.cmd_pub.publish(cmd)
            return
            
        control = self.compute_control_action()
        cmd = Twist()
        cmd.linear.x = float(control[0])
        cmd.angular.z = float(control[1])
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = NMPCController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':  # Fixed: was _name_ and _main_
    main()