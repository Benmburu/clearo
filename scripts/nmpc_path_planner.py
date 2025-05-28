#!/usr/bin/env python3
# File: improved_nmpc_path_planner.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import casadi as ca

class ImprovedNMPCPathPlanner(Node):
    def __init__(self):
        super().__init__('improved_nmpc_path_planner')

        # State and planning parameters
        self.x = np.zeros(3)  # [x, y, theta]
        self.goal_offset = 1.5  # Distance ahead of robot for dynamic goal
        self.obstacles = []
        
        # NMPC parameters - OPTIMIZED for better performance
        self.N = 20  # Shorter horizon for faster computation
        self.dt = 0.2  # Slightly larger time step
        self.max_speed = 0.3
        self.max_angular_speed = 0.8
        self.obstacle_detection_range = 2.0  # Closer range for relevant obstacles
        self.safe_distance = 0.4  # Minimum safe distance from obstacles
        self.obstacle_weight = 50.0  # Simplified obstacle avoidance weight
        
        # Track if we have valid odometry
        self.odom_received = False

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.goal_callback, 10)
        
        # Publishers for visualization
        self.path_pub = self.create_publisher(Path, '/nmpc_path', 10)
        self.obstacles_pub = self.create_publisher(MarkerArray, '/obstacles_markers', 10)
        self.goal_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.nmpc_traj_pub = self.create_publisher(Float64MultiArray, '/nmpc_trajectory', 10)

        self.setup_nmpc()
        
        # Timer for path planning
        self.timer = self.create_timer(0.3, self.planning_loop)  # Faster updates
        
        self.get_logger().info("🚀 Improved NMPC Path Planner Started!")
        self.get_logger().info("📍 Robot will maintain forward motion with obstacle avoidance")

    def goal_callback(self, msg):
        """Set new goal offset from RViz clicked point"""
        # Calculate distance from robot to clicked point
        clicked_point = np.array([msg.point.x, msg.point.y])
        robot_pos = self.x[:2]
        distance = np.linalg.norm(clicked_point - robot_pos)
        
        if distance > 0.5:  # Only update if click is reasonably far
            self.goal_offset = min(distance, 3.0)  # Cap at 3 meters
            self.get_logger().info(f"🎯 Goal offset updated to: {self.goal_offset:.2f}m ahead")

    def lidar_callback(self, msg):
        """Process LiDAR data to extract relevant obstacles"""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Only consider obstacles in front of the robot (forward-facing arc)
        forward_mask = np.abs(angles) < np.pi/2  # ±90 degrees
        
        obstacles_local = []
        for i, (r, a) in enumerate(zip(ranges, angles)):
            if (forward_mask[i] and np.isfinite(r) and 
                0.3 < r < self.obstacle_detection_range):
                
                # Convert to robot frame
                x_local = r * np.cos(a)
                y_local = r * np.sin(a)
                
                # Transform to global frame
                cos_theta = np.cos(self.x[2])
                sin_theta = np.sin(self.x[2])
                x_global = self.x[0] + x_local * cos_theta - y_local * sin_theta
                y_global = self.x[1] + x_local * sin_theta + y_local * cos_theta
                
                obstacles_local.append((x_global, y_global))
        
        # Cluster close obstacles to reduce computation
        self.obstacles = self.cluster_obstacles(obstacles_local)
        self.publish_obstacle_markers()

    def cluster_obstacles(self, obstacles, cluster_radius=0.3):
        """Cluster nearby obstacles to reduce computational load"""
        if not obstacles:
            return []
        
        obstacles = np.array(obstacles)
        clustered = []
        used = np.zeros(len(obstacles), dtype=bool)
        
        for i, obs in enumerate(obstacles):
            if used[i]:
                continue
                
            # Find all obstacles within cluster radius
            distances = np.linalg.norm(obstacles - obs, axis=1)
            cluster_mask = distances < cluster_radius
            cluster_points = obstacles[cluster_mask]
            
            # Use centroid of cluster
            centroid = np.mean(cluster_points, axis=0)
            clustered.append(tuple(centroid))
            
            # Mark all points in this cluster as used
            used |= cluster_mask
        
        return clustered[:10]  # Limit to 10 obstacles max

    def odom_callback(self, msg):
        """Update robot pose"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        theta = np.arctan2(2.0 * (ori.w * ori.z + ori.x * ori.y),
                           1.0 - 2.0 * (ori.y**2 + ori.z**2))
        self.x = np.array([pos.x, pos.y, theta])
        self.odom_received = True

    def get_dynamic_goal(self):
        """Calculate goal position ahead of robot in its current direction"""
        # Goal is offset distance ahead in the robot's current orientation
        goal_x = self.x[0] + self.goal_offset * np.cos(self.x[2])
        goal_y = self.x[1] + self.goal_offset * np.sin(self.x[2])
        return np.array([goal_x, goal_y])

    def setup_nmpc(self):
        """Setup the NMPC optimization problem with simplified cost function"""
        self.opti = ca.Opti()
        
        # Decision variables
        self.x_var = self.opti.variable(3, self.N + 1)  # [x, y, theta]
        self.u_var = self.opti.variable(2, self.N)      # [v, omega]
        
        # Parameters
        self.x0_param = self.opti.parameter(3)
        self.goal_param = self.opti.parameter(2)
        self.obstacles_param = self.opti.parameter(2, 10)  # Max 10 obstacles
        self.num_obstacles_param = self.opti.parameter(1)
        
        # SIMPLIFIED Cost function
        cost = 0
        
        # 1. Goal tracking cost - encourage forward motion toward goal
        for t in range(self.N + 1):
            goal_error = ca.sumsqr(self.x_var[:2, t] - self.goal_param)
            weight = 1.0 + 2.0 * t / self.N  # Increase weight over time
            cost += weight * goal_error
        
        # 2. Control smoothness (minimize control effort and changes)
        for t in range(self.N):
            # Prefer forward motion
            cost += 0.1 * (self.u_var[0, t] - 0.2)**2  # Prefer ~0.2 m/s forward speed
            cost += 0.05 * self.u_var[1, t]**2  # Minimize angular velocity
            
            # Smooth control changes
            if t > 0:
                cost += 0.1 * ca.sumsqr(self.u_var[:, t] - self.u_var[:, t-1])
        
        # 3. SIMPLIFIED Obstacle avoidance
        for t in range(self.N + 1):
            for j in range(10):  # Max 10 obstacles
                # Distance to obstacle
                dist = ca.norm_2(self.x_var[:2, t] - self.obstacles_param[:, j])
                
                # Simple barrier function - becomes large when dist < safe_distance
                barrier = ca.if_else(dist < self.safe_distance * 2, 
                                   self.obstacle_weight / (dist + 0.1),
                                   0)
                cost += barrier
        
        self.opti.minimize(cost)
        
        # Dynamics constraints (bicycle model)
        for t in range(self.N):
            x_next = self.x_var[:, t] + self.dt * ca.vertcat(
                self.u_var[0, t] * ca.cos(self.x_var[2, t]),
                self.u_var[0, t] * ca.sin(self.x_var[2, t]),
                self.u_var[1, t]
            )
            self.opti.subject_to(self.x_var[:, t+1] == x_next)
        
        # Control constraints
        self.opti.subject_to(self.opti.bounded(0.0, self.u_var[0, :], self.max_speed))  # Forward only
        self.opti.subject_to(self.opti.bounded(-self.max_angular_speed, self.u_var[1, :], self.max_angular_speed))
        
        # Initial condition
        self.opti.subject_to(self.x_var[:, 0] == self.x0_param)
        
        # Solver options - more tolerant for real-time performance
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.sb': 'yes',
            'ipopt.max_iter': 100,  # Reduced iterations
            'ipopt.tol': 1e-2,      # More tolerant
            'ipopt.acceptable_tol': 1e-1,
            'ipopt.acceptable_iter': 5
        }
        self.opti.solver('ipopt', opts)

    def compute_optimal_path(self):
        """Solve NMPC optimization and return the planned path"""
        try:
            # Get dynamic goal (always ahead of robot)
            current_goal = self.get_dynamic_goal()
            
            # Prepare obstacle array with padding for unused slots
            obs_array = np.full((2, 10), 1000.0)  # Far away default (1km)
            n_obstacles = min(len(self.obstacles), 10)
            
            if n_obstacles > 0:
                obs_data = np.array(self.obstacles).T
                obs_array[:, :n_obstacles] = obs_data
            
            # Set parameters
            self.opti.set_value(self.x0_param, self.x)
            self.opti.set_value(self.goal_param, current_goal)
            self.opti.set_value(self.obstacles_param, obs_array)
            self.opti.set_value(self.num_obstacles_param, n_obstacles)
            
            # Warm start with previous solution or simple initial guess
            if not hasattr(self, 'last_solution'):
                # Initial guess: straight line with constant forward velocity
                x_guess = np.zeros((3, self.N + 1))
                u_guess = np.zeros((2, self.N))
                
                for t in range(self.N + 1):
                    # Predict position assuming constant velocity
                    x_guess[0, t] = self.x[0] + t * self.dt * 0.2 * np.cos(self.x[2])
                    x_guess[1, t] = self.x[1] + t * self.dt * 0.2 * np.sin(self.x[2])
                    x_guess[2, t] = self.x[2]
                    
                for t in range(self.N):
                    u_guess[0, t] = 0.2  # Forward speed
                    u_guess[1, t] = 0.0  # No turning initially
                    
                self.opti.set_initial(self.x_var, x_guess)
                self.opti.set_initial(self.u_var, u_guess)
            else:
                # Shift previous solution for warm start
                self.opti.set_initial(self.x_var, self.last_solution['states'])
                self.opti.set_initial(self.u_var, self.last_solution['controls'])
            
            # Solve
            sol = self.opti.solve()
            
            # Extract and store solution
            planned_states = sol.value(self.x_var)
            planned_controls = sol.value(self.u_var)
            
            self.last_solution = {
                'states': planned_states,
                'controls': planned_controls
            }
            
            self.get_logger().info(f"✅ Path planned with {n_obstacles} obstacles")
            return planned_states, planned_controls, current_goal
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ NMPC failed: {str(e)[:100]}")
            return None, None, self.get_dynamic_goal()

    def publish_path(self, planned_states):
        """Publish the planned path for RViz visualization"""
        if planned_states is None:
            return
            
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(planned_states.shape[1]):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(planned_states[0, i])
            pose.pose.position.y = float(planned_states[1, i])
            pose.pose.position.z = 0.0
            
            # Set orientation from planned state
            yaw = float(planned_states[2, i])
            pose.pose.orientation.z = np.sin(yaw / 2.0)
            pose.pose.orientation.w = np.cos(yaw / 2.0)
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

    def publish_obstacle_markers(self):
        """Visualize detected obstacles in RViz"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        delete_marker = Marker()
        delete_marker.header.frame_id = "odom"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "obstacles"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, (x, y) in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i + 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.2
            
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime.sec = 1
            
            marker_array.markers.append(marker)
        
        self.obstacles_pub.publish(marker_array)

    def publish_goal_marker(self, goal_pos):
        """Visualize the current dynamic goal in RViz"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(goal_pos[0])
        marker.pose.position.y = float(goal_pos[1])
        marker.pose.position.z = 0.1
        
        # Arrow pointing in robot's direction
        yaw = self.x[2]
        marker.pose.orientation.z = np.sin(yaw / 2.0)
        marker.pose.orientation.w = np.cos(yaw / 2.0)
        
        marker.scale.x = 0.3
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.goal_pub.publish(marker)

    def planning_loop(self):
        """Main planning loop"""
        if not self.odom_received:
            return
        
        # Compute optimal path
        planned_states, planned_controls, current_goal = self.compute_optimal_path()
        
        # Publish visualizations
        self.publish_path(planned_states)
        self.publish_goal_marker(current_goal)
        
        # Publish trajectory data for debugging
        if planned_states is not None and planned_controls is not None:
            traj_msg = Float64MultiArray()
            states_flat = planned_states.flatten()
            controls_flat = planned_controls.flatten()
            traj_msg.data = [float(x) for x in states_flat] + [float(u) for u in controls_flat]
            self.nmpc_traj_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImprovedNMPCPathPlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()