#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from flask import Flask, render_template, request, jsonify
import threading
import subprocess
import psutil
import time
import os
import socket

class RobotWebInterface(Node):
    def __init__(self):
        super().__init__('robot_web_interface')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # Flask app
        self.app = Flask(__name__, template_folder=os.path.join(os.path.dirname(__file__), '..', 'web', 'templates'))
        self.setup_routes()
        
        # Robot status
        self.robot_ready = False
        self.startup_time = time.time()
        
        # Timer to periodically check status
        self.status_timer = self.create_timer(2.0, self.check_robot_status)
        
        # Get Pi IP address for display
        self.pi_ip = self.get_pi_ip()
        self.get_logger().info(f'Web interface will be available at: http://{self.pi_ip}:5000')
        
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('index.html')
        
        @self.app.route('/api/status')
        def get_status():
            uptime = time.time() - self.startup_time
            return jsonify({
                'ready': self.robot_ready,
                'nodes_running': self.check_ros_nodes(),
                'timestamp': time.time(),
                'uptime': uptime,
                'pi_ip': self.pi_ip
            })
        
        @self.app.route('/api/move', methods=['POST'])
        def move_robot():
            data = request.json
            action = data.get('action')
            
            if action == 'straight_line':
                distance = data.get('distance', 1.0)
                self.launch_straight_line(distance)
                return jsonify({'status': 'started', 'action': 'straight_line', 'distance': distance})
            
            elif action == 'stop':
                self.stop_robot()
                return jsonify({'status': 'stopped'})
            
            return jsonify({'status': 'unknown_action'})
        
        @self.app.route('/api/manual_control', methods=['POST'])
        def manual_control():
            data = request.json
            linear_x = data.get('linear_x', 0.0)
            angular_z = data.get('angular_z', 0.0)
            
            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.angular.z = float(angular_z)
            self.cmd_vel_pub.publish(twist)
            
            return jsonify({'status': 'command_sent'})
    
    def get_pi_ip(self):
        """Get the Pi's IP address"""
        try:
            # Connect to a remote server to get local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "localhost"
    
    def check_robot_status(self):
        """Check if all required ROS nodes are running"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            nodes = result.stdout.strip().split('\n')
            
            # Check for essential nodes (adjust based on your setup)
            required_nodes = ['/robot_state_publisher', '/joint_state_publisher']
            self.robot_ready = all(any(req in node for node in nodes) for req in required_nodes)
            
        except Exception as e:
            self.get_logger().error(f'Error checking robot status: {e}')
            self.robot_ready = False
    
    def check_ros_nodes(self):
        """Get list of running ROS nodes"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            return result.stdout.strip().split('\n')
        except:
            return []
    
    def launch_straight_line(self, distance):
        """Launch the straight line controller"""
        try:
            subprocess.Popen([
                'ros2', 'launch', 'clearo', 'pid_straight_line.launch.py', 
                f'distance:={distance}'
            ])
        except Exception as e:
            self.get_logger().error(f'Error launching straight line: {e}')
    
    def stop_robot(self):
        """Stop the robot by publishing zero velocity"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Kill any running movement controllers
        try:
            subprocess.run(['pkill', '-f', 'pid_straight_line'], timeout=5)
        except:
            pass
    
    def run_flask(self):
        """Run Flask app in a separate thread"""
        self.get_logger().info('Starting web server on http://0.0.0.0:5000')
        self.app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main():
    rclpy.init()
    
    robot_interface = RobotWebInterface()
    
    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=robot_interface.run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    # Run ROS node
    try:
        rclpy.spin(robot_interface)
    except KeyboardInterrupt:
        pass
    finally:
        robot_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()