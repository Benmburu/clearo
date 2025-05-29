#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import smbus
import math
import time

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        
        # Publisher for IMU data
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        
        # Timer for publishing IMU data
        self.timer = self.create_timer(0.02, self.publish_imu_data)  # 50Hz
        
        # MPU6050 I2C setup
        try:
            self.bus = smbus.SMBus(1)  # I2C bus 1
        except:
            try:
                import smbus2
                self.bus = smbus2.SMBus(1)
                self.get_logger().info('Using smbus2 library')
            except:
                self.get_logger().error('Neither smbus nor smbus2 available')
                return
                
        self.device_address = 0x68  # MPU6050 default address
        
        # Initialize MPU6050
        if not self.init_mpu6050():
            self.get_logger().error('Failed to initialize MPU6050, shutting down node')
            return
        
        # Calibration offsets (you'll need to calibrate these)
        self.accel_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]
        
        # Complementary filter for orientation
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.dt = 0.02  # 50Hz
        self.alpha = 0.98  # Complementary filter coefficient
        
        self.get_logger().info('MPU6050 node initialized')

    def init_mpu6050(self):
        """Initialize the MPU6050 sensor"""
        try:
            # Check if I2C device is available
            import os
            if not os.path.exists('/dev/i2c-1'):
                self.get_logger().error('I2C device /dev/i2c-1 not found. Enable I2C interface.')
                return False
                
            # Wake up the MPU6050
            self.bus.write_byte_data(self.device_address, 0x6B, 0)
            
            # Set accelerometer range to ±2g
            self.bus.write_byte_data(self.device_address, 0x1C, 0x00)
            
            # Set gyroscope range to ±250°/s
            self.bus.write_byte_data(self.device_address, 0x1B, 0x00)
            
            time.sleep(0.1)
            self.get_logger().info('MPU6050 initialized successfully')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU6050: {e}')
            self.get_logger().error('Make sure I2C is enabled and MPU6050 is connected to address 0x68')
            return False

    def read_raw_data(self, addr):
        """Read raw 16-bit data from MPU6050"""
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr + 1)
        
        # Combine high and low bytes
        value = (high << 8) | low
        
        # Convert to signed value
        if value > 32768:
            value = value - 65536
        
        return value

    def get_imu_data(self):
        """Read and process IMU data"""
        try:
            # Read accelerometer data
            accel_x = self.read_raw_data(0x3B) / 16384.0 - self.accel_offset[0]
            accel_y = self.read_raw_data(0x3D) / 16384.0 - self.accel_offset[1]
            accel_z = self.read_raw_data(0x3F) / 16384.0 - self.accel_offset[2]
            
            # Read gyroscope data (convert to rad/s)
            gyro_x = (self.read_raw_data(0x43) / 131.0 - self.gyro_offset[0]) * math.pi / 180.0
            gyro_y = (self.read_raw_data(0x45) / 131.0 - self.gyro_offset[1]) * math.pi / 180.0
            gyro_z = (self.read_raw_data(0x47) / 131.0 - self.gyro_offset[2]) * math.pi / 180.0
            
            return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
            
        except Exception as e:
            self.get_logger().error(f'Error reading IMU data: {e}')
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]

    def complementary_filter(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
        """Apply complementary filter for orientation estimation"""
        # Calculate angles from accelerometer
        accel_roll = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
        accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
        
        # Integrate gyroscope data
        self.roll += gyro_x * self.dt
        self.pitch += gyro_y * self.dt
        self.yaw += gyro_z * self.dt
        
        # Apply complementary filter
        self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch

    def publish_imu_data(self):
        """Publish IMU data"""
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.get_imu_data()
        
        # Apply complementary filter
        self.complementary_filter(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
        
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Set orientation (quaternion from Euler angles)
        quaternion = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        
        # Set angular velocity
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        
        # Set linear acceleration
        imu_msg.linear_acceleration.x = accel_x * 9.81  # Convert to m/s²
        imu_msg.linear_acceleration.y = accel_y * 9.81
        imu_msg.linear_acceleration.z = accel_z * 9.81
        
        # Set covariance matrices (adjust based on your sensor characteristics)
        imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        # Publish the message
        self.imu_publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        mpu6050_node = MPU6050Node()
        rclpy.spin(mpu6050_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'mpu6050_node' in locals():
            mpu6050_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()