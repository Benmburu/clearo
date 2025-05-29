#!/usr/bin/env python3

import smbus
import time
import math

class MPU6050Calibrator:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.device_address = 0x68
        self.init_mpu6050()
        
    def init_mpu6050(self):
        """Initialize the MPU6050 sensor"""
        # Wake up the MPU6050
        self.bus.write_byte_data(self.device_address, 0x6B, 0)
        
        # Set accelerometer range to ±2g
        self.bus.write_byte_data(self.device_address, 0x1C, 0x00)
        
        # Set gyroscope range to ±250°/s
        self.bus.write_byte_data(self.device_address, 0x1B, 0x00)
        
        time.sleep(0.1)
        print("MPU6050 initialized for calibration")

    def read_raw_data(self, addr):
        """Read raw 16-bit data from MPU6050"""
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr + 1)
        
        value = (high << 8) | low
        
        if value > 32768:
            value = value - 65536
        
        return value

    def calibrate_gyroscope(self, samples=1000):
        """Calibrate gyroscope by finding zero offsets"""
        print(f"Calibrating gyroscope with {samples} samples...")
        print("Keep the sensor perfectly still during calibration!")
        
        time.sleep(3)  # Give time to place sensor
        
        gyro_x_sum = 0
        gyro_y_sum = 0
        gyro_z_sum = 0
        
        for i in range(samples):
            gyro_x_sum += self.read_raw_data(0x43)
            gyro_y_sum += self.read_raw_data(0x45)
            gyro_z_sum += self.read_raw_data(0x47)
            
            if i % 100 == 0:
                print(f"Progress: {i}/{samples}")
            
            time.sleep(0.01)
        
        gyro_x_offset = gyro_x_sum / samples / 131.0  # Convert to degrees/s
        gyro_y_offset = gyro_y_sum / samples / 131.0
        gyro_z_offset = gyro_z_sum / samples / 131.0
        
        return gyro_x_offset, gyro_y_offset, gyro_z_offset

    def calibrate_accelerometer(self, samples=1000):
        """Calibrate accelerometer"""
        print(f"Calibrating accelerometer with {samples} samples...")
        print("Place sensor on a level surface with Z-axis pointing up!")
        
        time.sleep(3)
        
        accel_x_sum = 0
        accel_y_sum = 0
        accel_z_sum = 0
        
        for i in range(samples):
            accel_x_sum += self.read_raw_data(0x3B)
            accel_y_sum += self.read_raw_data(0x3D)
            accel_z_sum += self.read_raw_data(0x3F)
            
            if i % 100 == 0:
                print(f"Progress: {i}/{samples}")
            
            time.sleep(0.01)
        
        accel_x_offset = accel_x_sum / samples / 16384.0
        accel_y_offset = accel_y_sum / samples / 16384.0
        accel_z_offset = accel_z_sum / samples / 16384.0 - 1.0  # Subtract 1g
        
        return accel_x_offset, accel_y_offset, accel_z_offset

    def test_readings(self, duration=10):
        """Test sensor readings for a specified duration"""
        print(f"Testing sensor readings for {duration} seconds...")
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            # Read raw values
            accel_x = self.read_raw_data(0x3B) / 16384.0
            accel_y = self.read_raw_data(0x3D) / 16384.0
            accel_z = self.read_raw_data(0x3F) / 16384.0
            
            gyro_x = self.read_raw_data(0x43) / 131.0
            gyro_y = self.read_raw_data(0x45) / 131.0
            gyro_z = self.read_raw_data(0x47) / 131.0
            
            print(f"Accel: X={accel_x:6.3f} Y={accel_y:6.3f} Z={accel_z:6.3f} | "
                  f"Gyro: X={gyro_x:6.1f} Y={gyro_y:6.1f} Z={gyro_z:6.1f}")
            
            time.sleep(0.5)

def main():
    try:
        calibrator = MPU6050Calibrator()
        
        print("=== MPU6050 Calibration Tool ===")
        print("1. Test current readings")
        print("2. Calibrate gyroscope")
        print("3. Calibrate accelerometer")
        print("4. Full calibration")
        print("5. Exit")
        
        while True:
            choice = input("\nEnter your choice (1-5): ")
            
            if choice == '1':
                calibrator.test_readings()
                
            elif choice == '2':
                gyro_offsets = calibrator.calibrate_gyroscope()
                print(f"\nGyroscope offsets:")
                print(f"X: {gyro_offsets[0]:.6f}")
                print(f"Y: {gyro_offsets[1]:.6f}")
                print(f"Z: {gyro_offsets[2]:.6f}")
                
            elif choice == '3':
                accel_offsets = calibrator.calibrate_accelerometer()
                print(f"\nAccelerometer offsets:")
                print(f"X: {accel_offsets[0]:.6f}")
                print(f"Y: {accel_offsets[1]:.6f}")
                print(f"Z: {accel_offsets[2]:.6f}")
                
            elif choice == '4':
                gyro_offsets = calibrator.calibrate_gyroscope()
                accel_offsets = calibrator.calibrate_accelerometer()
                
                print(f"\n=== CALIBRATION RESULTS ===")
                print(f"Add these values to your MPU6050 node:")
                print(f"self.gyro_offset = [{gyro_offsets[0]:.6f}, {gyro_offsets[1]:.6f}, {gyro_offsets[2]:.6f}]")
                print(f"self.accel_offset = [{accel_offsets[0]:.6f}, {accel_offsets[1]:.6f}, {accel_offsets[2]:.6f}]")
                
            elif choice == '5':
                break
                
            else:
                print("Invalid choice. Please enter 1-5.")
                
    except KeyboardInterrupt:
        print("\nCalibration interrupted by user")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()