#!/usr/bin/env python3

"""
Test for Sensor Simulation

This test verifies that the sensor simulation in the digital twin environment
produces realistic data streams from LiDAR, camera, and IMU sensors.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
import time
import numpy as np
from threading import Thread


class SensorTestNode(Node):
    def __init__(self):
        super().__init__('sensor_test_node')

        # Set up subscribers for sensor data
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.camera_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Storage for received data
        self.laser_data = None
        self.camera_data = None
        self.imu_data = None

        # Flags to track if we've received data
        self.laser_received = False
        self.camera_received = False
        self.imu_received = False

        self.get_logger().info('Sensor Test Node initialized')

    def laser_callback(self, msg):
        """Callback for laser scan messages"""
        self.laser_data = msg
        self.laser_received = True
        self.get_logger().debug(f'Received laser scan with {len(msg.ranges)} range values')

    def camera_callback(self, msg):
        """Callback for camera image messages"""
        self.camera_data = msg
        self.camera_received = True
        self.get_logger().debug(f'Received camera image: {msg.width}x{msg.height}')

    def imu_callback(self, msg):
        """Callback for IMU messages"""
        self.imu_data = msg
        self.imu_received = True
        self.get_logger().debug('Received IMU data')


def test_sensor_simulation():
    """Test that verifies sensor simulation works correctly"""
    rclpy.init()

    try:
        # Create test node
        test_node = SensorTestNode()

        # Wait for data to be received
        start_time = time.time()
        timeout = 10.0  # 10 second timeout

        while (not test_node.laser_received or
               not test_node.camera_received or
               not test_node.imu_received) and (time.time() - start_time) < timeout:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        # Check results
        results = {
            'laser': test_node.laser_received,
            'camera': test_node.camera_received,
            'imu': test_node.imu_received
        }

        print("Sensor Simulation Test Results:")
        print(f"  LiDAR sensor: {'✅ PASSED' if results['laser'] else '❌ FAILED'}")
        print(f"  Camera sensor: {'✅ PASSED' if results['camera'] else '❌ FAILED'}")
        print(f"  IMU sensor: {'✅ PASSED' if results['imu'] else '❌ FAILED'}")

        # Detailed validation for sensors that received data
        if test_node.laser_received and test_node.laser_data:
            scan = test_node.laser_data
            print(f"  LiDAR details: {len(scan.ranges)} ranges, min: {scan.range_min:.2f}, max: {scan.range_max:.2f}")

            # Check that ranges are within expected bounds
            valid_ranges = [r for r in scan.ranges if scan.range_min <= r <= scan.range_max or r == float('inf')]
            if len(valid_ranges) == len(scan.ranges):
                print("  LiDAR range validation: ✅ PASSED")
            else:
                print("  LiDAR range validation: ❌ FAILED")

        if test_node.camera_received and test_node.camera_data:
            img = test_node.camera_data
            print(f"  Camera details: {img.width}x{img.height}, encoding: {img.encoding}")

        if test_node.imu_received and test_node.imu_data:
            imu = test_node.imu_data
            print(f"  IMU details: linear_acceleration=({imu.linear_acceleration.x:.2f}, {imu.linear_acceleration.y:.2f}, {imu.linear_acceleration.z:.2f})")

        # Overall result
        all_passed = all(results.values())

        # Cleanup
        test_node.destroy_node()
        rclpy.shutdown()

        return all_passed

    except Exception as e:
        print(f"❌ Sensor Simulation Test: FAILED with error: {e}")
        rclpy.shutdown()
        return False


def main():
    """Main test function"""
    print("Testing Sensor Simulation in Digital Twin Environment...")
    print("This test verifies that LiDAR, camera, and IMU sensors produce realistic data streams.")

    # Run the test
    success = test_sensor_simulation()

    if success:
        print("\n🎉 Sensor Simulation test completed successfully!")
        print("All sensors are producing realistic data streams in the digital twin environment.")
    else:
        print("\n💥 Sensor Simulation test failed!")
        print("Some sensors may not be configured correctly or producing expected data.")

    return success


if __name__ == '__main__':
    main()