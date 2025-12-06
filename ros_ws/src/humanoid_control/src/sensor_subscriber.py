#!/usr/bin/env python3

"""
Sensor Subscriber Node for Humanoid Robot

This node subscribes to sensor data from the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, LaserScan
from geometry_msgs.msg import PointStamped


class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Subscribe to laser scan data
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Subscribe to point cloud data (for depth camera simulation)
        self.point_sub = self.create_subscription(
            PointStamped,
            '/camera/depth/points',
            self.point_callback,
            10
        )

        self.get_logger().info('Sensor Subscriber Node has been started')

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        self.get_logger().info(f'Received joint state with {len(msg.name)} joints')

    def imu_callback(self, msg):
        """Callback for IMU messages"""
        self.get_logger().info('Received IMU data')

    def laser_callback(self, msg):
        """Callback for laser scan messages"""
        self.get_logger().info(f'Received laser scan with {len(msg.ranges)} range values')

    def point_callback(self, msg):
        """Callback for point cloud messages"""
        self.get_logger().info(f'Received point data at ({msg.point.x}, {msg.point.y}, {msg.point.z})')


def main(args=None):
    rclpy.init(args=args)

    sensor_subscriber = SensorSubscriber()

    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()