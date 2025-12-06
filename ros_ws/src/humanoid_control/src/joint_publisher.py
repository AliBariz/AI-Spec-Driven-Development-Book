#!/usr/bin/env python3

"""
Joint Publisher Node for Humanoid Robot

This node publishes joint commands to control the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time


class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')

        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Create publisher for joint trajectory commands
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        # Timer to publish joint states at 50Hz
        self.timer = self.create_timer(0.02, self.publish_joint_states)

        # Initialize joint positions
        self.joint_positions = {}
        self.initialize_joints()

        self.get_logger().info('Joint Publisher Node has been started')

    def initialize_joints(self):
        """Initialize joint positions to zero or default values"""
        # Torso joints
        self.joint_positions['neck_joint'] = 0.0

        # Arm joints
        self.joint_positions['left_shoulder_joint'] = 0.0
        self.joint_positions['left_elbow_joint'] = 0.0
        self.joint_positions['right_shoulder_joint'] = 0.0
        self.joint_positions['right_elbow_joint'] = 0.0

        # Leg joints
        self.joint_positions['left_hip_joint'] = 0.0
        self.joint_positions['left_knee_joint'] = 0.0
        self.joint_positions['right_hip_joint'] = 0.0
        self.joint_positions['right_knee_joint'] = 0.0

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Set joint names
        msg.name = list(self.joint_positions.keys())

        # Set joint positions
        msg.position = list(self.joint_positions.values())

        # Set velocities and efforts to zero for now
        msg.velocity = [0.0] * len(msg.position)
        msg.effort = [0.0] * len(msg.position)

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    joint_publisher = JointPublisher()

    try:
        rclpy.spin(joint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()