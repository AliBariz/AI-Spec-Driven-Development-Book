#!/usr/bin/env python3

"""
Python Agent for Humanoid Robot Control

This agent connects to ROS controllers and demonstrates how to send commands
and receive sensor data from the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import math


class HumanoidAgent(Node):
    def __init__(self):
        super().__init__('humanoid_agent')

        # Publishers for different types of commands
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers for sensor data
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Timer for periodic actions
        self.timer = self.create_timer(1.0, self.periodic_action)

        # Store latest sensor data
        self.latest_joint_state = None
        self.latest_imu_data = None

        self.get_logger().info('Humanoid Agent initialized and ready to control the robot')

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        self.latest_joint_state = msg
        self.get_logger().debug(f'Received joint state with {len(msg.name)} joints')

    def imu_callback(self, msg):
        """Callback for IMU messages"""
        self.latest_imu_data = msg
        self.get_logger().debug('Received IMU data')

    def periodic_action(self):
        """Perform periodic actions - for demonstration purposes"""
        self.get_logger().info('Agent performing periodic check...')

        # Print current joint information if available
        if self.latest_joint_state:
            self.get_logger().info(f'Current joint count: {len(self.latest_joint_state.name)}')

    def move_to_position(self, joint_names, positions, duration=5.0):
        """
        Move specified joints to target positions

        Args:
            joint_names: List of joint names to move
            positions: List of target positions (in radians)
            duration: Time to reach the target (in seconds)
        """
        if len(joint_names) != len(positions):
            self.get_logger().error('joint_names and positions must have the same length')
            return

        # Create joint trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)  # Start with zero velocity
        point.accelerations = [0.0] * len(positions)  # Start with zero acceleration
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        trajectory_msg.points.append(point)

        # Publish the trajectory
        self.joint_trajectory_pub.publish(trajectory_msg)
        self.get_logger().info(f'Moving joints {joint_names} to positions {positions}')

    def send_velocity_command(self, linear_x=0.0, angular_z=0.0):
        """
        Send velocity command to the robot

        Args:
            linear_x: Linear velocity in x direction (m/s)
            angular_z: Angular velocity around z axis (rad/s)
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z

        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f'Sent velocity command: linear.x={linear_x}, angular.z={angular_z}')

    def wave_arm(self):
        """Make the robot wave its arm"""
        self.get_logger().info('Making robot wave its arm')

        # Wave left arm
        joint_names = ['left_shoulder_joint', 'left_elbow_joint']
        positions = [0.5, 0.5]  # Wave position
        self.move_to_position(joint_names, positions, duration=2.0)

        # Wait and return to neutral
        time.sleep(2)
        positions = [0.0, 0.0]  # Neutral position
        self.move_to_position(joint_names, positions, duration=2.0)

    def get_robot_state(self):
        """Get current state of the robot"""
        state = {
            'joint_state': self.latest_joint_state,
            'imu_data': self.latest_imu_data
        }
        return state


def main(args=None):
    rclpy.init(args=args)

    agent = HumanoidAgent()

    try:
        # Example: Make the robot wave after 3 seconds
        def wave_callback():
            agent.wave_arm()

        # Create a one-shot timer to wave after 3 seconds
        wave_timer = agent.create_timer(3.0, wave_callback)

        # Cancel the timer after it executes once
        def cancel_timer():
            agent.destroy_timer(wave_timer)

        agent.create_timer(3.1, cancel_timer)

        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()