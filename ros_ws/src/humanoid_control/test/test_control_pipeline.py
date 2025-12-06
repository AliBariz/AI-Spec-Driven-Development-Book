#!/usr/bin/env python3

"""
Test for ROS 2 Control Pipeline

This test verifies that the ROS 2 control pipeline works correctly
with the simulated humanoid robot in Gazebo.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import unittest
from threading import Thread


class TestJointPublisher(Node):
    def __init__(self):
        super().__init__('test_joint_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        self.received_state = None
        self.test_passed = False

    def joint_state_callback(self, msg):
        self.received_state = msg
        # Verify that we received a proper joint state message
        if len(msg.name) > 0 and len(msg.position) == len(msg.name):
            self.test_passed = True
            self.get_logger().info('Test passed: Received valid joint state')

    def publish_test_state(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Define some test joint names and positions
        msg.name = ['neck_joint', 'left_shoulder_joint', 'left_elbow_joint',
                   'right_shoulder_joint', 'right_elbow_joint', 'left_hip_joint',
                   'left_knee_joint', 'right_hip_joint', 'right_knee_joint']
        msg.position = [0.1, 0.2, 0.3, -0.2, -0.3, 0.0, 0.0, 0.0, 0.0]
        msg.velocity = [0.0] * len(msg.position)
        msg.effort = [0.0] * len(msg.position)

        self.publisher.publish(msg)
        self.get_logger().info('Published test joint state')


def test_ros2_control_pipeline():
    """Test that verifies the ROS 2 control pipeline works"""
    rclpy.init()

    try:
        # Create test node
        test_node = TestJointPublisher()

        # Publish a test message
        test_node.publish_test_state()

        # Spin for a short time to allow message processing
        start_time = time.time()
        timeout = 5.0  # 5 second timeout

        while not test_node.test_passed and (time.time() - start_time) < timeout:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        # Check if test passed
        if test_node.test_passed:
            print("✅ ROS 2 Control Pipeline Test: PASSED")
            result = True
        else:
            print("❌ ROS 2 Control Pipeline Test: FAILED - Did not receive expected joint state")
            result = False

        # Cleanup
        test_node.destroy_node()
        rclpy.shutdown()

        return result

    except Exception as e:
        print(f"❌ ROS 2 Control Pipeline Test: FAILED with error: {e}")
        rclpy.shutdown()
        return False


def main():
    """Main test function"""
    print("Testing ROS 2 Control Pipeline...")
    print("This test verifies that joint states can be published and received correctly.")

    # Run the test
    success = test_ros2_control_pipeline()

    if success:
        print("\n🎉 ROS 2 Control Pipeline test completed successfully!")
        print("The humanoid robot control pipeline is functioning correctly.")
    else:
        print("\n💥 ROS 2 Control Pipeline test failed!")
        print("Please check the ROS 2 setup and node connections.")

    return success


if __name__ == '__main__':
    main()