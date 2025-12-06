#!/usr/bin/env python3

"""
Test script for VSLAM Pipeline

This test verifies that the VSLAM pipeline can process image data and produce
pose estimates for the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import cv2


class VSLAMTestNode(Node):
    def __init__(self):
        super().__init__('vslam_test_node')

        # Create publisher for test images
        self.image_pub = self.create_publisher(
            Image,
            '/camera/rgb/image_raw',
            10
        )

        # Create subscriber for pose estimates
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/pose',
            self.pose_callback,
            10
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Test state
        self.pose_received = False
        self.test_image = None

        self.get_logger().info('VSLAM Test Node initialized')

    def pose_callback(self, msg):
        """Callback for pose messages"""
        self.pose_received = True
        self.get_logger().info(f'Received pose estimate: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})')

    def generate_test_image(self):
        """Generate a simple test image"""
        # Create a simple test image with some features
        image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Add some colored rectangles to serve as features
        cv2.rectangle(image, (100, 100), (200, 200), (255, 0, 0), -1)  # Blue
        cv2.rectangle(image, (300, 150), (400, 250), (0, 255, 0), -1)  # Green
        cv2.rectangle(image, (200, 300), (300, 400), (0, 0, 255), -1)  # Red

        return image

    def run_test(self):
        """Run the VSLAM test"""
        self.get_logger().info('Running VSLAM test...')

        # Generate and publish a test image
        test_image = self.generate_test_image()
        ros_image = self.bridge.cv2_to_imgmsg(test_image, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_rgb_frame'

        # Publish the image
        self.image_pub.publish(ros_image)
        self.get_logger().info('Published test image')

        # Wait for a pose estimate
        start_time = self.get_clock().now()
        timeout = 5.0  # 5 second timeout

        while not self.pose_received:
            # Publish another image periodically to keep the pipeline active
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > 1.0:
                ros_image.header.stamp = self.get_clock().now().to_msg()
                self.image_pub.publish(ros_image)
                start_time = self.get_clock().now()

            # Small delay
            self.get_clock().sleep_for_nanoseconds(int(0.1 * 1e9))

            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                break

        return self.pose_received


def main():
    rclpy.init()

    test_node = VSLAMTestNode()

    try:
        success = test_node.run_test()

        if success:
            print("\n✅ VSLAM Pipeline test: PASSED")
            print("VSLAM node is successfully processing images and publishing pose estimates")
        else:
            print("\n❌ VSLAM Pipeline test: FAILED")
            print("VSLAM node did not produce pose estimates within the timeout period")

    except Exception as e:
        print(f"\n❌ VSLAM Pipeline test: FAILED with error: {e}")

    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()