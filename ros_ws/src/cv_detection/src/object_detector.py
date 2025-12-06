#!/usr/bin/env python3

"""
Computer Vision Object Detection Node

This node implements a conceptual object detection system for the humanoid robot.
In a real implementation, this would use deep learning models for object detection.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import cv2


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')

        # Create subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        # Create publisher for object detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10
        )

        # Create publisher for object detection status
        self.status_pub = self.create_publisher(
            String,
            '/object_detection/status',
            10
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Object detection state
        self.is_detecting = False

        self.get_logger().info('Object Detector Node initialized')

    def image_callback(self, msg):
        """Callback for image data"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process image for object detection
            detections = self.detect_objects(cv_image)

            # Publish detections
            self.publish_detections(detections, msg.header)

            # Publish status
            status_msg = String()
            status_msg.data = f'Detected {len(detections)} objects'
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_objects(self, image):
        """
        Detect objects in the image.

        In a real implementation, this would use a deep learning model like YOLO,
        SSD, or similar for object detection. For this conceptual implementation,
        we'll use simple color-based detection to identify "red objects".
        """
        detections = []

        # Convert BGR to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color (in HSV)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Find contours of red objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Filter by area to avoid tiny detections
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate center point
                center_x = x + w // 2
                center_y = y + h // 2

                # Create detection
                detection = {
                    'bbox': (x, y, w, h),
                    'center': (center_x, center_y),
                    'area': area,
                    'label': 'red_object'
                }
                detections.append(detection)

        # Also detect other simple shapes/colors if needed
        # For example, detect blue objects
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours_blue:
            area = cv2.contourArea(contour)
            if area > 500:
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                detection = {
                    'bbox': (x, y, w, h),
                    'center': (center_x, center_y),
                    'area': area,
                    'label': 'blue_object'
                }
                detections.append(detection)

        return detections

    def publish_detections(self, detections, header):
        """Publish object detections"""
        detection_array = Detection2DArray()
        detection_array.header = header

        for detection in detections:
            detection_msg = Detection2D()
            detection_msg.header = header

            # Set bounding box
            detection_msg.bbox.size_x = detection['bbox'][2]
            detection_msg.bbox.size_y = detection['bbox'][3]

            # Set center point
            center_point = Point()
            center_point.x = float(detection['center'][0])
            center_point.y = float(detection['center'][1])
            center_point.z = 0.0  # Depth would come from stereo/depth camera
            detection_msg.bbox.center = center_point

            # Set object hypothesis (confidence and ID)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.object_id = detection['label']
            hypothesis.hypothesis.score = 0.8  # Fixed confidence for demo
            detection_msg.results.append(hypothesis)

            detection_array.detections.append(detection_msg)

        self.detection_pub.publish(detection_array)

        if detections:
            self.get_logger().info(f'Published {len(detections)} object detections')


def main(args=None):
    rclpy.init(args=args)

    detector_node = ObjectDetectorNode()

    try:
        rclpy.spin(detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        detector_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()