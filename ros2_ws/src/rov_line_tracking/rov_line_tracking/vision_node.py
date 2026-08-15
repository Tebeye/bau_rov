#!/usr/bin/env python3
"""
==============================================================================
FAZ 3: OpenCV Image Processing Node (vision_node)
==============================================================================
Task Requirements:
- T-3.1: ROS 2 node subscribing to /camera/image_raw using cv_bridge.
- T-3.2: BGR -> Grayscale conversion & cv2.adaptiveThreshold (Gaussian).
- T-3.3: Line contour extraction (cv2.findContours) & Moments calculation (Cx, Cy).
- T-3.4: Yaw error calculation (Hata_Yaw = X_center - Cx) published on /vision/line_error.
==============================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('line_error_topic', '/vision/line_error')
        self.declare_parameter('adaptive_threshold_block_size', 19)
        self.declare_parameter('adaptive_threshold_c', 5)
        self.declare_parameter('min_contour_area', 500.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('debug_view', False)

        # Retrieve parameter values
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        line_error_topic = self.get_parameter('line_error_topic').get_parameter_value().string_value
        self.block_size = self.get_parameter('adaptive_threshold_block_size').get_parameter_value().integer_value
        self.c_val = self.get_parameter('adaptive_threshold_c').get_parameter_value().integer_value
        self.min_area = self.get_parameter('min_contour_area').get_parameter_value().double_value
        self.img_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.img_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.debug_view = self.get_parameter('debug_view').get_parameter_value().bool_value

        # cv_bridge initialization (T-3.1)
        self.bridge = CvBridge()

        # Publisher for line error (T-3.4)
        self.error_pub = self.create_publisher(Float64, line_error_topic, 10)

        # Subscriber for camera raw image (T-3.1)
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f"Vision Node started. Subscribed to '{image_topic}', publishing error to '{line_error_topic}'")

    def image_callback(self, msg: Image):
        """Processes incoming camera frame to compute centroid offset (Hata_Yaw)."""
        try:
            # Convert ROS Image message to OpenCV BGR image (T-3.1)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        height, width, _ = frame.shape
        x_center = width / 2.0

        # T-3.2: BGR -> Grayscale conversion
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # T-3.2: cv2.adaptiveThreshold (Gaussian)
        thresh = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            self.block_size if self.block_size % 2 == 1 else self.block_size + 1,
            self.c_val
        )

        # T-3.3: cv2.findContours for line contour detection
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        error_msg = Float64()

        if contours:
            # Filter contours by minimum area and find the largest line segment
            valid_contours = [c for c in contours if cv2.contourArea(c) >= self.min_area]

            if valid_contours:
                largest_contour = max(valid_contours, key=cv2.contourArea)

                # T-3.3: cv2.moments for Center of Mass (Cx, Cy)
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cx = float(M['m10'] / M['m00'])
                    cy = float(M['m01'] / M['m00'])

                    # T-3.4: Calculate Yaw Error: Hata_Yaw = X_center - Cx
                    hata_yaw = x_center - cx
                    error_msg.data = hata_yaw

                    self.error_pub.publish(error_msg)
                    self.get_logger().debug(f"Centroid: ({cx:.1f}, {cy:.1f}), Hata_Yaw: {hata_yaw:.2f}")

                    if self.debug_view:
                        cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                        cv2.line(frame, (int(x_center), 0), (int(x_center), height), (255, 0, 0), 1)
                        cv2.putText(frame, f"Error: {hata_yaw:.1f}px", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.imshow("Vision Line Tracking", frame)
                        cv2.waitKey(1)
                    return

        # If no line valid contour is detected, publish NaN or designated sentinel signal
        error_msg.data = float('nan')
        self.error_pub.publish(error_msg)
        self.get_logger().warn("Line lost: No valid contour detected.")


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
