#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraZoneNode(Node):

    def __init__(self):
        super().__init__('camera_zone_node')

        self.bridge = CvBridge()

        # Subscribe to raw camera image
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish camera image with zones
        self.publisher = self.create_publisher(
            Image,
            '/camera/zone',
            10
        )

        self.get_logger().info('Camera Zone Node Started')


    def image_callback(self, msg):

        try:
            # Convert ROS Image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            height, width = frame.shape[:2]

            # -----------------------------
            # GREEN ZONE (Far)
            # -----------------------------
            green_pts = np.array([
                [int(width * 0.30), int(height * 0.65)],
                [int(width * 0.70), int(height * 0.65)],
                [width, height],
                [0, height]
            ])

            # -----------------------------
            # YELLOW ZONE (Medium)
            # -----------------------------
            yellow_pts = np.array([
                [int(width * 0.38), int(height * 0.52)],
                [int(width * 0.62), int(height * 0.52)],
                [int(width * 0.70), int(height * 0.65)],
                [int(width * 0.30), int(height * 0.65)]
            ])

            # -----------------------------
            # RED ZONE (Near / Danger)
            # -----------------------------
            red_pts = np.array([
                [int(width * 0.45), int(height * 0.42)],
                [int(width * 0.55), int(height * 0.42)],
                [int(width * 0.62), int(height * 0.52)],
                [int(width * 0.38), int(height * 0.52)]
            ])

            # Create transparent overlay
            overlay = frame.copy()

            # Draw zones
            cv2.fillPoly(
                overlay,
                [green_pts],
                (0, 0, 255)
            )

            cv2.fillPoly(
                overlay,
                [yellow_pts],
                (0, 255, 255)
            )

            cv2.fillPoly(
                overlay,
                [red_pts],
                (0, 255, 0)
            )

            # Add transparency
            alpha = 0.35

            frame = cv2.addWeighted(
                overlay,
                alpha,
                frame,
                1 - alpha,
                0
            )

            # Draw zone boundaries
            cv2.polylines(
                frame,
                [green_pts],
                True,
                (0, 0, 255),
                2
            )

            cv2.polylines(
                frame,
                [yellow_pts],
                True,
                (0, 255, 255),
                2
            )

            cv2.polylines(
                frame,
                [red_pts],
                True,
                (0, 255, 0),
                2
            )

            # Convert OpenCV image back to ROS Image
            zone_msg = self.bridge.cv2_to_imgmsg(
                frame,
                encoding='bgr8'
            )

            # Keep original timestamp and frame ID
            zone_msg.header = msg.header

            # Publish
            self.publisher.publish(zone_msg)

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')


def main(args=None):

    rclpy.init(args=args)

    node = CameraZoneNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
