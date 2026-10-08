#!/usr/bin/env python3
"""
Publishes frames from a video file onto a ROS2 image topic (default
camera/image_raw), so downstream nodes like yolox_detector_node can be
exercised/checked without a live camera attached.
"""
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class VideoFilePublisherNode(Node):
    def __init__(self):
        super().__init__('video_file_publisher_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('video_path', '')
        self.declare_parameter('output_topic', 'camera/image_raw')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('loop', True)
        self.declare_parameter('fps', 0.0)  # 0.0 = use the video's native fps

        video_path = self.get_parameter('video_path').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.loop = bool(self.get_parameter('loop').value)
        fps_override = float(self.get_parameter('fps').value)

        if not video_path:
            self.get_logger().fatal("Parameter 'video_path' is required, e.g. "
                                     "-p video_path:=/path/to/video.mp4")
            raise SystemExit(1)

        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().fatal(f'Failed to open video file: {video_path}')
            raise SystemExit(1)

        native_fps = self.cap.get(cv2.CAP_PROP_FPS)
        fps = fps_override if fps_override > 0.0 else (native_fps if native_fps > 0.0 else 30.0)
        frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f'Opened video: {video_path} ({width}x{height}, {frame_count} frames, '
            f'{native_fps:.2f} native fps, publishing at {fps:.2f} fps, loop={self.loop})'
        )

        self.bridge = CvBridge()

        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Image, output_topic, camera_qos)

        self.frame_num = 0
        self.timer = self.create_timer(1.0 / fps, self._on_timer)
        self.get_logger().info(f'Publishing to {output_topic}')

    def _on_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            if self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.frame_num = 0
                self.get_logger().info('End of video reached, looping back to start')
                ok, frame = self.cap.read()
                if not ok:
                    self.get_logger().error('Failed to read frame after loop reset')
                    return
            else:
                self.get_logger().info('End of video reached, stopping publisher')
                self.timer.cancel()
                return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)
        self.frame_num += 1

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VideoFilePublisherNode()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
