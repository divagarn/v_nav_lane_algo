#!/usr/bin/env python3

import time
import cv2
import numpy as np
import gi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

gi.require_version('Gst', '1.0')
gi.require_version('GLib', '2.0')

from gi.repository import Gst, GLib


Gst.init(None)


class CameraRawNode(Node):

    def __init__(self):
        super().__init__('camera_raw_node')

        # -----------------------------
        # Camera configuration
        # -----------------------------
        self.device = '/dev/video0'

        self.capture_width = 1920
        self.capture_height = 1080
        self.fps = 30

        self.output_width = 960
        self.output_height = 540

        # -----------------------------
        # ROS publisher
        # -----------------------------
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.bridge = CvBridge()

        # Statistics
        self.count = 0
        self.start_time = None
        self.copy_times = []
        self.convert_times = []

        # -----------------------------
        # GStreamer pipeline
        # -----------------------------
        pipeline_str = (
            f'v4l2src device={self.device} do-timestamp=true io-mode=2 ! '
            f'video/x-raw,format=UYVY,'
            f'width={self.capture_width},'
            f'height={self.capture_height},'
            f'framerate={self.fps}/1 ! '
            'queue max-size-buffers=2 leaky=downstream ! '
            'appsink name=sink '
            'emit-signals=true '
            'sync=false '
            'max-buffers=2 '
            'drop=true'
        )

        self.get_logger().info(
            f'GStreamer pipeline:\n{pipeline_str}'
        )

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except Exception as e:
            self.get_logger().error(
                f'Failed to create GStreamer pipeline: {e}'
            )
            raise

        self.sink = self.pipeline.get_by_name('sink')

        if self.sink is None:
            self.get_logger().error(
                'Could not find appsink'
            )
            raise RuntimeError('appsink not found')

        self.sink.connect(
            'new-sample',
            self.on_new_sample
        )

        # -----------------------------
        # Start GStreamer
        # -----------------------------
        ret = self.pipeline.set_state(
            Gst.State.PLAYING
        )

        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error(
                'Failed to start GStreamer pipeline'
            )
            raise RuntimeError(
                'GStreamer pipeline failed'
            )

        self.get_logger().info(
            'Camera started'
        )

        self.get_logger().info(
            'Publishing images on /camera/image_raw'
        )

    def on_new_sample(self, sink):

        sample = sink.emit('pull-sample')

        if sample is None:
            return Gst.FlowReturn.ERROR

        buffer = sample.get_buffer()
        caps = sample.get_caps()

        structure = caps.get_structure(0)

        width = structure.get_value('width')
        height = structure.get_value('height')

        expected = width * height * 2

        # --------------------------------
        # Copy GStreamer buffer
        # --------------------------------
        t0 = time.perf_counter()

        raw = buffer.extract_dup(
            0,
            expected
        )

        t1 = time.perf_counter()

        # --------------------------------
        # UYVY -> BGR
        # --------------------------------
        yuv = np.frombuffer(
            raw,
            dtype=np.uint8
        ).reshape(
            (height, width, 2)
        )

        bgr = cv2.cvtColor(
            yuv,
            cv2.COLOR_YUV2BGR_UYVY
        )

        # --------------------------------
        # Resize
        # --------------------------------
        if (
            self.output_width,
            self.output_height
        ) != (
            width,
            height
        ):
            bgr = cv2.resize(
                bgr,
                (
                    self.output_width,
                    self.output_height
                ),
                interpolation=cv2.INTER_LINEAR
            )

        # Ensure contiguous memory
        bgr = np.ascontiguousarray(bgr)

        t2 = time.perf_counter()

        # --------------------------------
        # Publish ROS Image
        # --------------------------------
        msg = self.bridge.cv2_to_imgmsg(
            bgr,
            encoding='bgr8'
        )

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'

        self.publisher.publish(msg)

        # --------------------------------
        # Performance statistics
        # --------------------------------
        self.copy_times.append(
            t1 - t0
        )

        self.convert_times.append(
            t2 - t1
        )

        self.count += 1

        if self.start_time is None:
            self.start_time = time.time()

        if self.count % 60 == 0:

            elapsed = (
                time.time() -
                self.start_time
            )

            rc = self.copy_times[-60:]
            rv = self.convert_times[-60:]

            avg_copy_ms = (
                sum(rc) / len(rc)
            ) * 1000.0

            avg_convert_ms = (
                sum(rv) / len(rv)
            ) * 1000.0

            max_convert_ms = (
                max(rv)
            ) * 1000.0

            avg_fps = (
                self.count / elapsed
            )

            # self.get_logger().info(
            #     f'{self.count} frames | '
            #     f'avg fps {avg_fps:.2f} | '
            #     f'avg copy {avg_copy_ms:.2f} ms | '
            #     f'avg convert+resize '
            #     f'{avg_convert_ms:.2f} ms | '
            #     f'max convert '
            #     f'{max_convert_ms:.2f} ms | '
            #     f'out shape {bgr.shape}'
            # )

        return Gst.FlowReturn.OK

    def destroy_node(self):

        self.get_logger().info(
            'Stopping camera...'
        )

        if self.pipeline is not None:
            self.pipeline.set_state(
                Gst.State.NULL
            )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = CameraRawNode()

        # GStreamer appsink callbacks are handled
        # by the GLib/GStreamer thread.
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(
            f'Camera node error: {e}'
        )

    finally:

        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()