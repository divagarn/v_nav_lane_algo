#!/usr/bin/env python3
"""
Static TF broadcaster: base_link -> camera_link.

ROUGH ESTIMATE, not a real calibration - flagged clearly since this feeds
camera/LiDAR fusion for a safety-stop feature. Built from the only figures
available so far:
  - front axle to camera: 0.5 m forward of the front axle
  - camera height: 1.9 m above ground
  - pitch: 30 deg, assumed TILTED DOWN toward the road ahead (a level or
    upward-tilted mount would make no sense for a forward obstacle/ground
    camera at this height) - if the real mount tilts up instead, flip the
    sign of CAMERA_PITCH_DEG below.
  - roll/yaw not measured - assumed 0.

Sign convention: this workspace's quaternion_from_euler(roll, pitch, yaw)
uses ROS REP103 body axes (X forward, Y left, Z up). Under that convention,
positive pitch about Y rotates the forward axis DOWNWARD (verified via the
standard Ry rotation matrix: a point at +X maps to (cos(theta), 0, -sin(theta))
for positive theta), so CAMERA_PITCH_DEG is positive here for a downward tilt.

base_link (this workspace's convention, matching velodyne_static_tf.py) is
at the rear axle, ground level. steering_calculator_node's wheelbase
parameter (1.6 m, front-to-rear axle) is used to convert the front-axle-
relative measurement into a base_link-relative one:
  x = wheelbase + front_axle_to_camera = 1.6 + 0.5 = 2.1 m
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

if not hasattr(np, "float"):
    np.float = float

from tf_transformations import quaternion_from_euler

WHEELBASE = 1.6            # matches steering_calculator_node's default wheelbase param
FRONT_AXLE_TO_CAMERA = 0.5  # given: front axle to camera, forward
CAMERA_HEIGHT = 1.9         # given: camera height above ground
CAMERA_PITCH_DEG = 30.0     # given: tilt from level - see docstring for sign/direction


class CameraStaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('camera_static_tf_broadcaster')

        broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "camera_link"

        t.transform.translation.x = WHEELBASE + FRONT_AXLE_TO_CAMERA  # 2.1 m
        t.transform.translation.y = 0.0
        t.transform.translation.z = CAMERA_HEIGHT  # 1.75 m

        roll, yaw = 0.0, 0.0  # not measured - see module docstring
        pitch = math.radians(CAMERA_PITCH_DEG)
        quat = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        broadcaster.sendTransform(t)

        self.get_logger().warn(
            "Broadcasting ROUGH (uncalibrated) static transform base_link -> camera_link: "
            f"x={t.transform.translation.x:.2f}m z={t.transform.translation.z:.2f}m, "
            f"pitch={CAMERA_PITCH_DEG:.1f}deg (assumed tilted down), roll/yaw assumed 0 - "
            "replace with a real calibration before trusting camera/LiDAR fusion distances "
            "for anything safety-critical."
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraStaticTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
