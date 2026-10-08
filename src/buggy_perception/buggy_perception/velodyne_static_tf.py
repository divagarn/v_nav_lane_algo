#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

# NumPy compatibility for older transforms3d used by tf_transformations.
if not hasattr(np, "float"):
    np.float = float

from tf_transformations import quaternion_from_euler


class VelodyneStaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('velodyne_static_tf_broadcaster')

        broadcaster = StaticTransformBroadcaster(self)

        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = "base_link"   # parent frame
        static_transformStamped.child_frame_id = "velodyne"     # child frame

        # Translation: x=1.73m (from rear axle to LiDAR), y=0, z=1.00m (height from ground)
        static_transformStamped.transform.translation.x = 1.73  # 173cm from vehicle rear axle to LiDAR
        static_transformStamped.transform.translation.y = 0.0   # Centered on vehicle
        static_transformStamped.transform.translation.z = 1.00  # 100cm height from ground

        # Euler angles (roll, pitch, yaw) in radians
        roll = 0.0
        # pitch = 0.436332
        # pitch  = 0.296706
        pitch = 0.0
        yaw = 0.0
        # Convert Euler to quaternion
        quat = quaternion_from_euler(roll, pitch, yaw)

        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        # Broadcast once (static transform)
        broadcaster.sendTransform(static_transformStamped)

        self.get_logger().info("Broadcasted static transform from base_link to velodyne: x=1.73m, z=1.00m")


def main(args=None):
    rclpy.init(args=args)
    node = VelodyneStaticTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
