#!/usr/bin/env python3
"""Relays ackermann_steering_controller's odom->base_link transform onto the
standard /tf topic.

ackermann_steering_controller publishes its odometry transform on
/ackermann_steering_controller/tf_odometry (tf2_msgs/TFMessage) instead of
broadcasting through the standard tf2 mechanism - so tf2_ros buffer clients
(Nav2's costmap, in particular) can't see it via the usual canTransform/
lookupTransform calls no matter what frame names are configured correctly.
This just re-publishes each transform it receives onto /tf.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TfOdomRelay(Node):

    def __init__(self):
        super().__init__('tf_odom_relay')

        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.create_subscription(
            TFMessage, '/ackermann_steering_controller/tf_odometry', self.relay_callback, 10)

        self.get_logger().info('Relaying /ackermann_steering_controller/tf_odometry -> /tf')

    def relay_callback(self, msg):
        self.tf_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
