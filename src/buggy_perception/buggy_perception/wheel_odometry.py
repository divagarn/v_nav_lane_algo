#!/usr/bin/env python3
"""Real-hardware odometry integrator.

Neither the real vehicle nor its existing reactive nav pipeline
(front_view_occupancy_map -> obstacle_detector -> steering_calculator_node)
has ever needed a persistent odom->base_link transform or a
nav_msgs/Odometry - that pipeline works entirely in sensor-relative space
(ROI boxes in the LiDAR/base_link frame). Our costmap/NavFn/DWB stack does
need one: both costmap configs set global_frame: odom, and
controller_server's own "odom_topic" param (default "odom") is what DWB
reads for current-velocity feedback, separately from TF.

This integrates /wheel_uart (ground_segmentation_msgs/SteerSpeed -
steering in degrees, positive = right, and speed in km/h - both already
calibrated by the vehicle's own MCU before the packet is sent, see
SteerSpeed.msg and uart_bridge.py) into a standard bicycle-model
dead-reckoning odometry: the same relationship used everywhere else in
this project (sim_actuation_bridge.py, steering_uart_bridge.py), just
integrated forward instead of solved for steering angle, and broadcasts
the odom->base_link TF that a real Gazebo-free run has no other source
for.

Dead reckoning only - no correction for wheel slip or drift. Fine for a
short local-planning horizon (this stack never plans further than
~10m ahead), but this is not a substitute for real localization (GPS/IMU
fusion) if one becomes available later.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from ground_segmentation_msgs.msg import SteerSpeed


class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        self.wheelbase = self.declare_parameter('wheelbase', 1.6).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # False when kiss_icp is supplying the real odom->base_link TF
        # instead (enable_kiss_icp:=true in real_sensors_bringup.launch.py)
        # - this node keeps integrating and publishing /odom either way
        # (still consumed by DWB/controller_server's odom_topic), it just
        # stops fighting kiss_icp over the same TF.
        self.publish_tf = self.declare_parameter('publish_tf', True).value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(SteerSpeed, '/wheel_uart', self.feedback_callback, 10)

        tf_note = f'{self.odom_frame}->{self.base_frame} TF' if self.publish_tf else 'no TF (publish_tf=false)'
        self.get_logger().info(
            f'Wheel odometry integrator initialized (wheelbase={self.wheelbase}m, '
            f'dead-reckoning from /wheel_uart -> /odom + {tf_note})'
        )

    def feedback_callback(self, msg):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        speed_ms = msg.speed / 3.6
        steering_rad = math.radians(msg.steering)

        # Same SteerSpeed positive=right vs ROS angular.z positive=left
        # flip as sim_actuation_bridge.py / steering_uart_bridge.py.
        angular_z = (
            -math.tan(steering_rad) * speed_ms / self.wheelbase
            if abs(speed_ms) > 1e-6 else 0.0
        )

        # Bicycle-model dead-reckoning integration.
        self.x += speed_ms * math.cos(self.yaw) * dt
        self.y += speed_ms * math.sin(self.yaw) * dt
        self.yaw += angular_z * dt

        stamp = now.to_msg()
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = speed_ms
        odom.twist.twist.angular.z = angular_z
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
