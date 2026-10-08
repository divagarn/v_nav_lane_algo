#!/usr/bin/env python3
"""
Bridges steering_calculator_node's output (/steering_angle in degrees,
/vehicle_speed in km/h) into the geometry_msgs/Twist the simulated
ackermann_steering_controller's reference_unstamped topic expects - the sim
equivalent of the real cart's steering_uart_bridge path, which does the
same job for real UART output. Ported from cart_navigation_ws (same
Gazebo ackermann_steering_controller wiring, same topic name).

Steering sign convention: steering_calculator_node's positive means RIGHT
(see its own "Flip sign: positive becomes right turn" comment). Twist's
angular.z follows the standard REP-103 convention where positive means LEFT
(counter-clockwise) - negated here so a right-steering command actually
turns the buggy right in Gazebo, not left.

Safety: if either input goes stale (no message within command_timeout_sec),
publishes a zero Twist (stop) rather than continuing to drive on an old
command - same timeout-then-stop pattern uart_sender_node uses for the real
UART path.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class SimAckermannBridge(Node):
    def __init__(self):
        super().__init__('sim_ackermann_bridge')

        self.declare_parameter('wheelbase', 1.6)
        self.declare_parameter('cmd_vel_topic', '/ackermann_steering_controller/reference_unstamped')
        self.declare_parameter('command_timeout_sec', 1.0)
        self.declare_parameter('publish_rate', 10.0)

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.steering_deg = 0.0
        self.speed_kmh = 0.0
        self.last_steering_time = self.get_clock().now()
        self.last_speed_time = self.get_clock().now()
        self.have_steering = False
        self.have_speed = False

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 1)
        self.create_subscription(Float32, '/steering_angle', self._on_steering, 1)
        self.create_subscription(Float32, '/vehicle_speed', self._on_speed, 1)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._publish_cmd)

        self.get_logger().info(
            f'Sim Ackermann bridge: /steering_angle + /vehicle_speed -> Twist on '
            f'"{cmd_vel_topic}" (wheelbase={self.wheelbase}m, '
            f'timeout={self.timeout_sec}s).')

    def _on_steering(self, msg):
        self.steering_deg = msg.data
        self.last_steering_time = self.get_clock().now()
        self.have_steering = True

    def _on_speed(self, msg):
        self.speed_kmh = msg.data
        self.last_speed_time = self.get_clock().now()
        self.have_speed = True

    def _secs_since(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _publish_cmd(self):
        cmd = Twist()
        fresh = (self.have_steering and self.have_speed
                  and self._secs_since(self.last_steering_time) <= self.timeout_sec
                  and self._secs_since(self.last_speed_time) <= self.timeout_sec)
        if fresh:
            v = self.speed_kmh / 3.6  # km/h -> m/s
            steering_rad = math.radians(self.steering_deg)
            cmd.linear.x = v
            cmd.angular.z = -v * math.tan(steering_rad) / self.wheelbase
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimAckermannBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
