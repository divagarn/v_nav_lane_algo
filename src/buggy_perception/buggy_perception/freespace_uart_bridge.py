#!/usr/bin/env python3
"""Maps freespace_grid_node's /nav_status (OK/RECOVERING/STUCK) onto this
cart's real UART stop protocol (/final_red_detected, /final_yellow_detected
- consumed directly by buggy_uart's uart_sender_node).

IMPORTANT hardware limitation: the outbound UART protocol has no reverse/
speed field at all - only steering + red(stop)/yellow(caution) flags, and
the MCU applies its own fixed "go" speed whenever neither flag is set.
freespace_grid_node's recovery maneuver (reverse briefly, then retry) is
SIM-ONLY for that reason - it cannot be physically commanded over this real
protocol. On real hardware, RECOVERING and STUCK are both mapped to a plain
stop (red=True) instead: the vehicle holds still and waits rather than
attempting an uncommandable reverse. Getting going again after a STUCK
still requires a human (drive it clear, or fix localization/mapping) - this
bridge only ever asks the vehicle to stop, it never un-stops it on its own.

Also gates on /navigation_enabled (Bool, published by manual_pose_aligner's
'g'/'x' keys once alignment is confirmed) - red=True whenever navigation
isn't enabled, regardless of nav_status, since /vehicle_speed (what
steering_calculator_node's own navigation_enabled gate controls) has no
effect on this hardware at all - the red/yellow flags are the ONLY way to
actually stop the real vehicle, so this bridge has to enforce the same
gate independently rather than relying on steering_calculator_node to do
it upstream.

Fails safe: defaults to red=True (stop) until at least one /nav_status
message has actually been received, and again if /nav_status goes stale -
same "unknown state -> stop" reasoning used throughout this project's
UART-facing nodes.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class FreespaceUartBridge(Node):

    def __init__(self):
        super().__init__('freespace_uart_bridge')

        self.declare_parameter('nav_status_timeout_sec', 2.0)
        self.nav_status_timeout_sec = float(self.get_parameter('nav_status_timeout_sec').value)
        # manual_pose_aligner republishes /navigation_enabled every 0.2s
        # while enabled specifically so this can require a LIVE confirming
        # tool, not a one-time flag stuck true forever if that tool
        # crashes or its terminal is closed - same reasoning as
        # nav_status_timeout_sec above.
        self.declare_parameter('navigation_enabled_timeout_sec', 1.0)
        self.navigation_enabled_timeout_sec = float(
            self.get_parameter('navigation_enabled_timeout_sec').value)

        self.red_pub = self.create_publisher(Bool, '/final_red_detected', 1)
        self.yellow_pub = self.create_publisher(Bool, '/final_yellow_detected', 1)

        self._have_status = False
        self._last_status = 'STUCK'  # fail-safe default before anything arrives
        self._last_status_time = self.get_clock().now()
        self.create_subscription(String, '/nav_status', self._on_nav_status, 1)

        self._navigation_enabled = False
        self._last_navigation_enabled_time = self.get_clock().now()
        self.create_subscription(Bool, '/navigation_enabled', self._on_navigation_enabled, 1)

        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            'freespace_uart_bridge: mapping /nav_status -> /final_red_detected '
            '(RECOVERING and STUCK both mean stop - this hardware cannot reverse, '
            'see module docstring). Defaulting to STOP until /nav_status is seen AND '
            '/navigation_enabled is true.')

    def _on_nav_status(self, msg):
        self._have_status = True
        self._last_status = msg.data
        self._last_status_time = self.get_clock().now()

    def _on_navigation_enabled(self, msg):
        self._navigation_enabled = msg.data
        self._last_navigation_enabled_time = self.get_clock().now()

    def _tick(self):
        age = (self.get_clock().now() - self._last_status_time).nanoseconds / 1e9
        stale = (not self._have_status) or age > self.nav_status_timeout_sec
        status = 'STUCK' if stale else self._last_status

        nav_age = (self.get_clock().now() - self._last_navigation_enabled_time).nanoseconds / 1e9
        navigation_fresh = nav_age <= self.navigation_enabled_timeout_sec

        red = not (self._navigation_enabled and navigation_fresh) or (status != 'OK')
        self.red_pub.publish(Bool(data=red))
        self.yellow_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = FreespaceUartBridge()
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
