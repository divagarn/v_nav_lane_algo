#!/usr/bin/env python3
"""Sits between controller_server's raw cmd_vel output and Gazebo, forcing
the sim vehicle to match the REAL buggy's actual command contract instead
of TEB's freely-optimized velocity profile.

Confirmed from the real UART protocol (old workspace, uart_sender_node.py /
uart_interface.py): the real hardware does NOT accept a commanded speed at
all - the 6-byte packet is START, yellow, red, degree(16-bit), END. Only
steering angle and a stop/caution flag pair are sent; speed is fixed in
the vehicle itself, not software-controlled. TEB's own max_vel_x (up to
2.5 m/s / 9 km/h, tuned for a vehicle that doesn't exist) and its
continuously-varying speed-for-cornering behavior have no real-hardware
equivalent - simply capping max_vel_x lower would still leave TEB free to
slow down for turns, which the real buggy can't do. This node instead
overwrites the commanded speed's MAGNITUDE outright: exactly
target_speed_ms when TEB commands any meaningful forward motion, exactly
0 when TEB's own linear.x drops below min_forward_speed (same "stopped"
threshold and reasoning as steering_uart_bridge.py's final_red_detected
in the old workspace).

Steering angle is NOT recovered from TEB's Twist at all anymore (the old
atan2(angular_z*wheelbase, speed) approach - see git history / this
file's own docstring before this change for why that was ever needed).
That approach required knowing which SPEED TEB's angular.z was computed
for, and got it wrong in practice: TEB regularly commands a small
linear.x while cornering (its own free-speed optimization slowing for
the turn), and with that small value in atan2's denominator, a perfectly
ordinary bend recovered as a near-maximum steering angle (confirmed
live: ~20deg commands for slight turns). Using target_speed_ms instead
of TEB's own transient speed fixed the worst of it, but the whole
approach was solving a problem (recovering a steering angle from a
velocity command) that pure_pursuit.py's geometry avoids needing to ask
in the first place.

Instead: steering angle is computed DIRECTLY from geometry - current
pose (from odom_topic) + the active plan (/plan, the same path
carrot_path_publisher already produces) - using pure_pursuit.py's
compute_steering_for_path(). It picks a lookahead point on the path and
solves the bicycle-model curvature to reach it. No speed value is
involved in that calculation at all, so there is no "which speed"
ambiguity to get wrong - verified standalone (buggy_nav/pure_pursuit.py
+ its own test) before being wired in here.

TEB/controller_server still runs and still decides WHETHER to move
(commanded_speed_ms below min_forward_speed means TEB found no safe
trajectory / goal reached -> stop) - only the STEERING VALUE itself no
longer comes from TEB's Twist.

Output steering is still slew-rate limited (max_steering_rate_deg_s,
default 30 deg/s) - even a legitimate sharp curvature change (new carrot
point, a replan) ramps in over successive cycles instead of jumping
instantly, since no real steering actuator can snap either.

Also publishes a live TEXT_VIEW_FACING telemetry marker above the vehicle
- target distance (to the current plan's final waypoint), speed, and
steering angle - the same three-field format ground_segmentation/
steering_calculator_node.py and local_nav/steering_uart_bridge.py both
used in the old workspace ("Command: X deg | Speed: Y km/h"), extended
with target distance since carrot_path_publisher already computes exactly
that. A fourth line ("Feedback: ...") is intentionally left for later:
once a real UART bridge exists in this workspace, the ACTUAL steering
angle decoded from hardware feedback (not just what was commanded) goes
here - matching uart_bridge.py's own feedback decode in the old codebase,
which this node cannot show yet since nothing in this workspace talks to
real UART hardware.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker

from buggy_nav.pure_pursuit import compute_steering_for_path


class SpeedGovernor(Node):

    def __init__(self):
        super().__init__('speed_governor')

        self.wheelbase = self.declare_parameter('wheelbase', 1.6).value
        self.max_steering_deg = self.declare_parameter('max_steering_deg', 20.0).value
        # Real hardware fixed-speed presets are 2 or 4 km/h (buggy's own
        # physical setting) - this is the sim-side equivalent selection,
        # one value per run, not a runtime switch.
        self.target_speed_kmph = self.declare_parameter('target_speed_kmph', 4.0).value
        self.target_speed_ms = self.target_speed_kmph / 3.6
        # Below this, treat TEB's own commanded speed as "not really
        # trying to move" and stop outright - same value/reasoning as
        # steering_uart_bridge.py's min_forward_speed in the old workspace.
        self.min_forward_speed = self.declare_parameter('min_forward_speed', 0.05).value
        self.max_steering_rate_deg_s = self.declare_parameter('max_steering_rate_deg_s', 30.0).value
        # How far ahead along /plan to look for the pure-pursuit target
        # point. Shorter = tighter path tracking but jerkier, longer =
        # smoother but cuts corners more - independent of
        # carrot_path_publisher's own carrot_distance (that's how far
        # ahead it PICKS a new goal/candidate, this is how far ahead THIS
        # node looks along the resulting path for steering purposes).
        self.lookahead_distance = self.declare_parameter('lookahead_distance', 4.0).value

        self.latest_odom = None
        self.latest_plan = None
        self._last_cmd_time = None

        self.create_subscription(Twist, 'cmd_vel_in', self.cmd_vel_callback, 10)
        self.create_subscription(
            Odometry, self.declare_parameter('odom_topic', '/ackermann_steering_controller/odometry').value,
            self.odom_callback, 1)
        self.create_subscription(Path, '/plan', self.plan_callback, 1)

        self.cmd_vel_pub = self.create_publisher(
            Twist, self.declare_parameter('cmd_vel_out', '/ackermann_steering_controller/reference_unstamped').value,
            10)
        marker_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.marker_pub = self.create_publisher(Marker, '/steering_telemetry', marker_qos)

        self.latest_steering_deg = 0.0
        self.latest_speed_kmph = 0.0

        self.get_logger().info(
            f'speed_governor initialized (target_speed={self.target_speed_kmph}km/h, '
            f'max_steering=+-{self.max_steering_deg}deg, max_steering_rate='
            f'{self.max_steering_rate_deg_s}deg/s, wheelbase={self.wheelbase}m, '
            f'min_forward_speed={self.min_forward_speed}m/s)'
        )

    def odom_callback(self, msg):
        self.latest_odom = msg

    def plan_callback(self, msg):
        self.latest_plan = msg

    def _yaw_from_quaternion(self, q):
        # Same formula/convention as carrot_path_publisher.py's own helper.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _pure_pursuit_steering_deg(self):
        """Current pose + /plan -> steering angle, geometrically (see this
        file's docstring). Returns None if odom/plan aren't available yet
        or the plan is empty - caller falls back to 0.0 (safe default,
        same as the old TEB-Twist-unavailable case)."""
        if self.latest_odom is None or self.latest_plan is None or not self.latest_plan.poses:
            return None
        pose = self.latest_odom.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation)
        path = [(p.pose.position.x, p.pose.position.y) for p in self.latest_plan.poses]
        result = compute_steering_for_path(
            pose.position.x, pose.position.y, yaw, path,
            self.wheelbase, self.max_steering_deg, self.lookahead_distance)
        return result['steering_deg']

    def cmd_vel_callback(self, msg):
        commanded_speed_ms = msg.linear.x
        now = self.get_clock().now()
        dt = None
        if self._last_cmd_time is not None:
            dt = (now - self._last_cmd_time).nanoseconds / 1e9
        self._last_cmd_time = now

        if abs(commanded_speed_ms) < self.min_forward_speed:
            steering_deg = 0.0
            out_speed_ms = 0.0
        else:
            # Steering comes directly from geometry (pose + /plan), not
            # from TEB's Twist - see this file's docstring for why.
            steering_deg = self._pure_pursuit_steering_deg()
            if steering_deg is None:
                steering_deg = 0.0
            # Slew-rate limit: cap how many degrees this can change by
            # since the last cycle, so a legitimate sharp curvature change
            # ramps in instead of jumping instantly (no real steering
            # actuator can snap either).
            if dt is not None and dt > 0.0:
                max_delta = self.max_steering_rate_deg_s * dt
                steering_deg = max(
                    self.latest_steering_deg - max_delta,
                    min(self.latest_steering_deg + max_delta, steering_deg))
            # Real hardware never reverses (max_vel_x_backwards: 0.0 is
            # already enforced upstream in teb_controller.yaml) - sign is
            # always forward here regardless of what TEB commanded.
            out_speed_ms = self.target_speed_ms

        out = Twist()
        out.linear.x = out_speed_ms
        if out_speed_ms > 0.0:
            # Re-derive angular.z for the FIXED speed from the recovered
            # steering angle, inverse of the atan2 above - keeps the
            # executed curvature matching what TEB actually planned
            # instead of an accidental side effect of overwriting speed
            # alone (same angular.z at a different linear.x is a
            # different turning radius).
            out.angular.z = -math.tan(math.radians(steering_deg)) * out_speed_ms / self.wheelbase
        else:
            out.angular.z = 0.0
        self.cmd_vel_pub.publish(out)

        self.latest_steering_deg = steering_deg
        self.latest_speed_kmph = out_speed_ms * 3.6
        self._publish_telemetry_marker()

    def _target_distance(self):
        if self.latest_plan is None or not self.latest_plan.poses or self.latest_odom is None:
            return None
        last = self.latest_plan.poses[-1].pose.position
        cur = self.latest_odom.pose.pose.position
        return math.hypot(last.x - cur.x, last.y - cur.y)

    def _publish_telemetry_marker(self):
        dist = self._target_distance()
        dist_text = f'{dist:.2f}m' if dist is not None else 'n/a'
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'steering_telemetry'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.2
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.4
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 1.0, 1.0
        marker.text = (
            f'Target: {dist_text} | Speed: {self.latest_speed_kmph:.1f} km/h | '
            f'Steering: {self.latest_steering_deg:+.1f} deg'
        )
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedGovernor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
