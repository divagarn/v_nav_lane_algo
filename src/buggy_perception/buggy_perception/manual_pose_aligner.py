#!/usr/bin/env python3
"""Keyboard-driven fine alignment for slam_toolbox's manual relocalization
(the /initialpose -> LOCALIZE_AT_POSE bridge, initialpose_to_slam_toolbox -
already running via closed_loop_navigate_slam.launch.py /
real_closed_loop_navigate_slam.launch.py).

Workflow: use RViz's "2D Pose Estimate" first to give a rough position -
this node subscribes to the very same /initialpose topic and adopts
whatever you click as its starting point automatically. Then use the
keyboard to nudge it: each keypress moves the estimate by one small step
and immediately republishes /initialpose, so slam_toolbox's map->odom
jumps to match right away (same mechanism as a real click - the vehicle
does not need to move for this to take effect, see
initialpose_to_slam_toolbox's own docstring) and you can watch the live
scan slide into alignment with the map in RViz between keystrokes, instead
of trying to land the exact pose with one drag-and-drop click.

Keys:
  w / s   move forward / backward, along the CURRENT yaw estimate
  a / d   rotate left / right
  q / e   strafe left / right (perpendicular to current yaw) - forward/
          back + rotate alone can't reach an arbitrary lateral offset
          without repeatedly rotating first, so this is a real third
          degree of freedom, not just a convenience
  [ / ]   smaller / larger linear step
  , / .   smaller / larger angular step
  space   re-publish the current estimate unchanged (e.g. after
          slam_toolbox restarted, or just to confirm what's live)
  g       CONFIRM alignment and publish /navigation_enabled=true
  x       publish /navigation_enabled=false again (e.g. to realign
          mid-run)
  Ctrl-C  quit (does not touch localization or /navigation_enabled on
          exit - whatever was last published stays in effect)

/navigation_enabled is published here (repeatedly, not once, so a late
subscriber doesn't miss it) as a general-purpose "operator has confirmed
alignment" signal - ported as-is from cart_navigation_ws's version of this
node, where steering_calculator_node/freespace_uart_bridge gate on it
before the vehicle is allowed to drive. Nothing in THIS workspace
currently subscribes to /navigation_enabled - wire it into pure_pursuit.py
/ speed_governor.py (or wherever this workspace's own drive-gating lives)
if the same "don't move until the operator confirms alignment" behavior
is wanted here too.

Runs rclpy.spin() on a background thread (to actually receive the seed
/initialpose message, and any later external one) while the main thread
blocks on raw terminal keypresses - same threading split every ROS2
keyboard-teleop tool uses (e.g. teleop_twist_keyboard), since ROS
callbacks and a blocking stdin read can't share one thread.
"""
import math
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool

KEY_BINDINGS_HELP = """
Manual pose alignment - nudge the localization estimate with the keyboard.
Give a rough starting pose with RViz's "2D Pose Estimate" first (this tool
picks it up automatically), then fine-tune:

  w / s   forward / backward        a / d   rotate left / right
  q / e   strafe left / right       [ / ]   smaller / larger linear step
  , / .   smaller / larger angular step
  space   re-publish current estimate unchanged
  g       CONFIRM alignment - publishes /navigation_enabled=true (only
          meaningful if something in this workspace subscribes to it -
          see this file's own module docstring)
  x       publish /navigation_enabled=false again
  Ctrl-C  quit (does not change /navigation_enabled - press 'x' first if
          you want it held false before exiting)

  linear step={lin:.3f}m  angular step={ang:.2f}deg
"""


class ManualPoseAligner(Node):

    def __init__(self):
        super().__init__('manual_pose_aligner')

        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('linear_step', 0.05)
        self.declare_parameter('angular_step_deg', 1.0)
        # slam_toolbox's deserialize_map service does real work on every
        # call (not a cheap topic echo) - this caps how often a held/
        # auto-repeating key can actually trigger it, so a terminal's key
        # auto-repeat can't flood the service faster than it can keep up.
        self.declare_parameter('min_republish_interval_sec', 0.03)

        self.map_frame = self.get_parameter('map_frame').value
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.angular_step = math.radians(float(self.get_parameter('angular_step_deg').value))
        self.min_interval = float(self.get_parameter('min_republish_interval_sec').value)

        topic = self.get_parameter('initialpose_topic').value
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, topic, 1)
        self.create_subscription(PoseWithCovarianceStamped, topic, self._on_initialpose, 10)

        # See module docstring - published here for parity with
        # cart_navigation_ws's version, not yet consumed by anything in
        # this workspace.
        self.nav_enabled_pub = self.create_publisher(Bool, '/navigation_enabled', 1)
        self.navigation_enabled = False
        self.create_timer(0.2, lambda: self.nav_enabled_pub.publish(Bool(data=self.navigation_enabled)))

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_seed = False
        self._last_publish_time = 0.0
        self._lock = threading.Lock()

    def _on_initialpose(self, msg):
        """Adopts ANY message on this topic as the new seed - including our
        own republishes (a harmless no-op: they carry the exact (x,y,yaw)
        we just set), so an external "2D Pose Estimate" click always wins
        immediately and resets where nudging continues from."""
        with self._lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.have_seed = True

    def nudge(self, dx_forward=0.0, dy_left=0.0, dyaw=0.0):
        with self._lock:
            cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)
            # dx_forward/dy_left are in the CURRENT yaw's own local frame
            # (forward=x, left=y), rotated into map frame before applying -
            # 'w' always means "forward from wherever it's currently
            # facing", not a fixed world direction.
            self.x += dx_forward * cos_y - dy_left * sin_y
            self.y += dx_forward * sin_y + dy_left * cos_y
            self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
            x, y, yaw = self.x, self.y, self.yaw
        self._publish(x, y, yaw)

    def republish(self):
        with self._lock:
            x, y, yaw = self.x, self.y, self.yaw
        self._publish(x, y, yaw)

    def set_navigation_enabled(self, enabled):
        self.navigation_enabled = enabled
        self.nav_enabled_pub.publish(Bool(data=enabled))
        print(f'\n{"NAVIGATION ENABLED (/navigation_enabled=true)" if enabled else "NAVIGATION HELD (/navigation_enabled=false)"}')

    def _publish(self, x, y, yaw):
        now = time.monotonic()
        if now - self._last_publish_time < self.min_interval:
            return
        self._last_publish_time = now
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Covariance is never read by initialpose_to_slam_toolbox (only
        # position/orientation matter for LOCALIZE_AT_POSE) - left at zero.
        self.pose_pub.publish(msg)
        print(f'\rPose: x={x:7.3f}  y={y:7.3f}  yaw={math.degrees(yaw):7.1f}deg   ', end='', flush=True)


def _getch(settings):
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return ch


def main(args=None):
    rclpy.init(args=args)
    node = ManualPoseAligner()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)
    print(KEY_BINDINGS_HELP.format(lin=node.linear_step, ang=math.degrees(node.angular_step)))
    print('Waiting for an initial "2D Pose Estimate" click in RViz (or nudge straight away '
          'from (0,0,0) if you already know that\'s close)...')

    try:
        while rclpy.ok():
            key = _getch(settings)
            if key == '\x03':  # Ctrl-C
                break
            elif key == 'w':
                node.nudge(dx_forward=node.linear_step)
            elif key == 's':
                node.nudge(dx_forward=-node.linear_step)
            elif key == 'a':
                node.nudge(dyaw=node.angular_step)
            elif key == 'd':
                node.nudge(dyaw=-node.angular_step)
            elif key == 'q':
                node.nudge(dy_left=node.linear_step)
            elif key == 'e':
                node.nudge(dy_left=-node.linear_step)
            elif key == '[':
                node.linear_step = max(0.005, node.linear_step * 0.5)
                print(f'\nlinear step -> {node.linear_step:.3f} m')
            elif key == ']':
                node.linear_step = min(1.0, node.linear_step * 2.0)
                print(f'\nlinear step -> {node.linear_step:.3f} m')
            elif key == ',':
                node.angular_step = max(math.radians(0.1), node.angular_step * 0.5)
                print(f'\nangular step -> {math.degrees(node.angular_step):.2f} deg')
            elif key == '.':
                node.angular_step = min(math.radians(30.0), node.angular_step * 2.0)
                print(f'\nangular step -> {math.degrees(node.angular_step):.2f} deg')
            elif key == ' ':
                node.republish()
            elif key == 'g':
                node.set_navigation_enabled(True)
            elif key == 'x':
                node.set_navigation_enabled(False)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
