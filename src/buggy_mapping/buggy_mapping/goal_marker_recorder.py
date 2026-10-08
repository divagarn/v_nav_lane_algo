#!/usr/bin/env python3
"""Records RViz clicks against a loaded map into persistent markers, for
checking/picking start and goal coordinates BEFORE running any real
navigation - no Gazebo, no sensors, no localization, just the saved map
and RViz's own click tools (map_viewer.launch.py brings up only
nav2_map_server + this node + RViz).

  - "2D Pose Estimate" (/initialpose) -> ONE start marker (green arrow),
    overwritten on every new click - there's only ever one start point.
  - "2D Goal Pose" (/goal_pose) -> ACCUMULATES: every click adds a new
    numbered goal marker (amber sphere + text label) without erasing
    earlier ones, so multiple candidate goals can be compared on the map
    at once.
  - "Publish Point" (/clicked_point) -> CLEARS all accumulated goal
    markers (start marker untouched) - a free reset using a tool RViz
    already has, no extra service/UI needed.

Every click's x/y(/yaw) is also logged to the console so exact numbers
can be copied straight into a launch command's spawn_x/spawn_y/spawn_yaw
or used as a goal for the real navigation launch later.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GoalMarkerRecorder(Node):

    def __init__(self):
        super().__init__('goal_marker_recorder')

        self.start_marker = None
        self.goal_markers = []

        markers_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.marker_pub = self.create_publisher(MarkerArray, '/goal_markers', markers_qos)

        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._on_start, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_subscription(PointStamped, '/clicked_point', self._on_clear, 10)

        self.get_logger().info(
            'goal_marker_recorder ready - "2D Pose Estimate" sets the start marker, '
            '"2D Goal Pose" adds a goal marker (repeatable), "Publish Point" clears '
            'accumulated goal markers.')

    def _on_start(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self.get_logger().info(f'START set: x={x:.3f} y={y:.3f} yaw={yaw:.3f} rad')

        marker = Marker()
        marker.header = msg.header
        marker.ns = 'start'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = msg.pose.pose
        marker.scale.x, marker.scale.y, marker.scale.z = 1.2, 0.25, 0.25
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.15, 0.9, 0.35, 1.0
        self.start_marker = marker
        self._publish_all()

    def _on_goal(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        yaw = _yaw_from_quaternion(msg.pose.orientation)
        idx = len(self.goal_markers) // 2  # two markers (sphere + label) per goal
        self.get_logger().info(f'GOAL #{idx} added: x={x:.3f} y={y:.3f} yaw={yaw:.3f} rad')

        sphere = Marker()
        sphere.header = msg.header
        sphere.ns = 'goals'
        sphere.id = idx * 2
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = msg.pose
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.6
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.65, 0.05, 0.95

        label = Marker()
        label.header = msg.header
        label.ns = 'goals'
        label.id = idx * 2 + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = x
        label.pose.position.y = y
        label.pose.position.z = 0.8
        label.pose.orientation.w = 1.0
        label.scale.z = 0.5
        label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 1.0
        label.text = f'#{idx}  ({x:.2f}, {y:.2f})'

        self.goal_markers.append(sphere)
        self.goal_markers.append(label)
        self._publish_all()

    def _on_clear(self, msg):
        count = len(self.goal_markers) // 2
        self.get_logger().info(f'Clearing {count} goal marker(s) (Publish Point clicked).')
        delete_all = Marker()
        delete_all.header.frame_id = msg.header.frame_id
        delete_all.ns = 'goals'
        delete_all.action = Marker.DELETEALL
        array = MarkerArray()
        array.markers.append(delete_all)
        self.marker_pub.publish(array)
        self.goal_markers = []

    def _publish_all(self):
        array = MarkerArray()
        if self.start_marker is not None:
            array.markers.append(self.start_marker)
        array.markers.extend(self.goal_markers)
        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = GoalMarkerRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
