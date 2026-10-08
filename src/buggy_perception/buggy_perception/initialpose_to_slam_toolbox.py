"""Bridges RViz's "2D Pose Estimate" tool to slam_toolbox's actual manual
relocalization mechanism, for the pause/pose-correction workflow in
closed_loop_navigate_slam.launch.py / real_closed_loop_navigate_slam.launch.py.

RViz's "2D Pose Estimate" always publishes to /initialpose - that's a fixed
part of the tool, not configurable. But slam_toolbox's LOCALIZATION mode
does NOT subscribe to /initialpose at all (confirmed via `ros2 node info
/slam_toolbox` - its subscriptions are /clock, /map, /scan,
/parameter_events, /slam_toolbox/feedback only). An earlier assumption in
this project that it did was wrong. The actual supported mechanism for
manually (re)anchoring localization is calling /slam_toolbox/deserialize_map
again with match_type: LOCALIZE_AT_POSE and the desired initial_pose - the
same service/mechanism that loads the map at startup (map_start_pose), just
invoked again mid-run with a new pose instead of the launch-time one.

This ONLY works against localization_slam_toolbox_node - confirmed live
that calling it against async_slam_toolbox_node (even configured with
mode: localization) logs "Requested a localization deserialization in
non-localization mode" and does not update map->odom, while the identical
call against localization_slam_toolbox_node updates it immediately. See
closed_loop_navigate_slam.launch.py's comment on its slam_toolbox Node for
where that executable is chosen.

This node is the missing link: it subscribes to /initialpose (so "2D Pose
Estimate" keeps working the way an operator expects) and translates each
message into that service call. Because deserialize_map directly resets
slam_toolbox's localization anchor rather than going through the normal
scan-processing path, the TF (map->odom) jumps immediately - the vehicle
does not need to move for this part, unlike the ordinary scan-matching
update path (which is still gated by minimum_travel_distance/
minimum_travel_heading for anything AFTER this reset).
"""

import math

import rclpy
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph


class InitialPoseToSlamToolbox(Node):

    LOCALIZE_AT_POSE = 3

    def __init__(self):
        super().__init__('initialpose_to_slam_toolbox')

        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('deserialize_service', '/slam_toolbox/deserialize_map')
        self.declare_parameter('map_file_name', '')

        self.map_file_name = self.get_parameter('map_file_name').value
        if not self.map_file_name:
            self.get_logger().error(
                'map_file_name parameter not set - cannot relocalize slam_toolbox. '
                'This must match the map_file_name given to the slam_toolbox node itself.')

        self.client = self.create_client(
            DeserializePoseGraph, self.get_parameter('deserialize_service').value)
        self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter('initialpose_topic').value,
            self.initialpose_callback, 10)

        self.get_logger().info(
            f'Bridging {self.get_parameter("initialpose_topic").value} -> '
            f'{self.get_parameter("deserialize_service").value} (LOCALIZE_AT_POSE) for '
            f'map_file_name={self.map_file_name}.')

    def initialpose_callback(self, msg):
        if not self.map_file_name:
            self.get_logger().error('map_file_name not set - ignoring pose.')
            return
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('deserialize_map service not available - is slam_toolbox up?')
            return

        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        request = DeserializePoseGraph.Request()
        request.filename = self.map_file_name
        request.match_type = self.LOCALIZE_AT_POSE
        request.initial_pose = Pose2D(
            x=msg.pose.pose.position.x, y=msg.pose.pose.position.y, theta=yaw)

        future = self.client.call_async(request)
        future.add_done_callback(self._on_response)
        self.get_logger().info(
            f'Relocalizing slam_toolbox at x={request.initial_pose.x:.2f}, '
            f'y={request.initial_pose.y:.2f}, theta={yaw:.3f}.')

    def _on_response(self, future):
        try:
            future.result()
            self.get_logger().info('slam_toolbox relocalized.')
        except Exception as exc:
            self.get_logger().error(f'Relocalization FAILED: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseToSlamToolbox()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
