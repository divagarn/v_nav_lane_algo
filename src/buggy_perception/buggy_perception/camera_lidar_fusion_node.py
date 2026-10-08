#!/usr/bin/env python3
"""
Camera + LiDAR fusion for distance-based safety stop.

Projects a LiDAR point cloud into the camera image using the fisheye
calibration + the (currently rough/uncalibrated - see camera_static_tf.py)
base_link->camera_link static TF, matches projected points to each camera
detection's 2D box (from a Detection2DArray topic, e.g. camera_processing's
yolox_detector_node or unified_perception_node), and estimates a real-world
distance per detected object from the LiDAR points that fall inside its
box. Any detection closer than stop_distance triggers /camera_red_detected
the same way obstacle_detector.py's LiDAR clustering already triggers
red_zone - two independent triggers, not one fused distance, since neither
sensor's stop decision should depend on the other one working.

Default points_topic is /non_ground_points_filtered (self_hit_filter's
output, always running per real_sensors_bringup.launch.py), not the
pre-self-hit-filter /non_ground_points this node used in its original form
- fusing against the vehicle's own unfiltered chassis returns would read
as a permanently "detected" object a few tens of cm from the camera,
risking a spurious stuck-red state.

This node only estimates distance from whatever Detection2DArray it's
given - it does NOT run object detection itself. Something publishing to
detection_array_topic (yolox_detector_node/unified_perception_node) must
be running separately for this to ever produce a non-empty fused list.

Coordinate convention note: 'velodyne'/'base_link'/'camera_link' all use
ROS body convention (x=forward, y=left, z=up). OpenCV's fisheye model
expects optical convention (x=right, y=down, z=forward). The fixed
body->optical axis remap (x_opt=-y, y_opt=-z, z_opt=x) is applied directly
in code below rather than via an extra TF frame, since it's a fixed,
well-known constant, not something that needs calibrating.

IMPORTANT: base_link->camera_link is a ROUGH ESTIMATE (see
camera_static_tf.py's docstring), and the fisheye intrinsics below were
given calibrated at 1920x1080 - image_width/image_height parameters MUST
match whatever resolution camera/image_raw actually is, or the projected
points will land on the wrong pixels entirely.
"""
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros

from ground_segmentation_msgs.msg import Detection2DArray

# Fisheye calibration, given at this resolution - see module docstring.
CALIB_WIDTH, CALIB_HEIGHT = 1920, 1080
CALIB_K = np.array([
    [865.8168558653789, 0.0, 946.40124792976326],
    [0.0, 866.93611621490459, 539.44936044049723],
    [0.0, 0.0, 1.0],
])
CALIB_D = np.array([-0.02884450658109073, 0.0037397682678126918,
                     -0.011255146215826033, 0.0045604499621032029])


def _body_to_optical(points_xyz):
    """ROS body convention (x fwd, y left, z up) -> OpenCV optical (x right, y down, z fwd)."""
    x, y, z = points_xyz[:, 0], points_xyz[:, 1], points_xyz[:, 2]
    return np.stack([-y, -z, x], axis=1)


class CameraLidarFusionNode(Node):
    def __init__(self):
        super().__init__('camera_lidar_fusion_node')

        self.declare_parameter('points_topic', '/non_ground_points_filtered')
        self.declare_parameter('detection_array_topic', 'camera/detections/array')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('image_width', 960)
        self.declare_parameter('image_height', 540)
        self.declare_parameter('stop_distance', 3.0)
        self.declare_parameter('warning_distance', 6.0)
        self.declare_parameter('min_points_per_box', 3)
        self.declare_parameter('distance_percentile', 20.0)
        self.declare_parameter('debug_image_topic', 'camera/detections/fused/image_raw')
        self.declare_parameter('input_image_topic', 'camera/image_raw')
        self.declare_parameter('publish_debug_image', True)

        self.points_topic = self.get_parameter('points_topic').value
        detection_array_topic = self.get_parameter('detection_array_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        image_width = int(self.get_parameter('image_width').value)
        image_height = int(self.get_parameter('image_height').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.warning_distance = float(self.get_parameter('warning_distance').value)
        self.min_points_per_box = int(self.get_parameter('min_points_per_box').value)
        self.distance_percentile = float(self.get_parameter('distance_percentile').value)
        debug_image_topic = self.get_parameter('debug_image_topic').value
        input_image_topic = self.get_parameter('input_image_topic').value
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)

        # Scale K for the actual running resolution (D is resolution-independent).
        sx, sy = image_width / CALIB_WIDTH, image_height / CALIB_HEIGHT
        self.K = CALIB_K.copy()
        self.K[0, 0] *= sx  # fx
        self.K[1, 1] *= sy  # fy
        self.K[0, 2] *= sx  # cx
        self.K[1, 2] *= sy  # cy
        self.D = CALIB_D.copy()
        self.image_width, self.image_height = image_width, image_height
        if (image_width, image_height) != (CALIB_WIDTH, CALIB_HEIGHT):
            self.get_logger().warn(
                f'image_width/height ({image_width}x{image_height}) differs from the '
                f'calibration resolution ({CALIB_WIDTH}x{CALIB_HEIGHT}) - scaling K '
                'accordingly. Make sure this actually matches camera/image_raw\'s real '
                'resolution or projected points will land on the wrong pixels.'
            )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

        self.latest_cloud = None
        self.latest_frame = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Separate callback groups so the cheap, frequent image callback can't
        # get starved behind the heavier point-cloud-projection callback when
        # spun with a MultiThreadedExecutor (see main()) - same group would
        # still serialize them even across threads.
        fusion_group = MutuallyExclusiveCallbackGroup()
        image_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            PointCloud2, self.points_topic, self._on_points, qos, callback_group=fusion_group)
        self.create_subscription(
            Detection2DArray, detection_array_topic, self._on_detections, qos,
            callback_group=fusion_group)
        if self.publish_debug_image:
            self.create_subscription(
                Image, input_image_topic, self._on_image, qos, callback_group=image_group)

        self.red_pub = self.create_publisher(Bool, '/camera_red_detected', 1)
        self.yellow_pub = self.create_publisher(Bool, '/camera_yellow_detected', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/camera_fused_markers', 1)
        self.debug_image_pub = self.create_publisher(Image, debug_image_topic, qos) \
            if self.publish_debug_image else None

        self.get_logger().info(
            f'Camera/LiDAR fusion node initialized: {self.points_topic} + '
            f'{detection_array_topic} -> /camera_red_detected, /camera_yellow_detected '
            f'(stop<{self.stop_distance}m, warn<{self.warning_distance}m, image '
            f'{image_width}x{image_height})'
        )

    def _on_points(self, msg):
        self.latest_cloud = msg

    def _on_image(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')

    def _project_cloud_to_pixels(self, cloud_frame_id):
        """Returns (uv (N,2) int, forward_dist (N,) float) for points in front of the camera."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame, cloud_frame_id, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF lookup {cloud_frame_id}->{self.camera_frame} failed: {e}')
            return None, None

        # read_points_numpy() requires every field in the message to share one
        # datatype, even fields we don't ask for - real LiDAR clouds commonly
        # carry an extra field (e.g. 'rgb', a different datatype than x/y/z)
        # that breaks that assumption. read_points() returns a structured
        # array instead and only cares about the fields actually requested.
        structured = point_cloud2.read_points(
            self.latest_cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        if structured.size == 0:
            return None, None
        pts = np.column_stack(
            [structured['x'], structured['y'], structured['z']]).astype(np.float64)

        t = tf.transform.translation
        q = tf.transform.rotation
        # quaternion -> rotation matrix
        R = np.array([
            [1 - 2 * (q.y**2 + q.z**2), 2 * (q.x*q.y - q.z*q.w), 2 * (q.x*q.z + q.y*q.w)],
            [2 * (q.x*q.y + q.z*q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y*q.z - q.x*q.w)],
            [2 * (q.x*q.z - q.y*q.w), 2 * (q.y*q.z + q.x*q.w), 1 - 2 * (q.x**2 + q.y**2)],
        ])
        pts_cam_body = (R @ pts.T).T + np.array([t.x, t.y, t.z])

        pts_opt = _body_to_optical(pts_cam_body)
        in_front = pts_opt[:, 2] > 0.1
        pts_opt = pts_opt[in_front]
        if pts_opt.shape[0] == 0:
            return None, None

        rvec = np.zeros(3)
        tvec = np.zeros(3)
        uv, _ = cv2.fisheye.projectPoints(
            pts_opt.reshape(-1, 1, 3), rvec, tvec, self.K, self.D)
        uv = uv.reshape(-1, 2)
        forward_dist = pts_opt[:, 2]

        valid = (uv[:, 0] >= 0) & (uv[:, 0] < self.image_width) & \
                (uv[:, 1] >= 0) & (uv[:, 1] < self.image_height)
        return uv[valid], forward_dist[valid]

    def _on_detections(self, msg):
        if self.latest_cloud is None:
            return
        if not msg.detections:
            self._publish_status(False, False, [])
            return

        uv, dist = self._project_cloud_to_pixels(self.latest_cloud.header.frame_id)
        if uv is None:
            self._publish_status(False, False, [])
            return

        fused = []  # (class_name, score, box, distance)
        for det in msg.detections:
            in_box = (uv[:, 0] >= det.x1) & (uv[:, 0] <= det.x2) & \
                     (uv[:, 1] >= det.y1) & (uv[:, 1] <= det.y2)
            box_dists = dist[in_box]
            if box_dists.shape[0] < self.min_points_per_box:
                continue
            d = float(np.percentile(box_dists, self.distance_percentile))
            fused.append((det.class_name, det.score, (det.x1, det.y1, det.x2, det.y2), d))

        red = any(d <= self.stop_distance for *_, d in fused)
        yellow = any(self.stop_distance < d <= self.warning_distance for *_, d in fused)
        self._publish_status(red, yellow, fused)

    def _publish_status(self, red, yellow, fused):
        self.red_pub.publish(Bool(data=red))
        self.yellow_pub.publish(Bool(data=yellow))
        self._publish_markers(fused)
        self._publish_debug_image(fused)
        if red:
            closest = min((d for *_, d in fused), default=None)
            self.get_logger().warn(f'camera_red_detected: closest fused object at {closest:.2f}m')

    def _publish_markers(self, fused):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, (name, score, box, d) in enumerate(fused):
            m = Marker()
            m.header.frame_id = self.camera_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'camera_fused'
            m.id = i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = d, 0.0, 0.0
            m.pose.orientation.w = 1.0
            m.scale.z = 0.4
            m.color.a = 1.0
            m.color.r = 1.0 if d <= self.stop_distance else (1.0 if d <= self.warning_distance else 0.0)
            m.color.g = 1.0 if d > self.stop_distance else 0.0
            m.text = f'{name} {d:.1f}m ({score:.2f})'
            markers.markers.append(m)
        self.marker_pub.publish(markers)

    def _publish_debug_image(self, fused):
        if not self.publish_debug_image or self.latest_frame is None:
            return
        frame = self.latest_frame.copy()
        for name, score, (x1, y1, x2, y2), d in fused:
            color = (0, 0, 255) if d <= self.stop_distance else \
                    (0, 255, 255) if d <= self.warning_distance else (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame, f'{name}:{d:.1f}m', (x1, max(0, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA,
            )
        try:
            out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge conversion failed (debug image): {e}')
            return
        self.debug_image_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarFusionNode()
    # MultiThreadedExecutor so the (cheap, frequent) camera/image_raw callback
    # isn't starved behind the heavier point-cloud-projection callback under
    # load - with a single-threaded executor the small QoS depth on the image
    # subscription meant frames could get evicted before ever being serviced,
    # leaving the debug image topic silently empty even though everything
    # else (the actual safety signal) worked fine.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
