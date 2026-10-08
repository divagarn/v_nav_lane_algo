#!/usr/bin/env python3
"""
Monocular depth estimation node (MiDaS via torch.hub).

Subscribes to a raw camera image topic and publishes:
  - a depth image (32FC1) on `depth_topic`
  - a colored 3D point cloud on `points_topic`, back-projected from the
    depth map using a pinhole model (fx, fy, cx, cy)

IMPORTANT - this is *relative* depth, not metric range: MiDaS predicts
scale-ambiguous inverse depth from a single image - there is no physical
baseline (stereo, LiDAR, known object size, ...) to anchor it to real
metres. `depth_scale`/`max_depth` below only rescale the network's raw
output into a range that looks sane in RViz; treat every distance this node
outputs as "relatively near/far", not ground truth. Do not feed this straight
into collision-avoidance or costmap logic that assumes metric range.

No camera_info topic exists for this camera yet (no calibration has been
run), so fx/fy/cx/cy default to a placeholder pinhole model derived from
the image size and `hfov_deg` (an assumed horizontal field of view) instead
of a real calibration. This only affects the point cloud's x/y spread, not
the depth (z) values. Run a real calibration (`ros2 run camera_calibration
cameracalibrator ...`) and pass the resulting fx/fy/cx/cy as parameters once
one exists.

Requires network access the first time it runs (torch.hub fetches the
intel-isl/MiDaS repo + model weights, then caches them under
~/.cache/torch/hub) - UNTESTED on the Orin itself; this was written/checked
by inspection against the documented torch.hub/MiDaS API on a dev machine
with no GPU. Watch the log line reporting which device (CUDA/CPU) it ended
up on when first run on real hardware.
"""
import math

import cv2
import numpy as np
import torch
import torch.nn.functional as F

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError

_PC_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
]


class MonoDepthNode(Node):
    def __init__(self):
        super().__init__('mono_depth_node')

        # -- Parameters --
        self.declare_parameter('input_topic', 'camera/image_raw')
        self.declare_parameter('depth_topic', 'camera/depth/image_raw')
        self.declare_parameter('points_topic', 'camera/depth/points')
        self.declare_parameter('model_type', 'MiDaS_small')
        self.declare_parameter('depth_scale', 5.0)
        self.declare_parameter('max_depth', 20.0)
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('point_stride', 4)
        self.declare_parameter('process_every_n', 1)
        # Pinhole intrinsics - <=0 means "derive from image size + hfov_deg"
        self.declare_parameter('fx', -1.0)
        self.declare_parameter('fy', -1.0)
        self.declare_parameter('cx', -1.0)
        self.declare_parameter('cy', -1.0)
        self.declare_parameter('hfov_deg', 90.0)

        input_topic = self.get_parameter('input_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        points_topic = self.get_parameter('points_topic').value
        model_type = self.get_parameter('model_type').value
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.stride = max(1, int(self.get_parameter('point_stride').value))
        self.every_n = max(1, int(self.get_parameter('process_every_n').value))
        self._fx_param = float(self.get_parameter('fx').value)
        self._fy_param = float(self.get_parameter('fy').value)
        self._cx_param = float(self.get_parameter('cx').value)
        self._cy_param = float(self.get_parameter('cy').value)
        self.hfov_deg = float(self.get_parameter('hfov_deg').value)

        self.bridge = CvBridge()
        self.frame_count = 0
        self._intrinsics = None  # (fx, fy, cx, cy) cached for the current (w, h)
        self._intrinsics_wh = None

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Loading MiDaS ({model_type}) on {self.device} ...')
        self.model = torch.hub.load('intel-isl/MiDaS', model_type, trust_repo=True)
        self.model.to(self.device).eval()
        midas_transforms = torch.hub.load('intel-isl/MiDaS', 'transforms', trust_repo=True)
        self.transform = (
            midas_transforms.small_transform if 'small' in model_type
            else midas_transforms.dpt_transform
        )
        self.get_logger().info(f'MiDaS loaded on {self.device}')

        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.depth_pub = self.create_publisher(Image, depth_topic, camera_qos)
        self.points_pub = self.create_publisher(PointCloud2, points_topic, camera_qos)
        self.create_subscription(Image, input_topic, self._on_image, camera_qos)

        self.get_logger().info(
            f'mono_depth_node initialized: {input_topic} -> {depth_topic}, {points_topic}')

    def _intrinsics_for(self, w, h):
        if self._intrinsics_wh == (w, h):
            return self._intrinsics
        if self._fx_param > 0 and self._fy_param > 0:
            fx, fy = self._fx_param, self._fy_param
        else:
            fx = fy = (w / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
            self.get_logger().warn(
                f'No fx/fy given - using a placeholder pinhole model from '
                f'hfov_deg={self.hfov_deg} ({w}x{h} -> fx=fy={fx:.1f}). Point cloud x/y '
                f'will be wrong until this camera is actually calibrated.')
        cx = self._cx_param if self._cx_param > 0 else w / 2.0
        cy = self._cy_param if self._cy_param > 0 else h / 2.0
        self._intrinsics = (fx, fy, cx, cy)
        self._intrinsics_wh = (w, h)
        return self._intrinsics

    def _on_image(self, msg):
        self.frame_count += 1
        if self.frame_count % self.every_n != 0:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        inp = self.transform(rgb).to(self.device)
        with torch.no_grad():
            pred = self.model(inp)
            pred = F.interpolate(
                pred.unsqueeze(1), size=(h, w), mode='bicubic', align_corners=False,
            ).squeeze()
        inv_depth = pred.clamp(min=1e-6)
        depth = (self.depth_scale / inv_depth).clamp(0.0, self.max_depth)
        depth_np = depth.cpu().numpy().astype(np.float32)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding='32FC1')
        depth_msg.header = msg.header
        self.depth_pub.publish(depth_msg)

        self._publish_points(msg.header, rgb, depth_np, w, h)

    def _publish_points(self, header, rgb, depth, w, h):
        fx, fy, cx, cy = self._intrinsics_for(w, h)
        s = self.stride
        us = np.arange(0, w, s)
        vs = np.arange(0, h, s)
        uu, vv = np.meshgrid(us, vs)
        d = depth[vv, uu]
        valid = (d > self.min_depth) & (d < self.max_depth)
        if not valid.any():
            return

        x = (uu.astype(np.float32) - cx) * d / fx
        y = (vv.astype(np.float32) - cy) * d / fy
        z = d

        r = rgb[vv, uu, 0].astype(np.uint32)
        g = rgb[vv, uu, 1].astype(np.uint32)
        b = rgb[vv, uu, 2].astype(np.uint32)
        rgb_packed = (r << 16) | (g << 8) | b

        pts = np.stack([x[valid], y[valid], z[valid]], axis=1).astype(np.float32)
        colors = np.ascontiguousarray(rgb_packed[valid])

        n = len(pts)
        buf = np.empty((n, 4), dtype=np.float32)
        buf[:, :3] = pts
        buf[:, 3] = colors.view(np.float32)

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = n
        cloud.fields = _PC_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 16 * n
        cloud.data = buf.tobytes()
        cloud.is_dense = False
        self.points_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = MonoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
