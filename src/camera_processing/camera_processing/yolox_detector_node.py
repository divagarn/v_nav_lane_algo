#!/usr/bin/env python3
"""
YOLOX object detection node (ONNX Runtime, PyTorch .pth, or TensorRT backend).

Subscribes to a raw camera image topic, runs inference with a YOLOX model
(18-class IDD road-object detector), draws the resulting bounding boxes on
each frame, and republishes the annotated frames on a new topic.

`model_path`'s extension selects the backend:
  - '.pth'          -> raw YOLOX-s state_dict via PyTorch (needs
                        `yolox_repo_path` pointing at the edgeai-yolox-
                        simplified checkout, since a bare state_dict carries
                        no architecture info on its own)
  - '.trt'/'.engine' -> a TensorRT engine, e.g. built via:
                        trtexec --onnx=model.onnx --saveEngine=model.trt --fp16
                        See trt_utils.py - this path is UNTESTED here (no
                        GPU/TensorRT on the dev machine this was written on),
                        needs validating on the Orin.
  - anything else    -> ONNX Runtime

All three backends emit the same output tensor shape (1, 8400,
5+num_classes) for a given model - trtexec compiles the same graph, it
doesn't change what the graph computes - so decode_grid_stride below
applies identically regardless of backend; it depends on which *model* you
converted, not which backend loads it:
  - yolox_s_ti_18class_ti_today_qat.onnx: decode IS baked in (verified
    empirically - re-applying grid/stride decode on top produces
    overflowing coordinates). Default: decode_grid_stride=false.
  - yolox_s_ti_18class_twillight_clean_tda4vh.onnx: decode is NOT baked in
    (verified empirically - raw cx/cy/w/h are tiny grid-relative/log-space
    values; its own reference script explicitly calls demo_postprocess()).
    Set decode_grid_stride:=true for this one.
Set the `decode_grid_stride` parameter to match whichever model you point
`model_path` at (an engine built from either .onnx keeps that .onnx's
convention).
"""
import os

import cv2
import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ground_segmentation_msgs.msg import Detection2D, Detection2DArray

CLASS_NAMES = [
    "animal", "autorickshaw", "barricade", "bicycle", "bicyclist", "bus", "car",
    "debris", "motorcycle", "motorcyclist", "partial pedestrian", "pedestrian",
    "pothole", "road damage", "traffic light", "traffic sign", "truck",
    "vehicle fallback",
]


def _letterbox(img_bgr, size):
    """Resize+pad to `size` (h, w) keeping aspect ratio, pad value 114, CHW float32 blob."""
    h, w = size
    padded = np.full((h, w, 3), 114, dtype=np.uint8)
    r = min(h / img_bgr.shape[0], w / img_bgr.shape[1])
    nh, nw = int(round(img_bgr.shape[0] * r)), int(round(img_bgr.shape[1] * r))
    resized = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    padded[:nh, :nw] = resized
    blob = padded.transpose(2, 0, 1).astype(np.float32)[None, ...]
    return blob, r


def _decode_grid_stride(outputs, input_size, strides=(8, 16, 32)):
    """Grid/stride decode for exports that don't bake it into the graph.

    outputs: (N, 5+num_classes) raw predictions, cx/cy grid-relative,
    w/h in log-space. Mutates and returns outputs with cx/cy/w/h now in
    absolute pixel coordinates (of the letterboxed input_size image).
    """
    h, w = input_size
    grids, expanded_strides = [], []
    for stride in strides:
        hsize, wsize = h // stride, w // stride
        xv, yv = np.meshgrid(np.arange(wsize), np.arange(hsize))
        grid = np.stack((xv, yv), 2).reshape(-1, 2)
        grids.append(grid)
        expanded_strides.append(np.full((grid.shape[0], 1), stride))
    grids = np.concatenate(grids, axis=0)
    expanded_strides = np.concatenate(expanded_strides, axis=0)
    outputs[:, :2] = (outputs[:, :2] + grids) * expanded_strides
    outputs[:, 2:4] = np.exp(outputs[:, 2:4]) * expanded_strides
    return outputs


def _nms(boxes, scores, nms_thr):
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= nms_thr)[0]
        order = order[inds + 1]
    return keep


def _multiclass_nms(boxes, scores, nms_thr, score_thr):
    """Class-agnostic multiclass NMS: keep each box's best-scoring class only."""
    cls_inds = scores.argmax(1)
    cls_scores = scores[np.arange(len(cls_inds)), cls_inds]
    mask = cls_scores > score_thr
    if not mask.any():
        return None
    valid_scores = cls_scores[mask]
    valid_boxes = boxes[mask]
    valid_cls_inds = cls_inds[mask]
    keep = _nms(valid_boxes, valid_scores, nms_thr)
    if not keep:
        return None
    return np.concatenate(
        [valid_boxes[keep], valid_scores[keep, None], valid_cls_inds[keep, None]],
        axis=1,
    )


class YoloxDetectorNode(Node):
    def __init__(self):
        super().__init__('yolox_detector_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('input_topic', 'camera/image_raw')
        self.declare_parameter('output_topic', 'camera/detections/image_raw')
        self.declare_parameter('detection_array_topic', 'camera/detections/array')
        self.declare_parameter(
            'model_path',
            '/home/divagar/edgeai-yolox-simplified/build_edgeai_custom/'
            'yolox_s_ti_18class_ti_today_qat.onnx',
        )
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('conf_threshold', 0.2)
        self.declare_parameter('nms_threshold', 0.65)
        self.declare_parameter('num_classes', 18)
        self.declare_parameter('decode_grid_stride', False)
        self.declare_parameter(
            'yolox_repo_path',
            '/home/divagar/edgeai-yolox-simplified/git_dir/edgeai-yolox',
        )

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        detection_array_topic = self.get_parameter('detection_array_topic').value
        model_path = self.get_parameter('model_path').value
        self.input_size = (
            int(self.get_parameter('input_height').value),
            int(self.get_parameter('input_width').value),
        )
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.nms_threshold = float(self.get_parameter('nms_threshold').value)
        num_classes = int(self.get_parameter('num_classes').value)
        self.decode_grid_stride = bool(self.get_parameter('decode_grid_stride').value)
        yolox_repo_path = self.get_parameter('yolox_repo_path').value

        self.bridge = CvBridge()

        # ── Backend selection ────────────────────────────────────────
        ext = os.path.splitext(model_path)[1]
        if ext == '.pth':
            self.backend = 'pth'
        elif ext in ('.trt', '.engine'):
            self.backend = 'trt'
        else:
            self.backend = 'onnx'

        if self.backend == 'onnx':
            available = ort.get_available_providers()
            providers = [p for p in ('CUDAExecutionProvider', 'CPUExecutionProvider') if p in available]
            self.get_logger().info(f'Loading ONNX model: {model_path}')
            self.session = ort.InferenceSession(model_path, providers=providers)
            self.input_name = self.session.get_inputs()[0].name
            self.get_logger().info(f'ONNX execution providers in use: {self.session.get_providers()}')
        elif self.backend == 'trt':
            from camera_processing.trt_utils import TrtEngine
            self.get_logger().info(f'Loading TensorRT engine: {model_path}')
            self.trt_engine = TrtEngine(model_path, logger=self.get_logger())
        else:
            self._load_pth_model(model_path, yolox_repo_path, num_classes)

        # ── ROS interfaces ────────────────────────────────────────────
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Image, output_topic, camera_qos)
        self.det_array_pub = self.create_publisher(Detection2DArray, detection_array_topic, camera_qos)
        self.subscription = self.create_subscription(
            Image, input_topic, self._on_image, camera_qos)

        self.get_logger().info(
            f'YOLOX detector node initialized ({self.backend} backend): '
            f'{input_topic} -> {output_topic}')

    def _load_pth_model(self, model_path, yolox_repo_path, num_classes):
        import sys
        if yolox_repo_path not in sys.path:
            sys.path.insert(0, yolox_repo_path)
        import torch
        from yolox.models import YOLOX, YOLOPAFPN, YOLOXHead

        self.torch = torch
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.get_logger().info(f'Loading PyTorch checkpoint: {model_path}')
        ckpt = torch.load(model_path, map_location='cpu')
        state_dict = ckpt['model'] if isinstance(ckpt, dict) and 'model' in ckpt else ckpt

        in_channels = [256, 512, 1024]
        backbone = YOLOPAFPN(
            0.33, 0.50, in_channels=in_channels, act='relu',
            conv_focus=True, split_max_pool_kernel=True,
        )
        head = YOLOXHead(num_classes, 0.50, in_channels=in_channels, act='relu')
        model = YOLOX(backbone, head)
        model.load_state_dict(state_dict, strict=True)
        model.eval()
        self.pth_model = model.to(self.device)
        self.get_logger().info(f'PyTorch model loaded on device: {self.device}')

    def _infer(self, blob):
        """Run the loaded backend on a (1,3,H,W) float32 blob, return (N, 5+num_classes)."""
        if self.backend == 'onnx':
            return self.session.run(None, {self.input_name: blob})[0][0]
        if self.backend == 'trt':
            outputs = self.trt_engine.infer(blob)
            # single-output model - grab whatever the engine calls its one output.
            out = next(iter(outputs.values()))
            return out.reshape(-1, out.shape[-1])
        tensor = self.torch.from_numpy(blob).to(self.device)
        with self.torch.no_grad():
            out = self.pth_model(tensor)
        return out[0].cpu().numpy()

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge conversion failed (input): {e}')
            return

        blob, ratio = _letterbox(frame, self.input_size)
        outputs = self._infer(blob)  # (N, 5+num_classes)
        if self.decode_grid_stride:
            outputs = _decode_grid_stride(outputs, self.input_size)

        boxes = outputs[:, :4]
        objectness = outputs[:, 4:5]
        cls_scores = outputs[:, 5:]
        scores = objectness * cls_scores

        boxes_xyxy = np.empty_like(boxes)
        boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.0
        boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.0
        boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2.0
        boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2.0
        boxes_xyxy /= ratio

        dets = _multiclass_nms(boxes_xyxy, scores, self.nms_threshold, self.conf_threshold)

        det_array = Detection2DArray()
        det_array.header = msg.header
        if dets is not None:
            h, w = frame.shape[:2]
            for x1, y1, x2, y2, score, cls_id in dets:
                x1 = int(np.clip(x1, 0, w - 1))
                y1 = int(np.clip(y1, 0, h - 1))
                x2 = int(np.clip(x2, 0, w - 1))
                y2 = int(np.clip(y2, 0, h - 1))
                cls_id = int(cls_id)
                label = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else str(cls_id)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame, f'{label}:{score:.2f}', (x1, max(0, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
                )
                det_array.detections.append(Detection2D(
                    class_name=label, score=float(score), x1=x1, y1=y1, x2=x2, y2=y2,
                ))
        self.det_array_pub.publish(det_array)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge conversion failed (output): {e}')
            return
        out_msg.header = msg.header
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloxDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
