#!/usr/bin/env python3
"""
Unified detection + segmentation node (CenterNet-style multi-task ONNX model).

Subscribes to a raw camera image topic, runs a single ONNX model that
produces object detections (lmv/person/two_wheeler) AND a 3-class semantic
segmentation map (background/lane/road) in one forward pass, and publishes
three separate annotated topics: detections, full segmentation, and a
lane-only view.

Ported from unified_pipeline_onnx_package_20260902/inference_onnx.py +
utils/utils.py + utils/augmentations.py (RescaleTo + Normalize) + configs/config.py,
kept self-contained (pure NumPy/OpenCV, no torch/torchvision) so this node
only needs the .onnx weights file at runtime, not the reference repo.

Preprocessing (matches RescaleTo + Normalize exactly):
  - plain stretch-resize (NOT letterboxed - the reference training size is
    512x768, not square) BGR(cv_bridge) -> RGB (the reference uses imageio,
    which reads RGB, not cv2's BGR)
  - normalize: (pixel - 127) / 127

Detection postprocessing (matches utils.postprocess exactly, ported to NumPy):
  - heatmap channel 0 is treated as background/unused (the reference's
    `class_names_list[clses - 1]` indexing implies classes 1..3 map to
    lmv/person/two_wheeler; channel 0 has no valid name and is dropped here
    rather than reproducing what looks like an unintentional Python
    negative-index wraparound onto the last class name)
  - 3x3 local-max peak suppression on the heatmap, top-K global peaks,
    gather width/height + sub-pixel offset at each peak, per-class NMS
  - boxes are computed directly in the model's stride-4 feature space
    (128x192) and scaled straight to the source frame's native resolution
    (independent x/y scale factors, since preprocessing has no letterbox
    padding to account for)

Segmentation output is already an argmaxed per-pixel class-id map from the
graph itself (no softmax/argmax needed here), resized to native resolution
with nearest-neighbor to keep discrete class ids.

Backend: `model_path`'s extension selects it - '.trt'/'.engine' loads a
TensorRT engine (e.g. built via `trtexec --onnx=up_20260903.onnx
--saveEngine=up.trt --fp16`, see trt_utils.py), anything else uses ONNX
Runtime. The TRT path is UNTESTED here (no GPU/TensorRT on the dev machine
this was written on) - validate on the Orin. One likely-harmless quirk to
expect: the segmentation output is int64 in the ONNX graph, but TensorRT
doesn't support int64 natively and silently downcasts it to int32 while
parsing (usually with a warning during the trtexec build) - the code below
just calls `.astype(np.uint8)` on it either way, so this shouldn't actually
break anything, just don't be alarmed by that warning if you see it.
"""
import os

import cv2
import numpy as np
import onnxruntime as ort
from scipy.ndimage import maximum_filter

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

TRAIN_H, TRAIN_W = 512, 768  # configs/config.py get_training_size()

CLASS_NAMES = ['lmv', 'person', 'two_wheeler']  # heatmap channels 1,2,3 (channel 0 unused)

# id_to_color in configs/config.py is RGB (fed straight into PIL there);
# converted to BGR here since this node draws with OpenCV.
SEG_COLOR_BGR = {
    0: (0, 0, 0),      # Background
    1: (0, 0, 255),    # Lane   (RGB 255,0,0 -> BGR)
    2: (255, 0, 0),    # Road   (RGB 0,0,255 -> BGR)
}
LANE_CLASS_ID = 1


def _nms(boxes, scores, iou_thr):
    """Standard single-class NMS, no +1 discretization (matches torchvision.ops.nms)."""
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        inds = np.where(ovr <= iou_thr)[0]
        order = order[inds + 1]
    return keep


def _class_based_nms(boxes, scores, clses, iou_thr, score_thr):
    kept_boxes, kept_scores, kept_clses = [], [], []
    for c in np.unique(clses):
        m = clses == c
        b, s = boxes[m], scores[m]
        keep = _nms(b, s, iou_thr)
        kept_boxes.append(b[keep])
        kept_scores.append(s[keep])
        kept_clses.append(np.full(len(keep), c))
    if not kept_boxes:
        return None
    boxes = np.concatenate(kept_boxes, axis=0)
    scores = np.concatenate(kept_scores, axis=0)
    clses = np.concatenate(kept_clses, axis=0)
    mask = scores > score_thr
    return boxes[mask], scores[mask], clses[mask]


def _decode_detections(hm, wh, offset, k, score_floor, nms_thr, score_thr):
    """hm:(1,C,Ho,Wo) wh:(1,2,Ho,Wo) offset:(1,2,Ho,Wo) -> boxes in feature-map units."""
    _, c, ho, wo = hm.shape

    # 3x3 local-max peak suppression per channel
    hm_max = maximum_filter(hm, size=(1, 1, 3, 3), mode='constant', cval=-np.inf)
    hm_peaks = np.where(hm_max == hm, hm, 0.0)

    flat = hm_peaks.reshape(-1)  # (C*Ho*Wo,)
    k = min(k, flat.size)
    top_idx = np.argpartition(-flat, k - 1)[:k]
    top_idx = top_idx[np.argsort(-flat[top_idx])]

    scores = flat[top_idx]
    clses = top_idx // (ho * wo)
    rem = top_idx % (ho * wo)
    ys = (rem // wo).astype(np.float32)
    xs = (rem % wo).astype(np.float32)

    valid = scores > score_floor
    scores, clses, ys, xs = scores[valid], clses[valid], ys[valid], xs[valid]
    # channel 0 has no corresponding class name - treat as background, drop it
    keep_named = clses > 0
    scores, clses, ys, xs = scores[keep_named], clses[keep_named], ys[keep_named], xs[keep_named]
    if scores.size == 0:
        return None

    yi = ys.astype(np.int64)
    xi = xs.astype(np.int64)
    dx = offset[0, 0, yi, xi]
    dy = offset[0, 1, yi, xi]
    w = wh[0, 0, yi, xi]
    h = wh[0, 1, yi, xi]

    cx = xs + dx
    cy = ys + dy
    boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1)

    result = _class_based_nms(boxes, scores, clses, nms_thr, score_thr)
    if result is None:
        return None
    boxes, scores, clses = result
    return boxes, scores, clses, (ho, wo)


class UnifiedPerceptionNode(Node):
    def __init__(self):
        super().__init__('unified_perception_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('input_topic', 'camera/image_raw')
        self.declare_parameter('detection_topic', 'camera/detections/image_raw')
        self.declare_parameter('detection_array_topic', 'camera/detections/array')
        self.declare_parameter('segmentation_topic', 'camera/segmentation/image_raw')
        self.declare_parameter('lane_topic', 'camera/lane/image_raw')
        self.declare_parameter(
            'model_path',
            '/home/divagar/Downloads/up_20260907.onnx',
            #'/home/divagar/Downloads/up_20260903.onnx',
            #'/home/divagar/Downloads/unified_pipeline_onnx_package_20260902/weights/up.onnx',
            
        )
        self.declare_parameter('conf_threshold', 0.3)
        self.declare_parameter('nms_threshold', 0.3)
        self.declare_parameter('topk', 100)
        self.declare_parameter('seg_alpha', 0.4)
        self.declare_parameter('enable_detection', True)

        input_topic = self.get_parameter('input_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        detection_array_topic = self.get_parameter('detection_array_topic').value
        segmentation_topic = self.get_parameter('segmentation_topic').value
        lane_topic = self.get_parameter('lane_topic').value
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.nms_threshold = float(self.get_parameter('nms_threshold').value)
        self.topk = int(self.get_parameter('topk').value)
        self.seg_alpha = float(self.get_parameter('seg_alpha').value)
        self.enable_detection = bool(self.get_parameter('enable_detection').value)

        self.bridge = CvBridge()

        ext = os.path.splitext(model_path)[1]
        self.backend = 'trt' if ext in ('.trt', '.engine') else 'onnx'

        if self.backend == 'onnx':
            available = ort.get_available_providers()
            providers = [p for p in ('CUDAExecutionProvider', 'CPUExecutionProvider') if p in available]
            self.get_logger().info(f'Loading unified perception ONNX model: {model_path}')
            self.session = ort.InferenceSession(model_path, providers=providers)
            self.input_name = self.session.get_inputs()[0].name
            self.get_logger().info(f'ONNX execution providers in use: {self.session.get_providers()}')
        else:
            from camera_processing.trt_utils import TrtEngine
            self.get_logger().info(f'Loading TensorRT engine: {model_path}')
            self.trt_engine = TrtEngine(model_path, logger=self.get_logger())

        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.det_pub = self.create_publisher(Image, detection_topic, camera_qos) \
            if self.enable_detection else None
        self.det_array_pub = self.create_publisher(Detection2DArray, detection_array_topic, camera_qos) \
            if self.enable_detection else None
        self.seg_pub = self.create_publisher(Image, segmentation_topic, camera_qos)
        self.lane_pub = self.create_publisher(Image, lane_topic, camera_qos)
        self.subscription = self.create_subscription(
            Image, input_topic, self._on_image, camera_qos)

        outputs = [segmentation_topic, lane_topic]
        detection_note = 'disabled (expecting another node to publish it)'
        if self.enable_detection:
            outputs.insert(0, detection_topic)
            detection_note = 'enabled'
        self.get_logger().info(
            f'Unified perception node initialized: {input_topic} -> {outputs} '
            f'(detection: {detection_note})')

    def _preprocess(self, frame_bgr):
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).astype(np.float32)
        resized = cv2.resize(rgb, (TRAIN_W, TRAIN_H), interpolation=cv2.INTER_LINEAR)
        normalized = (resized - 127.0) / 127.0
        blob = normalized.transpose(2, 0, 1)[None, ...].astype(np.float32)
        return blob

    def _infer(self, blob):
        """Returns (hm, wh, offset, seg) in the same shapes as the ONNX graph's own
        4 declared outputs, regardless of backend."""
        if self.backend == 'onnx':
            return self.session.run(None, {self.input_name: blob})
        outputs = self.trt_engine.infer(blob)
        # ONNX graph's own output tensor names (from `onnxruntime` introspection of
        # up_20260903.onnx) - trtexec normally preserves these as binding names.
        # index_fallback assumes trtexec kept the graph's declared output order if
        # the names don't match for some reason - UNVERIFIED, check against your
        # engine's actual output order if results look wrong.
        hm = self.trt_engine.get_output(outputs, '651', index_fallback=0)
        wh = self.trt_engine.get_output(outputs, '657', index_fallback=1)
        offset = self.trt_engine.get_output(outputs, '663', index_fallback=2)
        seg = self.trt_engine.get_output(outputs, '746', index_fallback=3)
        return hm, wh, offset, seg

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge conversion failed (input): {e}')
            return

        h, w = frame.shape[:2]
        blob = self._preprocess(frame)
        hm, wh, offset, seg = self._infer(blob)

        # ── Detections (skipped entirely when another node owns detection_topic) ─
        if self.enable_detection:
            det_frame = frame.copy()
            det_array = Detection2DArray()
            det_array.header = msg.header
            decoded = _decode_detections(
                hm, wh, offset, self.topk, score_floor=0.01,
                nms_thr=self.nms_threshold, score_thr=self.conf_threshold,
            )
            if decoded is not None:
                boxes, scores, clses, (fh, fw) = decoded
                sx, sy = w / fw, h / fh
                for (x1, y1, x2, y2), score, cls_id in zip(boxes, scores, clses):
                    x1 = int(np.clip(x1 * sx, 0, w - 1))
                    y1 = int(np.clip(y1 * sy, 0, h - 1))
                    x2 = int(np.clip(x2 * sx, 0, w - 1))
                    y2 = int(np.clip(y2 * sy, 0, h - 1))
                    name = CLASS_NAMES[int(cls_id) - 1]
                    cv2.rectangle(det_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        det_frame, f'{name}:{score:.2f}', (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
                    )
                    det_array.detections.append(Detection2D(
                        class_name=name, score=float(score), x1=x1, y1=y1, x2=x2, y2=y2,
                    ))
            self._publish(self.det_pub, det_frame, msg.header)
            self.det_array_pub.publish(det_array)

        # ── Segmentation (native resolution, nearest-neighbor to keep class ids) ─
        seg_map = seg[0, 0].astype(np.uint8)  # (TRAIN_H, TRAIN_W), values 0/1/2
        seg_native = cv2.resize(seg_map, (w, h), interpolation=cv2.INTER_NEAREST)

        seg_color = np.zeros((h, w, 3), dtype=np.uint8)
        for cid, color in SEG_COLOR_BGR.items():
            seg_color[seg_native == cid] = color
        seg_overlay = cv2.addWeighted(frame, 1.0 - self.seg_alpha, seg_color, self.seg_alpha, 0)
        self._publish(self.seg_pub, seg_overlay, msg.header)

        # ── Lane only (highlight lane pixels, leave everything else untouched) ─
        lane_overlay = frame.copy()
        lane_mask = seg_native == LANE_CLASS_ID
        lane_color = np.array(SEG_COLOR_BGR[LANE_CLASS_ID], dtype=np.float32)
        lane_overlay[lane_mask] = (
            frame[lane_mask].astype(np.float32) * (1.0 - self.seg_alpha)
            + lane_color * self.seg_alpha
        ).astype(np.uint8)
        self._publish(self.lane_pub, lane_overlay, msg.header)

    def _publish(self, publisher, frame, header):
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge conversion failed (output): {e}')
            return
        out_msg.header = header
        publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
