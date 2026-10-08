#!/usr/bin/env python3
import queue as _Queue
import threading
import time
 
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
 
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image
 
 
class CameraH265DecoderNode(Node):
    def __init__(self):
        super().__init__('camera_h265_decoder_node')
 
        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('input_topic',             'camera/compressed')
        self.declare_parameter('output_topic',            'camera/image_raw')
        self.declare_parameter('allow_software_fallback', False)
        self.declare_parameter('output_width',            960)
        self.declare_parameter('output_height',           540)
        self.declare_parameter('diagnostic_mode',         False)   # set True to enable timing logs
 
        self.input_topic            = self.get_parameter('input_topic').value
        self.output_topic           = self.get_parameter('output_topic').value
        self.allow_software_fallback = self.get_parameter('allow_software_fallback').value
        self.output_width           = int(self.get_parameter('output_width').value)
        self.output_height          = int(self.get_parameter('output_height').value)
        self.diagnostic_mode        = self.get_parameter('diagnostic_mode').value
 
        # ── QoS ───────────────────────────────────────────────────────────────
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
 
        # ── ROS interfaces ────────────────────────────────────────────────────
        self.publisher   = self.create_publisher(Image, self.output_topic, camera_qos)
        self.subscription = self.create_subscription(
            CompressedImage,
            self.input_topic,
            self._on_compressed_image,
            camera_qos,
        )
 
        # ── State ─────────────────────────────────────────────────────────────
        Gst.init(None)
        self.pipeline  = None
        self.appsrc    = None
        self.appsink   = None
        self.running   = True
 
        self._header_lock        = threading.Lock()
        self._last_header        = None
        self._last_bad_fmt_warn  = 0.0
 
        # Publish queue — decouples GStreamer callback thread from ROS publisher
        self._pub_queue  = _Queue.SimpleQueue()
        self._pub_thread = threading.Thread(
            target=self._publish_loop, name='ros_publish', daemon=True
        )
        self._pub_thread.start()
 
        # ── Pipeline ──────────────────────────────────────────────────────────
        self._create_pipeline()
 
        self.get_logger().info('🎞️  H265 Decoder Node started')
        self.get_logger().info(f'📥 Subscribed to:        /{self.input_topic}')
        self.get_logger().info(f'📤 Publishing raw to:    /{self.output_topic}')
        self.get_logger().info(f'🖼️  Output size:          {self.output_width}x{self.output_height}')
        self.get_logger().info(f'🔬 Diagnostic mode:      {"ON" if self.diagnostic_mode else "OFF"}')
 
    # ── Pipeline creation ─────────────────────────────────────────────────────
 
    def _create_pipeline(self):
        """
        Build the GStreamer decode pipeline.
 
        Pipeline design decisions
        ─────────────────────────
        • appsrc block=true          — producer is throttled to decoder speed;
                                       eliminates the burst-drop-burst stutter.
        • format=bytes               — avoids GStreamer clock conflicts; we are
                                       delivering raw byte-stream, not timed packets.
        • queue max-size-buffers=1   — with block=true one slot is enough to
                                       decouple the ROS thread from the GStreamer
                                       streaming thread without adding latency.
        • nvv4l2decoder disable-dpb=1 — skip DPB reordering; live H.265 streams
                                        do not need B-frame reordering and it only
                                        adds multi-frame latency.
        • Single nvvidconv → RGBA     — one colour-space conversion instead of two;
                                        RGBA is directly consumable by RViz/OpenCV.
        • appsink max-buffers=1 drop=true — keep only the freshest decoded frame.
        """
 
        hw_pipeline = (
            # ── Source ──────────────────────────────────────────────────────
            "appsrc name=src is-live=true format=bytes block=true "
            # ONE slot: blocks the ROS callback when the decoder is busy
            "! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
            # ── Parse ───────────────────────────────────────────────────────
            "! h265parse "
            # ── HW decode (Orin VIC) ────────────────────────────────────────
            # disable-dpb=1  → no display-picture-buffer reorder (live stream)
            # enable-max-performance=1 → max hw utilization for throughput
            "! nvv4l2decoder enable-max-performance=1 disable-dpb=1 "
            # ── Scale on VIC (stays in NVMM, zero CPU copy) ─────────────────
            "! nvvidconv flip-method=0 "
            f"! video/x-raw(memory:NVMM),width={self.output_width},height={self.output_height},format=NV12 "
            # ── Single colour-space conversion to system memory ─────────────
            # RGBA avoids the extra BGRx→BGR strip that was happening before
            "! nvvidconv "
            "! video/x-raw,format=RGBA "
            # ── Output queue ────────────────────────────────────────────────
            "! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
            # ── Sink ────────────────────────────────────────────────────────
            # max-buffers=1 drop=true → always hand out the newest frame
            "! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
        )
 
        sw_pipeline = (
            "appsrc name=src is-live=true format=bytes block=true "
            "! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
            "! h265parse "
            "! avdec_h265 "
            "! videoconvert "
            "! video/x-raw,format=RGBA "
            "! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
        )
 
        pipelines = [hw_pipeline]
        if self.allow_software_fallback:
            pipelines.append(sw_pipeline)
 
        last_error = None
        for idx, pipeline_str in enumerate(pipelines):
            try:
                self.pipeline = Gst.parse_launch(pipeline_str)
                self.appsrc   = self.pipeline.get_by_name('src')
                self.appsink  = self.pipeline.get_by_name('sink')
 
                if self.appsrc is None or self.appsink is None:
                    raise RuntimeError('Missing appsrc/appsink in pipeline')
 
                # Caps: byte-stream H.265 with AU alignment
                self.appsrc.set_property(
                    'caps',
                    Gst.Caps.from_string(
                        'video/x-h265,stream-format=byte-stream,alignment=au'
                    ),
                )
 
                self.appsink.connect('new-sample', self._on_new_sample)
 
                ret = self.pipeline.set_state(Gst.State.PLAYING)
                if ret == Gst.StateChangeReturn.FAILURE:
                    raise RuntimeError('Failed to set pipeline to PLAYING')
 
                label = 'Jetson HW decoder + VIC (Orin)' if idx == 0 else 'software decoder (avdec_h265)'
                self.get_logger().info(
                    f'✅ Using {label} | {self.output_width}x{self.output_height} → RGBA'
                )
                self.get_logger().info(
                    '   QoS: RELIABLE depth=2 | appsrc block=true | disable-dpb=1'
                )
                return
 
            except Exception as exc:
                last_error = exc
                if self.pipeline:
                    self.pipeline.set_state(Gst.State.NULL)
                self.pipeline = None
                self.appsrc   = None
                self.appsink  = None
                self.get_logger().warn(f'Pipeline {idx} failed: {exc}')
 
        if self.allow_software_fallback:
            raise RuntimeError(f'All pipelines failed. Last error: {last_error}')
        raise RuntimeError(
            f'Hardware pipeline failed. '
            f'Pass allow_software_fallback:=true to permit avdec_h265. '
            f'Last error: {last_error}'
        )
 
    # ── ROS → GStreamer ───────────────────────────────────────────────────────
 
    def _on_compressed_image(self, msg: CompressedImage):
        """
        Called on the ROS executor thread.
 
        With appsrc block=true this call will block when the decoder is
        busy — that is intentional; it back-pressures the subscription so
        the node never builds up a backlog of unprocessed frames.
        """
        if not self.running or self.appsrc is None:
            return
 
        # ── Format guard ──────────────────────────────────────────────────────
        msg_format = (msg.format or '').lower()
        if 'h265' not in msg_format and 'hevc' not in msg_format:
            now = time.time()
            if now - self._last_bad_fmt_warn > 5.0:
                self.get_logger().warn(
                    f"Expected H265/HEVC compressed frames, got '{msg.format}'"
                )
                self._last_bad_fmt_warn = now
            return
 
        frame = bytes(msg.data)
        if not frame:
            return
 
        # ── Build GStreamer buffer ────────────────────────────────────────────
        gst_buf = Gst.Buffer.new_allocate(None, len(frame), None)
        gst_buf.fill(0, frame)
 
        # Set PTS only — let GStreamer derive DTS.
        # Do NOT set both PTS and DTS to the same value; it confuses the DPB.
        stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        if stamp_ns > 0:
            gst_buf.pts = stamp_ns
 
        # Cache header for use in _on_new_sample
        with self._header_lock:
            self._last_header = msg.header
 
        # ── Push (blocks when decoder is busy — desired behaviour) ────────────
        t0 = time.monotonic() if self.diagnostic_mode else None
 
        flow_ret = self.appsrc.emit('push-buffer', gst_buf)
 
        if self.diagnostic_mode and t0 is not None:
            elapsed_ms = (time.monotonic() - t0) * 1000
            if elapsed_ms > 20.0:
                self.get_logger().warn(
                    f'push-buffer blocked for {elapsed_ms:.1f} ms '
                    f'— decoder back-pressure active (normal at startup)'
                )
 
        if flow_ret != Gst.FlowReturn.OK:
            self.get_logger().warn(f'push-buffer returned {flow_ret}')
 
    # ── GStreamer → publish queue ──────────────────────────────────────────────
 
    def _on_new_sample(self, sink) -> Gst.FlowReturn:
        """
        Called on the GStreamer streaming thread.
 
        Keep this method as fast as possible:
          1. pull-sample
          2. map buffer  → copy bytes  → unmap immediately
          3. post Image message to _pub_queue (non-blocking)
          4. return OK
 
        The actual ROS publish happens on _publish_loop (a separate thread)
        so the GStreamer thread is never blocked by the ROS middleware.
        """
        if not self.running:
            return Gst.FlowReturn.OK
 
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
 
        caps = sample.get_caps()
        if caps is None or caps.get_size() == 0:
            return Gst.FlowReturn.OK
 
        structure = caps.get_structure(0)
        width  = int(structure.get_value('width'))
        height = int(structure.get_value('height'))
 
        buf = sample.get_buffer()
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
 
        try:
            # Copy frame bytes — unavoidable, but done here while the
            # buffer is still mapped; unmap immediately after.
            raw = bytes(mapinfo.data)
        finally:
            buf.unmap(mapinfo)   # release ASAP — never hold across publish
 
        # Build the ROS message (lightweight, no heavy ops)
        msg = Image()
        with self._header_lock:
            if self._last_header is not None:
                msg.header = self._last_header
            else:
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera'
 
        msg.height       = height
        msg.width        = width
        msg.encoding     = 'rgba8'   # matches RGBA caps negotiated above
        msg.is_bigendian = 0
        msg.step         = width * 4
        msg.data         = raw
 
        # Non-blocking hand-off to publish thread
        self._pub_queue.put_nowait(msg)
 
        return Gst.FlowReturn.OK
 
    # ── Dedicated publish thread ──────────────────────────────────────────────
 
    def _publish_loop(self):
        """
        Runs on a dedicated daemon thread.
        Drains _pub_queue and calls self.publisher.publish().
        Isolates ROS middleware latency from the GStreamer pipeline.
        """
        while self.running:
            try:
                msg = self._pub_queue.get(timeout=0.05)
                self.publisher.publish(msg)
            except _Queue.Empty:
                continue
            except Exception as exc:
                if self.running:
                    self.get_logger().error(f'Publish error: {exc}')
 
    # ── Shutdown ──────────────────────────────────────────────────────────────
 
    def stop(self):
        if not self.running:
            return
        self.running = False
 
        if self.appsrc is not None:
            self.appsrc.emit('end-of-stream')
 
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
 
        self.get_logger().info('🛑 H265 Decoder Node stopped')
 
 
# ── Entry point ───────────────────────────────────────────────────────────────
 
def main(args=None):
    rclpy.init(args=args)
    node = CameraH265DecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()