#!/usr/bin/env python3
"""
ROS2 Camera H265 Publisher Node - QUALITY OPTIMIZED
Zero drops + High visual quality for AGX Orin
Published to topic for bag file recording
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import json
import time
import signal
import sys
import subprocess
import re
import os
import threading
from pathlib import Path
from datetime import datetime
from collections import deque


class CameraH265Node(Node):
    def __init__(self):
        
        super().__init__('camera_h265_node')
        
        # Declare ROS2 parameters
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('bitrate', 15000000)
        self.declare_parameter('topic_name', 'camera/compressed')
        self.declare_parameter('enable_monitoring', True)
        
        # Get parameters
        self.device = self.get_parameter('device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.bitrate = self.get_parameter('bitrate').value
        self.enable_monitoring = self.get_parameter('enable_monitoring').value
        self.topic_name = self.get_parameter('topic_name').value
        
        # ROS2 Publishers
        self.publisher = self.create_publisher(CompressedImage, self.topic_name, 10)
        self.metadata_publisher = self.create_publisher(String, f'{self.topic_name}/metadata', 10)
        self.performance_publisher = self.create_publisher(String, f'{self.topic_name}/performance', 10)
        
        self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        self.frame_count = 0
        self.frame_lock = threading.Lock()
        self.running = True
        self.pipeline_start_time = None
        self._last_pts = 0
        
        self.perf_log = deque(maxlen=1000)
        self.drop_count = 0
        self.max_delta_seen = 0
        
        # Performance tracking
        self.last_perf_publish = time.time()
        self.perf_publish_interval = 5.0  # Publish performance metrics every 5 seconds
        
        Gst.init(None)
        self.pipeline = None
        self.appsink = None
        self.loop = None
        self.glib_context = GLib.MainContext()
        self.glib_thread = None
        
        self.ntp_offset_ns = 0
        self.last_ntp_check = 0
        
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self._check_system_config()
        
        self.get_logger().info(f"🎬 Quality-Optimized Camera Node (ROS2 Topic Publisher)")
        self.get_logger().info(f"📁 Session: {self.session_id}")
        self.get_logger().info(f"📢 Publishing video to: /{self.topic_name}")
        self.get_logger().info(f"📊 Metadata topic: /{self.topic_name}/metadata")
        self.get_logger().info(f"📈 Performance topic: /{self.topic_name}/performance")
        self.get_logger().info(f"📊 Bitrate: {self.bitrate/1e6:.1f} Mbps")
        
    def _check_system_config(self):
        """Check system configuration"""
        self.get_logger().info(f"\n🔍 System Check:")
        
        warnings = []
        
        try:
            nvpmodel = subprocess.check_output(['nvpmodel', '-q'], stderr=subprocess.DEVNULL, timeout=1).decode()
            if 'NV Power Mode: MAXN' in nvpmodel or 'mode: 0' in nvpmodel:
                self.get_logger().info(f"   ✅ Power mode: MAXN")
            else:
                warnings.append("Run: sudo nvpmodel -m 0")
        except:
            pass
        
        try:
            governors = []
            for i in range(12):
                try:
                    gov = open(f'/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_governor').read().strip()
                    governors.append(gov)
                except:
                    break
            
            if governors and all(g == 'performance' for g in governors):
                self.get_logger().info(f"   ✅ CPU governor: performance")
            elif governors:
                warnings.append("Run: for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance | sudo tee $cpu; done")
        except:
            pass
        
        if Path(self.device).exists():
            self.get_logger().info(f"   ✅ Camera: {self.device}")
        else:
            self.get_logger().error(f"   ❌ Camera NOT FOUND: {self.device}")
        
        if warnings:
            self.get_logger().warn(f"\n⚠️  Recommendations:")
            for w in warnings:
                self.get_logger().warn(f"   • {w}")
        
    def _publish_frame_metadata(self, frame_id, gst_pts, gst_dts, sys_time, buf_size, delta, proc_time, ntp_off):
        """Publish frame metadata to ROS2 topic"""
        metadata = {
            'frame_id': frame_id,
            'gst_pts_ns': gst_pts,
            'gst_dts_ns': gst_dts,
            'system_time_ns': sys_time,
            'ntp_offset_ns': ntp_off,
            'buffer_size_bytes': buf_size,
            'delta_pts_ns': delta,
            'processing_time_us': proc_time
        }
        msg = String()
        msg.data = json.dumps(metadata)
        self.metadata_publisher.publish(msg)
    
    def _publish_performance_metrics(self):
        """Publish performance metrics periodically"""
        with self.frame_lock:
            frames = self.frame_count
            drops = self.drop_count
        
        if self.pipeline_start_time:
            dur = (time.time_ns() - self.pipeline_start_time) / 1e9
            fps = frames / dur if dur > 0 else 0
        else:
            fps = 0
            dur = 0
        
        perf_data = {
            'timestamp': time.time(),
            'total_frames': frames,
            'dropped_frames': drops,
            'fps': round(fps, 2),
            'duration_sec': round(dur, 2),
            'max_delta_ms': round(self.max_delta_seen / 1e6, 2),
            'subscribers': self.publisher.get_subscription_count()
        }
        msg = String()
        msg.data = json.dumps(perf_data)
        self.performance_publisher.publish(msg)
    
    def _glib_context_pump(self):
        """Poll GStreamer bus messages"""
        bus = self.pipeline.get_bus()
        while self.running:
            msg = bus.timed_pop_filtered(0, Gst.MessageType.ANY)  # Non-blocking pop
            if msg:
                self._on_bus_message(bus, msg)
            time.sleep(0.01)  # 10ms poll interval
            
    def _get_ntp_offset_ns(self):
        if time.time() - self.last_ntp_check < 5.0:
            return self.ntp_offset_ns
        
        self.last_ntp_check = time.time()
        
        try:
            out = subprocess.check_output(['chronyc', 'tracking'], timeout=0.5, stderr=subprocess.DEVNULL).decode()
            match = re.search(r'System time\s*:\s*([\d.]+)\s*seconds\s+(fast|slow)', out)
            if match:
                offset = float(match.group(1))
                if match.group(2) == 'slow':
                    offset = -offset
                self.ntp_offset_ns = int(offset * 1e9)
        except:
            pass
        
        return self.ntp_offset_ns
    
    def _create_pipeline(self):
        """Quality-optimized pipeline with appsink for publishing"""
        
        queue_buffers = self.fps * 2
        quality_bitrate = max(self.bitrate, 12000000)
        
        pipeline_str = f"""
        v4l2src device={self.device} do-timestamp=true io-mode=2 !
        video/x-raw,format=UYVY,width={self.width},height={self.height},framerate={self.fps}/1 !
        queue name=q_src max-size-buffers={queue_buffers} max-size-time=0 max-size-bytes=0 leaky=no !
        nvvidconv flip-method=0 ! 
        video/x-raw(memory:NVMM),format=NV12,width={self.width},height={self.height},framerate={self.fps}/1 !
        nvv4l2h265enc 
            bitrate={quality_bitrate}
            control-rate=0
            profile=0
            preset-level=2
            iframeinterval=15
            insert-sps-pps=true
            insert-vui=true
            insert-aud=true
            maxperf-enable=0
            MeasureEncoderLatency=1
            EnableTwopassCBR=0 ! 
        h265parse ! 
        queue name=q_app max-size-buffers={queue_buffers} max-size-time=0 max-size-bytes=0 leaky=no ! 
        appsink 
            name=sink 
            emit-signals=true 
            max-buffers=10 
            drop=false 
            sync=false
            enable-last-sample=false
            async=false
        """
        
        self.get_logger().info(f"🔧 Creating pipeline (bitrate: {quality_bitrate/1e6:.1f} Mbps, VBR, qtmux)...")
        
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except Exception as e:
            self.get_logger().error(f"❌ Failed: {e}")
            raise
        
        self.appsink = self.pipeline.get_by_name('sink')
        if not self.appsink:
            raise RuntimeError("No appsink")
        
        self.appsink.connect('new-sample', self._on_new_sample)
        
        bus = self.pipeline.get_bus()
        
        self.get_logger().info(f"✅ Pipeline created")
        
    def _on_new_sample(self, sink):
        if not self.running:
            return Gst.FlowReturn.OK
        
        t_start = time.perf_counter_ns()
        
        try:
            sample = sink.emit('pull-sample')
            if not sample:
                return Gst.FlowReturn.OK
            
            buf = sample.get_buffer()
            caps = sample.get_caps()
            
            gst_pts = buf.pts if buf.pts != Gst.CLOCK_TIME_NONE else 0
            gst_dts = buf.dts if buf.dts != Gst.CLOCK_TIME_NONE else 0
            sys_time = time.clock_gettime_ns(time.CLOCK_REALTIME)
            buf_size = buf.get_size()
            ntp_off = self._get_ntp_offset_ns()
            
            with self.frame_lock:
                if self.frame_count > 0:
                    delta = gst_pts - self._last_pts
                    self.max_delta_seen = max(self.max_delta_seen, delta)
                    
                    if delta > int(1.5e9 / self.fps):
                        drops = int(delta / (1e9 / self.fps)) - 1
                        self.drop_count += drops
                        self.get_logger().warn(f"⚠️  DROP: {drops} frames at {self.frame_count}")
                else:
                    delta = 0
                
                self._last_pts = gst_pts
                fid = self.frame_count
                self.frame_count += 1
            
            proc_time = (time.perf_counter_ns() - t_start) // 1000
            
            # Publish frame metadata to ROS2 topic
            self._publish_frame_metadata(fid, gst_pts, gst_dts, sys_time, buf_size, delta, proc_time, ntp_off)
            
            # Publish performance metrics periodically
            if time.time() - self.last_perf_publish >= self.perf_publish_interval:
                self._publish_performance_metrics()
                self.last_perf_publish = time.time()
            
            # ================== PUBLISH TO ROS2 ==================
            # Extract buffer data
            success, mapinfo = buf.map(Gst.MapFlags.READ)
            if success:
                frame_data = bytes(mapinfo.data)
                buf.unmap(mapinfo)
                
                # Create CompressedImage message
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f'frame_{fid}'
                msg.format = 'h265'  # H265/HEVC format
                msg.data = frame_data
                
                self.publisher.publish(msg)
            # ====================================================
            
            if fid % (self.fps * 10) == 0 and fid > 0:
                dur = gst_pts / 1e9
                fps = fid / dur if dur > 0 else 0
                self.get_logger().info(f"📹 {fid:6d} | {dur:7.2f}s | {fps:5.2f}fps | "
                      f"Drops:{self.drop_count:3d} | MaxΔ:{self.max_delta_seen/1e6:5.2f}ms")
        
        except Exception as e:
            self.get_logger().error(f"⚠️  Error: {e}")
        
        return Gst.FlowReturn.OK
    
    def _on_bus_message(self, bus, msg):
        if msg.type == Gst.MessageType.EOS:
            self.stop()
        elif msg.type == Gst.MessageType.ERROR:
            err, _ = msg.parse_error()
            self.get_logger().error(f"❌ Error: {err}")
            self.stop()
        elif msg.type == Gst.MessageType.STATE_CHANGED:
            if msg.src == self.pipeline:
                _, new, _ = msg.parse_state_changed()
                if new == Gst.State.PLAYING:
                    self.get_logger().info(f"✅ Pipeline PLAYING")
        return True
    
    def start(self):
        self.get_logger().info(f"\n🚀 Starting...")
        
        time.sleep(0.3)
        self._create_pipeline()
        
        # Start GLib context pump thread after pipeline creation
        self.glib_thread = threading.Thread(target=self._glib_context_pump, daemon=True)
        self.glib_thread.start()
        
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to start")
        
        self.pipeline_start_time = time.time_ns()
        self.get_logger().info(f"✅ Recording | Publishing to /{self.topic_name}\n")
    
    def stop(self):
        if not self.running:
            return
        
        self.get_logger().info(f"\n🛑 Stopping...")
        self.running = False
        
        if self.pipeline:
            self.pipeline.send_event(Gst.Event.new_eos())
            bus = self.pipeline.get_bus()
            bus.timed_pop_filtered(Gst.SECOND * 3, Gst.MessageType.EOS)
            self.pipeline.set_state(Gst.State.NULL)
        
        # Wait for GLib thread to finish (avoid self-join)
        if self.glib_thread and threading.current_thread() is not self.glib_thread:
            self.glib_thread.join(timeout=2)
        
        # Publish final performance metrics
        self._publish_performance_metrics()
        
        self._print_summary()
        self.get_logger().info(f"\n✅ Stopped")
    
    def _print_summary(self):
        with self.frame_lock:
            frames = self.frame_count
            drops = self.drop_count
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"📊 SUMMARY")
        self.get_logger().info(f"{'='*60}")
        self.get_logger().info(f"Frames:      {frames:,}")
        self.get_logger().info(f"Drops:       {drops}")
        
        if frames > 0:
            self.get_logger().info(f"Drop rate:   {(drops/frames)*100:.4f}%")
        
        if self.pipeline_start_time:
            dur = (time.time_ns() - self.pipeline_start_time) / 1e9
            self.get_logger().info(f"Duration:    {dur:.2f}s")
            self.get_logger().info(f"Avg FPS:     {frames/dur:.2f}")
            self.get_logger().info(f"Max delta:   {self.max_delta_seen/1e6:.2f}ms")
        
        self.get_logger().info(f"\n{'🏆 PERFECT!' if drops == 0 else '⚠️ Needs tuning'}")
        self.get_logger().info(f"{'='*60}")
    
    def _signal_handler(self, sig, frame):
        self.stop()


def main(args=None):
    rclpy.init(args=args)
    
    # Create node (parameters are declared and read from launch file or CLI)
    node = CameraH265Node()
    
    node.start()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
