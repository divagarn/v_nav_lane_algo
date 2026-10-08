# Camera Processing - ROS2 H265 Camera Node

High-quality H265 video capture node for AGX Orin, optimized for zero frame drops. Publishes encoded video and metadata to ROS2 topics (no local file storage).

## Features

- **H265/HEVC Encoding**: High-quality video compression via NVIDIA NVENC
- **ROS2 Publishers**: 
  - Video frames as `CompressedImage` messages
  - Frame metadata (timestamps, PTS, DTS, buffer info) as JSON on metadata topic
  - Performance metrics as JSON on performance topic
- **Bag File Support**: Full compatibility with ROS2 bag recording
- **Zero Local Storage**: All data published to topics (no file writes except optional logging)
- **Quality Monitoring**: Integrated frame dropping detection
- **Timestamp Tracking**: Comprehensive timing information (GStreamer PTS, DTS, system time, NTP offset)
- **Performance Monitoring**: Real-time FPS and performance metrics

## Requirements

### System Requirements
- **AGX Orin** (Jetson platform)
- GStreamer 1.0 with NVIDIA plugins
- Python 3.8+

### System Configuration

Before running, ensure optimal performance:

```bash
# Set to MAXN power mode
sudo nvpmodel -m 0

# Set CPU governor to performance
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  echo performance | sudo tee $cpu > /dev/null
done
```

## Installation

1. **Clone/Build the package**:
```bash
cd ~/ros2_ws
source /opt/ros/<distro>/setup.bash
colcon build --packages-select camera_processing
source install/setup.bash
```

2. **Verify camera device**:
```bash
ls -la /dev/video*
```

## Usage

### Option 1: Launch File (Recommended)

```bash
ros2 launch camera_processing camera_h265.launch.py \
  device:=/dev/video0 \
  width:=1920 \
  height:=1080 \
  fps:=30 \
  bitrate:=15000000 \
  topic_name:=camera/compressed
```

### Option 2: Direct Command Line

```bash
ros2 run camera_processing camera_h265_node \
  --ros-args \
  -p device:=/dev/video0 \
  -p width:=1920 \
  -p height:=1080 \
  -p fps:=30 \
  -p bitrate:=15000000 \
  -p topic_name:=camera/compressed
```

### Option 3: Record to ROS2 Bag

```bash
# Terminal 1: Start camera node
ros2 launch camera_processing camera_h265.launch.py device:=/dev/video0

# Terminal 2: Record to bag file
ros2 bag record camera/compressed camera/compressed/metadata camera/compressed/performance \
  -o camera_session
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `device` | `/dev/video0` | V4L2 video device path |
| `width` | `1920` | Video frame width |
| `height` | `1080` | Video frame height |
| `fps` | `30` | Frames per second |
| `bitrate` | `15000000` | H265 bitrate in bits/sec (15 Mbps) |
| `topic_name` | `camera/compressed` | ROS2 topic name for video frames |
| `enable_monitoring` | `true` | Enable performance monitoring |

## Output Topics

The node publishes to three ROS2 topics:

* **`/<topic_name>`** - Video frames as CompressedImage (H265 format)
  * Message type: `sensor_msgs/msg/CompressedImage`
  * Format: `h265`
  
* **`/<topic_name>/metadata`** - Frame metadata (JSON)
  * Message type: `std_msgs/msg/String`
  * Contains: frame_id, timestamps (PTS, DTS, system), buffer size, delta, processing time
  
* **`/<topic_name>/performance`** - Performance metrics (JSON, published ~every 5 seconds)
  * Message type: `std_msgs/msg/String`
  * Contains: FPS, frame count, drops, duration, subscriber count

## Performance Notes

- **Bitrate**: 15 Mbps default (adjust based on quality/bandwidth needs)
- **Frame Rate**: 30 FPS standard (adjustable)
- **Resolution**: 1920x1080 standard (adjustable)
- **CPU Usage**: Optimized for AGX Orin with NVIDIA NVENC hardware encoding
- **Memory**: ~200MB for typical configuration
- **Network**: All data flows through ROS2 topics (local system or network)

## Troubleshooting

**Camera Device Not Found**
```bash
# Check available video devices
ls -l /dev/video*

# If needed, identify correct device
v4l2-ctl --list-devices
```

**Power Settings Warning**
The node will warn if not in MAXN mode. Apply recommendations to optimize performance.

**Frame Drops**
Monitor the performance metrics topic:
```bash
ros2 topic echo camera/compressed/performance
```

**High Topic Latency**
Check number of subscribers and network conditions:
```bash
ros2 topic info camera/compressed
ros2 topic hz camera/compressed
```

**Cannot Connect to Camera**
- Ensure camera device exists: `ls /dev/video*`
- Check device permissions
- Verify Gstreamer installation: `gst-inspect-1.0 v4l2src`

## Development

To run tests:
```bash
cd ~/ros2_ws
colcon test --packages-select camera_processing
```

## License

MIT License
