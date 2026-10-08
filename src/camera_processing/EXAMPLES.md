# Example Camera Processing Configurations

## High Quality (15 Mbps, 1080p, 30fps)
```bash
ros2 launch camera_processing camera_h265.launch.py \
  device:=/dev/video0 \
  width:=1920 \
  height:=1080 \
  fps:=30 \
  bitrate:=15000000 \
  topic_name:=camera/compressed
```

## High Speed (720p, 60fps)
```bash
ros2 launch camera_processing camera_h265.launch.py \
  device:=/dev/video0 \
  width:=1280 \
  height:=720 \
  fps:=60 \
  bitrate:=12000000 \
  topic_name:=camera/compressed
```

## Low Bandwidth (8 Mbps, 720p, 30fps)
```bash
ros2 launch camera_processing camera_h265.launch.py \
  device:=/dev/video0 \
  width:=1280 \
  height:=720 \
  fps:=30 \
  bitrate:=8000000 \
  topic_name:=camera/compressed
```

## Record to ROS2 Bag

### Terminal 1: Start Camera Node
```bash
ros2 launch camera_processing camera_h265.launch.py
```

### Terminal 2: Record topics to bag file
```bash
ros2 bag record camera/compressed camera/compressed/metadata camera/compressed/performance \
  -o /home/orin2/demo/bags/camera_session
```

### Terminal 3: Monitor Performance Metrics (optional)
```bash
# Subscribe and display performance metrics
ros2 topic echo camera/compressed/performance
```

## View Live Performance Stream

```bash
# Display all performance updates in real-time
ros2 topic echo camera/compressed/performance --no-prettify
```

## Parse Metadata on a Consumer Node

Example Python node that subscribes to metadata:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class CameraMetricsSubscriber(Node):
    def __init__(self):
        super().__init__('camera_metrics_subscriber')
        self.subscription = self.create_subscription(
            String,
            'camera/compressed/metadata',
            self.metadata_callback,
            10
        )
    
    def metadata_callback(self, msg):
        data = json.loads(msg.data)
        print(f"Frame {data['frame_id']}: PTS={data['gst_pts_ns']} ns, "
              f"Processing={data['processing_time_us']} µs")

def main(args=None):
    rclpy.init(args=args)
    subscriber = CameraMetricsSubscriber()
    rclpy.spin(subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Playback from Bag

```bash
# Play back the recorded bag
ros2 bag play /home/orin2/demo/bags/camera_session

# In another terminal, view the compressed image stream
ros2 run image_view image_view image:=camera/compressed
```

## Compare with Local Node

Monitor frame rate and metrics in real-time:

```bash
# Terminal 1: Start camera
ros2 launch camera_processing camera_h265.launch.py device:=/dev/video0

# Terminal 2: Monitor performance
watch -n 1 'ros2 topic echo camera/compressed/performance --no-prettify | tail -5'

# Terminal 3: Count frames per second
ros2 topic hz camera/compressed
```

## Notes

- H265 format string is set as 'h265' for compatibility with ROS2 tools
- Metadata is published as JSON strings for flexible parsing
- No local files are created - all data flows through ROS2 topics
- All parameters can be overridden from command line with ROS2 launch syntax
- Performance metrics published approximately every 5 seconds
