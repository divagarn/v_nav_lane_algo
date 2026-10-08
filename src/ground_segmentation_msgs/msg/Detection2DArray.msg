# All detections from one camera frame. header.frame_id/stamp match the
# source image message so a fusion node can match this to a LiDAR scan.
std_msgs/Header header
Detection2D[] detections
