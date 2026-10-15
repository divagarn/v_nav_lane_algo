std_msgs/Header header   # stamp = image stamp, frame_id = camera frame
string source            # "json_replay" | "yolo_opencv_dnn" | "yolo_tensorrt"
uint32 image_width
uint32 image_height
CameraObject[] objects
