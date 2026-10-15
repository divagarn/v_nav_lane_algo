# One detected (and tracked) object in the camera image.
uint32 track_id          # 2D track id (radar_fusion camera_tracks); 0 = raw detection
int32 class_id           # detector class index (COCO for the default YOLO)
string class_name        # person, car, truck, bus, bicycle, motorcycle, ...
float32 score
# bounding box in pixels of the source image
float32 x_min
float32 y_min
float32 x_max
float32 y_max
# box-centre velocity from the 2D tracker [px/s]
float32 vu
float32 vv
uint32 age               # frames since the track was created
