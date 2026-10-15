std_msgs/Header header   # frame_id of Cartesian fields/path, stamp = time of the cycle
string trigger           # sensor that triggered this cycle: "radar" | "camera"
string radar_frame_id    # frame of the *_radar fields of each object (the radar, e.g. umrr)
FusedObject[] objects
# predicted ego path in header.frame_id, at ground level, used for in_path
float32 path_curvature   # [1/m], + left
string path_source       # steering | odom
geometry_msgs/Point[] path_left
geometry_msgs/Point[] path_right
# stop zone: corridor from the radar to stop_zone_length ahead (closed outline, header.frame_id)
float32 stop_zone_length # [m] = stop_zone.min_distance + ego speed * stop_zone.time_headway
geometry_msgs/Point[] stop_zone
