# Cartesian fields are expressed in FusedObjectArray.header.frame_id.
uint32 id
string class_name                    # "unknown" until a camera association votes a class
float32 class_confidence
geometry_msgs/Point position         # object reference point
geometry_msgs/Vector3 velocity       # over-ground velocity [m/s]
float32 yaw_rate                     # IMM coordinated-turn rate [rad/s]
float32 heading                      # [rad], from velocity if moving else 0
geometry_msgs/Vector3 dimensions     # length, width, height used for display [m]
float32 range                        # [m] from radar origin (radar-relative)
float32 azimuth                      # [rad] in the radar frame
float32[4] position_covariance       # xx, xy, yx, yy
# the same object in the radar frame (FusedObjectArray.radar_frame_id), never transformed:
# relative to the radar at header.stamp; velocity is still over ground
geometry_msgs/Point position_radar
geometry_msgs/Vector3 velocity_radar
geometry_msgs/Point predicted_position_radar
float32[] model_probabilities        # IMM mode probabilities (order = imm.models param)
float32 existence_probability        # hit: pe += gain (1 - pe), miss: pe *= factor
bool ghost                           # double-bounce multipath ghost (only in ~/tracks, never in ~/objects)
bool is_moving
string motion_state                  # stationary | moving | turning (IMM mode probabilities)
# Motion relative to the ego vehicle. Unlike motion_state, this includes ego velocity.
# same_direction may still have positive closing_speed when the lead object is slower.
string relative_motion_state         # approaching | moving_away | same_direction | stationary | crossing | unknown
float32 closing_speed                # signed LOS closing speed [m/s], positive = range decreasing
float32 time_to_collision            # range / closing_speed [s], -1 when not closing
# Over-ground position at prediction_horizon seconds (constant velocity; stationary objects stay),
# in FusedObjectArray.header.frame_id like position.
geometry_msgs/Point predicted_position
float32 prediction_horizon           # seconds after FusedObjectArray.header.stamp
# ego path (steering / odometry curvature, corridor of +-half width)
bool in_path
float32 path_distance                # along the path [m]
float32 path_lateral                 # from the path centre, + left [m]
# in the ego corridor at prediction_horizon, relative to where the ego will be then
bool predicted_in_path
float32 predicted_path_distance
float32 predicted_path_lateral
# stop zone (FusedObjectArray.stop_zone): object predicted with constant velocity, ego holding its
# speed and yaw rate; planners stop on in_stop_zone or a small time_to_stop_zone
bool in_stop_zone
float32 time_to_stop_zone            # [s] 0 = inside now, -1 = not within the prediction horizon
uint32 age                           # number of tracker cycles
uint32 hits
# sensors that updated this track in the latest cycle / ever
bool radar_updated
bool camera_associated
bool lidar_updated
uint32 camera_track_id               # associated 2D track (0 = none)
# Fusion: every object is a confirmed radar track. status "radar" or "radar+camera"
# (camera box associated within the hold time). The camera adds the class and a bearing
# correction (Bayes on the bearing, radar keeps >= radar_min_weight); range, velocity and
# existence are radar.
string status
float32 assoc_probability            # bounded cheap-JPDA probability of the camera association
float32 bearing_radar                # [rad] in the radar frame
float32 bearing_fused                # [rad] in the radar frame (before Cartesian output transform)
float32 height                       # mean measured height above ground [m]
# Image plane (pixels of the rectified detector image)
bool in_image
float32 image_u                      # fused object centre
float32 image_v
float32 projected_u                  # radar-only position
float32 projected_v
# box: projected class-sized 3D box at the fused position
bool has_bbox
float32 bbox_x_min
float32 bbox_y_min
float32 bbox_x_max
float32 bbox_y_max
# class vote shares (temporal filter), same order as class_names
string[] class_names
float32[] class_probabilities
