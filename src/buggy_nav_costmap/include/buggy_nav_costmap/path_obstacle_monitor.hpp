#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

namespace buggy_nav_costmap
{

// Collision stop gate. The "ahead zone" is a corridor in front of the
// vehicle, ahead_zone_half_width_m on each side (default 1.3 m = buggy
// footprint half-width 1.0 + margin 0.3), covering arc-length
// [zone_start_offset_m, ahead_zone_length_m] (default 0-5 m). Any obstacle
// inside it stops the vehicle:
//   - moving LiDAR points  (/moving_obstacle_points, dynamic_obstacle_detector)
//   - static LiDAR points  (/static_obstacle_points, same node)
//   - camera fusion        (/camera_red_detected, a boolean - blocks while
//                           true and fresh)
//
// The corridor BENDS along the live planner path (/suggested_lane) when
// one is actually usable this cycle - fresh, long enough, close to the
// vehicle, and pointing roughly the way the vehicle is actually facing
// (see evaluatePath()) - so a turn is checked along where the vehicle is
// about to go, not just straight-ahead of its current heading. It falls
// back to a straight corridor from the vehicle's current pose (fixed_frame
// -> vehicle_frame TF) whenever the path isn't usable - in particular
// while the planner is STUCK, since costmap_lane_planner simply stops
// publishing /suggested_lane in that state rather than clearing it, so an
// unchecked "path exists" test would silently steer the zone using a
// frozen path from before the obstacle that caused the STUCK. The straight
// fallback has no such dependency and keeps working regardless.
// enable_path_following_zone=false disables the bend entirely (straight
// corridor always) as an instant rollback switch, independent of a rebuild.
//
// If the vehicle pose is unavailable, the gate holds the vehicle (fail-safe).
// Blocking never triggers a replan; it only gates speed. Points are
// assumed to be in fixed_frame (dynamic_obstacle_detector publishes them
// there); a frame mismatch holds the vehicle.
//
// Outputs:
//   /moving_obstacle_blocking -> steering_calculator_node (speed/steering = 0)
//   /final_red_detected       -> uart_sender_node (real hardware STOP, red=1)
//   /ahead_zone_marker        -> RViz overlay: red while blocked, green clear
//
// Setting the stop: a LiDAR source needs lidar_set_debounce_messages
// consecutive ARRIVED messages with an in-zone point (evaluated in the
// subscription callbacks, since these topics publish far slower than the
// timer under this project's Gazebo real-time factor). Clearing is debounced
// by clear_debounce_cycles timer ticks.
class PathObstacleMonitor : public rclcpp::Node
{
public:
  PathObstacleMonitor();

private:
  struct LidarSource
  {
    bool confirmed = false;
    int consecutive = 0;
  };

  // One straight piece of the (possibly bent) corridor, in fixed_frame.
  // dir is a unit vector; cum_start is the arc length from the corridor's
  // own zone_start_offset_m origin to this segment's start - the straight
  // fallback is always exactly one segment, so BENT and STRAIGHT modes
  // share the same inZone()/publishZoneMarker() code with no special-casing.
  struct CorridorSegment
  {
    double start_x, start_y;
    double dir_x, dir_y;
    double length;
    double cum_start;
  };

  void movingPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void staticPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void cameraRedCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void navigationEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void suggestedLaneCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
  void evaluateSource(LidarSource & src, const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    const std::string & label);
  void processCycle();
  bool getVehiclePose(double & x, double & y, double & yaw);
  bool buildBentCorridor(const rclcpp::Time & stamp);
  void buildStraightCorridor();
  bool inZone(double px, double py) const;
  void publishZoneMarker(const rclcpp::Time & stamp, bool blocking, bool have_pose);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr moving_points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr static_points_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_red_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_enabled_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr suggested_lane_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr blocking_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr red_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr zone_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Zone geometry for the current cycle, set by processCycle() before the
  // LiDAR callbacks read it.
  bool have_pose_ = false;
  double pose_x_ = 0.0, pose_y_ = 0.0;
  double fwd_x_ = 1.0, fwd_y_ = 0.0;  // unit forward vector in fixed_frame
  std::vector<CorridorSegment> corridor_segments_;
  bool corridor_is_bent_ = false;  // for logging/diagnostics only

  // Latest /suggested_lane, written by suggestedLaneCallback(), read once
  // per processCycle() - same latest-wins pattern as the LiDAR sources'
  // zone geometry inputs below.
  std::vector<std::pair<double, double>> path_points_;
  rclcpp::Time last_path_time_;
  bool has_path_ = false;
  bool path_frame_ok_ = false;

  LidarSource moving_src_;
  LidarSource static_src_;

  bool camera_red_ = false;
  rclcpp::Time last_camera_red_time_;
  bool has_camera_red_ = false;

  // /navigation_enabled (manual_pose_aligner g/x). Real hardware ignores speed
  // commands, so the stop flag must also be set while navigation is not
  // confirmed and fresh.
  bool navigation_enabled_ = false;
  bool has_navigation_enabled_ = false;
  rclcpp::Time last_navigation_enabled_time_;
  double navigation_enabled_timeout_sec_ = 1.0;

  bool blocking_latched_ = false;
  int consecutive_clear_ = 0;

  std::string fixed_frame_;
  std::string vehicle_frame_;
  std::string moving_points_topic_;
  std::string static_points_topic_;
  std::string camera_red_topic_;
  std::string suggested_lane_topic_;
  double zone_start_offset_m_ = 0.0;
  double ahead_zone_length_m_ = 5.0;
  double ahead_zone_half_width_m_ = 1.3;
  int lidar_set_debounce_messages_ = 2;
  // A camera_red_detected=true older than this is ignored (fail-open), so a
  // crashed/closed camera_lidar_fusion_node can't hold the vehicle forever.
  double camera_timeout_sec_ = 1.0;
  int clear_debounce_cycles_ = 5;
  double process_rate_hz_ = 10.0;

  // Path-following zone (see class comment). true: use /suggested_lane to
  // bend the corridor when it passes evaluatePath()'s checks; false:
  // always use the straight pose-based corridor (instant rollback switch).
  bool enable_path_following_zone_ = true;
  // /suggested_lane older than this is treated as unusable (STUCK, or the
  // planner just isn't running) - must be tighter than process_rate_hz_'s
  // own period by a comfortable margin, not just "roughly matching" it,
  // else ordinary one-cycle publish jitter would flap the mode every cycle.
  double path_timeout_sec_ = 0.5;
  // Distance from the vehicle to the path's nearest point beyond which the
  // path is distrusted (vehicle has drifted too far from the planned lane
  // for "bend along it" to mean anything).
  double path_trust_max_lateral_m_ = 1.5;
  // Minimum points needed to form even one corridor segment.
  static constexpr size_t kPathTrustMinPoints = 2;
};

}  // namespace buggy_nav_costmap
