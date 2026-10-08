#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

namespace buggy_nav_costmap
{

// Collision stop gate. The "ahead zone" is a straight corridor in front of
// the vehicle, measured from its current pose (fixed_frame -> vehicle_frame
// TF): from zone_start_offset_m ahead of the vehicle out to
// ahead_zone_length_m, ahead_zone_half_width_m on each side (default 5 m
// long, 1.3 m half-width = buggy footprint half-width 1.0 + margin 0.3).
// Any obstacle inside it stops the vehicle. It does not depend on the
// planned path, so it keeps working when the planner has no lane:
//   - moving LiDAR points  (/moving_obstacle_points, dynamic_obstacle_detector)
//   - static LiDAR points  (/static_obstacle_points, same node)
//   - camera fusion        (/camera_red_detected, a boolean - blocks while
//                           true and fresh)
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

  void movingPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void staticPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void cameraRedCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void navigationEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void evaluateSource(LidarSource & src, const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    const std::string & label);
  void processCycle();
  bool getVehiclePose(double & x, double & y, double & yaw);
  bool inZone(double px, double py) const;
  void publishZoneMarker(const rclcpp::Time & stamp, bool blocking, bool have_pose);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr moving_points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr static_points_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_red_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_enabled_sub_;
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
  double zone_start_offset_m_ = 0.0;
  double ahead_zone_length_m_ = 5.0;
  double ahead_zone_half_width_m_ = 1.3;
  int lidar_set_debounce_messages_ = 2;
  // A camera_red_detected=true older than this is ignored (fail-open), so a
  // crashed/closed camera_lidar_fusion_node can't hold the vehicle forever.
  double camera_timeout_sec_ = 1.0;
  int clear_debounce_cycles_ = 5;
  double process_rate_hz_ = 10.0;
};

}  // namespace buggy_nav_costmap
