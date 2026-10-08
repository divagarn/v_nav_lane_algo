#include "buggy_nav_costmap/path_obstacle_monitor.hpp"

#include <cmath>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace buggy_nav_costmap
{

PathObstacleMonitor::PathObstacleMonitor()
: rclcpp::Node("path_obstacle_monitor")
{
  fixed_frame_ = declare_parameter("fixed_frame", std::string("odom"));
  vehicle_frame_ = declare_parameter("vehicle_frame", std::string("base_link"));
  moving_points_topic_ = declare_parameter(
    "moving_points_topic", std::string("/moving_obstacle_points"));
  static_points_topic_ = declare_parameter(
    "static_points_topic", std::string("/static_obstacle_points"));
  camera_red_topic_ = declare_parameter("camera_red_topic", std::string("/camera_red_detected"));
  zone_start_offset_m_ = declare_parameter("zone_start_offset_m", 0.0);
  ahead_zone_length_m_ = declare_parameter("ahead_zone_length_m", 5.0);
  ahead_zone_half_width_m_ = declare_parameter("ahead_zone_half_width_m", 1.3);
  lidar_set_debounce_messages_ = declare_parameter("lidar_set_debounce_messages", 2);
  camera_timeout_sec_ = declare_parameter("camera_timeout_sec", 1.0);
  clear_debounce_cycles_ = declare_parameter("clear_debounce_cycles", 5);
  process_rate_hz_ = declare_parameter("process_rate_hz", 10.0);
  navigation_enabled_timeout_sec_ = declare_parameter("navigation_enabled_timeout_sec", 1.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  moving_points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    moving_points_topic_, 1,
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      movingPointsCallback(msg);
    });
  static_points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    static_points_topic_, 1,
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      staticPointsCallback(msg);
    });
  camera_red_sub_ = create_subscription<std_msgs::msg::Bool>(
    camera_red_topic_, 1,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      cameraRedCallback(msg);
    });

  navigation_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/navigation_enabled", 1,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      navigationEnabledCallback(msg);
    });

  blocking_pub_ = create_publisher<std_msgs::msg::Bool>("/moving_obstacle_blocking", 1);
  red_pub_ = create_publisher<std_msgs::msg::Bool>("/final_red_detected", 1);
  zone_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/ahead_zone_marker", 1);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / process_rate_hz_),
    std::bind(&PathObstacleMonitor::processCycle, this));

  RCLCPP_INFO(
    get_logger(),
    "path_obstacle_monitor: straight ahead zone %.1fm (from +%.1fm) x %.2fm half-width in %s "
    "from %s; sources: %s, %s, %s -> /moving_obstacle_blocking + /final_red_detected",
    ahead_zone_length_m_, zone_start_offset_m_, ahead_zone_half_width_m_, fixed_frame_.c_str(),
    vehicle_frame_.c_str(), moving_points_topic_.c_str(), static_points_topic_.c_str(),
    camera_red_topic_.c_str());
}

bool PathObstacleMonitor::getVehiclePose(double & x, double & y, double & yaw)
{
  try {
    auto t = tf_buffer_->lookupTransform(
      fixed_frame_, vehicle_frame_, tf2::TimePointZero, tf2::durationFromSec(0.2));
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    const auto & q = t.transform.rotation;
    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void PathObstacleMonitor::navigationEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  navigation_enabled_ = msg->data;
  last_navigation_enabled_time_ = get_clock()->now();
  has_navigation_enabled_ = true;
}

void PathObstacleMonitor::cameraRedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  camera_red_ = msg->data;
  last_camera_red_time_ = get_clock()->now();
  has_camera_red_ = true;
}

bool PathObstacleMonitor::inZone(double px, double py) const
{
  // Coordinates relative to the zone start, projected onto the forward axis
  // (along) and its left normal (lateral).
  if (!have_pose_) {
    return false;
  }
  double sx = pose_x_ + zone_start_offset_m_ * fwd_x_;
  double sy = pose_y_ + zone_start_offset_m_ * fwd_y_;
  double dx = px - sx, dy = py - sy;
  double along = dx * fwd_x_ + dy * fwd_y_;
  double lateral = -dx * fwd_y_ + dy * fwd_x_;
  double zone_len = ahead_zone_length_m_ - zone_start_offset_m_;
  return along >= 0.0 && along <= zone_len && std::abs(lateral) <= ahead_zone_half_width_m_;
}

void PathObstacleMonitor::evaluateSource(
  LidarSource & src, const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  const std::string & label)
{
  // Evaluated once per ARRIVED message (see class comment).
  if (!have_pose_) {
    src.consecutive = 0;
    src.confirmed = false;
    return;
  }
  if (!msg->header.frame_id.empty() && msg->header.frame_id != fixed_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Frame mismatch: %s is in '%s' but this node uses '%s' - holding the vehicle until "
      "fixed_frame params match.",
      label.c_str(), msg->header.frame_id.c_str(), fixed_frame_.c_str());
    src.consecutive = lidar_set_debounce_messages_;
    src.confirmed = true;
    return;
  }

  bool in_zone = false;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y)) {
      continue;
    }
    if (inZone(*iter_x, *iter_y)) {
      in_zone = true;
      break;
    }
  }

  src.consecutive = in_zone ? src.consecutive + 1 : 0;
  src.confirmed = src.consecutive >= lidar_set_debounce_messages_;
}

void PathObstacleMonitor::movingPointsCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  evaluateSource(moving_src_, msg, "moving points");
}

void PathObstacleMonitor::staticPointsCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  evaluateSource(static_src_, msg, "static points");
}

void PathObstacleMonitor::publishZoneMarker(
  const rclcpp::Time & stamp, bool blocking, bool have_pose)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = fixed_frame_;
  m.header.stamp = stamp;
  m.ns = "ahead_zone";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  m.pose.orientation.w = 1.0;

  if (!have_pose) {
    m.action = visualization_msgs::msg::Marker::DELETE;
    zone_marker_pub_->publish(m);
    return;
  }

  m.action = visualization_msgs::msg::Marker::ADD;
  m.color.a = 0.35f;
  if (blocking) {
    m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f;
  } else {
    m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f;
  }

  // Corners of the rectangle, in the same frame the in-zone test uses.
  double sx = pose_x_ + zone_start_offset_m_ * fwd_x_;
  double sy = pose_y_ + zone_start_offset_m_ * fwd_y_;
  double ex = pose_x_ + ahead_zone_length_m_ * fwd_x_;
  double ey = pose_y_ + ahead_zone_length_m_ * fwd_y_;
  double nx = -fwd_y_ * ahead_zone_half_width_m_;
  double ny = fwd_x_ * ahead_zone_half_width_m_;
  auto point = [](double x, double y) {
      geometry_msgs::msg::Point p;
      p.x = x; p.y = y; p.z = 0.05;
      return p;
    };
  auto la = point(sx + nx, sy + ny);
  auto ra = point(sx - nx, sy - ny);
  auto lb = point(ex + nx, ey + ny);
  auto rb = point(ex - nx, ey - ny);
  m.points = {la, ra, rb, la, rb, lb};
  zone_marker_pub_->publish(m);

  // Opaque outline on top of the translucent fill so the zone stays visible
  // even when the fill is hard to see against the map.
  visualization_msgs::msg::Marker outline = m;
  outline.id = 1;
  outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
  outline.scale.x = 0.08;
  outline.color.a = 1.0f;
  auto lift = [](geometry_msgs::msg::Point p) { p.z = 0.3; return p; };
  outline.points = {lift(la), lift(ra), lift(rb), lift(lb), lift(la)};
  zone_marker_pub_->publish(outline);
}

void PathObstacleMonitor::processCycle()
{
  rclcpp::Time stamp = get_clock()->now();

  double x = 0.0, y = 0.0, yaw = 0.0;
  have_pose_ = getVehiclePose(x, y, yaw);
  if (have_pose_) {
    pose_x_ = x;
    pose_y_ = y;
    fwd_x_ = std::cos(yaw);
    fwd_y_ = std::sin(yaw);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No %s->%s pose - holding the vehicle (fail-safe) until TF is available.",
      fixed_frame_.c_str(), vehicle_frame_.c_str());
  }

  bool camera_blocking = false;
  if (has_camera_red_ &&
    (stamp - last_camera_red_time_).seconds() <= camera_timeout_sec_)
  {
    camera_blocking = camera_red_;
  }

  bool blocking_now = !have_pose_ || moving_src_.confirmed || static_src_.confirmed ||
    camera_blocking;

  if (blocking_now) {
    blocking_latched_ = true;
    consecutive_clear_ = 0;
  } else {
    consecutive_clear_ += 1;
    if (consecutive_clear_ >= clear_debounce_cycles_) {
      blocking_latched_ = false;
    }
  }

  bool nav_ok = has_navigation_enabled_ && navigation_enabled_ &&
    (stamp - last_navigation_enabled_time_).seconds() <= navigation_enabled_timeout_sec_;

  std_msgs::msg::Bool out;
  out.data = blocking_latched_;
  blocking_pub_->publish(out);

  // Real stop: red whenever the zone blocks OR navigation is not confirmed.
  std_msgs::msg::Bool stop;
  stop.data = blocking_latched_ || !nav_ok;
  red_pub_->publish(stop);
  publishZoneMarker(stamp, blocking_latched_, have_pose_);
}

}  // namespace buggy_nav_costmap
