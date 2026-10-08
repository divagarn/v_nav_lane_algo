#include "buggy_nav_costmap/path_obstacle_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

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
  suggested_lane_topic_ = declare_parameter("suggested_lane_topic", std::string("/suggested_lane"));
  enable_path_following_zone_ = declare_parameter("enable_path_following_zone", true);
  path_timeout_sec_ = declare_parameter("path_timeout_sec", 0.5);
  path_trust_max_lateral_m_ = declare_parameter("path_trust_max_lateral_m", 1.5);

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

  suggested_lane_sub_ = create_subscription<visualization_msgs::msg::Marker>(
    suggested_lane_topic_, 1,
    [this](const visualization_msgs::msg::Marker::SharedPtr msg) {
      suggestedLaneCallback(msg);
    });

  blocking_pub_ = create_publisher<std_msgs::msg::Bool>("/moving_obstacle_blocking", 1);
  red_pub_ = create_publisher<std_msgs::msg::Bool>("/final_red_detected", 1);
  zone_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/ahead_zone_marker", 1);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / process_rate_hz_),
    std::bind(&PathObstacleMonitor::processCycle, this));

  RCLCPP_INFO(
    get_logger(),
    "path_obstacle_monitor: ahead zone %.1fm (from +%.1fm) x %.2fm half-width in %s from %s; "
    "bends along %s when fresh/close/aligned (enable_path_following_zone=%s), else straight; "
    "sources: %s, %s, %s -> /moving_obstacle_blocking + /final_red_detected",
    ahead_zone_length_m_, zone_start_offset_m_, ahead_zone_half_width_m_, fixed_frame_.c_str(),
    vehicle_frame_.c_str(), suggested_lane_topic_.c_str(),
    enable_path_following_zone_ ? "true" : "false", moving_points_topic_.c_str(),
    static_points_topic_.c_str(), camera_red_topic_.c_str());
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

void PathObstacleMonitor::suggestedLaneCallback(
  const visualization_msgs::msg::Marker::SharedPtr msg)
{
  path_frame_ok_ = msg->header.frame_id.empty() || msg->header.frame_id == fixed_frame_;
  path_points_.clear();
  path_points_.reserve(msg->points.size());
  for (const auto & p : msg->points) {
    path_points_.emplace_back(p.x, p.y);
  }
  last_path_time_ = msg->header.stamp;
  has_path_ = true;
}

bool PathObstacleMonitor::inZone(double px, double py) const
{
  if (!have_pose_ || corridor_segments_.empty()) {
    return false;
  }
  double zone_len = ahead_zone_length_m_ - zone_start_offset_m_;
  size_t n = corridor_segments_.size();

  // Nearest-segment search using each segment's own CLAMPED closest point
  // (standard polyline nearest-point search - correct at interior joints,
  // since the segments already tile the corridor continuously there).
  size_t best_i = 0;
  double best_dist2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < n; ++i) {
    const auto & seg = corridor_segments_[i];
    double dx = px - seg.start_x, dy = py - seg.start_y;
    double t_raw = dx * seg.dir_x + dy * seg.dir_y;
    double t_clamped = std::clamp(t_raw, 0.0, seg.length);
    double cx = seg.start_x + t_clamped * seg.dir_x;
    double cy = seg.start_y + t_clamped * seg.dir_y;
    double d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy);
    if (d2 < best_dist2) {
      best_dist2 = d2;
      best_i = i;
    }
  }

  // Final along/lateral from the winning segment's own direction. Only the
  // very first/last segment get the UNCLAMPED projection, so the corridor's
  // two true ends are flat planes (not rounded caps - the bug an earlier
  // path-based version of this zone had), while interior joints (handled
  // above, not here) stay exact because the segments already tile
  // continuously with no gap to round over.
  const auto & seg = corridor_segments_[best_i];
  double dx = px - seg.start_x, dy = py - seg.start_y;
  double t_raw = dx * seg.dir_x + dy * seg.dir_y;
  bool is_end = (best_i == 0 || best_i == n - 1);
  double t_final = is_end ? t_raw : std::clamp(t_raw, 0.0, seg.length);
  double along = seg.cum_start + t_final;
  double lateral = -dx * seg.dir_y + dy * seg.dir_x;

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

void PathObstacleMonitor::buildStraightCorridor()
{
  corridor_segments_.clear();
  corridor_is_bent_ = false;
  double zone_len = ahead_zone_length_m_ - zone_start_offset_m_;
  if (zone_len <= 0.0) {
    return;
  }
  CorridorSegment seg;
  seg.start_x = pose_x_ + zone_start_offset_m_ * fwd_x_;
  seg.start_y = pose_y_ + zone_start_offset_m_ * fwd_y_;
  seg.dir_x = fwd_x_;
  seg.dir_y = fwd_y_;
  seg.length = zone_len;
  seg.cum_start = 0.0;
  corridor_segments_.push_back(seg);
}

bool PathObstacleMonitor::buildBentCorridor(const rclcpp::Time & stamp)
{
  if (!enable_path_following_zone_ || !has_path_ || !path_frame_ok_ ||
    path_points_.size() < kPathTrustMinPoints)
  {
    return false;
  }
  if ((stamp - last_path_time_).seconds() > path_timeout_sec_) {
    return false;  // STUCK (or planner not running yet) - path is frozen, don't trust it.
  }

  // Nearest path point to the vehicle's OWN live pose (not the path's
  // embedded origin, which can be a cycle or two stale relative to this
  // node's own TF read) - this is where "arc length 0" starts from.
  size_t nearest_idx = 0;
  double nearest_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path_points_.size(); ++i) {
    double dx = path_points_[i].first - pose_x_;
    double dy = path_points_[i].second - pose_y_;
    double d2 = dx * dx + dy * dy;
    if (d2 < nearest_d2) {
      nearest_d2 = d2;
      nearest_idx = i;
    }
  }
  if (std::sqrt(nearest_d2) > path_trust_max_lateral_m_) {
    return false;  // Vehicle has drifted too far from this path to trust it.
  }
  if (nearest_idx + 1 >= path_points_.size()) {
    return false;  // No point ahead of the vehicle left to build a segment from.
  }

  // Local path direction must roughly match where the vehicle is actually
  // facing - guards against a stale/wrong-direction path slipping through
  // the timeout check (e.g. a very short STUCK blip right at the timeout
  // boundary).
  double pdx = path_points_[nearest_idx + 1].first - path_points_[nearest_idx].first;
  double pdy = path_points_[nearest_idx + 1].second - path_points_[nearest_idx].second;
  double plen = std::hypot(pdx, pdy);
  if (plen < 1e-6 || (pdx * fwd_x_ + pdy * fwd_y_) <= 0.0) {
    return false;
  }

  double zone_len = ahead_zone_length_m_ - zone_start_offset_m_;
  if (zone_len <= 0.0) {
    return false;
  }

  std::vector<CorridorSegment> segs;
  double arc = 0.0;        // cumulative arc length from nearest_idx, before trimming to zone_start
  double cum_start = 0.0;  // cumulative arc length of the ZONE (0 at zone_start_offset_m_)
  size_t i = nearest_idx;
  while (cum_start < zone_len) {
    double sx, sy, dx, dy, seg_len;
    if (i + 1 < path_points_.size()) {
      sx = path_points_[i].first; sy = path_points_[i].second;
      dx = path_points_[i + 1].first - sx;
      dy = path_points_[i + 1].second - sy;
      seg_len = std::hypot(dx, dy);
    } else {
      // Path ran out before covering the full zone length - extend using
      // the last segment's own direction rather than leaving the corridor
      // short of its configured safety margin.
      if (segs.empty()) {
        return false;
      }
      const auto & last = segs.back();
      sx = last.start_x + last.length * last.dir_x;
      sy = last.start_y + last.length * last.dir_y;
      dx = last.dir_x; dy = last.dir_y;
      seg_len = zone_len - cum_start + zone_start_offset_m_;  // just enough to finish the zone
    }
    if (seg_len < 1e-6) {
      ++i;
      continue;
    }
    double ux = dx / seg_len, uy = dy / seg_len;
    double seg_arc_start = arc;
    double seg_arc_end = arc + seg_len;
    arc = seg_arc_end;

    // Trim this raw path segment to the part that actually falls inside
    // [zone_start_offset_m_, ahead_zone_length_m_] of arc length from
    // nearest_idx, re-expressed as [0, zone_len] of the zone's own
    // cum_start - skips entirely if wholly before zone_start_offset_m_.
    double trim_start = std::max(seg_arc_start, zone_start_offset_m_);
    double trim_end = std::min(seg_arc_end, ahead_zone_length_m_);
    if (trim_end > trim_start) {
      double off_start = trim_start - seg_arc_start;
      CorridorSegment cs;
      cs.start_x = sx + off_start * ux;
      cs.start_y = sy + off_start * uy;
      cs.dir_x = ux;
      cs.dir_y = uy;
      cs.length = trim_end - trim_start;
      cs.cum_start = trim_start - zone_start_offset_m_;
      segs.push_back(cs);
      cum_start = trim_end - zone_start_offset_m_;
    }
    ++i;
  }

  if (segs.empty()) {
    return false;
  }
  corridor_segments_ = std::move(segs);
  corridor_is_bent_ = true;
  return true;
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

  if (!have_pose || corridor_segments_.empty()) {
    m.action = visualization_msgs::msg::Marker::DELETE;
    zone_marker_pub_->publish(m);
    visualization_msgs::msg::Marker outline_del = m;
    outline_del.id = 1;
    zone_marker_pub_->publish(outline_del);
    return;
  }

  m.action = visualization_msgs::msg::Marker::ADD;
  m.color.a = 0.35f;
  if (blocking) {
    m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f;
  } else {
    m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f;
  }

  // One quad per corridor segment, same frame the in-zone test uses - a
  // straight corridor is always exactly one segment (one quad, same shape
  // as before); a bent one is several consecutive quads following the
  // path, so what's drawn always matches what's actually being checked.
  auto point = [](double x, double y) {
      geometry_msgs::msg::Point p;
      p.x = x; p.y = y; p.z = 0.05;
      return p;
    };
  std::vector<geometry_msgs::msg::Point> left_edge, right_edge;
  for (size_t i = 0; i < corridor_segments_.size(); ++i) {
    const auto & seg = corridor_segments_[i];
    double nx = -seg.dir_y * ahead_zone_half_width_m_;
    double ny = seg.dir_x * ahead_zone_half_width_m_;
    double ex = seg.start_x + seg.length * seg.dir_x;
    double ey = seg.start_y + seg.length * seg.dir_y;
    auto la = point(seg.start_x + nx, seg.start_y + ny);
    auto ra = point(seg.start_x - nx, seg.start_y - ny);
    auto lb = point(ex + nx, ey + ny);
    auto rb = point(ex - nx, ey - ny);
    m.points.push_back(la); m.points.push_back(ra); m.points.push_back(rb);
    m.points.push_back(la); m.points.push_back(rb); m.points.push_back(lb);
    // Consecutive segments share an endpoint (trimmed from one continuous
    // polyline), so each segment contributes only its start corner to the
    // running edge polylines - except the last, which also contributes
    // its end corner to close them off.
    left_edge.push_back(la);
    right_edge.push_back(ra);
    if (i + 1 == corridor_segments_.size()) {
      left_edge.push_back(lb);
      right_edge.push_back(rb);
    }
  }
  zone_marker_pub_->publish(m);

  // Opaque outline on top of the translucent fill so the zone stays visible
  // even when the fill is hard to see against the map.
  visualization_msgs::msg::Marker outline = m;
  outline.id = 1;
  outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
  outline.scale.x = 0.08;
  outline.color.a = 1.0f;
  outline.points.clear();
  auto lift = [](geometry_msgs::msg::Point p) { p.z = 0.3; return p; };
  for (const auto & p : left_edge) {
    outline.points.push_back(lift(p));
  }
  for (auto it = right_edge.rbegin(); it != right_edge.rend(); ++it) {
    outline.points.push_back(lift(*it));
  }
  if (!outline.points.empty()) {
    outline.points.push_back(outline.points.front());
  }
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
    if (!buildBentCorridor(stamp)) {
      buildStraightCorridor();
    }
  } else {
    corridor_segments_.clear();
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
