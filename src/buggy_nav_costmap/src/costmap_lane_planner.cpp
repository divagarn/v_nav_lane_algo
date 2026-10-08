#include "buggy_nav_costmap/costmap_lane_planner.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace buggy_nav_costmap
{

CostmapLanePlanner::CostmapLanePlanner()
: rclcpp::Node("costmap_lane_planner")
{
  fixed_frame_ = declare_parameter("fixed_frame", std::string("odom"));
  base_frame_ = declare_parameter("base_frame", std::string("base_link"));
  obstacle_topic_ = declare_parameter("obstacle_topic", std::string("/velodyne_points"));
  resolution_ = declare_parameter("resolution", 0.3);
  lane_lookahead_ = declare_parameter("lane_lookahead", 12.0);
  lane_num_candidates_ = declare_parameter("lane_num_candidates", 15);
  corridor_scan_limit_ = declare_parameter("corridor_scan_limit", 8.0);
  lane_straight_preference_ratio_ = declare_parameter("lane_straight_preference_ratio", 0.85);
  lane_smoothing_alpha_ = declare_parameter("lane_smoothing_alpha", 0.3);
  steering_lookahead_distance_ = declare_parameter("steering_lookahead_distance", 6.0);
  vehicle_half_width_ = declare_parameter("vehicle_half_width", 1.0);
  vehicle_half_length_ = declare_parameter("vehicle_half_length", 1.8);
  vehicle_footprint_margin_ = declare_parameter("vehicle_footprint_margin", 0.3);
  min_obstacle_height_ = declare_parameter("min_obstacle_height", -0.6);
  max_obstacle_height_ = declare_parameter("max_obstacle_height", 2.0);
  obstacle_inflation_radius_ = declare_parameter("obstacle_inflation_radius", 0.25);
  obstacle_safety_margin_ = declare_parameter("obstacle_safety_margin", 0.3);
  obstacle_search_radius_ = declare_parameter("obstacle_search_radius", 25.0);
  double wheelbase = declare_parameter("wheelbase", 1.6);
  double max_steering_deg = declare_parameter("max_steering_deg", 20.0);
  direction_reference_gain_ = declare_parameter("direction_reference_gain", 6.0);
  double publish_rate = declare_parameter("publish_rate", 10.0);
  enable_pose_graph_bias_ = declare_parameter("enable_pose_graph_bias", true);
  std::string pose_graph_topic = declare_parameter(
    "pose_graph_topic", std::string("/slam_toolbox/graph_visualization"));
  start_stop_origin_ = declare_parameter("start_stop_origin", false);
  origin_stop_radius_m_ = declare_parameter("origin_stop_radius_m", 1.5);
  origin_min_travel_m_ = declare_parameter("origin_min_travel_m", 10.0);

  double steer_rad_clamped = std::max(max_steering_deg, 0.5) * M_PI / 180.0;
  min_turn_radius_ = wheelbase / std::tan(steer_rad_clamped);
  max_curvature_ = 1.0 / min_turn_radius_;
  clearance_ = vehicle_half_width_ + vehicle_footprint_margin_;

  PoseGraphWalker::Params pg_params;
  pg_params.fixed_frame = fixed_frame_;
  pg_params.map_frame = declare_parameter("map_frame", std::string("map"));
  pg_params.turn_lookahead_m = declare_parameter("turn_lookahead_m", 9.0);
  pg_params.route_preview_distance_m = declare_parameter("route_preview_distance_m", 40.0);
  pg_params.nearest_vertex_max_dist = declare_parameter("nearest_vertex_max_dist", 5.0);
  pg_params.snap_deg = declare_parameter("snap_deg", 25.0);
  pg_params.direction_disambiguation_vertices =
    declare_parameter("direction_disambiguation_vertices", 5);
  pg_params.pose_graph_stabilize_count = declare_parameter("pose_graph_stabilize_count", 2);
  pg_params.pose_graph_history_depth =
    static_cast<size_t>(declare_parameter("pose_graph_history_depth", 3));
  pg_params.pose_graph_reset_streak_threshold =
    static_cast<size_t>(declare_parameter("pose_graph_reset_streak_threshold", 3));
  pg_params.route_preview_max_jump_m = declare_parameter("route_preview_max_jump_m", 3.0);
  pg_params.route_preview_cache_timeout_sec =
    declare_parameter("route_preview_cache_timeout_sec", 999999.0);
  pg_params.turn_signal_timeout_sec = declare_parameter("turn_signal_timeout_sec", 2.0);
  pg_params.loop_closure_max_gap_m = declare_parameter("loop_closure_max_gap_m", 2.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pose_graph_walker_ = std::make_unique<PoseGraphWalker>(this, tf_buffer_.get(), pg_params);
  pose_graph_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    pose_graph_topic, 1,
    [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      pose_graph_walker_->poseGraphCallback(msg);
    });

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    obstacle_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      cloudCallback(msg);
    });

  // /navigation_enabled is the same topic manual_pose_aligner publishes on
  // 'g' - plain volatile QoS to match its publisher (manual_pose_aligner
  // republishes every 0.2s specifically so a late/volatile subscriber never
  // misses the current state - see its own module docstring).
  navigation_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/navigation_enabled", 1,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      navigationEnabledCallback(msg);
    });

  lane_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/suggested_lane", 1);
  obstacle_area_pub_ = create_publisher<visualization_msgs::msg::Marker>("/obstacle_area", 1);
  free_lane_area_pub_ = create_publisher<visualization_msgs::msg::Marker>("/free_lane_area", 1);
  target_lateral_pub_ = create_publisher<std_msgs::msg::Float32>("/target_lateral_position", 1);
  nav_status_pub_ = create_publisher<std_msgs::msg::String>("/nav_status", 1);
  lap_complete_pub_ = create_publisher<std_msgs::msg::Bool>("/lap_complete", 1);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate),
    std::bind(&CostmapLanePlanner::planCycle, this));

  RCLCPP_INFO(
    get_logger(),
    "costmap_lane_planner: wheelbase=%.2fm max_steering=%.1fdeg -> min_turn_radius=%.2fm, "
    "lane_lookahead=%.1fm, %d candidates, direction_reference_gain=%.1f, "
    "enable_pose_graph_bias=%s, obstacle_topic=%s (no costmap - direct point-cloud "
    "occupancy, rebuilt every cycle).",
    wheelbase, max_steering_deg, min_turn_radius_, lane_lookahead_, lane_num_candidates_,
    direction_reference_gain_, enable_pose_graph_bias_ ? "true" : "false",
    obstacle_topic_.c_str());
}

void CostmapLanePlanner::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_cloud_ = msg;
}

void CostmapLanePlanner::navigationEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  navigation_enabled_ = msg->data;
  if (navigation_enabled_ && !prev_navigation_enabled_) {
    // Rising edge ('g' just pressed) - snap wherever the vehicle currently
    // is onto the nearest recorded pose-graph vertex and remember that as
    // the lap origin, regardless of where on the loop (or in which
    // direction) the vehicle actually starts.
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(
        fixed_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.2));
      auto nearest = pose_graph_walker_->nearestVertex(
        t.transform.translation.x, t.transform.translation.y);
      if (nearest.has_value()) {
        auto [vx, vy, vd] = *nearest;
        start_position_ = Vec2{vx, vy};
        RCLCPP_INFO(
          get_logger(), "Navigation enabled: lap origin snapped to pose-graph vertex "
          "(%.2f, %.2f), %.2fm from current position.", vx, vy, vd);
      } else {
        start_position_.reset();
        RCLCPP_WARN(
          get_logger(),
          "Navigation enabled but no pose graph is available yet - start_stop_origin "
          "will have no effect until one arrives.");
      }
    } catch (const std::exception & e) {
      start_position_.reset();
      RCLCPP_WARN(
        get_logger(), "Navigation enabled but TF lookup failed, could not capture lap "
        "origin: %s", e.what());
    }
    has_left_origin_ = false;
    lap_complete_ = false;
  }
  prev_navigation_enabled_ = navigation_enabled_;
}

int64_t CostmapLanePlanner::gridKey(int cx, int cy) const
{
  return (static_cast<int64_t>(cx) << 32) ^ (static_cast<uint32_t>(cy));
}

void CostmapLanePlanner::rebuildObstacleGrid(const Vec2 & pos)
{
  obstacle_grid_.clear();
  obstacle_points_.clear();

  if (!latest_cloud_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No point cloud received yet on %s - treating everything as unknown/unblocked.",
      obstacle_topic_.c_str());
    return;
  }

  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform(
      fixed_frame_, latest_cloud_->header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(0.2));
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "TF lookup for obstacle cloud frame '%s' failed, skipping this cycle's obstacles: %s",
      latest_cloud_->header.frame_id.c_str(), e.what());
    return;
  }

  // Manual rigid transform (rotation matrix + translation) applied per
  // point, instead of tf2::doTransform per-point - one quaternion-to-
  // matrix conversion for the whole cloud, then plain float multiplies.
  const auto & tr = t.transform.translation;
  const auto & q = t.transform.rotation;
  double r00 = 1 - 2 * (q.y * q.y + q.z * q.z);
  double r01 = 2 * (q.x * q.y - q.z * q.w);
  double r02 = 2 * (q.x * q.z + q.y * q.w);
  double r10 = 2 * (q.x * q.y + q.z * q.w);
  double r11 = 1 - 2 * (q.x * q.x + q.z * q.z);
  double r12 = 2 * (q.y * q.z - q.x * q.w);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud_, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud_, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_cloud_, "z");

  double search_radius_sq = obstacle_search_radius_ * obstacle_search_radius_;
  double cell = resolution_;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    double lx = *iter_x, ly = *iter_y, lz = *iter_z;
    if (!std::isfinite(lx) || !std::isfinite(ly) || !std::isfinite(lz)) {
      continue;
    }
    // Height filter in the SENSOR's own frame, before transforming -
    // confirmed live the ground plane clusters around Z=-1.0 there
    // (sensor mounted ~1m up), not around Z=0.
    if (lz < min_obstacle_height_ || lz > max_obstacle_height_) {
      continue;
    }
    double wx = tr.x + r00 * lx + r01 * ly + r02 * lz;
    double wy = tr.y + r10 * lx + r11 * ly + r12 * lz;

    double dx = wx - pos.x, dy = wy - pos.y;
    if (dx * dx + dy * dy > search_radius_sq) {
      continue;
    }

    obstacle_points_.push_back({wx, wy});
    int cx = static_cast<int>(std::floor(wx / cell));
    int cy = static_cast<int>(std::floor(wy / cell));
    obstacle_grid_[gridKey(cx, cy)].push_back({wx, wy});
  }
}

bool CostmapLanePlanner::isBlockedWorld(double wx, double wy) const
{
  double cell = resolution_;
  int cx = static_cast<int>(std::floor(wx / cell));
  int cy = static_cast<int>(std::floor(wy / cell));
  double radius = obstacle_inflation_radius_ + obstacle_safety_margin_;
  double r2 = radius * radius;

  // Search however many cells out the radius actually reaches, not a
  // fixed +-1 - obstacle_safety_margin_ can easily exceed one cell width
  // (0.3m default), and a point just beyond the immediate neighbors would
  // otherwise be silently missed.
  int search_cells = std::max(1, static_cast<int>(std::ceil(radius / cell)));
  for (int dx = -search_cells; dx <= search_cells; ++dx) {
    for (int dy = -search_cells; dy <= search_cells; ++dy) {
      auto it = obstacle_grid_.find(gridKey(cx + dx, cy + dy));
      if (it == obstacle_grid_.end()) {
        continue;
      }
      for (const auto & p : it->second) {
        double ddx = p.x - wx, ddy = p.y - wy;
        if (ddx * ddx + ddy * ddy <= r2) {
          return true;
        }
      }
    }
  }
  return false;
}

double CostmapLanePlanner::lateralClearance(
  double x, double y, double theta, int sign, double max_search) const
{
  // Ported from _lateral_clearance: walk outward perpendicular to theta in
  // resolution_ steps, return the distance of the first unsafe cell (one
  // step short of it), or max_search if nothing along the way is unsafe.
  double step = resolution_;
  int n = static_cast<int>(max_search / step);
  if (n <= 0) {
    return max_search;
  }
  double sin_t = std::sin(theta);
  double cos_t = std::cos(theta);
  for (int i = 1; i <= n; ++i) {
    double d = i * step;
    double ex = x - sign * d * sin_t;
    double ey = y + sign * d * cos_t;
    if (isBlockedWorld(ex, ey)) {
      return d - step;
    }
  }
  return max_search;
}

CostmapLanePlanner::ArcResult CostmapLanePlanner::arcIsClear(
  const Vec2 & pos, double yaw, double curvature) const
{
  ArcResult result;
  result.curvature = curvature;
  double step = resolution_;
  int n_steps = static_cast<int>(lane_lookahead_ / step);
  double centering_error_sum = 0.0;
  int centering_samples = 0;

  for (int i = 1; i <= n_steps; ++i) {
    double s = i * step;
    double lx, ly, ltheta;
    if (std::abs(curvature) < 1e-4) {
      lx = s;
      ly = 0.0;
      ltheta = 0.0;
    } else {
      double r = 1.0 / curvature;
      ltheta = s * curvature;
      lx = r * std::sin(ltheta);
      ly = r * (1 - std::cos(ltheta));
    }
    double wx = pos.x + lx * std::cos(yaw) - ly * std::sin(yaw);
    double wy = pos.y + lx * std::sin(yaw) + ly * std::cos(yaw);
    double world_theta = yaw + ltheta;

    bool blocked = false;
    double sin_wt = std::sin(world_theta);
    double cos_wt = std::cos(world_theta);
    // Full footprint, not just width: checking only (0, +-clearance_) at
    // the reference point alone ignores the vehicle's 3.6m length
    // entirely - fine going straight (front/rear stay on the checked
    // line), but while turning the front and rear swing to a DIFFERENT
    // lateral offset than the reference point at the same arc-length
    // (vehicle off-tracking), so a curve could be approved here while the
    // real front/rear corner clips an obstacle this check never sampled.
    for (double along : {0.0, vehicle_half_length_, -vehicle_half_length_}) {
      double cx = wx + along * cos_wt;
      double cy = wy + along * sin_wt;
      for (double offset : {0.0, clearance_, -clearance_}) {
        double ex = cx - offset * sin_wt;
        double ey = cy + offset * cos_wt;
        if (isBlockedWorld(ex, ey)) {
          blocked = true;
          break;
        }
      }
      if (blocked) {
        break;
      }
    }
    if (blocked) {
      break;
    }
    result.clear_distance = s;
    result.points.push_back({wx, wy});

    if (i % 3 == 0) {
      double left = lateralClearance(wx, wy, world_theta, 1, corridor_scan_limit_);
      double right = lateralClearance(wx, wy, world_theta, -1, corridor_scan_limit_);
      centering_error_sum += std::abs(left - right);
      centering_samples += 1;
    }
  }

  result.centering_error = centering_samples ? centering_error_sum / centering_samples : 0.0;
  return result;
}

double CostmapLanePlanner::directionAndReferenceBonus(
  double curvature, double ex, double ey) const
{
  if (!enable_pose_graph_bias_) {
    return 0.0;
  }

  // Direction term: same shape as the Python original's
  // _direction_bias_bonus (+1 fully aligned with the commanded turn, -1
  // fully opposed, scaled by urgency as the turn gets closer), range
  // [-1, 1].
  double dir_term = 0.0;
  std::string dir = pose_graph_walker_->currentTurnDirection();
  double desired = (dir == "LEFT") ? 1.0 : (dir == "RIGHT" ? -1.0 : 0.0);
  if (desired != 0.0 && max_curvature_ > 0.0) {
    double turn_distance = pose_graph_walker_->turnDistanceM();
    double urgency = std::isnan(turn_distance)
      ? 1.0
      : std::clamp(1.0 - turn_distance / lane_lookahead_, 0.0, 1.0);
    double alignment = desired * (curvature / max_curvature_);
    dir_term = urgency * alignment;
  }

  // Reference-path term: same shape as _reference_path_bonus (negative
  // distance from this candidate's own endpoint to the recorded route, so
  // closer is better), in meters.
  double ref_term = 0.0;
  if (pose_graph_walker_->hasReferencePath()) {
    const auto & pts = pose_graph_walker_->referencePath();
    double min_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
      double ax = pts[i].x, ay = pts[i].y;
      double bx = pts[i + 1].x, by = pts[i + 1].y;
      double abx = bx - ax, aby = by - ay;
      double len_sq = abx * abx + aby * aby;
      double t = (len_sq > 1e-9)
        ? std::clamp(((ex - ax) * abx + (ey - ay) * aby) / len_sq, 0.0, 1.0)
        : 0.0;
      double cx = ax + t * abx, cy = ay + t * aby;
      min_dist = std::min(min_dist, std::hypot(cx - ex, cy - ey));
    }
    if (std::isfinite(min_dist)) {
      ref_term = -min_dist;
    }
  }

  // Combined into ONE dominant weight (direction_reference_gain_) instead
  // of the Python original's two separately-modest gains (1.5 each) - per
  // the "very strong bias, not absolute" design: this only re-ranks
  // candidates the safety filter in planCycle() already judged
  // comparably clear, it never removes an arc from candidacy.
  return direction_reference_gain_ * (dir_term + ref_term);
}

void CostmapLanePlanner::publishTargetLateralPosition(double curvature)
{
  double ly;
  if (std::abs(curvature) < 1e-4) {
    ly = 0.0;
  } else {
    double r = 1.0 / curvature;
    ly = r * (1 - std::cos(steering_lookahead_distance_ * curvature));
  }
  std_msgs::msg::Float32 msg;
  msg.data = static_cast<float>(ly);
  target_lateral_pub_->publish(msg);
}

void CostmapLanePlanner::publishLaneMarker(
  const rclcpp::Time & stamp, const Vec2 & pos, const ArcResult & chosen)
{
  visualization_msgs::msg::Marker lane;
  lane.header.frame_id = fixed_frame_;
  lane.header.stamp = stamp;
  lane.ns = "suggested_lane";
  lane.id = 0;
  lane.type = visualization_msgs::msg::Marker::LINE_STRIP;
  lane.action = visualization_msgs::msg::Marker::ADD;
  lane.pose.orientation.w = 1.0;
  lane.scale.x = 0.25;
  lane.color.r = 1.0;
  lane.color.g = 0.85;
  lane.color.b = 0.0;
  lane.color.a = 0.95;

  geometry_msgs::msg::Point origin;
  origin.x = pos.x;
  origin.y = pos.y;
  origin.z = 0.15;
  lane.points.push_back(origin);
  for (const auto & pt : chosen.points) {
    geometry_msgs::msg::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.15;
    lane.points.push_back(p);
  }
  lane_marker_pub_->publish(lane);
}

void CostmapLanePlanner::publishObstacleAreaMarker(const rclcpp::Time & stamp)
{
  // "Obstacle surrounded area" - every currently-indexed obstacle point
  // (already bounded to obstacle_search_radius_ around the vehicle),
  // drawn at (obstacle_inflation_radius_ + obstacle_safety_margin_) size -
  // the exact same combined radius isBlockedWorld() checks against, so
  // what's shown is what's really enforced, not a separately-eyeballed
  // visualization.
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = fixed_frame_;
  marker.header.stamp = stamp;
  marker.ns = "obstacle_area";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = obstacle_points_.empty()
    ? visualization_msgs::msg::Marker::DELETE
    : visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  double d = 2.0 * (obstacle_inflation_radius_ + obstacle_safety_margin_);
  marker.scale.x = d;
  marker.scale.y = d;
  marker.scale.z = 0.2;
  marker.color.r = 0.9;
  marker.color.g = 0.15;
  marker.color.b = 0.1;
  // Lower than before (0.5 -> 0.35): cubes are now larger and heavily
  // overlapping along dense obstacle surfaces (walls), so a solid alpha
  // here would just paint a single opaque red mass - the softer halo
  // still reads clearly as "occupied + safety margin" without hiding
  // the underlying wall/point structure in RViz.
  marker.color.a = 0.35;
  marker.points.reserve(obstacle_points_.size());
  for (const auto & p : obstacle_points_) {
    geometry_msgs::msg::Point gp;
    gp.x = p.x;
    gp.y = p.y;
    gp.z = 0.1;
    marker.points.push_back(gp);
  }
  obstacle_area_pub_->publish(marker);
}

void CostmapLanePlanner::publishFreeLaneAreaMarker(
  const rclcpp::Time & stamp, const Vec2 & pos, const ArcResult & chosen)
{
  // "Free area lane to navigate" - the chosen path's actual drivable
  // WIDTH (clearance_ either side), not just its centerline, rendered as
  // a filled semi-transparent green strip so the real navigable area
  // ahead is directly visible, matching the same clearance_ the arc
  // planner's own footprint check enforces.
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = fixed_frame_;
  marker.header.stamp = stamp;
  marker.ns = "free_lane_area";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;

  std::vector<Vec2> centerline;
  centerline.push_back(pos);
  centerline.insert(centerline.end(), chosen.points.begin(), chosen.points.end());

  if (centerline.size() < 2) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    free_lane_area_pub_->publish(marker);
    return;
  }

  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.1;
  marker.color.g = 0.9;
  marker.color.b = 0.2;
  marker.color.a = 0.35;

  auto perp_offset = [&](size_t i) -> std::pair<double, double> {
      double dx, dy;
      if (i + 1 < centerline.size()) {
        dx = centerline[i + 1].x - centerline[i].x;
        dy = centerline[i + 1].y - centerline[i].y;
      } else {
        dx = centerline[i].x - centerline[i - 1].x;
        dy = centerline[i].y - centerline[i - 1].y;
      }
      double len = std::hypot(dx, dy);
      if (len < 1e-6) {
        return {0.0, 0.0};
      }
      // Perpendicular unit vector, scaled to clearance_.
      return {-dy / len * clearance_, dx / len * clearance_};
    };

  std::vector<Vec2> left(centerline.size()), right(centerline.size());
  for (size_t i = 0; i < centerline.size(); ++i) {
    auto [ox, oy] = perp_offset(i);
    left[i] = {centerline[i].x + ox, centerline[i].y + oy};
    right[i] = {centerline[i].x - ox, centerline[i].y - oy};
  }

  auto add_point = [&](const Vec2 & p) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = 0.05;
      marker.points.push_back(gp);
    };

  for (size_t i = 0; i + 1 < centerline.size(); ++i) {
    // Quad (left[i], right[i], right[i+1], left[i+1]) as two triangles.
    add_point(left[i]);
    add_point(right[i]);
    add_point(right[i + 1]);

    add_point(left[i]);
    add_point(right[i + 1]);
    add_point(left[i + 1]);
  }

  free_lane_area_pub_->publish(marker);
}

void CostmapLanePlanner::planCycle()
{
  // ONE TF snapshot this cycle, shared by the pose-graph walker and the
  // arc planner below - see this class's own header comment for why.
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform(
      fixed_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.2));
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "TF lookup failed, skipping cycle: %s", e.what());
    nav_status_pub_->publish(std_msgs::msg::String().set__data("STUCK"));
    return;
  }
  Vec2 pos{t.transform.translation.x, t.transform.translation.y};
  const auto & q = t.transform.rotation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  rclcpp::Time stamp = get_clock()->now();

  if (start_stop_origin_ && start_position_.has_value() && !lap_complete_) {
    double d = std::hypot(pos.x - start_position_->x, pos.y - start_position_->y);
    if (!has_left_origin_) {
      // Don't even start checking for "back at origin" until the vehicle
      // has actually gone out and driven the loop - otherwise the very
      // first cycle after 'g' (still sitting on top of start_position_)
      // would latch lap_complete_ immediately.
      if (d >= origin_min_travel_m_) {
        has_left_origin_ = true;
      }
    } else if (d <= origin_stop_radius_m_) {
      lap_complete_ = true;
      RCLCPP_INFO(
        get_logger(), "Lap complete: back within %.2fm of the start pose-graph vertex - "
        "holding at origin (start_stop_origin=true).", origin_stop_radius_m_);
    }
  }
  lap_complete_pub_->publish(std_msgs::msg::Bool().set__data(lap_complete_));

  rebuildObstacleGrid(pos);
  publishObstacleAreaMarker(stamp);

  if (enable_pose_graph_bias_) {
    pose_graph_walker_->tick(pos.x, pos.y, yaw, stamp);
  }

  std::vector<double> curvatures(lane_num_candidates_);
  for (int i = 0; i < lane_num_candidates_; ++i) {
    double t_ = (lane_num_candidates_ > 1)
      ? static_cast<double>(i) / (lane_num_candidates_ - 1)
      : 0.0;
    curvatures[i] = -max_curvature_ + t_ * (2.0 * max_curvature_);
  }

  std::vector<ArcResult> results;
  results.reserve(curvatures.size());
  double max_distance = 0.0;
  for (double k : curvatures) {
    ArcResult r = arcIsClear(pos, yaw, k);
    max_distance = std::max(max_distance, r.clear_distance);
    results.push_back(std::move(r));
  }

  if (max_distance < resolution_) {
    smoothed_curvature_ = 0.0;
    nav_status_pub_->publish(std_msgs::msg::String().set__data("STUCK"));
    return;
  }

  double threshold = max_distance * lane_straight_preference_ratio_;
  std::vector<const ArcResult *> good_enough;
  for (const auto & r : results) {
    if (r.clear_distance >= threshold) {
      good_enough.push_back(&r);
    }
  }

  auto score = [&](const ArcResult * r) -> double {
      double ex = r->points.empty() ? pos.x : r->points.back().x;
      double ey = r->points.empty() ? pos.y : r->points.back().y;
      double bias = directionAndReferenceBonus(r->curvature, ex, ey);
      return r->centering_error - bias;
    };

  const ArcResult * chosen = good_enough.front();
  double best_score = score(chosen);
  for (const ArcResult * r : good_enough) {
    double s = score(r);
    if (s < best_score || (s == best_score && std::abs(r->curvature) < std::abs(chosen->curvature))) {
      best_score = s;
      chosen = r;
    }
  }

  double chosen_k = chosen->curvature;
  double alpha = lane_smoothing_alpha_;
  double smoothed_k = alpha * chosen_k + (1 - alpha) * smoothed_curvature_;
  ArcResult final_result = arcIsClear(pos, yaw, smoothed_k);
  if (final_result.clear_distance < resolution_) {
    // The smoothed curvature isn't actually clear right now (e.g. a new
    // obstacle just appeared where the old heading was taking us) - snap
    // to the unsmoothed choice instead of stalling.
    smoothed_k = chosen_k;
    final_result = *chosen;
  }
  smoothed_curvature_ = smoothed_k;

  if (final_result.points.empty()) {
    nav_status_pub_->publish(std_msgs::msg::String().set__data("STUCK"));
    return;
  }

  publishTargetLateralPosition(smoothed_k);
  publishLaneMarker(stamp, pos, final_result);
  publishFreeLaneAreaMarker(stamp, pos, final_result);
  nav_status_pub_->publish(std_msgs::msg::String().set__data("OK"));
}

}  // namespace buggy_nav_costmap
