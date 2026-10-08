#include "buggy_nav_costmap/pose_graph_walker.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <limits>

namespace buggy_nav_costmap
{

namespace
{
double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double median(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  size_t n = v.size();
  if (n == 0) {
    return 0.0;
  }
  if (n % 2 == 1) {
    return v[n / 2];
  }
  return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}
}  // namespace

PoseGraphWalker::PoseGraphWalker(
  rclcpp::Node * node, tf2_ros::Buffer * tf_buffer, const Params & params)
: node_(node), tf_buffer_(tf_buffer), p_(params), logger_(node->get_logger())
{
  direction_pub_ = node_->create_publisher<std_msgs::msg::String>("/turn_direction", 1);
  distance_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/turn_distance_m", 1);
  text_marker_pub_ =
    node_->create_publisher<visualization_msgs::msg::Marker>("/turn_direction_marker", 1);
  path_marker_pub_ =
    node_->create_publisher<visualization_msgs::msg::Marker>("/turn_direction_path", 1);
  nearest_vertex_pub_ =
    node_->create_publisher<visualization_msgs::msg::Marker>("/nearest_pose_graph_vertex", 1);
  nearest_link_pub_ =
    node_->create_publisher<visualization_msgs::msg::Marker>("/nearest_pose_graph_link", 1);
  nearest_text_pub_ =
    node_->create_publisher<visualization_msgs::msg::Marker>("/nearest_pose_graph_distance", 1);
  nearest_distance_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
    "/pose_graph_nearest_vertex_distance", 1);
  route_preview_pub_ =
    node_->create_publisher<visualization_msgs::msg::Marker>("/route_preview_path", 1);
  search_radius_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "/nearest_pose_graph_search_radius", 1);

  last_turn_signal_time_ = node_->get_clock()->now();
  last_reference_path_time_ = node_->get_clock()->now();
}

void PoseGraphWalker::poseGraphCallback(
  const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  // See this method's Python original (_pose_graph_callback) for why
  // freezing on the FIRST callback is fragile: slam_toolbox's LOCALIZATION
  // mode keeps appending the robot's own live trail onto this same array,
  // and if the freeze locks in before the initial full graph has finished
  // publishing, the nearest-vertex search is permanently stuck on a tiny
  // truncated graph. Only freeze once the vertex count has been unchanged
  // for pose_graph_stabilize_count consecutive callbacks.
  std::vector<Vec2> vertices;
  vertices.reserve(msg->markers.size());
  for (const auto & m : msg->markers) {
    if (m.ns == "slam_toolbox") {
      vertices.push_back({m.pose.position.x, m.pose.position.y});
    }
  }
  if (vertices.empty()) {
    return;
  }
  size_t n = vertices.size();

  if (!original_vertex_count_.has_value()) {
    if (pose_graph_last_len_ == n) {
      pose_graph_stable_count_ += 1;
    } else {
      pose_graph_stable_count_ = 1;
    }
    pose_graph_last_len_ = n;
    if (pose_graph_stable_count_ >= p_.pose_graph_stabilize_count) {
      original_vertex_count_ = n;
      RCLCPP_INFO(
        logger_, "Pose graph vertex count stabilized at %zu - freezing here "
        "(excludes any live trail appended from this point on).", n);
    }
    pose_graph_vertices_map_ = vertices;
  } else {
    size_t cap = std::min(n, *original_vertex_count_);
    pose_graph_vertices_map_.assign(vertices.begin(), vertices.begin() + cap);
  }
  has_vertices_ = true;
}

std::optional<std::vector<Vec2>> PoseGraphWalker::verticesInFixedFrame()
{
  if (!has_vertices_ || !original_vertex_count_.has_value()) {
    return std::nullopt;
  }
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform(
      p_.fixed_frame, p_.map_frame, tf2::TimePointZero, tf2::durationFromSec(0.2));
  } catch (const std::exception &) {
    return std::nullopt;
  }
  double tx = t.transform.translation.x;
  double ty = t.transform.translation.y;
  const auto & q = t.transform.rotation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  double cos_y = std::cos(yaw);
  double sin_y = std::sin(yaw);

  size_t cap = std::min(pose_graph_vertices_map_.size(), *original_vertex_count_);
  std::vector<Vec2> out;
  out.reserve(cap);
  for (size_t i = 0; i < cap; ++i) {
    double vx = pose_graph_vertices_map_[i].x;
    double vy = pose_graph_vertices_map_[i].y;
    out.push_back({tx + vx * cos_y - vy * sin_y, ty + vx * sin_y + vy * cos_y});
  }
  return out;
}

std::optional<size_t> PoseGraphWalker::safeNearestIndex(const std::vector<double> & d)
{
  // np.argmin propagates NaN (returns the index of the FIRST NaN it hits)
  // rather than the true nearest finite value - confirmed live this
  // silently corrupted nearest-vertex picking on a momentarily degenerate
  // map->odom transform. Only consider finite entries.
  std::optional<size_t> best;
  double best_d = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < d.size(); ++i) {
    if (std::isfinite(d[i]) && d[i] < best_d) {
      best_d = d[i];
      best = i;
    }
  }
  return best;
}

std::optional<std::tuple<double, double, double>> PoseGraphWalker::nearestVertex(
  double x0, double y0)
{
  auto vertices = verticesInFixedFrame();
  if (!vertices.has_value() || vertices->empty()) {
    return std::nullopt;
  }
  std::vector<double> d(vertices->size());
  for (size_t i = 0; i < vertices->size(); ++i) {
    d[i] = std::hypot((*vertices)[i].x - x0, (*vertices)[i].y - y0);
  }
  auto idx = safeNearestIndex(d);
  if (!idx.has_value()) {
    return std::nullopt;
  }
  return std::make_tuple((*vertices)[*idx].x, (*vertices)[*idx].y, d[*idx]);
}

std::optional<std::vector<Vec2>> PoseGraphWalker::lookaheadPath(
  double x0, double y0, double current_yaw, double max_distance)
{
  auto vertices_opt = verticesInFixedFrame();
  if (!vertices_opt.has_value() || vertices_opt->size() < 2) {
    return std::nullopt;
  }
  const auto & vertices = *vertices_opt;
  size_t n = vertices.size();
  long nn = static_cast<long>(n);

  std::vector<double> d(n);
  for (size_t i = 0; i < n; ++i) {
    d[i] = std::hypot(vertices[i].x - x0, vertices[i].y - y0);
  }

  // For a closed-loop track, vertex 0 and the last vertex are spatially
  // close (confirmed live on my_map2: 1.1m apart) even though they sit at
  // opposite ends of the array - that array boundary is a recording
  // artifact, not a real gap in the physical track, so a walk reaching
  // one end continues from the other instead of stopping dead - AS LONG
  // AS the gap is actually small (loop_closure_max_gap_m), so a genuinely
  // open (non-loop) track never gets bridged by mistake. Returns nullopt
  // only when there's truly nowhere to go (both the direct neighbor and
  // the wrap-around target are unavailable/too far).
  auto next_index = [&](long idx, long step_dir) -> std::optional<long> {
      long nj = idx + step_dir;
      if (nj >= 0 && nj < nn) {
        return nj;
      }
      long wrapped = (nj < 0) ? nn - 1 : 0;
      double gap = std::hypot(
        vertices[wrapped].x - vertices[idx].x, vertices[wrapped].y - vertices[idx].y);
      if (gap <= p_.loop_closure_max_gap_m) {
        return wrapped;
      }
      return std::nullopt;
    };

  // Bearing from vertices[i] to whichever vertex is reached after walking
  // up to direction_disambiguation_vertices steps in step_dir - NOT just
  // the single immediate neighbor, which a lone out-of-order/noisy vertex
  // could otherwise flip.
  auto local_direction_yaw = [&](long i, long step_dir) -> std::optional<double> {
      long j = i;
      long last_valid = i;
      for (int s = 0; s < p_.direction_disambiguation_vertices; ++s) {
        auto nj = next_index(j, step_dir);
        if (!nj.has_value()) {
          break;
        }
        last_valid = *nj;
        j = *nj;
      }
      if (last_valid == i) {
        return std::nullopt;
      }
      double vx = vertices[last_valid].x - vertices[i].x;
      double vy = vertices[last_valid].y - vertices[i].y;
      if (vx == 0.0 && vy == 0.0) {
        return std::nullopt;
      }
      return std::atan2(vy, vx);
    };

  auto alignment = [&](const std::optional<double> & yaw) -> double {
      if (!yaw.has_value()) {
        return -2.0;  // worse than any real cos() value
      }
      double diff = normalizeAngle(*yaw - current_yaw);
      return std::cos(diff);
    };

  // Starting vertex: among every vertex within nearest_vertex_max_dist (not
  // just the single closest one), prefer whichever best continues the
  // vehicle's CURRENT heading, tie-broken by distance. Confirmed live +
  // offline (route-crossing fuzz test) that picking by raw distance alone
  // let a route crossing/loop near itself hijack the walk onto a
  // far-index vertex (e.g. the recorded route's own start) whenever that
  // vertex happened to be physically closer at the crossing instant than
  // the one actually being walked toward - "continuing the same path,
  // suddenly jumped to vertex 0."
  std::optional<long> idx0;
  double best_alignment = -3.0;  // worse than any real alignment() value (range is -2..1)
  double best_d = std::numeric_limits<double>::infinity();
  for (long i = 0; i < nn; ++i) {
    if (!std::isfinite(d[i]) || d[i] > p_.nearest_vertex_max_dist) {
      continue;
    }
    double a = std::max(alignment(local_direction_yaw(i, 1)), alignment(local_direction_yaw(i, -1)));
    if (a > best_alignment || (a == best_alignment && d[i] < best_d)) {
      best_alignment = a;
      best_d = d[i];
      idx0 = i;
    }
  }
  if (!idx0.has_value()) {
    return std::nullopt;
  }

  auto fwd_yaw = local_direction_yaw(*idx0, 1);
  auto bwd_yaw = local_direction_yaw(*idx0, -1);
  long step = (alignment(fwd_yaw) >= alignment(bwd_yaw)) ? 1 : -1;

  double x = vertices[*idx0].x;
  double y = vertices[*idx0].y;
  std::vector<Vec2> points;
  points.push_back({x0, y0});
  points.push_back({x, y});
  double traveled = 0.0;
  long i = *idx0;
  while (true) {
    auto next = next_index(i, step);
    if (!next.has_value()) {
      break;
    }
    long ni = *next;
    double nx = vertices[ni].x;
    double ny = vertices[ni].y;
    double seg = std::hypot(nx - x, ny - y);
    if (traveled + seg >= max_distance) {
      double t = (seg > 0.0) ? (max_distance - traveled) / seg : 0.0;
      points.push_back({x + t * (nx - x), y + t * (ny - y)});
      return points;
    }
    traveled += seg;
    x = nx;
    y = ny;
    points.push_back({x, y});
    i = ni;
  }
  // Either max_distance was reached, or the graph genuinely ran out (open
  // track and hit an end, or hit an end with too large a gap to be a real
  // loop closure) - path just ends early either way.
  return points;
}

std::optional<std::vector<Vec2>> PoseGraphWalker::robustLookaheadPath(
  double x0, double y0, double current_yaw, double max_distance,
  const std::string & history_name)
{
  // 3-deep median-jump-rejection with self-poisoning recovery - see this
  // method's Python original (_robust_lookahead_path) for the full
  // derivation. A naive "reject anything far from history" rule
  // self-poisons if the very first accepted reading is wrong (nothing can
  // be rejected yet, so it's trusted forever after); rejected_streak_
  // tracks rejections that are mutually consistent WITH EACH OTHER, and
  // resets history to trust them once pose_graph_reset_streak_threshold
  // consecutive rejections agree - verified offline before this was
  // applied to recover from a bad seed while never falsely triggering on
  // genuinely scattered noise.
  auto path = lookaheadPath(x0, y0, current_yaw, max_distance);
  rclcpp::Time now = node_->get_clock()->now();
  auto & history = walk_history_[history_name];
  auto & rejected = rejected_streak_[history_name];

  if (path.has_value() && path->size() >= 2) {
    double start_x = (*path)[0].x;
    double start_y = (*path)[0].y;
    bool jump_ok = true;
    if (!history.empty()) {
      std::vector<double> hx, hy;
      for (const auto & h : history) {
        hx.push_back(h.x);
        hy.push_back(h.y);
      }
      double med_x = median(hx);
      double med_y = median(hy);
      jump_ok = std::hypot(start_x - med_x, start_y - med_y) <= p_.route_preview_max_jump_m;
    }

    if (jump_ok) {
      if (history.size() >= p_.pose_graph_history_depth) {
        history.pop_front();
      }
      history.push_back({start_x, start_y, *path, now});
      rejected.clear();
      return path;
    }

    bool consistent = true;
    if (!rejected.empty()) {
      std::vector<double> rx, ry;
      for (const auto & r : rejected) {
        rx.push_back(r.x);
        ry.push_back(r.y);
      }
      double rej_x = median(rx);
      double rej_y = median(ry);
      consistent = std::hypot(start_x - rej_x, start_y - rej_y) <= p_.route_preview_max_jump_m;
    }
    if (!consistent) {
      rejected.clear();
    }
    if (rejected.size() >= p_.pose_graph_reset_streak_threshold) {
      rejected.pop_front();
    }
    rejected.push_back({start_x, start_y});

    if (rejected.size() >= p_.pose_graph_reset_streak_threshold) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 2000,
        "Pose-graph walk (%s): %zu consecutive mutually-consistent readings disagreed "
        "with existing history - trusting the new cluster instead (existing history was "
        "likely a bad seed).", history_name.c_str(), rejected.size());
      history.clear();
      history.push_back({start_x, start_y, *path, now});
      rejected.clear();
      return path;
    }

    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "Pose-graph walk (%s) start point jumped implausibly from recent history - "
      "rejecting as a likely bad nearest-vertex pick, reusing the last accepted path "
      "instead.", history_name.c_str());
  }

  if (!history.empty()) {
    const auto & last = history.back();
    double age = (now - last.time).seconds();
    if (age <= p_.route_preview_cache_timeout_sec) {
      return last.path;
    }
  }
  return std::nullopt;
}

double PoseGraphWalker::distanceToTurn(
  const std::vector<Vec2> & points, double current_yaw) const
{
  double x0 = points[0].x;
  double y0 = points[0].y;
  double cumulative = 0.0;
  double prev_x = points[0].x;
  double prev_y = points[0].y;
  for (size_t i = 1; i < points.size(); ++i) {
    double x = points[i].x;
    double y = points[i].y;
    cumulative += std::hypot(x - prev_x, y - prev_y);
    prev_x = x;
    prev_y = y;
    double bearing = std::atan2(y - y0, x - x0);
    double diff_deg = normalizeAngle(bearing - current_yaw) * 180.0 / M_PI;
    if (std::abs(diff_deg) >= p_.snap_deg) {
      return cumulative;
    }
  }
  return p_.turn_lookahead_m;
}

void PoseGraphWalker::tick(double x0, double y0, double current_yaw, const rclcpp::Time & stamp)
{
  publishNearestVertexSearchRadius(stamp, x0, y0);
  publishNearestVertexMarker(stamp, x0, y0, nearestVertex(x0, y0));

  auto path = robustLookaheadPath(x0, y0, current_yaw, p_.turn_lookahead_m, "turn");
  if (!path.has_value()) {
    turn_direction_ = "STRAIGHT";
    turn_distance_m_ = std::numeric_limits<double>::quiet_NaN();
    last_turn_signal_time_ = node_->get_clock()->now();
    has_turn_signal_ = true;
    direction_pub_->publish(std_msgs::msg::String().set__data("STRAIGHT"));
    distance_pub_->publish(std_msgs::msg::Float32().set__data(
        std::numeric_limits<float>::quiet_NaN()));
    publishTurnDirectionTextMarker(
      stamp, x0, y0, "STRAIGHT", {1.0, 1.0, 1.0, 0.9},
      std::numeric_limits<double>::quiet_NaN());
    publishTurnDirectionPathMarker(stamp, nullptr, {1.0, 1.0, 1.0, 0.9});
    reference_path_points_.clear();
    has_reference_path_ = false;
    publishRoutePreviewMarker(stamp, nullptr);
    return;
  }

  double far_x = path->back().x;
  double far_y = path->back().y;
  double reference_yaw = std::atan2(far_y - y0, far_x - x0);
  double diff_deg = normalizeAngle(reference_yaw - current_yaw) * 180.0 / M_PI;

  std::string direction;
  std::array<double, 4> color;
  if (std::abs(diff_deg) < p_.snap_deg) {
    direction = "STRAIGHT";
    color = {1.0, 1.0, 1.0, 0.9};
  } else if (diff_deg > 0) {
    direction = "LEFT";
    color = {0.2, 0.4, 1.0, 0.95};
  } else {
    direction = "RIGHT";
    color = {1.0, 0.2, 0.2, 0.95};
  }

  double turn_distance = distanceToTurn(*path, current_yaw);

  turn_direction_ = direction;
  turn_distance_m_ = turn_distance;
  last_turn_signal_time_ = node_->get_clock()->now();
  has_turn_signal_ = true;
  direction_pub_->publish(std_msgs::msg::String().set__data(direction));
  distance_pub_->publish(std_msgs::msg::Float32().set__data(static_cast<float>(turn_distance)));
  publishTurnDirectionTextMarker(stamp, x0, y0, direction, color, turn_distance);
  publishTurnDirectionPathMarker(stamp, &(*path), color);

  auto preview_path = robustLookaheadPath(
    x0, y0, current_yaw, p_.route_preview_distance_m, "preview");
  if (preview_path.has_value()) {
    reference_path_points_ = *preview_path;
    has_reference_path_ = true;
    last_reference_path_time_ = node_->get_clock()->now();
    publishRoutePreviewMarker(stamp, &(*preview_path));
  } else {
    reference_path_points_.clear();
    has_reference_path_ = false;
    publishRoutePreviewMarker(stamp, nullptr);
  }
}

std::string PoseGraphWalker::currentTurnDirection() const
{
  if (!has_turn_signal_) {
    return "STRAIGHT";
  }
  double age = (node_->get_clock()->now() - last_turn_signal_time_).seconds();
  if (age > p_.turn_signal_timeout_sec) {
    return "STRAIGHT";
  }
  return turn_direction_;
}

bool PoseGraphWalker::hasReferencePath() const
{
  if (!has_reference_path_ || reference_path_points_.size() < 2) {
    return false;
  }
  double age = (node_->get_clock()->now() - last_reference_path_time_).seconds();
  return age <= p_.turn_signal_timeout_sec;
}

void PoseGraphWalker::publishNearestVertexSearchRadius(
  const rclcpp::Time & stamp, double x0, double y0)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = p_.fixed_frame;
  marker.header.stamp = stamp;
  marker.ns = "nearest_vertex_search_radius";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.color.r = 0.2;
  marker.color.g = 0.6;
  marker.color.b = 1.0;
  marker.color.a = 0.6;
  const int n_pts = 48;
  for (int i = 0; i <= n_pts; ++i) {
    double theta = 2.0 * M_PI * i / n_pts;
    geometry_msgs::msg::Point p;
    p.x = x0 + p_.nearest_vertex_max_dist * std::cos(theta);
    p.y = y0 + p_.nearest_vertex_max_dist * std::sin(theta);
    p.z = 0.05;
    marker.points.push_back(p);
  }
  search_radius_pub_->publish(marker);
}

void PoseGraphWalker::publishNearestVertexMarker(
  const rclcpp::Time & stamp, double x0, double y0,
  const std::optional<std::tuple<double, double, double>> & nearest)
{
  bool trusted = nearest.has_value() && std::get<2>(*nearest) <= p_.nearest_vertex_max_dist;
  std::array<double, 4> color =
    nearest.has_value() ? (trusted ? std::array<double, 4>{0.1, 0.9, 0.2, 0.9}
                                    : std::array<double, 4>{1.0, 0.6, 0.0, 0.9})
                         : std::array<double, 4>{0.5, 0.5, 0.5, 0.4};

  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = p_.fixed_frame;
  sphere.header.stamp = stamp;
  sphere.ns = "nearest_pose_graph_vertex";
  sphere.id = 0;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.pose.orientation.w = 1.0;
  sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.6;
  sphere.color.r = color[0];
  sphere.color.g = color[1];
  sphere.color.b = color[2];
  sphere.color.a = color[3];

  visualization_msgs::msg::Marker link;
  link.header.frame_id = p_.fixed_frame;
  link.header.stamp = stamp;
  link.ns = "nearest_pose_graph_link";
  link.id = 0;
  link.type = visualization_msgs::msg::Marker::LINE_STRIP;
  link.pose.orientation.w = 1.0;
  link.scale.x = 0.08;
  link.color.r = color[0];
  link.color.g = color[1];
  link.color.b = color[2];
  link.color.a = color[3];

  visualization_msgs::msg::Marker text;
  text.header.frame_id = p_.fixed_frame;
  text.header.stamp = stamp;
  text.ns = "nearest_pose_graph_distance";
  text.id = 0;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.pose.orientation.w = 1.0;
  text.scale.z = 0.4;
  text.color.r = color[0];
  text.color.g = color[1];
  text.color.b = color[2];
  text.color.a = color[3];

  std_msgs::msg::Float32 dist_msg;
  if (nearest.has_value()) {
    auto [vx, vy, dist] = *nearest;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = vx;
    sphere.pose.position.y = vy;
    sphere.pose.position.z = 0.3;

    link.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point p0, p1;
    p0.x = x0;
    p0.y = y0;
    p0.z = 0.1;
    p1.x = vx;
    p1.y = vy;
    p1.z = 0.1;
    link.points = {p0, p1};

    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = (x0 + vx) / 2.0;
    text.pose.position.y = (y0 + vy) / 2.0;
    text.pose.position.z = 0.8;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2fm", dist);
    text.text = buf;
    dist_msg.data = static_cast<float>(dist);
  } else {
    sphere.action = visualization_msgs::msg::Marker::DELETE;
    link.action = visualization_msgs::msg::Marker::DELETE;
    text.action = visualization_msgs::msg::Marker::DELETE;
    dist_msg.data = std::numeric_limits<float>::quiet_NaN();
  }

  nearest_vertex_pub_->publish(sphere);
  nearest_link_pub_->publish(link);
  nearest_text_pub_->publish(text);
  nearest_distance_pub_->publish(dist_msg);
}

void PoseGraphWalker::publishTurnDirectionTextMarker(
  const rclcpp::Time & stamp, double x, double y, const std::string & direction,
  const std::array<double, 4> & color, double turn_distance)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = p_.fixed_frame;
  marker.header.stamp = stamp;
  marker.ns = "turn_direction";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 1.8;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.6;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  if (std::isnan(turn_distance)) {
    marker.text = direction;
  } else {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s (%.1fm)", direction.c_str(), turn_distance);
    marker.text = buf;
  }
  text_marker_pub_->publish(marker);
}

void PoseGraphWalker::publishTurnDirectionPathMarker(
  const rclcpp::Time & stamp, const std::vector<Vec2> * points,
  const std::array<double, 4> & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = p_.fixed_frame;
  marker.header.stamp = stamp;
  marker.ns = "turn_direction_path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = (points != nullptr && !points->empty())
    ? visualization_msgs::msg::Marker::ADD
    : visualization_msgs::msg::Marker::DELETE;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  if (points != nullptr) {
    for (const auto & pt : *points) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = 0.2;
      marker.points.push_back(p);
    }
  }
  path_marker_pub_->publish(marker);
}

void PoseGraphWalker::publishRoutePreviewMarker(
  const rclcpp::Time & stamp, const std::vector<Vec2> * points)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = p_.fixed_frame;
  marker.header.stamp = stamp;
  marker.ns = "route_preview_path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = (points != nullptr && !points->empty())
    ? visualization_msgs::msg::Marker::ADD
    : visualization_msgs::msg::Marker::DELETE;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.15;
  marker.color.r = 0.1;
  marker.color.g = 0.9;
  marker.color.b = 0.2;
  marker.color.a = 0.85;
  if (points != nullptr) {
    for (const auto & pt : *points) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = 0.1;
      marker.points.push_back(p);
    }
  }
  route_preview_pub_->publish(marker);
}

}  // namespace buggy_nav_costmap
