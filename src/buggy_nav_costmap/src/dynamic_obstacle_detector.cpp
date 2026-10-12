#include "buggy_nav_costmap/dynamic_obstacle_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <unordered_set>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace buggy_nav_costmap
{

DynamicObstacleDetector::DynamicObstacleDetector()
: rclcpp::Node("dynamic_obstacle_detector")
{
  fixed_frame_ = declare_parameter("fixed_frame", std::string("odom"));
  base_frame_ = declare_parameter("base_frame", std::string("base_link"));
  obstacle_topic_ = declare_parameter(
    "obstacle_topic", std::string("/non_ground_points_filtered"));
  cell_size_ = declare_parameter("cell_size", 0.3);
  // Number of past scans (including the current one) merged for the
  // density comparison - matches the paper's own tuned m=6. More history
  // = more confident static classification but slower to recognize a
  // just-arrived moving object as no-longer-corroborated; see this node's
  // own header comment for the mechanism.
  history_length_ = declare_parameter("history_length", 6);
  obstacle_search_radius_ = declare_parameter("obstacle_search_radius", 30.0);
  moving_threshold_ = declare_parameter("moving_threshold", 0.3);
  min_moving_blob_points_ = declare_parameter("min_moving_blob_points", 15);
  max_moving_blob_extent_m_ = declare_parameter("max_moving_blob_extent_m", 2.5);
  moving_color_r_ = declare_parameter("moving_color_r", 0);
  moving_color_g_ = declare_parameter("moving_color_g", 0);
  moving_color_b_ = declare_parameter("moving_color_b", 255);
  process_rate_hz_ = declare_parameter("process_rate_hz", 10.0);
  max_track_jump_m_ = declare_parameter("max_track_jump_m", 2.0);
  max_missed_cycles_ = declare_parameter("max_missed_cycles", 5);
  prediction_horizon_sec_ = declare_parameter("prediction_horizon_sec", 2.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    obstacle_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      cloudCallback(msg);
    });

  // Plain reliable QoS (not SensorDataQoS/best-effort) - this is a
  // low-rate derived result, not a raw high-rate sensor feed, and
  // RViz's PointCloud2 display defaults to a Reliable subscription. A
  // best-effort publisher there is a classic "topic has real data but
  // RViz silently shows nothing" QoS-incompatibility trap - confirmed
  // live this session.
  moving_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/moving_obstacle_points", 1);
  static_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/static_obstacle_points", 1);
  tracking_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/moving_object_tracking", 1);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / process_rate_hz_),
    std::bind(&DynamicObstacleDetector::processCycle, this));

  RCLCPP_INFO(
    get_logger(),
    "dynamic_obstacle_detector: density-ratio classification of %s in %s "
    "(cell=%.2fm, history_length=%d, moving<=%.2f, min_blob_points=%d, "
    "max_blob_extent=%.2fm) -> /moving_obstacle_points (color=%d,%d,%d)",
    obstacle_topic_.c_str(), fixed_frame_.c_str(), cell_size_, history_length_,
    moving_threshold_, min_moving_blob_points_, max_moving_blob_extent_m_,
    moving_color_r_, moving_color_g_, moving_color_b_);
}

void DynamicObstacleDetector::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_cloud_ = msg;
}

int64_t DynamicObstacleDetector::cellKey(double x, double y) const
{
  int cx = static_cast<int>(std::floor(x / cell_size_));
  int cy = static_cast<int>(std::floor(y / cell_size_));
  return (static_cast<int64_t>(cx) << 32) ^ (static_cast<uint32_t>(cy));
}

void DynamicObstacleDetector::cellKeyToCoords(int64_t key, int & cx, int & cy) const
{
  cx = static_cast<int>(key >> 32);
  cy = static_cast<int>(static_cast<uint32_t>(key));
}

int DynamicObstacleDetector::coverageCount(int64_t key) const
{
  int cx, cy;
  cellKeyToCoords(key, cx, cy);
  double cell_wx = (cx + 0.5) * cell_size_;
  double cell_wy = (cy + 0.5) * cell_size_;
  double radius_sq = obstacle_search_radius_ * obstacle_search_radius_;

  int count = 0;
  for (const auto & entry : history_buffer_) {
    if (!entry.have_vpos) {
      // No vehicle-position record for that past scan (TF lookup failed
      // that cycle) - can't prove it was out of range, so don't penalize
      // the cell for it either; same fail-open behavior
      // transformLatestCloud itself uses when the vehicle position is
      // unavailable.
      count += 1;
      continue;
    }
    double dx = cell_wx - entry.vpos_x, dy = cell_wy - entry.vpos_y;
    if (dx * dx + dy * dy <= radius_sq) {
      count += 1;
    }
  }
  return count;
}

std::unordered_set<int64_t> DynamicObstacleDetector::filterSmallBlobs(
  const std::unordered_set<int64_t> & moving_cells,
  const std::unordered_map<int64_t, CellAgg> & current_grid) const
{
  std::unordered_set<int64_t> kept;
  std::unordered_set<int64_t> visited;
  visited.reserve(moving_cells.size());

  // Connected-component labelling (8-connectivity) over JUST the cells
  // already classified moving this cycle - same grid-hash flood-fill
  // idiom used elsewhere in this package, here purely as a one-shot
  // per-cycle size filter (nothing persists across cycles, no per-blob
  // identity/tracking).
  for (const auto & start_key : moving_cells) {
    if (visited.count(start_key)) {
      continue;
    }
    std::deque<int64_t> queue;
    queue.push_back(start_key);
    visited.insert(start_key);
    std::vector<int64_t> blob_cells;
    int total_points = 0;
    int min_cx = std::numeric_limits<int>::max(), max_cx = std::numeric_limits<int>::min();
    int min_cy = std::numeric_limits<int>::max(), max_cy = std::numeric_limits<int>::min();

    while (!queue.empty()) {
      int64_t key = queue.front();
      queue.pop_front();
      blob_cells.push_back(key);
      auto it = current_grid.find(key);
      if (it != current_grid.end()) {
        total_points += it->second.count;
      }

      int cx, cy;
      cellKeyToCoords(key, cx, cy);
      min_cx = std::min(min_cx, cx); max_cx = std::max(max_cx, cx);
      min_cy = std::min(min_cy, cy); max_cy = std::max(max_cy, cy);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          int64_t nkey =
            (static_cast<int64_t>(cx + dx) << 32) ^ (static_cast<uint32_t>(cy + dy));
          if (moving_cells.count(nkey) && !visited.count(nkey)) {
            visited.insert(nkey);
            queue.push_back(nkey);
          }
        }
      }
    }

    // Longest side of the blob's bounding box, in meters - a newly-
    // revealed WALL SEGMENT (occluded until the vehicle drove far enough
    // to see around whatever was blocking it - confirmed live this is
    // the dominant source of large moving-classified blobs during
    // continuous forward driving, since coverageCount()'s radius-based
    // model has no actual line-of-sight/occlusion awareness) is
    // spatially elongated; a real moving object this project cares about
    // (person, vehicle) is comparatively compact. Rejecting oversized
    // blobs here is a pragmatic shape heuristic, not a perfect
    // classifier - a genuinely wide moving obstacle could be excluded
    // too - but it directly targets the specific, confirmed-live failure
    // mode without needing real raycasting-based visibility history.
    double extent_m = std::max(max_cx - min_cx, max_cy - min_cy) * cell_size_;

    if (total_points >= min_moving_blob_points_ && extent_m <= max_moving_blob_extent_m_) {
      for (const auto & k : blob_cells) {
        kept.insert(k);
      }
    }
  }
  return kept;
}

bool DynamicObstacleDetector::transformLatestCloud(
  std::vector<Point3> & out, double & vpos_x, double & vpos_y, bool & have_vpos)
{
  out.clear();
  have_vpos = false;
  if (!latest_cloud_) {
    return false;
  }

  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform(
      fixed_frame_, latest_cloud_->header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(0.2));
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "TF lookup for obstacle cloud frame '%s' failed, skipping this cycle: %s",
      latest_cloud_->header.frame_id.c_str(), e.what());
    return false;
  }

  try {
    auto vt = tf_buffer_->lookupTransform(
      fixed_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.2));
    vpos_x = vt.transform.translation.x;
    vpos_y = vt.transform.translation.y;
    have_vpos = true;
  } catch (const std::exception &) {
    // Fine - process everything, unbounded, this cycle.
  }

  const auto & tr = t.transform.translation;
  const auto & q = t.transform.rotation;
  double r00 = 1 - 2 * (q.y * q.y + q.z * q.z);
  double r01 = 2 * (q.x * q.y - q.z * q.w);
  double r02 = 2 * (q.x * q.z + q.y * q.w);
  double r10 = 2 * (q.x * q.y + q.z * q.w);
  double r11 = 1 - 2 * (q.x * q.x + q.z * q.z);
  double r12 = 2 * (q.y * q.z - q.x * q.w);
  double r20 = 2 * (q.x * q.z - q.y * q.w);
  double r21 = 2 * (q.y * q.z + q.x * q.w);
  double r22 = 1 - 2 * (q.x * q.x + q.y * q.y);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud_, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud_, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_cloud_, "z");

  double search_radius_sq = obstacle_search_radius_ * obstacle_search_radius_;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    double lx = *iter_x, ly = *iter_y, lz = *iter_z;
    if (!std::isfinite(lx) || !std::isfinite(ly) || !std::isfinite(lz)) {
      continue;
    }
    double wx = tr.x + r00 * lx + r01 * ly + r02 * lz;
    double wy = tr.y + r10 * lx + r11 * ly + r12 * lz;
    double wz = tr.z + r20 * lx + r21 * ly + r22 * lz;

    if (have_vpos) {
      double dx = wx - vpos_x, dy = wy - vpos_y;
      if (dx * dx + dy * dy > search_radius_sq) {
        continue;
      }
    }
    out.push_back({wx, wy, wz});
  }
  return true;
}

void DynamicObstacleDetector::publishMovingPoints(
  const rclcpp::Time & stamp, const std::vector<Point3> & moving_points)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = fixed_frame_;
  cloud_msg.header.stamp = stamp;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(moving_points.size());

  // Packed-float32 "rgb" field, the standard PCL/RViz XYZRGB convention
  // (RViz's PointCloud2 display picks this up automatically under the
  // "RGB8" color transformer) - a single flat tint applied to every
  // point, not intensity-mapped, so moving points read as visually
  // distinct regardless of whatever intensity data (if any) the original
  // sensor points carried.
  uint32_t packed_rgb =
    (static_cast<uint32_t>(moving_color_r_) << 16) |
    (static_cast<uint32_t>(moving_color_g_) << 8) |
    static_cast<uint32_t>(moving_color_b_);
  float rgb_float;
  std::memcpy(&rgb_float, &packed_rgb, sizeof(float));

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_rgb(cloud_msg, "rgb");
  for (const auto & p : moving_points) {
    *iter_x = static_cast<float>(p.x);
    *iter_y = static_cast<float>(p.y);
    *iter_z = static_cast<float>(p.z);
    *iter_rgb = rgb_float;
    ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
  }

  moving_points_pub_->publish(cloud_msg);
}

void DynamicObstacleDetector::publishStaticPoints(
  const rclcpp::Time & stamp, const std::vector<Point3> & static_points)
{
  // Plain XYZ (no rgb field) - this is costmap_lane_planner's obstacle
  // input when path_obstacle_monitor_node is enabled, not a visualization
  // aid, so it matches the same field layout obstacle_topic already
  // expects (no point in paying for an unused colour field on what can be
  // a large cloud).
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = fixed_frame_;
  cloud_msg.header.stamp = stamp;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(static_points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  for (const auto & p : static_points) {
    *iter_x = static_cast<float>(p.x);
    *iter_y = static_cast<float>(p.y);
    *iter_z = static_cast<float>(p.z);
    ++iter_x; ++iter_y; ++iter_z;
  }

  static_points_pub_->publish(cloud_msg);
}

void DynamicObstacleDetector::processCycle()
{
  rclcpp::Time stamp = get_clock()->now();
  if (has_last_cycle_time_ && (stamp - last_cycle_time_).seconds() <= 0.0) {
    // Sim time hasn't actually advanced since the last cycle (see this
    // project's known-degraded Gazebo real-time factor) - nothing new to
    // process until it does. Confirmed necessary live in the earlier,
    // tracking-based version of this node; kept here defensively even
    // though this node has no per-cycle state that a no-op cycle could
    // corrupt the way track ages/spawns could.
    return;
  }
  last_cycle_time_ = stamp;
  has_last_cycle_time_ = true;

  std::vector<Point3> current_points;
  double vpos_x = 0.0, vpos_y = 0.0;
  bool have_vpos = false;
  if (!transformLatestCloud(current_points, vpos_x, vpos_y, have_vpos)) {
    return;
  }

  HistoryEntry entry;
  entry.points = current_points;
  entry.vpos_x = vpos_x;
  entry.vpos_y = vpos_y;
  entry.have_vpos = have_vpos;
  history_buffer_.push_back(std::move(entry));
  while (static_cast<int>(history_buffer_.size()) > history_length_) {
    history_buffer_.pop_front();
  }

  std::unordered_map<int64_t, CellAgg> current_grid;
  for (const auto & p : current_points) {
    current_grid[cellKey(p.x, p.y)].count += 1;
  }

  std::unordered_map<int64_t, int> merged_counts;
  for (const auto & hist_entry : history_buffer_) {
    for (const auto & p : hist_entry.points) {
      merged_counts[cellKey(p.x, p.y)] += 1;
    }
  }

  // Classify per CELL first (density-ratio needs the aggregate counts
  // above), then collect every RAW point belonging to a moving cell in a
  // second pass - full sensor resolution, not one point per cell, so a
  // moving object's actual shape shows up in the output cloud rather than
  // a blocky per-cell approximation.
  std::unordered_set<int64_t> moving_cells;
  for (const auto & kv : current_grid) {
    const CellAgg & agg = kv.second;
    int merged_count = merged_counts[kv.first];  // >= agg.count, current scan is in the buffer too
    // Denominator scaled by how many buffered scans could actually have
    // SEEN this cell (within obstacle_search_radius_ of the vehicle at
    // that scan's time), not just how many scans are buffered - see
    // coverageCount()'s own comment for why a flat history_length_ scale
    // misclassifies a dense wall patch that just entered range.
    int coverage = coverageCount(kv.first);
    double expected_full = static_cast<double>(agg.count) * coverage;
    double static_score = expected_full > 0.0 ? merged_count / expected_full : 1.0;

    if (static_score <= moving_threshold_) {
      moving_cells.insert(kv.first);
    }
    // static_score >= static_threshold_ (static) and the undecided gap in
    // between are both simply not included - this node only ever outputs
    // moving points (see this class's own header comment).
  }

  moving_cells = filterSmallBlobs(moving_cells, current_grid);

  std::vector<Point3> moving_points, static_points;
  moving_points.reserve(moving_cells.size());
  for (const auto & p : current_points) {
    if (moving_cells.count(cellKey(p.x, p.y))) {
      moving_points.push_back(p);
    } else {
      static_points.push_back(p);
    }
  }

  publishMovingPoints(stamp, moving_points);
  publishStaticPoints(stamp, static_points);

  auto centroids = computeBlobCentroids(moving_cells, current_points);
  updateTracks(stamp, centroids);
  publishTrackingMarkers(stamp);
}

std::vector<DynamicObstacleDetector::Point3> DynamicObstacleDetector::computeBlobCentroids(
  const std::unordered_set<int64_t> & moving_cells,
  const std::vector<Point3> & current_points) const
{
  // Per-cell point lists, so each blob's centroid is a mean over RAW
  // points (not cell centers) - same source data the flood fill below
  // walks, just grouped for an easy per-blob sum.
  std::unordered_map<int64_t, std::vector<const Point3 *>> cell_points;
  for (const auto & p : current_points) {
    int64_t key = cellKey(p.x, p.y);
    if (moving_cells.count(key)) {
      cell_points[key].push_back(&p);
    }
  }

  std::vector<Point3> centroids;
  std::unordered_set<int64_t> visited;
  visited.reserve(moving_cells.size());

  for (const auto & start_key : moving_cells) {
    if (visited.count(start_key)) {
      continue;
    }
    std::deque<int64_t> queue;
    queue.push_back(start_key);
    visited.insert(start_key);
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int n = 0;

    while (!queue.empty()) {
      int64_t key = queue.front();
      queue.pop_front();
      auto it = cell_points.find(key);
      if (it != cell_points.end()) {
        for (const auto * p : it->second) {
          sum_x += p->x; sum_y += p->y; sum_z += p->z;
          ++n;
        }
      }
      int cx, cy;
      cellKeyToCoords(key, cx, cy);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          int64_t nkey =
            (static_cast<int64_t>(cx + dx) << 32) ^ (static_cast<uint32_t>(cy + dy));
          if (moving_cells.count(nkey) && !visited.count(nkey)) {
            visited.insert(nkey);
            queue.push_back(nkey);
          }
        }
      }
    }

    if (n > 0) {
      centroids.push_back({sum_x / n, sum_y / n, sum_z / n});
    }
  }
  return centroids;
}

void DynamicObstacleDetector::updateTracks(
  const rclcpp::Time & stamp, const std::vector<Point3> & centroids)
{
  std::vector<bool> matched(tracks_.size(), false);

  for (const auto & c : centroids) {
    // Nearest existing (not-yet-matched-this-cycle) track within
    // max_track_jump_m_ - see this class's own header "TRACKING" comment.
    int best_i = -1;
    double best_d2 = max_track_jump_m_ * max_track_jump_m_;
    for (size_t i = 0; i < tracks_.size(); ++i) {
      if (matched[i]) {
        continue;
      }
      double dx = c.x - tracks_[i].x, dy = c.y - tracks_[i].y;
      double d2 = dx * dx + dy * dy;
      if (d2 <= best_d2) {
        best_d2 = d2;
        best_i = static_cast<int>(i);
      }
    }

    if (best_i >= 0) {
      Track & t = tracks_[best_i];
      double dt = (stamp - t.last_update).seconds();
      // processCycle() already guarantees dt > 0 (its own top-of-function
      // sim-stall guard) whenever this runs at all, but a dt near zero
      // would still divide a small (even noise-level) displacement into
      // an amplified, bogus velocity - 0.05s is half this node's own
      // nominal 10Hz cycle period, so a normal cycle always clears it;
      // only an abnormally-fast re-match would be skipped (keeping the
      // track's previous velocity estimate rather than replacing it with
      // a noisy one).
      if (dt > 0.05) {
        t.vx = (c.x - t.x) / dt;
        t.vy = (c.y - t.y) / dt;
        t.has_velocity = true;
      }
      t.x = c.x; t.y = c.y;
      t.last_update = stamp;
      t.missed_cycles = 0;
      matched[best_i] = true;
    } else {
      Track nt;
      nt.id = next_track_id_++;
      nt.x = c.x; nt.y = c.y;
      nt.last_update = stamp;
      tracks_.push_back(nt);
      matched.push_back(true);
    }
  }

  for (size_t i = 0; i < tracks_.size(); ++i) {
    if (!matched[i]) {
      tracks_[i].missed_cycles += 1;
    }
  }
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this](const Track & t) { return t.missed_cycles > max_missed_cycles_; }),
    tracks_.end());
}

void DynamicObstacleDetector::publishTrackingMarkers(const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = fixed_frame_;
  clear.header.stamp = stamp;
  clear.ns = "moving_object_tracking";
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  int next_id = 0;
  for (const auto & t : tracks_) {
    if (!t.has_velocity) {
      continue;  // First sighting only - no velocity to show yet.
    }
    double speed = std::hypot(t.vx, t.vy);

    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = fixed_frame_;
    arrow.header.stamp = stamp;
    arrow.ns = "moving_object_tracking";
    arrow.id = next_id++;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point start, end;
    start.x = t.x; start.y = t.y; start.z = 0.3;
    end.x = t.x + t.vx; end.y = t.y + t.vy; end.z = 0.3;  // 1s-ahead arrow length
    arrow.points = {start, end};
    arrow.scale.x = 0.15; arrow.scale.y = 0.3; arrow.scale.z = 0.3;
    arrow.color.r = 1.0f; arrow.color.g = 0.6f; arrow.color.b = 0.0f; arrow.color.a = 0.95f;
    array.markers.push_back(arrow);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = fixed_frame_;
    text.header.stamp = stamp;
    text.ns = "moving_object_tracking";
    text.id = next_id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = t.x;
    text.pose.position.y = t.y;
    text.pose.position.z = 1.0;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.35;
    text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f; text.color.a = 1.0f;
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f m/s", speed);
    text.text = buf;
    array.markers.push_back(text);

    visualization_msgs::msg::Marker path;
    path.header.frame_id = fixed_frame_;
    path.header.stamp = stamp;
    path.ns = "moving_object_tracking";
    path.id = next_id++;
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.action = visualization_msgs::msg::Marker::ADD;
    path.pose.orientation.w = 1.0;
    path.scale.x = 0.08;
    path.color.r = 1.0f; path.color.g = 0.6f; path.color.b = 0.0f; path.color.a = 0.5f;
    const int kPredictionPoints = 8;
    for (int i = 0; i <= kPredictionPoints; ++i) {
      double dt = prediction_horizon_sec_ * i / kPredictionPoints;
      geometry_msgs::msg::Point p;
      p.x = t.x + t.vx * dt;
      p.y = t.y + t.vy * dt;
      p.z = 0.25;
      path.points.push_back(p);
    }
    array.markers.push_back(path);
  }

  tracking_markers_pub_->publish(array);
}

}  // namespace buggy_nav_costmap
