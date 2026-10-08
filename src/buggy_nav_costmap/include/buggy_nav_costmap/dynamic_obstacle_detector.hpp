#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace buggy_nav_costmap
{

// Static/moving obstacle classification, ported from the core idea of
// Asvadi et al., "3D Lidar-based Static and Moving Obstacle Detection in
// Driving Environments" (RAS 2016): merge the last `history_length_`
// ego-motion-compensated scans into one point set, then compare each
// currently-occupied cell's MERGED point density against what the CURRENT
// scan alone contributed there. A cell every buffered scan corroborates
// accumulates close to history_length_x the density of one scan (static -
// something has reliably been there the whole time). A cell only the
// current scan touches stays at ~1x density (moving - the other buffered
// scans, taken moments ago, saw nothing there because the object wasn't
// there yet). No per-object clustering, tracking, or frame-to-frame ID
// association is used anywhere - classification is a per-cell property of
// this cycle's density ratio alone, which is what makes this immune to
// the two failure modes an earlier (reverted) tracking-based attempt hit
// live: large/elongated clusters' centroids jumping between frames, and
// track state getting corrupted on sim cycles where use_sim_time hadn't
// actually advanced (this node has no persistent per-object state to
// corrupt - only a rolling buffer of raw scans).
//
// The paper used GPS/IMU + per-scan ICP to align scans into a world
// frame; this project has neither GPS nor a bespoke per-scan ICP step -
// scans are aligned here using the SAME fixed_frame TF chain the rest of
// this package already relies on (kiss_icp when enabled via
// enable_kiss_icp, wheel_odometry/tf_odom_relay otherwise), which serves
// the identical role.
//
// Publishes TWO point clouds each cycle, both at full sensor resolution:
// /moving_obstacle_points (colour-tinted, moving cells only - for RViz)
// and /static_obstacle_points (plain XYZ, exact complement - everything
// NOT classified moving this cycle, i.e. confidently static OR still
// undecided). /static_obstacle_points exists specifically so
// costmap_lane_planner can be pointed at it instead of the raw
// obstacle_topic when path_obstacle_monitor_node is in the loop (see that
// node's own header comment): path SELECTION then only ever reacts to
// static obstacles, so a moving object crossing the chosen path can never
// itself trigger a reroute - only path_obstacle_monitor_node's separate
// stop/go gate reacts to it, and once it clears, the vehicle resumes
// EXACTLY the path that was already being computed the whole time,
// because that path's own obstacle input was never disturbed by it.
//
// Moving cells are additionally grouped into contiguous blobs (8-
// connectivity, same grid-hash flood-fill idiom used elsewhere in this
// package) and any blob with fewer than min_moving_blob_points_ total
// points is dropped before publishing - thin, few-point slivers (most
// search-radius-boundary flicker) are filtered this way. This is NOT
// sufficient on its own though - confirmed live that a DENSE patch of
// wall newly entering the search radius (small vehicle pose jitter
// shifting which cells are in range) produces a medium/large blob too,
// indistinguishable from a real object by size alone. The real fix for
// that is coverage-aware density (see history_buffer_'s own comment):
// a cell that simply wasn't within obstacle_search_radius_ of the
// vehicle in a given buffered scan couldn't have been recorded there
// regardless of what was actually present, so that scan must not count
// against it in the static_score denominator.
class DynamicObstacleDetector : public rclcpp::Node
{
public:
  DynamicObstacleDetector();

private:
  struct Point3
  {
    double x, y, z;
  };

  struct CellAgg
  {
    int count = 0;
  };

  struct HistoryEntry
  {
    std::vector<Point3> points;
    double vpos_x = 0.0, vpos_y = 0.0;
    bool have_vpos = false;
  };

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void processCycle();
  bool transformLatestCloud(
    std::vector<Point3> & out, double & vpos_x, double & vpos_y, bool & have_vpos);
  int64_t cellKey(double x, double y) const;
  void cellKeyToCoords(int64_t key, int & cx, int & cy) const;
  // Groups moving_cells into contiguous (8-connected) blobs and returns
  // only the cells belonging to a blob whose summed point count (via
  // current_grid) reaches min_moving_blob_points_.
  std::unordered_set<int64_t> filterSmallBlobs(
    const std::unordered_set<int64_t> & moving_cells,
    const std::unordered_map<int64_t, CellAgg> & current_grid) const;
  // Number of buffered scans whose vehicle position was within
  // obstacle_search_radius_ of this cell's center - see history_buffer_'s
  // own comment for why this (not history_buffer_.size()) is the correct
  // denominator scale for static_score.
  int coverageCount(int64_t key) const;
  void publishMovingPoints(const rclcpp::Time & stamp, const std::vector<Point3> & moving_points);
  void publishStaticPoints(const rclcpp::Time & stamp, const std::vector<Point3> & static_points);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr moving_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_points_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Rolling buffer of the last history_length_ scans, each already
  // transformed into fixed_frame at the time it was processed - the
  // "integrated multi-scan cloud" the paper's D (Section 2.1) plays the
  // role of, built here without a separate per-scan ICP step (see this
  // class's own header comment). Each entry also keeps the vehicle
  // position at that scan's time, so coverageCount() can tell "this cell
  // had no points in that past scan because nothing was there" apart from
  // "this cell had no points in that past scan because it wasn't even
  // within obstacle_search_radius_ yet" - confirmed live that without
  // this distinction, a dense patch of wall newly entering the search
  // radius (small vehicle pose jitter) reads as under-corroborated and
  // gets misclassified moving, same as a genuinely new object would.
  std::deque<HistoryEntry> history_buffer_;

  std::string fixed_frame_;
  std::string base_frame_;
  std::string obstacle_topic_;
  double cell_size_ = 0.3;
  int history_length_ = 6;
  double obstacle_search_radius_ = 30.0;
  // static_score = (points in this cell across ALL buffered scans) /
  // (points in this cell THIS scan x number of buffered scans) - 1.0 if
  // every buffered scan hit this cell equally hard, ~1/history_length_ if
  // only the current scan ever touched it. A cell is only ever emitted
  // (as moving) when static_score <= moving_threshold_ - everything else
  // (confidently static, or too ambiguous to call yet) is simply not
  // published, since this node only ever outputs moving points.
  double moving_threshold_ = 0.3;
  // A contiguous blob of moving cells must contain at least this many
  // total raw points to be published - see this class's own header
  // comment for why this is the filter against wall/boundary false
  // positives rather than a flat per-cell point-count minimum.
  int min_moving_blob_points_ = 15;
  // A blob's bounding-box longest side must NOT exceed this (meters) to
  // be published - confirmed live that a wall segment newly revealed by
  // continuous forward driving (occluded until the vehicle drove far
  // enough to see around whatever was blocking it) produces a large,
  // spatially ELONGATED moving-classified blob, since coverageCount()'s
  // radius-based model has no real line-of-sight/occlusion awareness.
  // Real moving objects this project cares about (person, vehicle) are
  // comparatively compact - this is a pragmatic shape heuristic against
  // that specific failure mode, not a perfect classifier.
  double max_moving_blob_extent_m_ = 2.5;
  // Fixed tint applied to every published moving point (0-255 each) - a
  // single flat colour, not intensity-mapped, so moving points read as
  // visually distinct from this project's other (uncoloured/intensity-
  // shaded) point cloud displays in RViz regardless of intensity data.
  int moving_color_r_ = 0;
  int moving_color_g_ = 0;
  int moving_color_b_ = 255;
  double process_rate_hz_ = 10.0;
  bool has_last_cycle_time_ = false;
  rclcpp::Time last_cycle_time_;
};

}  // namespace buggy_nav_costmap
