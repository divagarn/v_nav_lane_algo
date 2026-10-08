#pragma once

#include <deque>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace buggy_nav_costmap
{

struct Vec2
{
  double x = 0.0;
  double y = 0.0;
};

// Ported from freespace_grid_node.py's merged pose-graph turn-direction
// logic (originally the standalone pose_graph_turn_direction_node, merged
// in this session for a single-TF-snapshot-per-cycle guarantee - see that
// file's enable_pose_graph_bias comment for why). Faithfully ported
// (vertex-count stabilize-then-freeze, NaN-safe nearest vertex,
// direction-disambiguated graph walk, 3-deep median-jump-rejection with
// self-poisoning recovery) rather than reimplemented, since this logic was
// hardened through several real bugs already found and fixed this session.
class PoseGraphWalker
{
public:
  struct Params
  {
    std::string fixed_frame = "odom";
    std::string map_frame = "map";
    double turn_lookahead_m = 9.0;
    double route_preview_distance_m = 40.0;
    double nearest_vertex_max_dist = 5.0;
    double snap_deg = 25.0;
    int direction_disambiguation_vertices = 5;
    int pose_graph_stabilize_count = 2;
    size_t pose_graph_history_depth = 3;
    size_t pose_graph_reset_streak_threshold = 3;
    double route_preview_max_jump_m = 3.0;
    double route_preview_cache_timeout_sec = 999999.0;
    double turn_signal_timeout_sec = 2.0;
    // For a closed-loop track, the array's first and last vertex are
    // spatially close (confirmed live on my_map2: 1.1m apart) even though
    // they're at opposite ends of the vertex array - that array boundary
    // is a recording artifact, not a real gap in the physical track. If
    // the gap between the walk's current end and the wrap-around target
    // (vertex 0 or the last vertex) is within this distance, the walk
    // continues across it instead of stopping dead at the array edge.
    // Set below the shortest real gap you'd ever expect between separate
    // track segments, so an actually-open (non-loop) track, or two
    // genuinely distant parts of a loop that just happen to be adjacent
    // in recording order, never gets bridged by mistake.
    double loop_closure_max_gap_m = 2.0;
  };

  PoseGraphWalker(rclcpp::Node * node, tf2_ros::Buffer * tf_buffer, const Params & params);

  void poseGraphCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

  // Call once per planning cycle with the SAME (x0,y0,yaw) TF snapshot the
  // arc planner uses this cycle - do not give this its own separate TF read.
  void tick(double x0, double y0, double current_yaw, const rclcpp::Time & stamp);

  // Graceful-degrades to STRAIGHT/NaN if the signal is stale or absent -
  // same as _current_turn_direction()/_reference_path_bonus() in the
  // Python original.
  std::string currentTurnDirection() const;
  double turnDistanceM() const { return turn_distance_m_; }
  bool hasReferencePath() const;
  const std::vector<Vec2> & referencePath() const { return reference_path_points_; }

  // Nearest pose-graph vertex to (x0, y0) regardless of
  // nearest_vertex_max_dist, as (x, y, distance) - or nullopt if no graph
  // is available yet. Exposed publicly so callers can snap an arbitrary
  // live position (e.g. "where navigation was just enabled") onto the
  // recorded route.
  std::optional<std::tuple<double, double, double>> nearestVertex(double x0, double y0);

private:
  std::optional<std::vector<Vec2>> verticesInFixedFrame();
  static std::optional<size_t> safeNearestIndex(const std::vector<double> & d);
  std::optional<std::vector<Vec2>> lookaheadPath(
    double x0, double y0, double current_yaw, double max_distance);
  std::optional<std::vector<Vec2>> robustLookaheadPath(
    double x0, double y0, double current_yaw, double max_distance,
    const std::string & history_name);
  double distanceToTurn(const std::vector<Vec2> & points, double current_yaw) const;

  void publishNearestVertexSearchRadius(const rclcpp::Time & stamp, double x0, double y0);
  void publishNearestVertexMarker(
    const rclcpp::Time & stamp, double x0, double y0,
    const std::optional<std::tuple<double, double, double>> & nearest);
  void publishTurnDirectionTextMarker(
    const rclcpp::Time & stamp, double x, double y, const std::string & direction,
    const std::array<double, 4> & color, double turn_distance);
  void publishTurnDirectionPathMarker(
    const rclcpp::Time & stamp, const std::vector<Vec2> * points,
    const std::array<double, 4> & color);
  void publishRoutePreviewMarker(const rclcpp::Time & stamp, const std::vector<Vec2> * points);

  rclcpp::Node * node_;
  tf2_ros::Buffer * tf_buffer_;
  Params p_;
  rclcpp::Logger logger_;

  // Pose-graph vertex state (mirrors _pose_graph_callback's stabilize-then-
  // freeze fields exactly).
  std::vector<Vec2> pose_graph_vertices_map_;
  bool has_vertices_ = false;
  std::optional<size_t> original_vertex_count_;
  size_t pose_graph_last_len_ = 0;
  int pose_graph_stable_count_ = 0;

  struct WalkHistoryEntry
  {
    double x, y;
    std::vector<Vec2> path;
    rclcpp::Time time;
  };
  std::map<std::string, std::deque<WalkHistoryEntry>> walk_history_;
  std::map<std::string, std::deque<Vec2>> rejected_streak_;

  std::string turn_direction_ = "STRAIGHT";
  double turn_distance_m_ = std::numeric_limits<double>::quiet_NaN();
  rclcpp::Time last_turn_signal_time_;
  bool has_turn_signal_ = false;

  std::vector<Vec2> reference_path_points_;
  bool has_reference_path_ = false;
  rclcpp::Time last_reference_path_time_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr direction_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr text_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nearest_vertex_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nearest_link_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nearest_text_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr nearest_distance_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr route_preview_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr search_radius_pub_;
};

}  // namespace buggy_nav_costmap
