#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "buggy_nav_costmap/pose_graph_walker.hpp"

namespace buggy_nav_costmap
{

// Drop-in alternative to buggy_nav's freespace_grid_node.py: same arc-based
// lane planning logic (candidate curvatures, footprint-width clearance
// walk, centering preference, temporal smoothing) and the same ported
// pose-graph pipeline, but occupancy comes directly from the latest LiDAR
// point cloud (transformed once per cycle into a simple grid-hash) instead
// of a persistent costmap layer. A Nav2 Costmap2DROS was tried first, but
// its internal message filter kept dropping every observation ("earlier
// than all the data in the transform cache") regardless of obstacle
// source, mutex use, or update/publish rate - confirmed live across four
// separate fix attempts - so this drops that whole subsystem rather than
// keep chasing a Nav2-internal timing issue neither this node nor its
// launch file can control. Publishes the SAME topics as the Python node
// (/target_lateral_position, /suggested_lane, /turn_direction, etc.) so
// steering_calculator_node needs no changes - this replaces
// freespace_grid_node, it does not run alongside it.
class CostmapLanePlanner : public rclcpp::Node
{
public:
  CostmapLanePlanner();

private:
  struct ArcResult
  {
    double curvature = 0.0;
    double clear_distance = 0.0;
    std::vector<Vec2> points;
    double centering_error = 0.0;
    // Average of min(left, right) sensed clearance along the arc - how
    // much standoff this candidate keeps from the NEAREST obstacle on
    // either side, not just left/right symmetry (centering_error is 0 for
    // a path centered in a tight 2m gap just as much as one centered in a
    // wide-open 10m one - this is what actually distinguishes them). See
    // obstacle_clearance_preference_'s own comment for how it's used.
    double min_side_clearance = 0.0;
  };

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void navigationEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void rebuildObstacleGrid(const Vec2 & pos);
  int64_t gridKey(int cx, int cy) const;
  void planCycle();
  bool isBlockedWorld(double wx, double wy) const;
  double lateralClearance(double x, double y, double theta, int sign, double max_search) const;
  ArcResult arcIsClear(const Vec2 & pos, double yaw, double curvature) const;
  double directionAndReferenceBonus(double curvature, double ex, double ey) const;
  void publishTargetLateralPosition(double curvature);
  void publishLaneMarker(const rclcpp::Time & stamp, const Vec2 & pos, const ArcResult & chosen);
  void publishObstacleAreaMarker(const rclcpp::Time & stamp);
  void publishFreeLaneAreaMarker(
    const rclcpp::Time & stamp, const Vec2 & pos, const ArcResult & chosen);
  // Green - the chosen path's actual drivable WIDTH (clearance_ either
  // side), shared by publishFreeLaneAreaMarker (the visualization) and
  // checkNewObstacleIntrusion (the enforced-safety check) so both agree on
  // exactly the same shape - same reasoning as computeFreeSpaceCorridor
  // below, just at the fixed clearance_ width instead of the real sensed
  // one.
  void computeFixedWidthLane(
    const Vec2 & pos, const ArcResult & chosen,
    std::vector<Vec2> & centerline, std::vector<Vec2> & left, std::vector<Vec2> & right) const;
  // The REAL LiDAR-sensed free-space corridor along the chosen lane (+
  // lane_free_space_extra_forward_m past its end) - centerline plus the
  // left/right WORLD POINTS lateralClearance() actually reached at each
  // one, computed ONCE per cycle and shared by the orange visualization
  // marker below and checkMovingIntrusion()'s safety check, so both agree
  // on exactly the same shape (no drift between what's drawn and what's
  // enforced).
  void computeFreeSpaceCorridor(
    const Vec2 & pos, const ArcResult & chosen,
    std::vector<Vec2> & centerline, std::vector<Vec2> & left, std::vector<Vec2> & right) const;
  // Orange - draws the corridor computeFreeSpaceCorridor() built. If the
  // buggy is between two close walls this hugs that actual gap; where
  // it's wide open it extends out to corridor_scan_limit_. See
  // publishFreeLaneAreaMarker (green, fixed clearance_ width) for the
  // enforced-safety-margin strip this is NOT replacing - this is the
  // actual available space, that is the guaranteed-safe subset of it.
  void publishLaneFreeSpaceAreaMarker(
    const rclcpp::Time & stamp,
    const std::vector<Vec2> & centerline, const std::vector<Vec2> & left,
    const std::vector<Vec2> & right);
  void movingPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  // A moving obstacle crossing INTO the real free-space corridor above -
  // the one category of obstacle that corridor is otherwise blind to:
  // when enable_moving_obstacle_stop's /static_obstacle_points switch is
  // in effect, lateralClearance()'s own isBlockedWorld() only ever sees
  // STATIC points (by design, so path selection stays blind to moving
  // objects - see dynamic_obstacle_detector's own header comment), so the
  // corridor never narrows for a moving object the way it already does
  // for a static one. This checks /moving_obstacle_points against the
  // SAME corridor shape the orange marker draws and publishes
  // /lane_free_space_blocking (Bool) - path_obstacle_monitor_node ORs
  // this into its own stop decision, the same extension point its own
  // header comment already documents for "one more obstacle source".
  // Static obstacles are NOT re-checked here - they already define the
  // corridor's own boundary (lateralClearance stops at them), so
  // path_obstacle_monitor's separate /static_obstacle_points check against
  // its fixed-width ahead-zone remains the static-obstacle safety net.
  // Sets moving_intrusion_latched_ (see its own comment) - no longer
  // publishes directly; publishLaneFreeSpaceBlocking() below ORs it
  // together with new_obstacle_intrusion_latched_ into one publish, so
  // either source alone is enough to hold the latch.
  void checkMovingIntrusion(const std::vector<Vec2> & left, const std::vector<Vec2> & right);
  // A brand-NEW obstacle (see dynamic_obstacle_detector's own
  // /new_obstacle_points comment - this includes a blob before its track
  // has even had a chance to confirm as a real mover, unlike
  // /moving_obstacle_points) crossing into the GREEN fixed-width lane band
  // - deliberately the narrower, enforced-safety-margin band
  // (computeFixedWidthLane), not the wider real-sensed orange corridor,
  // since a new obstacle merely somewhere in a wide-open sensed corridor
  // isn't actually in the vehicle's way. Same no-reroute-only-stop
  // principle /moving_obstacle_points already gets: this obstacle source
  // is deliberately never fed into obstacle_topic_/isBlockedWorld (so it
  // can never itself narrow the corridor or change which arc gets chosen),
  // it only ever latches the ahead-zone stop gate via
  // publishLaneFreeSpaceBlocking().
  void checkNewObstacleIntrusion(const std::vector<Vec2> & left, const std::vector<Vec2> & right);
  // Combines moving_intrusion_latched_ and new_obstacle_intrusion_latched_
  // (either one alone holds the gate) into the single
  // /lane_free_space_blocking publish path_obstacle_monitor_node already
  // ORs into its own stop decision.
  void publishLaneFreeSpaceBlocking();
  void newObstaclePointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // This node's own tf buffer - one lookup per cycle, shared by the
  // pose-graph walker and the arc planner (see planCycle()'s own comment).
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<PoseGraphWalker> pose_graph_walker_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr pose_graph_sub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr moving_points_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_moving_cloud_;
  // /new_obstacle_points - see checkNewObstacleIntrusion's own comment.
  // Independent subscription for the same reason moving_points_sub_ is:
  // obstacle_topic_ only ever carries /static_obstacle_points (or the raw
  // cloud, feature off), never this.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr new_obstacle_points_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_new_obstacle_cloud_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lane_free_space_blocking_pub_;
  // Per-CYCLE consecutive confirmation (not per-message like path_obstacle_
  // monitor_node's own debounce) - this node already evaluates against
  // whatever the latest moving-points message is once per planCycle(),
  // same established pattern latest_cloud_ already uses, so there's no
  // "topic publishes slower than the timer" problem to work around here.
  int moving_intrusion_consecutive_ = 0;
  int moving_intrusion_clear_consecutive_ = 0;
  bool moving_intrusion_latched_ = false;
  // Same per-cycle hysteresis pattern as moving_intrusion_* above, its own
  // independent counters/latch so a new-obstacle intrusion and a moving
  // one don't reset each other's debounce.
  int new_obstacle_intrusion_set_cycles_ = 2;
  int new_obstacle_intrusion_clear_cycles_ = 5;
  int new_obstacle_intrusion_consecutive_ = 0;
  int new_obstacle_intrusion_clear_consecutive_ = 0;
  bool new_obstacle_intrusion_latched_ = false;

  // Captures "where the pose-graph route was when navigation was just
  // turned on" (manual_pose_aligner's 'g' key), snapped onto the nearest
  // recorded vertex - the origin used by the optional stop-at-start-point
  // behavior below. Independent of wherever the vehicle physically started;
  // re-captured every time /navigation_enabled has a false->true edge, so
  // re-pressing 'g' from a different spot on the loop re-anchors the origin
  // there instead of leaving a stale one from an earlier run.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_enabled_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lap_complete_pub_;
  bool navigation_enabled_ = false;
  bool prev_navigation_enabled_ = false;
  std::optional<Vec2> start_position_;
  // Only start checking "are we back at the origin" once the vehicle has
  // actually traveled origin_min_travel_m_ away from it - otherwise the
  // very first planCycle() after the rising edge (still sitting right on
  // top of start_position_) would immediately read as "lap complete".
  bool has_left_origin_ = false;
  bool lap_complete_ = false;

  // Simple uniform-grid spatial index of obstacle points in fixed_frame_,
  // rebuilt fresh every planCycle() from whatever the latest cloud is at
  // that moment - no persistence across cycles (matches the "reactive
  // only" dynamic-obstacle handling already in place elsewhere in this
  // project: a point is only ever "there" for as long as the sensor
  // currently sees it). Keyed by packed (cell_x, cell_y).
  std::unordered_map<int64_t, std::vector<Vec2>> obstacle_grid_;
  std::vector<Vec2> obstacle_points_;  // flat copy, for the visualization marker

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lane_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_area_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr free_lane_area_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lane_free_space_area_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_lateral_pub_;
  // Consumed directly by buggy_perception's existing freespace_uart_bridge
  // (real-hardware stop/go safety gate) - published every cycle so it's a
  // reliable heartbeat (staleness = stop), same "OK" heartbeat semantics
  // freespace_grid_node.py's own /nav_status already used. This node has
  // no stuck/recovery state machine of its own (unlike freespace_grid_
  // node), so it only ever publishes OK or STUCK, never RECOVERING -
  // freespace_uart_bridge already treats both the same (stop) regardless.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_status_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string fixed_frame_;
  std::string base_frame_;
  std::string obstacle_topic_;
  double resolution_ = 0.3;
  double lane_lookahead_ = 12.0;
  int lane_num_candidates_ = 15;
  double corridor_scan_limit_ = 8.0;
  double lane_straight_preference_ratio_ = 0.85;
  // Weight on ArcResult::min_side_clearance in the candidate score (see
  // its own comment) - rewards standing off further from the nearest
  // obstacle when multiple candidates are already comparably clear and
  // comparably centered, instead of always settling for the bare minimum
  // clearance_ margin right next to it. Deliberately small relative to
  // direction_reference_gain_ (6.0 default): this only re-ranks among
  // arcs the pose-graph bias already considers roughly as good for the
  // commanded turn direction - it never fights that bias for WHICH way to
  // turn, only how much room to leave once going that way.
  double obstacle_clearance_preference_ = 0.5;
  double lane_smoothing_alpha_ = 0.3;
  double steering_lookahead_distance_ = 6.0;
  double vehicle_half_width_ = 1.0;
  double vehicle_half_length_ = 1.8;
  double vehicle_footprint_margin_ = 0.3;
  double min_turn_radius_ = 1.0;
  double max_curvature_ = 1.0;
  double clearance_ = 1.3;

  // Points are in the sensor's own frame. Upstream (segment_ground) already
  // removes the ground plane, so this is only an overhead cutoff (signage,
  // branches, etc.) - no lower bound. max_obstacle_height_m is meters above
  // the GROUND (declared param, direct from the launch file); converted
  // once here into this sensor-frame value by subtracting the LiDAR
  // mounting height.
  static constexpr double kLidarMountHeightM = 1.0;
  double max_obstacle_height_ = 4.0;
  // Radius around each raw obstacle point that counts as "occupied" for
  // isBlockedWorld() - stands in for the point's own physical size/the
  // sensor's angular resolution at range, not the vehicle's safety margin
  // (that's clearance_/vehicle_half_length_, applied separately in
  // arcIsClear's footprint-corner walk).
  double obstacle_inflation_radius_ = 0.25;
  // Extra buffer added on top of obstacle_inflation_radius_ for BOTH the
  // actual isBlockedWorld() check and the obstacle_area marker's size -
  // the two always match exactly, so what's drawn is what's really
  // enforced, not a separately-eyeballed picture. Genuinely widens the
  // real blocking distance (not just the visualization), as real extra
  // margin against the residual steering/dynamics imprecision already
  // observed to cause collisions this session even with the footprint
  // checks and lookahead-matching both correct.
  double obstacle_safety_margin_ = 0.3;
  // Only points within this radius of the vehicle's current position get
  // indexed each cycle - bounds the working set to "a certain area around
  // where the vehicle actually is/is heading" instead of the whole cloud,
  // per the explicit request to scope this to the relevant forward area
  // rather than a full persistent map.
  double obstacle_search_radius_ = 30.0;
  // How much further forward (m) /lane_free_space_area extends past the
  // suggested lane's own endpoint (lane_lookahead_) - straight-continued
  // from the lane's last segment direction, still using REAL
  // lateralClearance() sensing at each extended point (not a flat
  // repeat of the last measured width). Lets an obstacle (moving or
  // static) just beyond where the lane currently ends show up in this
  // zone before the lane itself would have reached that point.
  double lane_free_space_extra_forward_m_ = 3.0;
  // Consecutive planCycle()s a moving point must be found inside the free-
  // space corridor before /lane_free_space_blocking sets, and consecutive
  // clear cycles before it releases - same debounce-against-single-cycle-
  // noise idea as path_obstacle_monitor_node's own thresholds.
  int moving_intrusion_set_cycles_ = 2;
  int moving_intrusion_clear_cycles_ = 5;

  // Single, much-stronger-than-the-Python-original weight replacing
  // direction_bias_gain (1.5) + reference_path_weight (1.5): dominates
  // candidate selection among everything the safety filter already passed,
  // per the user's explicit "very strong bias, not absolute" choice - see
  // directionAndReferenceBonus()'s own comment for the exact formula.
  double direction_reference_gain_ = 6.0;

  double smoothed_curvature_ = 0.0;
  bool enable_pose_graph_bias_ = true;

  // If true: once the vehicle has left and come back within
  // origin_stop_radius_m_ of start_position_, /lap_complete latches true and
  // stays true (steering_calculator_node then holds the vehicle stopped,
  // same as the existing not-yet-enabled gate). If false: /lap_complete is
  // never set, so the vehicle keeps navigating around the loop indefinitely.
  bool start_stop_origin_ = false;
  double origin_stop_radius_m_ = 1.5;
  double origin_min_travel_m_ = 10.0;
};

}  // namespace buggy_nav_costmap
