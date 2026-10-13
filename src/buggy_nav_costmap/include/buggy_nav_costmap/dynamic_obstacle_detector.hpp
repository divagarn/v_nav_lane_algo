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
#include <visualization_msgs/msg/marker_array.hpp>

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
// /moving_obstacle_points (colour-tinted - for RViz) and
// /static_obstacle_points (plain XYZ, exact complement). The split is
// decided by TRACKING below, not by this cycle's density-ratio
// classification alone - a tracked obstacle stays on
// /moving_obstacle_points for as long as it's still physically there, no
// matter how long, not just while it still looks "new" by density.
// /static_obstacle_points exists specifically so costmap_lane_planner can
// be pointed at it instead of the raw obstacle_topic when
// path_obstacle_monitor_node is in the loop (see that node's own header
// comment): path SELECTION then only ever reacts to obstacles that were
// never excluded as new/tracked, so ANY obstacle that appeared after the
// vehicle started navigating - moving or sitting dead still, for a
// fraction of a second or indefinitely - can never itself trigger a
// reroute. Only path_obstacle_monitor_node's separate stop/go gate reacts
// to it, and once it's gone, the vehicle resumes EXACTLY the path that
// was already being computed the whole time, because that path's own
// obstacle input was never disturbed by it.
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
//
// TRACKING (velocity + predicted trajectory, Autoware-style; also decides
// the /moving_obstacle_points split - see above): one persistent Track
// per physical object, kept alive two different ways depending on whether
// it's brand new or already being tracked -
//   STEP A - re-detect EXISTING tracks by proximity to their own last
//     position in the FULL current cloud (updateTracks()'s own
//     definition), independent of this cycle's density-ratio
//     classification entirely. This is what makes exclusion PERMANENT for
//     as long as the object is physically there: the density ratio that
//     first flagged an obstacle "new" naturally rises back toward
//     "static" once enough scans have corroborated it sitting in the same
//     spot (exactly the property that makes it useful for spotting newly-
//     revealed walls) - without this step, a motionless obstacle would
//     silently stop being new after history_length_ cycles (~0.6s),
//     its track would go unmatched and get dropped, and costmap_
//     lane_planner would start seeing it on /static_obstacle_points and
//     could replan around it. A moving object re-detects the same way;
//     this project's speeds move well under max_track_jump_m_ per cycle.
//   STEP B - spawn NEW tracks from this cycle's density+shape-filtered
//     moving_cells blobs, for whichever part isn't already claimed by
//     STEP A's re-detection (same object, already handled).
// A track is only dropped - and its location only then becomes eligible
// for /static_obstacle_points again - once STEP A finds nothing nearby
// for max_missed_cycles_ consecutive cycles, i.e. the object is actually
// gone. This is the same category of feature this class's own header
// comment above says was tried once already and reverted for two
// confirmed failure modes - this version exists specifically because both
// are now addressed structurally rather than re-attempted as-is:
//   1. Centroid jumping between frames: STEP A/B both gate matching by
//      max_track_jump_m_ - too far away is a NEW track, never (wrongly)
//      "the same object moved that far in one cycle".
//   2. Sim-time stalls corrupting velocity: processCycle() already
//      returns before reaching this code at all when the clock hasn't
//      advanced since the last cycle (see its own top-of-function guard)
//      - velocity is always dt/(genuinely elapsed time), never dt/~0.
// Published as /moving_object_tracking (MarkerArray): a velocity arrow +
// speed-label text + predicted-trajectory line per CONFIRMED track with a
// valid velocity estimate (a brand-new track has none yet - first
// sighting only gives a position).
//
// CONFIRMATION (vehicles/pedestrians only, not shaking leaves/wind-blown
// debris/sensor jitter) - VISUALIZATION ONLY, does not gate
// /moving_obstacle_points or /static_obstacle_points. A track counts as a
// confirmed real mover once it's been matched for min_confirm_cycles_
// consecutive cycles AND its net straight-line displacement from where it
// was first seen exceeds min_confirm_displacement_m_ - sustained net
// displacement is what actually separates a translating object from
// something jittering in place (leaves, a flag, grass all satisfy the
// density+shape filtering above just as well as a real mover does from a
// single cycle's density alone, since in-place jitter never accumulates
// distance no matter how long it keeps triggering the per-cell density
// check). publishTrackingMarkers() only draws confirmed tracks, so
// /moving_object_tracking's arrows/labels/predicted paths stay specific
// to vehicles/pedestrians.
//
// This does NOT decide /moving_obstacle_points vs /static_obstacle_points
// membership - that split uses the broader, pre-confirmation moving_cells
// set (everything density+shape filtering already called new-and-compact)
// deliberately: costmap_lane_planner only ever reads
// /static_obstacle_points for lane SELECTION, so a brand-new obstacle -
// moving or not yet proven moving - must never land there, or it could
// change which lane is chosen. The vehicle keeps driving its already-
// chosen lane for ANY new obstacle; only path_obstacle_monitor's ahead-
// zone gate (which already checks both topics) may stop it once it's
// actually in the way. Confirmation never un-sets once reached.
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

  struct Track
  {
    int id = 0;
    double x = 0.0, y = 0.0;
    double vx = 0.0, vy = 0.0;
    bool has_velocity = false;  // false until a second sighting confirms one
    rclcpp::Time last_update;
    int missed_cycles = 0;
    // CONFIRMATION (see this class's own header comment) - origin_x/y is
    // this track's first-ever centroid, age_cycles counts matched sightings
    // since then. confirmed only ever flips false->true, never back - once
    // a real mover, a brief missed-cycle gap shouldn't un-confirm it.
    double origin_x = 0.0, origin_y = 0.0;
    int age_cycles = 0;
    bool confirmed = false;
  };

  // One connected blob of moving cells, with the cell keys belonging to it
  // kept alongside the centroid - computeBlobs() flood-fills once and
  // updateTracks() needs both (centroid for matching, cells for routing
  // this blob's actual points once we know if its track got confirmed).
  struct Blob
  {
    Point3 centroid;
    std::unordered_set<int64_t> cells;
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
  // One Blob (centroid + cell set) per connected blob in the (already
  // fully filtered) final moving_cells set - a second, cheap flood-fill
  // pass purely to get per-object identity for tracking, same 8-
  // connectivity idiom as filterSmallBlobs. Does not re-apply any
  // size/extent filtering (that already happened).
  std::vector<Blob> computeBlobs(
    const std::unordered_set<int64_t> & moving_cells,
    const std::vector<Point3> & current_points) const;
  // Re-detects every EXISTING track by proximity to its own last position
  // in the FULL current cloud (current_points, not just this cycle's
  // moving_cells) within max_track_jump_m_, then spawns NEW tracks from
  // whatever part of new_blobs isn't already claimed by that re-detection
  // - see this class's own header "TRACKING" comment and this method's own
  // definition for why re-detection must not depend on the per-cycle
  // density-ratio test. Also runs each track's CONFIRMATION check (see
  // Track's own comment), setting t.confirmed directly on the Track -
  // publishTrackingMarkers() reads it from there. Returns the union of
  // every alive (re-detected or newly spawned) track's cells this cycle -
  // processCycle() publishes exactly these as moving, everything else as
  // static.
  std::unordered_set<int64_t> updateTracks(
    const rclcpp::Time & stamp, const std::vector<Blob> & new_blobs,
    const std::vector<Point3> & current_points);
  void publishTrackingMarkers(const rclcpp::Time & stamp);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr moving_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracking_markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Track> tracks_;
  int next_track_id_ = 0;

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
  // that specific failure mode, not a perfect classifier. Getting this
  // right matters more than it used to: a wall segment that slips through
  // this filter now spawns a track that re-detects itself indefinitely
  // (see updateTracks()'s "STEP A"), permanently excluding that stretch of
  // wall from /static_obstacle_points instead of just for the few cycles
  // it used to.
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

  // Tracking params - see this class's own "TRACKING" header comment.
  // Dual role: (1) the search radius updateTracks()'s STEP A uses to
  // re-detect an existing track by proximity to its own last position in
  // the full current cloud, independent of this cycle's density-ratio
  // test - this is what keeps a long-stationary obstacle permanently
  // excluded from /static_obstacle_points instead of it graduating back
  // after history_length_ cycles. (2) a detected new-blob centroid beyond
  // this of every track's last position is treated as a genuinely
  // different object rather than the same one having jumped - the
  // original guard against the "centroid jumped between frames" failure
  // mode. This project's speeds (walking pedestrian ~1m/s, vehicle a few
  // km/h, 10Hz cycle) move well under a meter per cycle, so the default
  // comfortably keeps pace with genuine motion, not just stationary
  // objects.
  double max_track_jump_m_ = 2.0;
  // STEP A counts current_points within max_track_jump_m_ of a track's
  // last position - this many are required for that to count as a
  // re-detection (not just a few stray/noise points at the search
  // radius's edge). Lower than min_moving_blob_points_ deliberately: this
  // only needs to confirm "yes, still substantially there", not re-run
  // the full new-object shape/size classification.
  int track_redetect_min_points_ = 5;
  // Consecutive cycles a track can go unmatched (STEP A found nothing
  // nearby) before being dropped - this is what actually ends the
  // exclusion, once the object is confirmed gone.
  int max_missed_cycles_ = 5;
  // How far ahead (seconds) the predicted-trajectory line extends.
  double prediction_horizon_sec_ = 2.0;

  // CONFIRMATION (see Track's own comment) - the density-ratio+shape
  // filtering above tells "this blob is new/transient", not "this blob is
  // a vehicle/pedestrian specifically" - shaking tree leaves, a flag
  // flapping, grass in wind all pass it too, since from one cycle's
  // density alone a jitter-in-place blob looks identical to a translating
  // one. The distinguishing signal is over TIME: a real mover accumulates
  // net displacement cycle over cycle; something jittering in place
  // oscillates back toward where it started, so its net displacement
  // stays small no matter how long you watch it. A track only counts as
  // confirmed once it's been matched for min_confirm_cycles_ consecutive
  // cycles AND its straight-line distance from where it was FIRST seen
  // exceeds min_confirm_displacement_m_ - defaults (3 cycles, 0.3m) confirm
  // a walking pedestrian (~1m/s) in well under half a second while an
  // in-place jitter never crosses the displacement bar at all, regardless
  // of how long it keeps triggering the per-cell density check.
  int min_confirm_cycles_ = 3;
  double min_confirm_displacement_m_ = 0.3;
};

}  // namespace buggy_nav_costmap
