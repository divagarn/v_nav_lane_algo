/**
 * @file fusion_core.hpp
 * @brief Radar-led fusion. Pure C++ (no ROS) so it can be unit-tested and reused.
 *
 * Radar cycle: ground filter -> DBSCAN -> IMM tracks (stationary / CV / coordinated turn).
 * Camera cycle: 2D ByteTrack on the detector boxes, then the confirmed radar tracks are
 * associated with the boxes (one box per track at most) and the box bearing is fused
 * into the radar bearing. The camera never creates or deletes a track: every fused
 * object is a radar track, status "radar" or "radar+camera". Range, velocity and
 * existence stay radar; the camera adds the class (decaying votes) and a bearing
 * correction. Ego path check and the final object list on top.
 */
#pragma once

#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "fusion_common/camera_model.hpp"
#include "radar_fusion/byte_tracker.hpp"
#include "radar_fusion/ego_motion.hpp"
#include "radar_fusion/imm_filter.hpp"
#include "radar_fusion/path_predictor.hpp"
#include "radar_fusion/preprocessors.hpp"

namespace radar_fusion
{

/// Radar tracker parameters (imm.* and tracker.* in radar_fusion.yaml).
struct TrackerConfig
{
  // IMM: st = stationary, cv = constant velocity, ct = coordinated turn
  std::vector<std::string> model_types{"st", "cv", "ct"};
  double pos_std_st{0.1};          // stationary position random walk [m/sqrt(s)]
  double accel_std_cv{1.5};
  double accel_std_ct{1.0};
  double yaw_accel_std{0.5};
  std::vector<double> transition{   // row-major MxM
    0.95, 0.04, 0.01,
    0.03, 0.92, 0.05,
    0.01, 0.07, 0.92};
  std::vector<double> initial_mode_probs{0.5, 0.4, 0.1};
  // motion state reported per track: stationary | moving | turning
  double stationary_prob{0.6};     // P(st) above this -> stationary
  double moving_min_sigma{2.0};    // moving only if speed > this many std of the speed estimate
  // motion state temporal filter: EMA of the per-scan "moving" evidence + hysteresis
  double motion_alpha{0.3};
  double motion_enter{0.7};        // stationary -> moving above this
  double motion_exit{0.3};         // moving -> stationary below this
  // existence probability: hit pe += hit_gain (1 - pe), miss pe *= miss_factor
  double existence_init{0.3};
  double existence_hit_gain{0.3};
  double existence_miss_factor{0.7};
  // double-bounce ghost: a track at ~2x the range and ~2x the relative radial speed of a
  // closer track on the same bearing (radar -> object -> ego -> object -> radar)
  bool ghost_check{true};
  double ghost_az_tol_deg{3.0};
  double ghost_range_tol{0.1};     // relative tolerance on r_ghost = 2 r_real
  double ghost_doppler_tol{1.0};   // [m/s] on vr_ghost = 2 vr_real
  int ghost_min_count{3};          // consecutive scans before a track is flagged
  double turn_rate_threshold{0.1}; // |omega| [rad/s] with P(ct) dominant -> turning
  // init
  double init_vel_std_radial{0.7};
  double init_vel_std_tangential{4.0};
  double init_yaw_rate_std{0.3};
  // association
  double gate_chi2{13.8};          // 99.9% for 2 dof
  double absorb_gate{9.0};         // unmatched cluster this close to an updated track: no new track
  // lifecycle (radar scans)
  int confirm_hits{3};
  int confirm_window{5};
  int tentative_max_missed{1};
  int max_missed{5};
  double max_pos_std{4.0};         // delete if position std exceeds this
  double max_range{80.0};
  double moving_speed_threshold{0.6};
  double merge_distance{0.8};      // merge confirmed tracks closer than this
};

/// Camera-radar association and Bayes parameters (camera.* in radar_fusion.yaml).
struct CameraFusionConfig
{
  bool enabled{true};
  // detections are in the rectified (pinhole) image (object_detection rectifies)
  bool rectified{true};
  double rectify_balance{0.0};
  double rectify_fov_scale{1.0};
  double radar_height_above_ground{0.5};
  double min_detection_score{0.3};
  // gates per (box, radar track); pixel values are at 1920 px image width
  double margin_px{24.0};          // horizontal: track projects inside the box +- margin
  double object_height{2.5};       // vertical: [ground, ground + h] at the track range overlaps the box
  double range_ratio_min{0.5};     // radar range / ground range of the box bottom
  double range_ratio_max{2.0};
  double border_px{5.0};           // box edge this close to the image border is cut off
  // association: d2 = (bearing residual / sigma)^2 (gated) + ((r - r_closest) / range_pref)^2,
  // bounded cheap JPDA probabilities, then one-to-one Hungarian on -log(probability)
  double gate_d2{25.0};
  double range_pref{3.0};          // [m] prefer the closest track inside a box
  int jpda_max_candidates{4};      // per box, only the best few tracks
  double jpda_pd{0.9};
  double jpda_clutter{1e-3};
  // Bayes on the bearing
  double sigma_px{4.0};            // camera bearing noise: px (at 1920) + frac * box width
  double sigma_frac{0.03};
  double sigma_extent{0.5};        // [m] radar point vs box centre on an extended object
  double sigma_radar_min_deg{0.5};
  double radar_min_weight{0.6};    // radar keeps at least this share of the fused bearing
  double consistency_sigma{3.0};   // larger radar-camera disagreement -> no bearing correction
  double hold_s{0.3};              // radar cycles keep the last camera association this long
  // class: temporal filter (votes decay per associated frame)
  double class_vote_decay{0.97};
};

/// Relative-motion labelling and short-horizon prediction parameters.
struct RelativeMotionConfig
{
  double prediction_horizon{0.5};       // [s] future position exposed to downstream planners
  double radial_speed_threshold{0.3};   // [m/s] deadband for approaching / moving-away
  double min_ego_speed{0.3};            // [m/s] below this, same-direction is undefined
  double min_object_speed{0.3};         // [m/s] below this, object direction is unreliable
  double same_direction_max_angle{30.0};  // [deg] object and ego velocity heading difference
};

/// Stop zone: the ego corridor just ahead of the radar. Objects are predicted (constant velocity,
/// ego holding its current speed and yaw rate) to find when they would be inside it.
struct StopZoneConfig
{
  double horizon{3.0};        // [s] how far ahead object / ego motion is predicted
  double step{0.1};           // [s] prediction step
  double min_distance{3.0};   // [m] zone length ahead of the radar at standstill
  double time_headway{1.5};   // [s] zone length += ego speed * time_headway
};

/// Complete fusion configuration.
struct FusionConfig
{
  TrackerConfig tracker;
  RadarPreprocessorConfig radar;
  LidarPreprocessorConfig lidar;
  CameraFusionConfig camera;
  bool lidar_enabled{false};
  // ego speed source: "odom" (vehicle odometry) or "radar_doppler" (estimated
  // from static radar targets; yaw rate still from odometry)
  std::string ego_speed_source{"radar_doppler"};
  MountPose2D fusion_frame_mount;   // radar pose in vehicle base frame (ego motion, path)
  ByteTrackerConfig camera_tracker; // 2D tracking of the detector boxes
  PathConfig path;
  RelativeMotionConfig relative_motion;
  StopZoneConfig stop_zone;
  // final object list: every confirmed, non-ghost radar track in front of the vehicle,
  // closer than output_max_range and listed by existence (enter >= output_min_existence,
  // leave < output_keep_existence). in_path is only an attribute (the list never depends on the path).
  double output_max_range{40.0};
  double output_min_existence{0.6};
  double output_keep_existence{0.3};
  // drop stationary tracks whose mean height above ground is above this (tree canopy, facades)
  double output_max_static_height{2.0};
};

/// Radar-led radar/camera fusion: tracks, camera association, final object list.
class FusionCore
{
public:
  /**
   * @param[in] cfg  fusion configuration
   * @throws std::invalid_argument if the IMM set-up is inconsistent
   */
  explicit FusionCore(const FusionConfig & cfg);

  /**
   * @brief Sets the calibrated camera (raw image size); the model for the detector image
   *        is derived per frame size.
   * @param[in] cam  calibrated camera model
   */
  void setCamera(const fusion_common::CameraModel & cam) {base_camera_ = cam; have_camera_model_ = cam.valid(); cam_size_ = {0, 0};}

  /**
   * @brief Adds a vehicle odometry sample (ego motion and odom path curvature).
   * @param[in] s  speed / yaw-rate sample
   */
  void addEgo(const EgoSample & s);
  /**
   * @brief Adds a steering angle sample for the ego path.
   * @param[in] raw_angle  steering angle as published
   */
  void addSteering(double raw_angle) {path_.addSteering(raw_angle);}

  /**
   * @brief Radar cycle: preprocessing, prediction, association, update, track management.
   *
   * One cycle per sensor message, processed on arrival. Radar scans must not go back in time.
   * @param[in] stamp  scan time [s]
   * @param[in] dets   raw targets of the scan
   * @return number of measurements (clusters) used
   */
  std::size_t processRadar(Time stamp, const std::vector<RadarDetection> & dets);
  /**
   * @brief Lidar cycle (only if lidar_enabled).
   * @param[in] stamp  cloud time [s]
   * @param[in] pts    points in the fusion frame
   * @return number of measurements used
   */
  std::size_t processLidar(Time stamp, const std::vector<LidarPoint> & pts);
  /**
   * @brief Camera cycle: 2D tracking of the boxes and association with the radar tracks.
   *
   * The frame may be older or newer than the last scan: it uses the tracks predicted to its
   * stamp (snapshot), the radar filters themselves are not changed by the camera.
   * @param[in] f  detections of one frame
   * @return number of radar tracks fused with a box
   */
  std::size_t processCamera(const CameraFrame & f);

  /**
   * @brief All confirmed tracks at the output time (incl. ghosts and far tracks).
   * @return tracks
   */
  std::vector<FusedTrack> confirmedTracks() const;
  /**
   * @brief Final fused list: confirmed, non-ghost, listed (existence hysteresis) radar tracks in front within output_max_range,
   *        clutter rules applied, sorted by range, with the in-path flag.
   * @return objects
   */
  std::vector<FusedTrack> objects() const;
  /// @brief 2D tracks of the latest camera cycle (FusedTrack::camera_track_id = their track_id).
  const std::vector<TrackedBox> & cameraTracks() const {return camera_tracks_;}
  /// @brief Ego path predictor.
  const PathPredictor & path() const {return path_;}
  /**
   * @brief Radar frame at t0 -> radar frame at t1 from the ego motion (identity without odometry):
   *        p_t1 = R p_t0 + t.
   */
  void egoFrameTransform(Time t0, Time t1, Eigen::Matrix2d & R, Eigen::Vector2d & t) const
  {
    R.setIdentity();
    t.setZero();
    if (!ego_.empty()) {ego_.frameTransform(t0, t1, R, t);}
  }
  /// @brief Stop zone length ahead of the radar at the output time [m].
  double stopZoneLength() const;
  /// @brief Stop zone outline (closed polygon) in the radar frame at the output time.
  std::vector<Eigen::Vector2d> stopZone() const;

  /// @brief Time of the filter states (last radar scan) [s].
  Time lastStamp() const {return last_stamp_;}
  /// @brief Ego speed at the last radar scan [m/s].
  double lastEgoSpeed() const {return last_ego_speed_;}
  /// @brief Clusters of the last radar cycle.
  const std::vector<Measurement> & lastMeasurements() const {return last_meas_;}
  /// @brief Camera model of the detector image.
  const fusion_common::CameraModel & camera() const {return camera_;}
  /// @brief True once a valid camera calibration was set.
  bool hasCameraModel() const {return have_camera_model_;}
  /// @brief Deletes all radar and 2D tracks (e.g. bag restart).
  void reset();

private:
  /// Internal radar track.
  struct Track
  {
    uint32_t id;
    std::unique_ptr<ImmFilter> filter;
    int hits{0};
    int age{0};
    int missed{0};                       // consecutive radar scans without an update
    std::deque<bool> hit_history;
    bool confirmed{false};
    double z{0};
    double heading{0};
    bool radar_updated{false}, lidar_updated{false};
    double existence{0};
    double motion_score{0};              // EMA of the moving evidence
    bool moving{false};
    int ghost_count{0};
    bool listed{false};                  // in the final list (existence hysteresis)
    bool ghost{false};
    // camera
    uint32_t camera_track_id{0};
    Time camera_time{-1};                // last association
    double bearing_delta{0};             // fused - radar bearing at that time [rad]
    double assoc_prob{0};
    std::map<std::string, double> votes; // class votes (temporal filter)
  };

  /**
   * @brief Common radar/lidar cycle: predict, GNN association, update, births, management,
   *        motion state filter and ghost check.
   * @param[in] stamp   measurement time [s]
   * @param[in] meas    clusters
   * @param[in] sensor  source sensor
   * @return number of measurements
   */
  std::size_t runCycle(Time stamp, std::vector<Measurement> meas, SensorType sensor);
  /**
   * @brief Predicts all filters to stamp and compensates the ego motion.
   * @param[in] stamp  target time [s]
   */
  void predictTo(Time stamp);
  /**
   * @brief New filter from an unassociated cluster (velocity from Doppler along the line of sight).
   * @param[in] m  measurement
   * @return filter
   */
  std::unique_ptr<ImmFilter> initFilter(const Measurement & m) const;
  /// @brief M/N confirmation, heading, deletion and merging of duplicate tracks.
  void manageTracks();
  /**
   * @brief Derives the detector-image camera model when the frame size changes.
   * @param[in] width   image width [px]
   * @param[in] height  image height [px]
   */
  void updateCameraModel(int width, int height);
  /**
   * @brief True if the track was associated with a box within hold_s of the output time.
   * @param[in] t  track
   */
  bool cameraRecent(const Track & t) const;
  /// Track position / velocity / position covariance at a given time.
  struct Snapshot {Eigen::Vector2d p, v; Eigen::Matrix2d P;};
  /**
   * @brief Non-mutating prediction of a track to an arbitrary time (constant velocity + ego
   *        motion); used for camera frames and the output.
   * @param[in] t     track
   * @param[in] time  target time [s] (may be before or after last_stamp_)
   * @return snapshot in the sensor frame at time
   */
  Snapshot snapshot(const Track & t, Time time) const;
  /// Future ego frames for the stop-zone prediction: sensor frame at last_stamp_ -> sensor frame
  /// at output_time_ + k * step, cached per (last_stamp_, output_time_).
  struct FutureFrames {Time from{-1}, to{-1}; std::vector<double> dt; std::vector<Eigen::Matrix2d> R;
    std::vector<Eigen::Vector2d> t;};
  const FutureFrames & futureFrames() const;
  /**
   * @brief First time the track is inside the stop zone (constant velocity, ego holding its speed
   *        and yaw rate), -1 if not within stop_zone.horizon.
   * @param[in] t       track
   * @param[in] half_w  object half width [m] (corridor test)
   */
  double timeToStopZone(const Track & t, double half_w) const;
  /**
   * @brief Per-scan moving evidence: speed above threshold, significant and not stationary-model.
   * @param[in] t  track
   */
  bool movingEvidence(const Track & t) const;
  /**
   * @brief Flags double-bounce multipath ghosts (2x range, 2x radial speed on the bearing of a
   *        closer track) after ghost_min_count consecutive scans.
   * @param[in] stamp  scan time [s]
   */
  void checkGhosts(Time stamp);
  /**
   * @brief Radar position at the output time, rotated by the recent camera bearing correction.
   * @param[in] t  track
   * @return position [m]
   */
  Eigen::Vector2d outputPosition(const Track & t) const;
  /**
   * @brief 3D box centre from the radar point and the class size (radar sees the near face).
   * @param[in] t    track (class, heading)
   * @param[in] pos  radar position [m]
   * @return centre in the radar frame [m]
   */
  Eigen::Vector3d classCentre(const Track & t, const Eigen::Vector2d & pos) const;
  /**
   * @brief Horizontal range of the ground point seen by a pixel.
   * @param[in] u  pixel column
   * @param[in] v  pixel row
   * @return range [m], nullopt above the horizon
   */
  std::optional<double> groundRange(double u, double v) const;
  /**
   * @brief Radar-frame bearing of a pixel at a given horizontal range (camera parallax exact).
   * @param[in] u  pixel column
   * @param[in] v  pixel row
   * @param[in] r  range from the radar [m]
   * @return bearing [rad], nullopt if the ray never reaches range r
   */
  std::optional<double> bearingAtRange(double u, double v, double r) const;
  /**
   * @brief Image box of the class-sized 3D box at the fused position.
   * @param[in]  t    track
   * @param[in]  pos  fused position [m]
   * @param[out] box  x_min, y_min, x_max, y_max [px], clamped to the image
   * @return false if fewer than 4 corners project or the box is degenerate
   */
  bool constructBox(const Track & t, const Eigen::Vector2d & pos, double box[4]) const;
  /**
   * @brief Converts an internal track into the output object (state, motion state, camera, path, image).
   * @param[in] t  track
   * @return output object
   */
  FusedTrack toOutput(const Track & t) const;
  /**
   * @brief Class with the most (decayed) votes.
   * @param[in]  t     track
   * @param[out] conf  if not null, vote share of the best class
   * @return class name, "unknown" without votes
   */
  std::string bestClass(const Track & t, double * conf) const;

  FusionConfig cfg_;
  ImmConfig imm_cfg_;
  RadarPreprocessor radar_pre_;
  LidarPreprocessor lidar_pre_;
  EgoMotion ego_;
  fusion_common::CameraModel base_camera_, camera_;
  bool have_camera_model_{false};
  std::pair<int, int> cam_size_{0, 0};
  std::vector<Track> tracks_;
  ByteTracker box_tracker_;
  std::vector<TrackedBox> camera_tracks_;
  PathPredictor path_;
  int idx_st_{-1}, idx_ct_{-1};
  std::vector<Measurement> last_meas_;
  uint32_t next_id_{1};
  Time last_stamp_{-1};                  // time of the filter states (last radar scan)
  Time output_time_{-1};                 // time of the latest cycle (radar or camera)
  double last_ego_speed_{0};
  double last_yaw_rate_{0};
  mutable FutureFrames future_;          // stop-zone prediction cache (see futureFrames)
};

/**
 * @brief Fitzgerald's cheap JPDA association weights.
 * @param[in] lik      likelihoods (rows = boxes, cols = tracks; 0 = gated out)
 * @param[in] pd       detection probability
 * @param[in] clutter  clutter density
 * @return beta, same shape as lik, rows sum to <= 1
 */
Eigen::MatrixXd cheapJpda(const Eigen::MatrixXd & lik, double pd, double clutter);

}  // namespace radar_fusion
