/**
 * @file byte_tracker.hpp
 * @brief ByteTrack-style 2D multi-object tracker: constant-velocity Kalman filter on
 *        (cx, cy, w, h), IoU + Hungarian association in two stages (high then low score).
 */
#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <vector>

namespace radar_fusion
{

/// 2D box from the camera detector (pixels of the detector image).
struct Detection2D
{
  float x1{0}, y1{0}, x2{0}, y2{0};
  float score{0};
  int class_id{-1};
  std::string class_name;
};

/// Confirmed 2D track reported for one frame.
struct TrackedBox
{
  uint32_t track_id{0};
  Detection2D det;        // current (smoothed) box
  float vu{0}, vv{0};     // box-centre velocity [px/s]
  uint32_t age{0};
};

/// Thresholds of the 2D tracker (camera_tracker.* in radar_fusion.yaml).
struct ByteTrackerConfig
{
  float high_thresh{0.5f};
  float low_thresh{0.15f};
  float new_track_thresh{0.5f};
  float match_iou{0.3f};        // min IoU for a match (stage 1)
  float low_match_iou{0.5f};    // min IoU for low-score matches (stage 2)
  int min_hits{3};              // frames before a track is reported
  double max_lost_s{0.5};       // delete after being unmatched for this long
  bool class_consistent{true};  // only match detections of the same class
};

/// 2D box tracker giving stable ids and smoothed boxes to the camera fusion.
class ByteTracker
{
public:
  /// @param[in] c  tracker thresholds
  explicit ByteTracker(const ByteTrackerConfig & c = {}) : c_(c) {}
  /**
   * @brief Processes the detections of one frame (predict, two-stage match, correct, birth, delete).
   * @param[in] dets   detections of the frame
   * @param[in] stamp  frame time [s]; a backwards jump resets the tracker
   * @return confirmed tracks that were matched in this frame
   */
  std::vector<TrackedBox> update(const std::vector<Detection2D> & dets, double stamp);
  /// @brief Deletes all tracks.
  void reset() {tracks_.clear(); last_stamp_ = -1;}

private:
  using Vec8 = Eigen::Matrix<double, 8, 1>;
  using Mat8 = Eigen::Matrix<double, 8, 8>;
  struct Track
  {
    uint32_t id;
    Vec8 x;
    Mat8 P;
    Detection2D det;
    int hits{0};
    uint32_t age{0};
    double last_seen{0};
    bool confirmed{false};
    bool matched{false};
  };
  /**
   * @brief Kalman prediction with box-size-proportional noise.
   * @param[in,out] t   track
   * @param[in]     dt  interval [s]
   */
  void predict(Track & t, double dt) const;
  /**
   * @brief Kalman update with a matched detection.
   * @param[in,out] t  track
   * @param[in]     d  matched detection
   */
  void correct(Track & t, const Detection2D & d) const;
  /**
   * @brief Intersection over union of the predicted track box and a detection.
   * @param[in] x  track state (cx, cy, w, h, ...)
   * @param[in] d  detection
   * @return IoU in [0, 1]
   */
  static float iou(const Vec8 & x, const Detection2D & d);

  ByteTrackerConfig c_;
  std::vector<Track> tracks_;
  uint32_t next_id_{1};
  double last_stamp_{-1};
};

}  // namespace radar_fusion
