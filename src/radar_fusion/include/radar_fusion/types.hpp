/**
 * @file types.hpp
 * @brief Plain data types of the fusion core (no ROS dependency).
 */
#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace radar_fusion
{

using Time = double;  ///< time stamp [s]

/// Source sensor of a measurement.
enum class SensorType : uint8_t { RADAR = 0, LIDAR = 1, CAMERA = 2 };

/// One raw radar target (radar frame: x forward, y left, z up).
struct RadarDetection
{
  double x{0}, y{0}, z{0};
  double radial_speed{0};   // relative radial speed as reported by the sensor [m/s]
  double rcs{0}, snr{0}, power{0};
  double range{0}, azimuth{0}, elevation{0};
};

/// Point from a lidar cloud, already transformed into the radar (fusion) frame.
struct LidarPoint
{
  double x{0}, y{0}, z{0};
  float intensity{0};
};

/// Generic position measurement fed to the tracker by any ranging sensor (one radar/lidar cluster).
/// Expandable: radar provides position + over-ground Doppler, lidar position only.
struct Measurement
{
  SensorType sensor{SensorType::RADAR};
  Time stamp{0};
  Eigen::Vector2d pos{Eigen::Vector2d::Zero()};      // x, y in fusion frame
  Eigen::Matrix2d pos_cov{Eigen::Matrix2d::Identity()};
  bool has_doppler{false};
  double doppler{0};          // over-ground radial speed (ego compensated) [m/s]
  double doppler_var{0.25};
  double z{0};                // mean height of the cluster (informative only)
  double extent{0};           // cluster extent [m]
  int num_points{1};
  double rcs{0};
};

/// 2D object from the camera detector / tracker (pixels of the detector image).
struct CameraDetection
{
  uint32_t track_id{0};        // 0 from the detector; set by the 2D tracker
  int class_id{-1};
  std::string label;          // detector label (e.g. COCO "person")
  float score{0};
  float x_min{0}, y_min{0}, x_max{0}, y_max{0};
};

/// All detections of one camera image.
struct CameraFrame
{
  Time stamp{0};
  int width{0}, height{0};
  std::vector<CameraDetection> detections;
};

/// Vehicle motion sample (from /odom or wheel speed + steering).
struct EgoSample
{
  Time stamp{0};
  double speed{0};      // [m/s] forward
  double yaw_rate{0};   // [rad/s]
};

/// Output object of the fusion (fusion frame = radar frame).
struct FusedTrack
{
  uint32_t id{0};
  Eigen::Vector2d pos{Eigen::Vector2d::Zero()};
  Eigen::Vector2d vel{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d pos_cov{Eigen::Matrix2d::Identity()};
  double z{0};
  double yaw_rate{0};
  double heading{0};
  std::vector<double> model_probs;
  double existence{0};
  bool confirmed{false};
  bool ghost{false};                   // double-bounce multipath ghost (never in the final list)
  bool is_moving{false};
  std::string motion_state{"stationary"};   // stationary | moving | turning
  std::string relative_motion_state{"unknown"};
  double closing_speed{0};                  // signed LOS closing speed, positive = closing
  double time_to_collision{-1};             // [s], -1 when not closing
  Eigen::Vector2d predicted_pos{Eigen::Vector2d::Zero()};
  double prediction_horizon{0};
  // ego path
  bool in_path{false};
  double path_distance{0}, path_lateral{0};
  bool predicted_in_path{false};
  bool in_stop_zone{false};                 // inside the stop zone now
  double time_to_stop_zone{-1};             // [s] predicted entry into the stop zone, 0 = inside, -1 = not within horizon
  double predicted_path_distance{0}, predicted_path_lateral{0};
  uint32_t age{0}, hits{0};
  bool radar_updated{false}, lidar_updated{false}, camera_associated{false};
  uint32_t camera_track_id{0};
  std::string class_name{"unknown"};
  double class_confidence{0};
  std::vector<double> class_probs;     // vote shares, order = fusion_common::fusedClasses()
  // "radar" | "radar+camera" (camera associated within the hold time)
  std::string status{"radar"};
  double assoc_prob{0};                // JPDA probability of the camera association
  double bearing_radar{0}, bearing_fused{0};   // [rad]
  double height{0};                    // mean measured height above ground [m]
  // image plane (detector image): fused object centre, radar-only projection
  bool in_image{false};
  double u{0}, v{0};
  double proj_u{0}, proj_v{0};
  // box built from the fused position and the class dimensions
  bool has_bbox{false};
  double bbox[4]{0, 0, 0, 0};          // x_min, y_min, x_max, y_max
  /// @brief Range from the radar [m]: sqrt(x^2 + y^2).
  double range() const {return pos.norm();}
};

}  // namespace radar_fusion
