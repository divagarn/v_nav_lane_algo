/**
 * @file path_predictor.hpp
 * @brief Predicted ego path: a constant-curvature arc from the vehicle origin, with a
 *        corridor of +-half_width.
 *
 * Curvature from the steering angle (bicycle model, kappa = tan(delta / ratio) / wheelbase)
 * or from odometry (kappa = yaw_rate / speed).
 */
#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "radar_fusion/ego_motion.hpp"
#include "radar_fusion/types.hpp"

namespace radar_fusion
{

/// Ego path parameters (path.* in radar_fusion.yaml).
struct PathConfig
{
  bool enabled{true};
  // steering | odom | auto (steering once a non-zero angle was received, else odom)
  std::string curvature_source{"auto"};
  double wheelbase{2.0};            // [m]
  double steering_ratio{1.0};       // steering-wheel angle / road-wheel angle
  bool steering_in_degrees{true};
  double steering_offset{0.0};      // raw value that means "straight", same unit as the topic
  double steering_sign{1.0};        // +1 if a positive angle turns left
  double half_width{0.9};           // vehicle half width + safety margin [m]
  double length{40.0};              // path length checked [m]
  double min_speed_odom{0.5};       // below this the odom curvature is held
  double max_curvature{0.5};        // [1/m]
  double max_heading_change{1.6};   // [rad] the arc ends after this turn (tight turns)
  double smoothing{0.15};           // EMA weight of each new curvature sample (steering / odom are noisy)
};

/// Result of testing one object against the ego path.
struct PathCheck
{
  bool in_path{false};
  double distance{0};               // arc length along the path [m]
  double lateral{0};                // offset from the path centre, + left [m]
};

/// Ego path from steering or odometry; sets the in_path attribute (never filters objects).
class PathPredictor
{
public:
  /**
   * @param[in] c            path configuration
   * @param[in] radar_mount  radar pose in the vehicle base frame
   */
  PathPredictor(const PathConfig & c, const MountPose2D & radar_mount) : c_(c), mount_(radar_mount) {}

  /**
   * @brief Adds a steering sample and updates the smoothed steering curvature.
   * @param[in] raw_angle  steering angle as published (unit / offset / sign per config)
   */
  void addSteering(double raw_angle);
  /**
   * @brief Adds an odometry sample and updates the smoothed odom curvature (held below min_speed_odom).
   * @param[in] speed     forward speed [m/s]
   * @param[in] yaw_rate  yaw rate [rad/s]
   */
  void addOdom(double speed, double yaw_rate);

  /// @brief Current path curvature kappa [1/m] (+ = left) from the selected source.
  double curvature() const;
  /// @brief Curvature source in use: "steering" or "odom".
  std::string source() const;

  /**
   * @brief Tests whether an object overlaps the path corridor.
   * @param[in] p_radar            object centre in the radar frame [m]
   * @param[in] object_half_width  half width of the object [m]
   * @return in_path flag, arc distance and lateral offset
   */
  PathCheck check(const Eigen::Vector2d & p_radar, double object_half_width) const;
  /**
   * @brief Corridor border for display.
   * @param[in] side  +1 left, -1 right
   * @param[in] step  sample spacing along the arc [m]
   * @return border points in the radar frame
   */
  std::vector<Eigen::Vector2d> border(int side, double step = 1.0) const;
  /**
   * @brief Closed outline of the corridor between two arc lengths (stop-zone display).
   * @param[in] s0    start along the path from base_link [m]
   * @param[in] s1    end [m]
   * @param[in] step  sample spacing [m]
   * @return polygon in the radar frame
   */
  std::vector<Eigen::Vector2d> section(double s0, double s1, double step = 0.5) const;

  /// @brief Path configuration.
  const PathConfig & config() const {return c_;}
  /// @brief Length actually checked: min(path.length, max_heading_change / |kappa|) [m].
  double effectiveLength() const;

private:
  /// @brief Radar frame -> vehicle base frame.
  Eigen::Vector2d toBase(const Eigen::Vector2d & p_radar) const;
  /// @brief Vehicle base frame -> radar frame.
  Eigen::Vector2d toRadar(const Eigen::Vector2d & p_base) const;

  PathConfig c_;
  MountPose2D mount_;
  bool steering_seen_{false}, steering_nonzero_{false};
  double kappa_steer_{0}, kappa_odom_{0};
};

}  // namespace radar_fusion
