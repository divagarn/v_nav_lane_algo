/**
 * @file ego_motion.hpp
 * @brief Buffers vehicle speed / yaw rate and gives the rigid motion of the fusion
 *        (radar) frame between two time stamps.
 */
#pragma once

#include <deque>

#include "radar_fusion/types.hpp"

namespace radar_fusion
{

/// Planar pose of the sensor in the vehicle base frame.
struct MountPose2D
{
  double x{0}, y{0}, yaw{0};   // sensor (fusion frame) pose in the vehicle base frame
};

/// Ego-motion buffer used for motion compensation of tracks and Doppler.
class EgoMotion
{
public:
  /**
   * @param[in] mount     sensor pose in the vehicle base frame
   * @param[in] buffer_s  history kept [s]
   */
  explicit EgoMotion(MountPose2D mount = {}, double buffer_s = 5.0)
  : mount_(mount), buffer_s_(buffer_s) {}

  /**
   * @brief Appends a vehicle motion sample (older than buffer_s samples are dropped).
   * @param[in] s  speed / yaw-rate sample
   */
  void add(const EgoSample & s);
  /// @brief True when no sample has been received.
  bool empty() const {return buf_.empty();}

  /**
   * @brief Ego speed / yaw rate at time t (linear interpolation, hold at the ends).
   * @param[in] t  query time [s]
   * @return interpolated sample
   */
  EgoSample at(Time t) const;

  /**
   * @brief Rigid transform of the sensor frame from t0 to t1: p1 = R p0 + t.
   * @param[in]  t0  source time [s]
   * @param[in]  t1  target time [s]
   * @param[out] R   2x2 rotation
   * @param[out] t   translation [m]
   */
  void frameTransform(Time t0, Time t1, Eigen::Matrix2d & R, Eigen::Vector2d & t) const;

  /**
   * @brief Over-ground velocity of the sensor, expressed in the sensor frame.
   * @param[in] t  query time [s]
   * @return (vx, vy) [m/s]
   */
  Eigen::Vector2d sensorVelocity(Time t) const;

  /// @brief Sensor mount pose.
  const MountPose2D & mount() const {return mount_;}

private:
  MountPose2D mount_;
  double buffer_s_;
  std::deque<EgoSample> buf_;
};

}  // namespace radar_fusion
