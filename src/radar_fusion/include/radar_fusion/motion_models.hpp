/**
 * @file motion_models.hpp
 * @brief Motion models for the IMM. Common state: [x, y, vx, vy, omega].
 */
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

namespace radar_fusion
{

constexpr int kStateDim = 5;
using StateVec = Eigen::Matrix<double, kStateDim, 1>;
using StateMat = Eigen::Matrix<double, kStateDim, kStateDim>;

/// Interface of an IMM motion model.
class MotionModel
{
public:
  virtual ~MotionModel() = default;
  /// @brief Short model name (ST, CV, CT).
  virtual std::string name() const = 0;
  /**
   * @brief Propagates the state.
   * @param[in]  x   state at t
   * @param[in]  dt  interval [s]
   * @param[out] F   Jacobian d x(t+dt) / d x(t) for the EKF covariance
   * @return state at t + dt
   */
  virtual StateVec predict(const StateVec & x, double dt, StateMat & F) const = 0;
  /**
   * @brief Process noise covariance Q for the interval.
   * @param[in] x   state (used by state-dependent models)
   * @param[in] dt  interval [s]
   * @return Q
   */
  virtual StateMat processNoise(const StateVec & x, double dt) const = 0;
};

/// Stationary: velocity and turn rate forced to 0, small position random walk.
/// With ego-compensated Doppler it separates parked/static objects from movers.
class StationaryModel : public MotionModel
{
public:
  /// @param[in] pos_std  position random walk [m/sqrt(s)]
  explicit StationaryModel(double pos_std) : q_(pos_std * pos_std) {}
  /// @brief "ST"
  std::string name() const override {return "ST";}
  StateVec predict(const StateVec & x, double dt, StateMat & F) const override;
  StateMat processNoise(const StateVec & x, double dt) const override;

private:
  double q_;
};

/// Constant velocity: omega is forced towards 0.
class ConstantVelocityModel : public MotionModel
{
public:
  /// @param[in] accel_std  white acceleration noise [m/s^2]
  explicit ConstantVelocityModel(double accel_std) : q_(accel_std * accel_std) {}
  /// @brief "CV"
  std::string name() const override {return "CV";}
  StateVec predict(const StateVec & x, double dt, StateMat & F) const override;
  StateMat processNoise(const StateVec & x, double dt) const override;

private:
  double q_;
};

/// Coordinated turn with unknown, slowly varying turn rate.
class CoordinatedTurnModel : public MotionModel
{
public:
  /**
   * @param[in] accel_std      white acceleration noise [m/s^2]
   * @param[in] yaw_accel_std  white turn-rate acceleration noise [rad/s^2]
   */
  CoordinatedTurnModel(double accel_std, double yaw_accel_std)
  : q_(accel_std * accel_std), qw_(yaw_accel_std * yaw_accel_std) {}
  /// @brief "CT"
  std::string name() const override {return "CT";}
  StateVec predict(const StateVec & x, double dt, StateMat & F) const override;
  StateMat processNoise(const StateVec & x, double dt) const override;

private:
  double q_, qw_;
};

/**
 * @brief Factory for the configured models.
 * @param[in] type           st | cv | ct
 * @param[in] accel_std      acceleration noise (for st: position random walk [m/sqrt(s)])
 * @param[in] yaw_accel_std  turn-rate noise (ct only)
 * @return model
 * @throws std::invalid_argument for an unknown type
 */
std::unique_ptr<MotionModel> makeMotionModel(
  const std::string & type, double accel_std, double yaw_accel_std);

}  // namespace radar_fusion
