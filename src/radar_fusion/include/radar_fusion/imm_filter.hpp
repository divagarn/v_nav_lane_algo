/**
 * @file imm_filter.hpp
 * @brief Interacting Multiple Model filter (EKF per mode) over the common 5D state [x, y, vx, vy, omega].
 */
#pragma once

#include <memory>
#include <vector>

#include "radar_fusion/motion_models.hpp"
#include "radar_fusion/types.hpp"

namespace radar_fusion
{

/// IMM set-up shared by all tracks.
struct ImmConfig
{
  std::vector<std::shared_ptr<const MotionModel>> models;
  Eigen::MatrixXd transition;   // Markov mode transition matrix (rows sum to 1)
  std::vector<double> initial_mode_probs;
};

/// IMM estimator of one track.
class ImmFilter
{
public:
  /**
   * @param[in] cfg  model set and transition matrix (must outlive the filter)
   * @param[in] x0   initial state, used for every mode
   * @param[in] P0   initial covariance
   */
  ImmFilter(const ImmConfig * cfg, const StateVec & x0, const StateMat & P0);

  /**
   * @brief Interaction (mixing) + per-mode EKF prediction + combination.
   * @param[in] dt  prediction interval [s]
   */
  void predict(double dt);
  /**
   * @brief Applies a rigid 2D transform (ego-motion compensation) to every mode:
   *        p' = R p + t, v' = R v, P' = T P T^T.
   * @param[in] R  2x2 rotation
   * @param[in] t  translation [m]
   */
  void transform(const Eigen::Matrix2d & R, const Eigen::Vector2d & t);

  /**
   * @brief Innovation statistics of the combined estimate for gating / association.
   * @param[in]  m        measurement (position, optional Doppler)
   * @param[out] log_det  if not null, receives ln|S|
   * @return squared Mahalanobis distance d^2 = y^T S^-1 y
   */
  double gatingDistance(const Measurement & m, double * log_det = nullptr) const;
  /**
   * @brief EKF update of each mode + mode-probability update + combination.
   * @param[in] m  associated measurement
   */
  void update(const Measurement & m);

  /// @brief Combined state estimate.
  const StateVec & x() const {return x_;}
  /// @brief Combined covariance.
  const StateMat & P() const {return P_;}
  /// @brief Mode probabilities mu_j (order of ImmConfig::models).
  const std::vector<double> & modeProbs() const {return mu_;}

private:
  struct Mode
  {
    StateVec x;
    StateMat P;
  };
  /**
   * @brief Measurement model: position (+ Doppler) prediction, Jacobian and noise.
   * @param[in]  x       state
   * @param[in]  m       measurement (defines whether Doppler is used)
   * @param[out] z_pred  predicted measurement
   * @param[out] H       measurement Jacobian
   * @param[out] R       measurement noise
   */
  static void measModel(
    const StateVec & x, const Measurement & m, Eigen::VectorXd & z_pred, Eigen::MatrixXd & H,
    Eigen::MatrixXd & R);
  /// @brief Measurement vector (x, y[, doppler]).
  static Eigen::VectorXd measVector(const Measurement & m);
  /// @brief Combined x_, P_ from the modes weighted by mu_.
  void combine();

  const ImmConfig * cfg_;
  std::vector<Mode> modes_;
  std::vector<double> mu_;
  StateVec x_;
  StateMat P_;
};

}  // namespace radar_fusion
