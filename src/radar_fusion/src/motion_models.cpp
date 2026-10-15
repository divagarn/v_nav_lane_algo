#include "radar_fusion/motion_models.hpp"

#include <cmath>
#include <stdexcept>

namespace radar_fusion
{

namespace
{
/**
 * @brief Discrete white-noise acceleration covariance on x/y.
 * @param[in] q   acceleration variance [m^2/s^4]
 * @param[in] dt  interval [s]
 * @return Q with q [dt^4/4 dt^3/2; dt^3/2 dt^2] per axis
 */
StateMat accelNoise(double q, double dt)
{
  StateMat Q = StateMat::Zero();
  const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
  for (int i = 0; i < 2; ++i) {
    Q(i, i) = dt4 / 4 * q;
    Q(i, i + 2) = Q(i + 2, i) = dt3 / 2 * q;
    Q(i + 2, i + 2) = dt2 * q;
  }
  return Q;
}
}  // namespace

// x' = (x, y, 0, 0, 0)
StateVec StationaryModel::predict(const StateVec & x, double, StateMat & F) const
{
  F = StateMat::Zero();
  F(0, 0) = 1.0;
  F(1, 1) = 1.0;
  return F * x;
}

// Position random walk q dt, tiny velocity / turn-rate noise.
StateMat StationaryModel::processNoise(const StateVec &, double dt) const
{
  StateMat Q = StateMat::Zero();
  Q(0, 0) = Q(1, 1) = q_ * dt;
  Q(2, 2) = Q(3, 3) = 1e-3;   // keeps the velocity block positive definite for mixing
  Q(4, 4) = 1e-4;
  return Q;
}

// x' = x + vx dt, y' = y + vy dt, v' = v, omega' = 0
StateVec ConstantVelocityModel::predict(const StateVec & x, double dt, StateMat & F) const
{
  F = StateMat::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;
  F(4, 4) = 0.0;
  StateVec xn = F * x;
  return xn;
}

// White acceleration noise, tiny turn-rate noise.
StateMat ConstantVelocityModel::processNoise(const StateVec &, double dt) const
{
  StateMat Q = accelNoise(q_, dt);
  Q(4, 4) = 1e-4;
  return Q;
}

// Constant turn rate omega (falls back to CV + first-order omega terms for |omega| ~ 0).
StateVec CoordinatedTurnModel::predict(const StateVec & s, double dt, StateMat & F) const
{
  const double x = s[0], y = s[1], vx = s[2], vy = s[3], w = s[4];
  StateVec n;
  F = StateMat::Identity();
  if (std::abs(w) < 1e-4) {
    n << x + vx * dt, y + vy * dt, vx, vy, w;
    F(0, 2) = dt;
    F(1, 3) = dt;
    // first-order sensitivity to omega
    F(0, 4) = -0.5 * vy * dt * dt;
    F(1, 4) = 0.5 * vx * dt * dt;
    F(2, 4) = -vy * dt;
    F(3, 4) = vx * dt;
    return n;
  }
  const double sw = std::sin(w * dt), cw = std::cos(w * dt);
  // x' = x + (vx sin(w dt) - vy (1 - cos(w dt))) / w,  y' = y + (vx (1 - cos(w dt)) + vy sin(w dt)) / w
  // v' = Rot(w dt) v
  n[0] = x + (vx * sw - vy * (1 - cw)) / w;
  n[1] = y + (vx * (1 - cw) + vy * sw) / w;
  n[2] = vx * cw - vy * sw;
  n[3] = vx * sw + vy * cw;
  n[4] = w;
  F(0, 2) = sw / w;
  F(0, 3) = -(1 - cw) / w;
  F(1, 2) = (1 - cw) / w;
  F(1, 3) = sw / w;
  F(2, 2) = cw;
  F(2, 3) = -sw;
  F(3, 2) = sw;
  F(3, 3) = cw;
  // d/dw
  F(0, 4) = (vx * cw * dt - vy * sw * dt) / w - (vx * sw - vy * (1 - cw)) / (w * w);
  F(1, 4) = (vx * sw * dt + vy * cw * dt) / w - (vx * (1 - cw) + vy * sw) / (w * w);
  F(2, 4) = -vx * sw * dt - vy * cw * dt;
  F(3, 4) = vx * cw * dt - vy * sw * dt;
  return n;
}

// White acceleration noise + turn-rate random walk qw dt^2.
StateMat CoordinatedTurnModel::processNoise(const StateVec &, double dt) const
{
  StateMat Q = accelNoise(q_, dt);
  Q(4, 4) = qw_ * dt * dt;
  return Q;
}

// Model by type name.
std::unique_ptr<MotionModel> makeMotionModel(
  const std::string & type, double accel_std, double yaw_accel_std)
{
  if (type == "st" || type == "ST") {return std::make_unique<StationaryModel>(accel_std);}
  if (type == "cv" || type == "CV") {return std::make_unique<ConstantVelocityModel>(accel_std);}
  if (type == "ct" || type == "CT") {
    return std::make_unique<CoordinatedTurnModel>(accel_std, yaw_accel_std);
  }
  throw std::invalid_argument("unknown motion model '" + type + "' (use st | cv | ct)");
}

}  // namespace radar_fusion
