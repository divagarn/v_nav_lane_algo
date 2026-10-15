#include "radar_fusion/ego_motion.hpp"

#include <cmath>

namespace radar_fusion
{

namespace
{
/**
 * @brief 2D rotation matrix.
 * @param[in] a  angle [rad]
 * @return [cos -sin; sin cos]
 */
Eigen::Matrix2d rot(double a)
{
  Eigen::Matrix2d R;
  R << std::cos(a), -std::sin(a), std::sin(a), std::cos(a);
  return R;
}
}  // namespace

// Appends a sample, restarts on a backwards time jump, trims to buffer_s.
void EgoMotion::add(const EgoSample & s)
{
  if (!buf_.empty() && s.stamp < buf_.back().stamp) {
    buf_.clear();   // time jumped back (bag loop) -> restart
  }
  buf_.push_back(s);
  while (buf_.size() > 2 && buf_.back().stamp - buf_.front().stamp > buffer_s_) {buf_.pop_front();}
}

// Linear interpolation between neighbouring samples, hold outside the buffer.
EgoSample EgoMotion::at(Time t) const
{
  if (buf_.empty()) {return EgoSample{t, 0, 0};}
  if (t <= buf_.front().stamp) {return buf_.front();}
  if (t >= buf_.back().stamp) {return buf_.back();}
  for (std::size_t i = 1; i < buf_.size(); ++i) {
    if (buf_[i].stamp >= t) {
      const auto & a = buf_[i - 1];
      const auto & b = buf_[i];
      // w = (t - t_a) / (t_b - t_a),  s = a + w (b - a)
      const double w = (t - a.stamp) / std::max(1e-9, b.stamp - a.stamp);
      return EgoSample{t, a.speed + w * (b.speed - a.speed), a.yaw_rate + w * (b.yaw_rate - a.yaw_rate)};
    }
  }
  return buf_.back();
}

// Midpoint integration of speed / yaw rate in 10 ms steps, then mount transform.
void EgoMotion::frameTransform(Time t0, Time t1, Eigen::Matrix2d & R, Eigen::Vector2d & t) const
{
  // integrate base motion t0 -> t1 (pose of base@t1 in base@t0)
  double th = 0, px = 0, py = 0;
  const double dt_total = t1 - t0;
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(dt_total) / 0.01)));
  const double h = dt_total / steps;
  for (int k = 0; k < steps; ++k) {
    const EgoSample s = at(t0 + (k + 0.5) * h);
    // theta_mid = theta + w h / 2,  p += v (cos theta_mid, sin theta_mid) h,  theta += w h
    const double thm = th + 0.5 * s.yaw_rate * h;
    px += s.speed * std::cos(thm) * h;
    py += s.speed * std::sin(thm) * h;
    th += s.yaw_rate * h;
  }
  // T_s1_s0 = T_m^-1 * T_b1_b0 * T_m   with T_b1_b0 = (T_b0_b1)^-1
  const Eigen::Matrix2d Rm = rot(mount_.yaw);
  const Eigen::Vector2d tm(mount_.x, mount_.y);
  const Eigen::Matrix2d Rb = rot(th);
  const Eigen::Vector2d tb(px, py);
  // point in s0 -> b0 -> b1 -> s1
  const Eigen::Matrix2d R_b1_b0 = Rb.transpose();
  const Eigen::Vector2d t_b1_b0 = -Rb.transpose() * tb;
  R = Rm.transpose() * R_b1_b0 * Rm;
  t = Rm.transpose() * (R_b1_b0 * tm + t_b1_b0 - tm);
}

// Rigid-body velocity of the mount point, rotated into the sensor frame.
Eigen::Vector2d EgoMotion::sensorVelocity(Time t) const
{
  const EgoSample s = at(t);
  // velocity of the mount point in base frame: v + w x r
  const Eigen::Vector2d vb(s.speed - s.yaw_rate * mount_.y, s.yaw_rate * mount_.x);
  return rot(mount_.yaw).transpose() * vb;
}

}  // namespace radar_fusion
