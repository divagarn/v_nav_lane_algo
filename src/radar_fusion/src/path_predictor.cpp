#include "radar_fusion/path_predictor.hpp"

#include <algorithm>
#include <cmath>

namespace radar_fusion
{

// Steering angle -> road-wheel angle -> bicycle-model curvature, EMA smoothed.
void PathPredictor::addSteering(double raw)
{
  steering_seen_ = true;
  double a = raw - c_.steering_offset;
  if (std::abs(a) > 1e-3) {steering_nonzero_ = true;}
  if (c_.steering_in_degrees) {a *= M_PI / 180.0;}
  // delta = sign * (raw - offset) / steering_ratio
  const double delta = c_.steering_sign * a / std::max(1e-3, c_.steering_ratio);
  // kappa = tan(delta) / wheelbase, clamped to +-max_curvature
  const double k = std::clamp(std::tan(delta) / std::max(0.1, c_.wheelbase), -c_.max_curvature, c_.max_curvature);
  // EMA: kappa <- kappa + alpha (kappa_new - kappa)
  kappa_steer_ += c_.smoothing * (k - kappa_steer_);
}

// Odometry curvature, EMA smoothed; held at low speed.
void PathPredictor::addOdom(double speed, double yaw_rate)
{
  if (std::abs(speed) < c_.min_speed_odom) {return;}   // curvature undefined at standstill: hold
  // kappa = yaw_rate / speed, clamped to +-max_curvature
  const double k = std::clamp(yaw_rate / speed, -c_.max_curvature, c_.max_curvature);
  kappa_odom_ += c_.smoothing * (k - kappa_odom_);
}

// Configured source, or auto: steering once a non-zero angle was seen.
std::string PathPredictor::source() const
{
  if (c_.curvature_source == "steering") {return "steering";}
  if (c_.curvature_source == "odom") {return "odom";}
  return steering_seen_ && steering_nonzero_ ? "steering" : "odom";
}

// Smoothed curvature of the selected source.
double PathPredictor::curvature() const
{
  return source() == "steering" ? kappa_steer_ : kappa_odom_;
}

// p_base = t_mount + Rot(yaw_mount) p_radar
Eigen::Vector2d PathPredictor::toBase(const Eigen::Vector2d & p) const
{
  const double c = std::cos(mount_.yaw), s = std::sin(mount_.yaw);
  return {mount_.x + c * p.x() - s * p.y(), mount_.y + s * p.x() + c * p.y()};
}

// p_radar = Rot(yaw_mount)^T (p_base - t_mount)
Eigen::Vector2d PathPredictor::toRadar(const Eigen::Vector2d & p) const
{
  const double c = std::cos(mount_.yaw), s = std::sin(mount_.yaw);
  const Eigen::Vector2d d(p.x() - mount_.x, p.y() - mount_.y);
  return {c * d.x() + s * d.y(), -s * d.x() + c * d.y()};
}

// L_eff = min(L, max_heading_change / |kappa|)
double PathPredictor::effectiveLength() const
{
  const double k = std::abs(curvature());
  return k < 1e-4 ? c_.length : std::min(c_.length, c_.max_heading_change / k);
}

// Arc distance and lateral offset of the object w.r.t. the path centre line.
PathCheck PathPredictor::check(const Eigen::Vector2d & p_radar, double object_half_width) const
{
  PathCheck r;
  const Eigen::Vector2d p = toBase(p_radar);
  const double k = curvature();
  if (std::abs(k) < 1e-4) {
    r.distance = p.x();
    r.lateral = p.y();
  } else {
    // arc x = sin(k s) / k, y = (1 - cos(k s)) / k around the centre (0, R)
    const double R = 1.0 / k;
    const double sg = R > 0 ? 1.0 : -1.0;
    // circle centre (0, R): theta = angle travelled, s = |R| theta, lateral = sign (|R| - |p - c|)
    const double theta = std::atan2(p.x(), sg * (R - p.y()));
    r.distance = std::abs(R) * theta;
    r.lateral = sg * (std::abs(R) - std::hypot(p.x(), p.y() - R));
  }
  r.in_path = c_.enabled && r.distance >= 0.0 && r.distance <= effectiveLength() &&
    std::abs(r.lateral) <= c_.half_width + object_half_width;
  return r;
}

// Samples the centre line and offsets it by +-half_width along the left normal.
std::vector<Eigen::Vector2d> PathPredictor::border(int side, double step) const
{
  std::vector<Eigen::Vector2d> out;
  const double k = curvature();
  const double len = effectiveLength();
  for (double s = 0.0; s <= len + 1e-6; s += step) {
    const double psi = k * s;
    // centre(s) = (sin(kappa s) / kappa, (1 - cos(kappa s)) / kappa), left normal = (-sin psi, cos psi)
    const Eigen::Vector2d centre = std::abs(k) < 1e-4 ? Eigen::Vector2d(s, 0.0) :
      Eigen::Vector2d(std::sin(psi) / k, (1.0 - std::cos(psi)) / k);
    const Eigen::Vector2d left(-std::sin(psi), std::cos(psi));
    out.push_back(toRadar(centre + side * c_.half_width * left));
  }
  return out;
}

// Left border from s0 to s1, then the right border back: a closed outline of that stretch.
std::vector<Eigen::Vector2d> PathPredictor::section(double s0, double s1, double step) const
{
  const double k = curvature();
  s1 = std::min(s1, effectiveLength());
  auto point = [&](double s, int side) {
      const double psi = k * s;
      // centre(s) as in border(), offset by side * half_width along the left normal
      const Eigen::Vector2d centre = std::abs(k) < 1e-4 ? Eigen::Vector2d(s, 0.0) :
        Eigen::Vector2d(std::sin(psi) / k, (1.0 - std::cos(psi)) / k);
      return toRadar(centre + side * c_.half_width * Eigen::Vector2d(-std::sin(psi), std::cos(psi)));
    };
  std::vector<Eigen::Vector2d> out;
  if (s1 <= s0) {return out;}
  const int n = std::max(1, static_cast<int>(std::ceil((s1 - s0) / step)));
  for (int i = 0; i <= n; ++i) {out.push_back(point(s0 + (s1 - s0) * i / n, 1));}
  for (int i = n; i >= 0; --i) {out.push_back(point(s0 + (s1 - s0) * i / n, -1));}
  out.push_back(out.front());
  return out;
}

}  // namespace radar_fusion
