#include "radar_fusion/imm_filter.hpp"

#include <cmath>

namespace radar_fusion
{

// Every mode starts from the same state and covariance.
ImmFilter::ImmFilter(const ImmConfig * cfg, const StateVec & x0, const StateMat & P0)
: cfg_(cfg), modes_(cfg->models.size(), Mode{x0, P0}), mu_(cfg->initial_mode_probs), x_(x0), P_(P0)
{
}

// IMM step 1-2: interaction (mixing) and mode-matched EKF prediction.
void ImmFilter::predict(double dt)
{
  const int M = static_cast<int>(modes_.size());
  const Eigen::MatrixXd & T = cfg_->transition;
  // predicted mode probabilities c_j and mixing weights mu_{i|j}
  std::vector<double> c(M, 0.0);
  for (int j = 0; j < M; ++j) {
    // c_j = sum_i pi_ij mu_i
    for (int i = 0; i < M; ++i) {c[j] += T(i, j) * mu_[i];}
  }
  std::vector<Mode> mixed(M);
  for (int j = 0; j < M; ++j) {
    StateVec xm = StateVec::Zero();
    // x0_j = sum_i mu_{i|j} x_i,  mu_{i|j} = pi_ij mu_i / c_j
    for (int i = 0; i < M; ++i) {xm += (T(i, j) * mu_[i] / c[j]) * modes_[i].x;}
    StateMat Pm = StateMat::Zero();
    for (int i = 0; i < M; ++i) {
      const StateVec d = modes_[i].x - xm;
      // P0_j = sum_i mu_{i|j} (P_i + (x_i - x0_j)(x_i - x0_j)^T)
      Pm += (T(i, j) * mu_[i] / c[j]) * (modes_[i].P + d * d.transpose());
    }
    mixed[j] = {xm, Pm};
  }
  for (int j = 0; j < M; ++j) {
    StateMat F;
    const auto & mdl = *cfg_->models[j];
    modes_[j].x = mdl.predict(mixed[j].x, dt, F);
    // P_j = F P0_j F^T + Q_j
    modes_[j].P = F * mixed[j].P * F.transpose() + mdl.processNoise(mixed[j].x, dt);
  }
  mu_ = c;
  combine();
}

// Ego-motion compensation of every mode.
void ImmFilter::transform(const Eigen::Matrix2d & R, const Eigen::Vector2d & t)
{
  StateMat A = StateMat::Identity();
  A.block<2, 2>(0, 0) = R;
  A.block<2, 2>(2, 2) = R;
  for (auto & m : modes_) {
    // x' = A x + (t, 0, 0, 0),  P' = A P A^T,  A = diag(R, R, 1)
    m.x = A * m.x;
    m.x.head<2>() += t;
    m.P = A * m.P * A.transpose();
  }
  combine();
}

// z = (x, y[, doppler]).
Eigen::VectorXd ImmFilter::measVector(const Measurement & m)
{
  Eigen::VectorXd z(m.has_doppler ? 3 : 2);
  z.head<2>() = m.pos;
  if (m.has_doppler) {z[2] = m.doppler;}
  return z;
}

// h(x) = (x, y[, (x vx + y vy) / r]) and its Jacobian.
void ImmFilter::measModel(
  const StateVec & x, const Measurement & m, Eigen::VectorXd & zp, Eigen::MatrixXd & H,
  Eigen::MatrixXd & R)
{
  const int n = m.has_doppler ? 3 : 2;
  zp.resize(n);
  H = Eigen::MatrixXd::Zero(n, kStateDim);
  R = Eigen::MatrixXd::Zero(n, n);
  zp[0] = x[0];
  zp[1] = x[1];
  H(0, 0) = 1;
  H(1, 1) = 1;
  R.topLeftCorner<2, 2>() = m.pos_cov;
  if (m.has_doppler) {
    // over-ground radial speed: (x vx + y vy) / r
    const double r = std::max(0.5, std::hypot(x[0], x[1]));
    const double ux = x[0] / r, uy = x[1] / r;
    zp[2] = ux * x[2] + uy * x[3];
    // d vr / dx = (vx - vr ux) / r, d vr / dy = (vy - vr uy) / r, d vr / dvx = ux, d vr / dvy = uy
    H(2, 0) = (x[2] - zp[2] * ux) / r;
    H(2, 1) = (x[3] - zp[2] * uy) / r;
    H(2, 2) = ux;
    H(2, 3) = uy;
    R(2, 2) = m.doppler_var;
  }
}

// Innovation of the combined estimate.
double ImmFilter::gatingDistance(const Measurement & m, double * log_det) const
{
  Eigen::VectorXd zp;
  Eigen::MatrixXd H, R;
  measModel(x_, m, zp, H, R);
  // S = H P H^T + R,  d^2 = y^T S^-1 y
  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::VectorXd y = measVector(m) - zp;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
  if (log_det) {*log_det = std::log(std::max(1e-12, S.determinant()));}
  return y.dot(ldlt.solve(y));
}

// IMM step 3-4: mode-matched EKF update and mode-probability update.
void ImmFilter::update(const Measurement & m)
{
  const int M = static_cast<int>(modes_.size());
  const Eigen::VectorXd z = measVector(m);
  std::vector<double> lik(M, 0.0);
  for (int j = 0; j < M; ++j) {
    Eigen::VectorXd zp;
    Eigen::MatrixXd H, R;
    measModel(modes_[j].x, m, zp, H, R);
    const Eigen::MatrixXd S = H * modes_[j].P * H.transpose() + R;
    // K = P H^T S^-1,  x <- x + K y
    const Eigen::MatrixXd K = modes_[j].P * H.transpose() * S.inverse();
    const Eigen::VectorXd y = z - zp;
    modes_[j].x += K * y;
    const StateMat I_KH = StateMat::Identity() - K * H;
    // Joseph form for numerical stability
    // P <- (I - K H) P (I - K H)^T + K R K^T
    modes_[j].P = I_KH * modes_[j].P * I_KH.transpose() + K * R * K.transpose();
    const double d2 = y.dot(S.ldlt().solve(y));
    // Lambda_j = exp(-d^2 / 2) / sqrt((2 pi)^n |S_j|)
    lik[j] = std::exp(-0.5 * d2) / std::sqrt(std::pow(2 * M_PI, z.size()) * S.determinant());
  }
  double norm = 0;
  for (int j = 0; j < M; ++j) {norm += lik[j] * mu_[j];}
  if (norm > 1e-300) {
    // mu_j = Lambda_j c_j / sum_i Lambda_i c_i
    for (int j = 0; j < M; ++j) {mu_[j] = std::max(1e-6, lik[j] * mu_[j] / norm);}
  }
  double s = 0;
  for (double v : mu_) {s += v;}
  for (double & v : mu_) {v /= s;}
  combine();
}

// IMM step 5: x = sum_j mu_j x_j,  P = sum_j mu_j (P_j + (x_j - x)(x_j - x)^T).
void ImmFilter::combine()
{
  x_.setZero();
  for (std::size_t j = 0; j < modes_.size(); ++j) {x_ += mu_[j] * modes_[j].x;}
  P_.setZero();
  for (std::size_t j = 0; j < modes_.size(); ++j) {
    const StateVec d = modes_[j].x - x_;
    P_ += mu_[j] * (modes_[j].P + d * d.transpose());
  }
}

}  // namespace radar_fusion
