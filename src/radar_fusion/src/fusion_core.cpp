#include "radar_fusion/fusion_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "fusion_common/hungarian.hpp"
#include "fusion_common/object_classes.hpp"

namespace radar_fusion
{

// Builds the IMM model set and normalises the transition matrix and initial mode probabilities.
FusionCore::FusionCore(const FusionConfig & cfg)
: cfg_(cfg), radar_pre_(cfg.radar), lidar_pre_(cfg.lidar), ego_(cfg.fusion_frame_mount),
  box_tracker_(cfg.camera_tracker), path_(cfg.path, cfg.fusion_frame_mount)
{
  const auto & tc = cfg_.tracker;
  const int M = static_cast<int>(tc.model_types.size());
  if (M == 0) {throw std::invalid_argument("at least one IMM model required");}
  for (int i = 0; i < M; ++i) {
    std::string type = tc.model_types[i];
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);
    const double q = type == "ct" ? tc.accel_std_ct : type == "st" ? tc.pos_std_st : tc.accel_std_cv;
    imm_cfg_.models.push_back(makeMotionModel(type, q, tc.yaw_accel_std));
    if (type == "st") {idx_st_ = i;}
    if (type == "ct") {idx_ct_ = i;}
  }
  if (static_cast<int>(tc.transition.size()) != M * M) {
    throw std::invalid_argument("imm transition matrix must have models^2 entries");
  }
  imm_cfg_.transition = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
    tc.transition.data(), M, M);
  // pi_ij <- pi_ij / sum_j pi_ij
  for (int i = 0; i < M; ++i) {imm_cfg_.transition.row(i) /= imm_cfg_.transition.row(i).sum();}
  imm_cfg_.initial_mode_probs = tc.initial_mode_probs;
  imm_cfg_.initial_mode_probs.resize(M, 1.0 / M);
  double s = 0;
  for (double v : imm_cfg_.initial_mode_probs) {s += v;}
  for (double & v : imm_cfg_.initial_mode_probs) {v /= s;}
}

// Odometry: yaw rate always, speed only if ego_speed_source is odom.
void FusionCore::addEgo(const EgoSample & s)
{
  last_yaw_rate_ = s.yaw_rate;
  path_.addOdom(s.speed, s.yaw_rate);
  if (cfg_.ego_speed_source == "radar_doppler") {
    // keep odometry yaw rate; speed is filled per radar cycle
    EgoSample y = s;
    y.speed = ego_.empty() ? s.speed : ego_.at(s.stamp).speed;
    ego_.add(y);
    return;
  }
  ego_.add(s);
}

// Clears all state.
void FusionCore::reset()
{
  tracks_.clear();
  box_tracker_.reset();
  camera_tracks_.clear();
  last_stamp_ = -1;
}

// Ego speed (radar Doppler or odom), preprocessing, then the common tracker cycle.
std::size_t FusionCore::processRadar(Time stamp, const std::vector<RadarDetection> & dets)
{
  if (cfg_.ego_speed_source == "radar_doppler") {
    double v = 0;
    const EgoSample odo = ego_.at(stamp);
    if (RadarPreprocessor::estimateSensorSpeed(dets, v)) {
      // sensor speed -> base speed (radar mounted with yaw mount.yaw, ignore lever arm)
      // v_base = v_sensor / cos(mount_yaw)
      v /= std::max(0.3, std::cos(cfg_.fusion_frame_mount.yaw));
    } else {
      v = odo.speed;
    }
    ego_.add(EgoSample{stamp, v, ego_.empty() ? last_yaw_rate_ : odo.yaw_rate});
  }
  last_ego_speed_ = ego_.at(stamp).speed;
  auto meas = radar_pre_.process(dets, stamp, &ego_);
  return runCycle(stamp, std::move(meas), SensorType::RADAR);
}

// Lidar clusters into the same tracker cycle.
std::size_t FusionCore::processLidar(Time stamp, const std::vector<LidarPoint> & pts)
{
  if (!cfg_.lidar_enabled) {return 0;}
  auto meas = lidar_pre_.process(pts, stamp);
  return runCycle(stamp, std::move(meas), SensorType::LIDAR);
}

// Prediction of every track to the new scan time in the new sensor frame.
void FusionCore::predictTo(Time stamp)
{
  if (last_stamp_ < 0) {last_stamp_ = stamp; return;}
  const double dt = stamp - last_stamp_;
  if (dt <= 0) {return;}
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
  Eigen::Vector2d t = Eigen::Vector2d::Zero();
  if (!ego_.empty()) {ego_.frameTransform(last_stamp_, stamp, R, t);}
  for (auto & tr : tracks_) {
    tr.filter->predict(dt);
    tr.filter->transform(R, t);   // express in the sensor frame at `stamp`
  }
  last_stamp_ = stamp;
}

// Initial state at the cluster, velocity along the line of sight from Doppler.
std::unique_ptr<ImmFilter> FusionCore::initFilter(const Measurement & m) const
{
  const auto & tc = cfg_.tracker;
  StateVec x = StateVec::Zero();
  StateMat P = StateMat::Zero();
  x.head<2>() = m.pos;
  P.topLeftCorner<2, 2>() = m.pos_cov;
  const double r = std::max(0.5, m.pos.norm());
  const Eigen::Vector2d u = m.pos / r;           // radial unit vector
  const Eigen::Vector2d n(-u.y(), u.x());        // tangential
  // P_v = s_t^2 n n^T + s_r^2 u u^T  (tangential velocity unknown, radial from Doppler)
  Eigen::Matrix2d Pv = tc.init_vel_std_tangential * tc.init_vel_std_tangential * n * n.transpose();
  if (m.has_doppler) {
    // v0 = doppler * u
    x.segment<2>(2) = m.doppler * u;
    Pv += (tc.init_vel_std_radial * tc.init_vel_std_radial + m.doppler_var) * u * u.transpose();
  } else {
    Pv += tc.init_vel_std_tangential * tc.init_vel_std_tangential * u * u.transpose();
  }
  P.block<2, 2>(2, 2) = Pv;
  P(4, 4) = tc.init_yaw_rate_std * tc.init_yaw_rate_std;
  return std::make_unique<ImmFilter>(&imm_cfg_, x, P);
}


// ------------------------------------------------------------------ radar cycle

// Tracker cycle shared by radar and lidar.
std::size_t FusionCore::runCycle(Time stamp, std::vector<Measurement> meas, SensorType sensor)
{
  if (last_stamp_ >= 0 && stamp < last_stamp_ - 1.0) {reset();}   // bag restarted
  if (last_stamp_ >= 0 && stamp < last_stamp_) {return 0;}        // out of order, drop
  predictTo(stamp);
  last_meas_ = meas;
  for (auto & tr : tracks_) {
    if (sensor == SensorType::RADAR) {tr.radar_updated = false;}
    if (sensor == SensorType::LIDAR) {tr.lidar_updated = false;}
  }

  // --- association (global nearest neighbour, Mahalanobis gating incl. Doppler) ---
  const int nt = static_cast<int>(tracks_.size());
  const int nm = static_cast<int>(meas.size());
  Eigen::MatrixXd d2(nt, nm), cost = Eigen::MatrixXd::Constant(nt, nm, 1e9);
  for (int i = 0; i < nt; ++i) {
    for (int j = 0; j < nm; ++j) {
      double logdet = 0;
      d2(i, j) = tracks_[i].filter->gatingDistance(meas[j], &logdet);
      // cost = d^2 + ln|S| (negative log-likelihood up to a constant), gated by chi2
      if (d2(i, j) < cfg_.tracker.gate_chi2) {cost(i, j) = d2(i, j) + logdet;}
    }
  }
  const auto assign = fusion_common::hungarian(cost, 1e8);
  const auto & tc = cfg_.tracker;
  std::vector<char> used(nm, 0), updated(nt, 0);
  for (int i = 0; i < nt; ++i) {
    auto & tr = tracks_[i];
    tr.age++;
    const bool hit = assign[i] >= 0;
    tr.hit_history.push_back(hit);
    while (static_cast<int>(tr.hit_history.size()) > cfg_.tracker.confirm_window) {
      tr.hit_history.pop_front();
    }
    if (!hit) {
      tr.missed++;
      // miss: pe <- pe * miss_factor
      tr.existence *= tc.existence_miss_factor;
      continue;
    }
    // hit: pe <- pe + gain * (1 - pe)
    tr.existence += tc.existence_hit_gain * (1.0 - tr.existence);
    const auto & m = meas[assign[i]];
    used[assign[i]] = 1;
    updated[i] = 1;
    tr.filter->update(m);
    tr.hits++;
    tr.missed = 0;
    // height EMA: z <- 0.8 z + 0.2 z_meas
    tr.z = tr.hits == 1 ? m.z : 0.8 * tr.z + 0.2 * m.z;
    if (sensor == SensorType::RADAR) {tr.radar_updated = true;}
    if (sensor == SensorType::LIDAR) {tr.lidar_updated = true;}
  }
  // --- new tracks; a second cluster of an object that just updated a track is absorbed ---
  for (int j = 0; j < nm; ++j) {
    if (used[j]) {continue;}
    bool absorbed = false;
    for (int i = 0; i < nt && !absorbed; ++i) {
      absorbed = updated[i] && d2(i, j) < cfg_.tracker.absorb_gate;
    }
    if (absorbed) {continue;}
    Track tr;
    tr.id = next_id_++;
    tr.filter = initFilter(meas[j]);
    tr.hits = 1;
    tr.age = 1;
    tr.hit_history.push_back(true);
    tr.z = meas[j].z;
    tr.existence = tc.existence_init;
    tr.radar_updated = sensor == SensorType::RADAR;
    tr.lidar_updated = sensor == SensorType::LIDAR;
    tracks_.push_back(std::move(tr));
  }
  manageTracks();
  // motion state: EMA of the per-scan evidence with hysteresis (no stationary/moving flicker)
  for (auto & tr : tracks_) {
    // s <- s + alpha * (evidence - s), moving if s > enter, stationary if s < exit
    tr.motion_score += tc.motion_alpha * ((movingEvidence(tr) ? 1.0 : 0.0) - tr.motion_score);
    if (!tr.moving && tr.motion_score > tc.motion_enter) {tr.moving = true;}
    if (tr.moving && tr.motion_score < tc.motion_exit) {tr.moving = false;}
  }
  if (tc.ghost_check) {checkGhosts(stamp);}
  // final-list membership with hysteresis: enter at output_min_existence, leave below
  // output_keep_existence (weak targets such as pedestrians miss single scans)
  for (auto & tr : tracks_) {
    if (!tr.listed && tr.existence >= cfg_.output_min_existence) {tr.listed = true;}
    if (tr.listed && tr.existence < cfg_.output_keep_existence) {tr.listed = false;}
  }
  output_time_ = stamp;
  return meas.size();
}

// Instantaneous "moving" evidence: fast enough, significant, and not explained by the
// stationary model.
bool FusionCore::movingEvidence(const Track & t) const
{
  const auto & tc = cfg_.tracker;
  const Eigen::Vector2d v = t.filter->x().segment<2>(2);
  const double speed = v.norm();
  const Eigen::Matrix2d Pv = t.filter->P().block<2, 2>(2, 2);
  // sigma_speed = sqrt(v^T P_v v) / |v|  (speed std along the velocity direction)
  const double s_speed = speed > 1e-6 ? std::sqrt(std::max(0.0, v.dot(Pv * v))) / speed : 0.0;
  const double p_st = idx_st_ >= 0 ? t.filter->modeProbs()[idx_st_] : 0.0;
  return speed >= tc.moving_speed_threshold && p_st <= tc.stationary_prob && speed >= tc.moving_min_sigma * s_speed;
}

// Double bounce (radar -> object -> ego -> object -> radar): a ghost appears on the bearing
// of a real object at twice its range with twice its relative radial speed.
void FusionCore::checkGhosts(Time stamp)
{
  const auto & tc = cfg_.tracker;
  const Eigen::Vector2d vs = ego_.empty() ? Eigen::Vector2d::Zero() : ego_.sensorVelocity(stamp);
  const int n = static_cast<int>(tracks_.size());
  std::vector<double> r(n), az(n), vr(n);
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector2d p = tracks_[i].filter->x().head<2>();
    r[i] = std::max(0.1, p.norm());
    az[i] = std::atan2(p.y(), p.x());
    // relative radial speed vr = (v_obj - v_sensor) . p / |p|
    vr[i] = (tracks_[i].filter->x().segment<2>(2) - vs).dot(p / r[i]);   // relative radial speed
  }
  const double az_tol = tc.ghost_az_tol_deg * M_PI / 180.0;
  for (int g = 0; g < n; ++g) {
    bool candidate = false;
    for (int k = 0; k < n && !candidate; ++k) {
      if (k == g || !tracks_[k].confirmed || tracks_[k].ghost || r[k] >= r[g]) {continue;}
      // ghost if |az_g - az_k| < tol, |r_g - 2 r_k| < tol r_g, |vr_g - 2 vr_k| < tol and
      // |vr_g - vr_k| > tol (the Doppler must tell 2x from 1x: at low relative speed it cannot,
      // and any object at twice the range of another one would be flagged)
      candidate = std::abs(std::remainder(az[g] - az[k], 2 * M_PI)) < az_tol &&
        std::abs(r[g] - 2.0 * r[k]) < tc.ghost_range_tol * r[g] &&
        std::abs(vr[g] - 2.0 * vr[k]) < tc.ghost_doppler_tol &&
        std::abs(vr[g] - vr[k]) > tc.ghost_doppler_tol;
    }
    auto & t = tracks_[g];
    t.ghost_count = candidate ? t.ghost_count + 1 : std::max(0, t.ghost_count - 1);
    t.ghost = t.ghost_count >= tc.ghost_min_count;
  }
}

// Confirmation (M of N), heading, deletion, duplicate merge.
void FusionCore::manageTracks()
{
  const auto & tc = cfg_.tracker;
  for (auto & tr : tracks_) {
    if (!tr.confirmed) {
      const int h = static_cast<int>(std::count(tr.hit_history.begin(), tr.hit_history.end(), true));
      if (h >= tc.confirm_hits) {tr.confirmed = true;}
    }
    const Eigen::Vector2d v = tr.filter->x().segment<2>(2);
    if (v.norm() > tc.moving_speed_threshold) {tr.heading = std::atan2(v.y(), v.x());}
  }
  // delete
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track & tr) {
      const Eigen::Matrix2d Pp = tr.filter->P().topLeftCorner<2, 2>();
      const double pos_std = std::sqrt(std::max(Pp(0, 0), Pp(1, 1)));
      const bool window_failed = !tr.confirmed &&
      static_cast<int>(tr.hit_history.size()) >= tc.confirm_window;
      return tr.missed > (tr.confirmed ? tc.max_missed : tc.tentative_max_missed) ||
             pos_std > tc.max_pos_std || window_failed || tr.filter->x().head<2>().norm() > tc.max_range;
    }), tracks_.end());
  // merge duplicate confirmed tracks (keep the older one)
  std::sort(tracks_.begin(), tracks_.end(), [](const Track & a, const Track & b) {return a.id < b.id;});
  std::vector<char> dead(tracks_.size(), 0);
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (dead[i] || !tracks_[i].confirmed) {continue;}
    for (std::size_t j = i + 1; j < tracks_.size(); ++j) {
      if (dead[j]) {continue;}
      const Eigen::Vector2d d = tracks_[i].filter->x().head<2>() - tracks_[j].filter->x().head<2>();
      const Eigen::Vector2d dv = tracks_[i].filter->x().segment<2>(2) - tracks_[j].filter->x().segment<2>(2);
      if (d.norm() < tc.merge_distance && dv.norm() < 1.0) {dead[j] = 1;}
    }
  }
  std::vector<Track> keep;
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (!dead[i]) {keep.push_back(std::move(tracks_[i]));}
  }
  tracks_ = std::move(keep);
}

// ------------------------------------------------------------------ camera cycle

// Cached camera model per detector image size.
void FusionCore::updateCameraModel(int width, int height)
{
  if (!have_camera_model_ || width <= 0 || height <= 0) {return;}
  if (cam_size_ == std::make_pair(width, height)) {return;}
  const auto & cc = cfg_.camera;
  camera_ = fusion_common::cameraForImage(base_camera_, width, height, cc.rectified, cc.rectify_balance,
    cc.rectify_fov_scale);
  cam_size_ = {width, height};
}

// Ground range of a pixel (ray to the ground plane), nullopt above the horizon.
std::optional<double> FusionCore::groundRange(double u, double v) const
{
  const Eigen::Vector3d d = camera_.pixelToRadarRay({u, v});
  const Eigen::Vector3d o = camera_.cameraPositionInRadar();
  if (d.z() >= -1e-6) {return std::nullopt;}
  // ray o + s d hits the ground plane z = -h:  s = (-h - o_z) / d_z
  const double s = (-cfg_.camera.radar_height_above_ground - o.z()) / d.z();
  if (s <= 0) {return std::nullopt;}
  return (o + s * d).head<2>().norm();
}

// Bearing (radar frame) of pixel (u, v) at horizontal range r from the radar (parallax exact).
std::optional<double> FusionCore::bearingAtRange(double u, double v, double r) const
{
  const Eigen::Vector3d d3 = camera_.pixelToRadarRay({u, v});
  const Eigen::Vector2d o = camera_.cameraPositionInRadar().head<2>();
  if (d3.head<2>().norm() < 1e-6) {return std::nullopt;}
  const Eigen::Vector2d e = d3.head<2>().normalized();
  // |o + s e| = r  ->  s = -o.e + sqrt((o.e)^2 - |o|^2 + r^2)
  const double oe = o.dot(e), disc = oe * oe - o.squaredNorm() + r * r;
  if (disc < 0) {return std::nullopt;}
  const double s = -oe + std::sqrt(disc);
  if (s <= 0) {return std::nullopt;}
  const Eigen::Vector2d p = o + s * e;
  return std::atan2(p.y(), p.x());
}

// Camera cycle: ByteTrack, gating, bounded cheap JPDA, one-to-one assignment, bearing Bayes, class votes.
std::size_t FusionCore::processCamera(const CameraFrame & f)
{
  const auto & cc = cfg_.camera;
  if (last_stamp_ >= 0 && f.stamp < last_stamp_ - 1.0) {reset();}   // bag restarted
  // 2D tracking of the detector boxes (ByteTrack: high then low score stage)
  std::vector<Detection2D> raw;
  for (const auto & d : f.detections) {
    raw.push_back(Detection2D{d.x_min, d.y_min, d.x_max, d.y_max, d.score, d.class_id, d.label});
  }
  camera_tracks_ = box_tracker_.update(raw, f.stamp);
  if (!cc.enabled || !have_camera_model_) {return 0;}
  updateCameraModel(f.width, f.height);
  output_time_ = f.stamp;

  // confirmed radar tracks at the frame time
  std::vector<int> ti;
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (tracks_[i].confirmed) {ti.push_back(static_cast<int>(i));}
  }
  std::vector<const TrackedBox *> boxes;
  for (const auto & b : camera_tracks_) {
    if (b.det.score >= cc.min_detection_score) {boxes.push_back(&b);}
  }
  const int nb = static_cast<int>(boxes.size()), nt = static_cast<int>(ti.size());
  if (nb == 0 || nt == 0) {return 0;}

  const double W = cam_size_.first, H = cam_size_.second, px = W / 1920.0;
  const double fx = camera_.calib().fx;
  const double hg = -cc.radar_height_above_ground;
  std::vector<double> r(nt), th_r(nt), s_r(nt), u(nt, NAN), v_ground(nt, NAN), v_top(nt, NAN);
  for (int k = 0; k < nt; ++k) {
    const auto & tr = tracks_[ti[k]];
    const Snapshot sn = snapshot(tr, f.stamp);   // track at the frame time (tracker unchanged)
    const Eigen::Vector2d p = sn.p;
    r[k] = std::max(0.1, p.norm());
    th_r[k] = std::atan2(p.y(), p.x());
    // bearing std: J = d atan2(y, x) / d(x, y) = (-y, x) / r^2,  var = J P J^T
    const Eigen::Vector2d j(-p.y() / (r[k] * r[k]), p.x() / (r[k] * r[k]));
    const double var = j.dot(sn.P * j);
    const double smin = cc.sigma_radar_min_deg * M_PI / 180.0;
    s_r[k] = std::sqrt(std::max(var, smin * smin));
    const auto a = camera_.projectRadarPoint({p.x(), p.y(), tr.z}, 1e4);
    const auto g = camera_.projectRadarPoint({p.x(), p.y(), hg}, 1e4);
    const auto t = camera_.projectRadarPoint({p.x(), p.y(), hg + cc.object_height}, 1e4);
    if (a && g && t) {u[k] = a->x(); v_ground[k] = g->y(); v_top[k] = t->y();}
  }

  // gates + cost per (box, track)
  const double BLOCK = 1e6;
  Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(nb, nt, BLOCK);
  Eigen::MatrixXd th_c = Eigen::MatrixXd::Constant(nb, nt, NAN);
  std::vector<double> s_c(nb, 0.0);
  const double m = cc.margin_px * px;
  for (int i = 0; i < nb; ++i) {
    const auto & d = boxes[i]->det;
    const bool bottom_cut = d.y2 >= H - cc.border_px * px;
    const auto rg = bottom_cut ? std::nullopt : groundRange(0.5 * (d.x1 + d.x2), d.y2);
    std::vector<char> g(nt, 0);
    double rmin = 1e9;
    for (int k = 0; k < nt; ++k) {
      if (std::isnan(u[k])) {continue;}
      // gate: projected track column inside the box, [ground, ground + object_height] overlaps the box rows
      bool ok = u[k] >= d.x1 - m && u[k] <= d.x2 + m && v_top[k] <= d.y2 + m && v_ground[k] >= d.y1 - m;
      if (ok && rg) {
        // range gate: ratio_min <= r_radar / r_ground(box bottom) <= ratio_max
        const double ratio = r[k] / *rg;
        ok = ratio >= cc.range_ratio_min && ratio <= cc.range_ratio_max;
      }
      if (ok) {g[k] = 1; rmin = std::min(rmin, r[k]);}
    }
    // camera bearing std = (sigma_px + sigma_frac * box_width) / fx
    s_c[i] = (cc.sigma_px * px + cc.sigma_frac * (d.x2 - d.x1)) / fx;
    for (int k = 0; k < nt; ++k) {
      if (!g[k]) {continue;}
      const auto tc = bearingAtRange(0.5 * (d.x1 + d.x2), 0.5 * (d.y1 + d.y2), r[k]);
      if (!tc) {continue;}
      th_c(i, k) = *tc;
      const double dth = std::remainder(th_r[k] - *tc, 2 * M_PI);
      const double se = cc.sigma_extent / r[k];
      // d_b^2 = dtheta^2 / (s_r^2 + s_c^2 + (sigma_extent / r)^2)
      const double db = dth * dth / (s_r[k] * s_r[k] + s_c[i] * s_c[i] + se * se);
      // cost = d_b^2 + ((r - r_min) / range_pref)^2
      if (db < cc.gate_d2) {cost(i, k) = db + std::pow((r[k] - rmin) / cc.range_pref, 2);}
    }
  }
  // bounded cheap JPDA (best few tracks per box), then one-to-one on -log(probability)
  Eigen::MatrixXd lik = Eigen::MatrixXd::Zero(nb, nt);
  for (int i = 0; i < nb; ++i) {
    std::vector<int> idx;
    for (int k = 0; k < nt; ++k) {if (cost(i, k) < BLOCK) {idx.push_back(k);}}
    std::sort(idx.begin(), idx.end(), [&](int a, int b) {return cost(i, a) < cost(i, b);});
    if (static_cast<int>(idx.size()) > cc.jpda_max_candidates) {idx.resize(cc.jpda_max_candidates);}
    // likelihood = exp(-cost / 2)
    for (int k : idx) {lik(i, k) = std::exp(-0.5 * std::min(cost(i, k), 50.0));}
  }
  const Eigen::MatrixXd beta = cheapJpda(lik, cc.jpda_pd, cc.jpda_clutter);
  Eigen::MatrixXd nl = Eigen::MatrixXd::Constant(nt, nb, BLOCK);   // rows = tracks
  for (int i = 0; i < nb; ++i) {
    for (int k = 0; k < nt; ++k) {
      // assignment cost = -ln(beta)
      if (beta(i, k) > 1e-6) {nl(k, i) = -std::log(beta(i, k));}
    }
  }
  const auto assign = fusion_common::hungarian(nl, BLOCK / 10);

  std::size_t fused = 0;
  for (int k = 0; k < nt; ++k) {
    if (assign[k] < 0) {continue;}
    const int i = assign[k];
    auto & tr = tracks_[ti[k]];
    const auto & d = boxes[i]->det;
    // Gaussian Bayes on the bearing, radar keeps at least radar_min_weight
    // s_c' = sqrt(s_c^2 + (sigma_extent / r)^2)
    const double sc = std::hypot(s_c[i], cc.sigma_extent / r[k]);
    const double dth = std::remainder(th_c(i, k) - th_r[k], 2 * M_PI);
    double delta = 0.0;
    // consistency: |dtheta| <= k * sqrt(s_r^2 + s_c'^2)
    if (std::abs(dth) <= cc.consistency_sigma * std::hypot(s_r[k], sc)) {
      const double wr = 1.0 / (s_r[k] * s_r[k]), wc = 1.0 / (sc * sc);
      // radar weight a = max(w_r / (w_r + w_c), radar_min_weight), w = 1 / sigma^2;  delta = (1 - a) dtheta
      const double a = std::max(wr / (wr + wc), cc.radar_min_weight);
      delta = (1.0 - a) * dth;
    }
    tr.bearing_delta = delta;
    tr.camera_time = f.stamp;
    tr.camera_track_id = boxes[i]->track_id;
    tr.assoc_prob = beta(i, k);
    // class: temporal filter, votes decay per associated frame
    // votes_c <- decay * votes_c, then +1 for the detected class
    for (auto & kv : tr.votes) {kv.second *= cc.class_vote_decay;}
    tr.votes[fusion_common::canonicalClass(d.class_name)] += 1.0;
    ++fused;
  }
  return fused;
}

// Fitzgerald (1985): G = pd * lik, B = clutter (1 - pd).
// beta_tm = G_tm / (S_t + S_m - G_tm + B), S_t = row sum, S_m = column sum; rows renormalised to <= 1.
Eigen::MatrixXd cheapJpda(const Eigen::MatrixXd & lik, double pd, double clutter)
{
  const Eigen::MatrixXd G = pd * lik;
  const Eigen::VectorXd St = G.rowwise().sum();
  const Eigen::RowVectorXd Sm = G.colwise().sum();
  const double B = clutter * (1.0 - pd) + 1e-12;
  Eigen::MatrixXd beta = Eigen::MatrixXd::Zero(G.rows(), G.cols());
  for (int t = 0; t < G.rows(); ++t) {
    for (int m = 0; m < G.cols(); ++m) {
      if (G(t, m) > 0) {beta(t, m) = G(t, m) / (St[t] + Sm[m] - G(t, m) + B);}
    }
    const double s = beta.row(t).sum();
    if (s > 1.0) {beta.row(t) /= s;}
  }
  return beta;
}

// ------------------------------------------------------------------ output

// |t_output - t_camera| <= hold_s
bool FusionCore::cameraRecent(const Track & t) const
{
  return t.camera_time >= 0 && std::abs(output_time_ - t.camera_time) <= cfg_.camera.hold_s;
}

// Constant-velocity prediction of position and covariance, then ego-frame transform.
FusionCore::Snapshot FusionCore::snapshot(const Track & t, Time time) const
{
  Snapshot s;
  const auto & x = t.filter->x();
  const auto & P = t.filter->P();
  const double dt = last_stamp_ >= 0 ? time - last_stamp_ : 0.0;
  // p(t) = p + dt v,  P_p(t) = P_p + dt^2 P_v
  s.p = x.head<2>() + dt * x.segment<2>(2);
  s.v = x.segment<2>(2);
  s.P = P.topLeftCorner<2, 2>() + dt * dt * P.block<2, 2>(2, 2);
  if (std::abs(dt) > 1e-4 && !ego_.empty()) {   // into the sensor frame at `time`
    Eigen::Matrix2d R;
    Eigen::Vector2d tr;
    ego_.frameTransform(last_stamp_, time, R, tr);
    // p' = R p + t, v' = R v, P' = R P R^T
    s.p = R * s.p + tr;
    s.v = R * s.v;
    s.P = R * s.P * R.transpose();
  }
  return s;
}

// Radar position at the output time, rotated by the last camera bearing correction while it
// is recent.
Eigen::Vector2d FusionCore::outputPosition(const Track & t) const
{
  const Eigen::Vector2d p = snapshot(t, output_time_).p;
  if (!cameraRecent(t) || t.bearing_delta == 0.0) {return p;}
  // rotate p by the bearing correction delta: p' = Rot(delta) p
  const double c = std::cos(t.bearing_delta), s = std::sin(t.bearing_delta);
  return {c * p.x() - s * p.y(), s * p.x() + c * p.y()};
}

// Object centre: the radar sees the face towards the sensor, so the centre lies half
// the class extent further along the line of sight, at half the class height.
Eigen::Vector3d FusionCore::classCentre(const Track & t, const Eigen::Vector2d & p) const
{
  const auto & ci = fusion_common::classInfo(bestClass(t, nullptr));
  const double r = std::max(0.5, p.norm());
  const double rel = t.heading - std::atan2(p.y(), p.x());
  // extent along the line of sight = |L cos(rel)| + |W sin(rel)|,  centre = p + u * extent / 2
  const double extent = std::abs(ci.length * std::cos(rel)) + std::abs(ci.width * std::sin(rel));
  const Eigen::Vector2d c = p + p / r * 0.5 * extent;
  return {c.x(), c.y(), -cfg_.camera.radar_height_above_ground + 0.5 * ci.height};
}

// Argmax of the votes; confidence = best / sum.
std::string FusionCore::bestClass(const Track & t, double * conf) const
{
  if (conf) {*conf = 0.0;}
  std::string best = "unknown";
  double bv = 0, sum = 0;
  for (const auto & [k, v] : t.votes) {
    sum += v;
    if (v > bv) {bv = v; best = k;}
  }
  if (conf && sum > 0) {*conf = bv / sum;}
  return best;
}

// 2D box of the projected class-sized 3D box at the fused position.
bool FusionCore::constructBox(const Track & t, const Eigen::Vector2d & pos, double box[4]) const
{
  if (cam_size_.first == 0) {return false;}
  const auto & ci = fusion_common::classInfo(bestClass(t, nullptr));
  const Eigen::Vector3d c = classCentre(t, pos);
  const double ch = std::cos(t.heading), sh = std::sin(t.heading);
  double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  int n = 0;
  for (int i = 0; i < 8; ++i) {
    const double l = (i & 1 ? 0.5 : -0.5) * ci.length, w = (i & 2 ? 0.5 : -0.5) * ci.width;
    const double z = (i & 4 ? 0.5 : -0.5) * ci.height;
    // corner = centre + Rot(heading) (+-L/2, +-W/2) + (0, 0, +-H/2)
    const auto uv = camera_.projectRadarPoint(
      Eigen::Vector3d(c.x() + l * ch - w * sh, c.y() + l * sh + w * ch, c.z() + z), 1e4);
    if (!uv) {continue;}
    x0 = std::min(x0, uv->x()); y0 = std::min(y0, uv->y());
    x1 = std::max(x1, uv->x()); y1 = std::max(y1, uv->y());
    ++n;
  }
  if (n < 4) {return false;}
  box[0] = std::clamp(x0, 0.0, static_cast<double>(cam_size_.first - 1));
  box[1] = std::clamp(y0, 0.0, static_cast<double>(cam_size_.second - 1));
  box[2] = std::clamp(x1, 0.0, static_cast<double>(cam_size_.first - 1));
  box[3] = std::clamp(y1, 0.0, static_cast<double>(cam_size_.second - 1));
  return box[2] - box[0] >= 2 && box[3] - box[1] >= 2;
}

// Fills every output field of one track.
FusedTrack FusionCore::toOutput(const Track & t) const
{
  FusedTrack o;
  const auto & x = t.filter->x();
  o.id = t.id;
  o.pos = outputPosition(t);
  o.vel = snapshot(t, output_time_).v;
  o.pos_cov = t.filter->P().topLeftCorner<2, 2>();
  o.yaw_rate = x[4];
  o.z = t.z;
  o.height = t.z + cfg_.camera.radar_height_above_ground;
  o.model_probs = t.filter->modeProbs();
  o.confirmed = t.confirmed;
  o.ghost = t.ghost;
  const auto & mu = o.model_probs;
  const double p_ct = idx_ct_ >= 0 ? mu[idx_ct_] : 0.0;
  if (!t.moving) {   // filtered state (hysteresis), see runCycle
    o.motion_state = "stationary";
  } else if (p_ct >= *std::max_element(mu.begin(), mu.end()) &&
    std::abs(o.yaw_rate) > cfg_.tracker.turn_rate_threshold)
  {
    o.motion_state = "turning";
  } else {
    o.motion_state = "moving";
  }
  o.is_moving = o.motion_state != "stationary";
  o.heading = t.heading;

  // Relative range rate uses over-ground object velocity minus the radar mount's
  // over-ground velocity. Positive closing_speed means the measured separation is
  // shrinking. This remains meaningful for a stationary obstacle while ego moves.
  const Eigen::Vector2d ego_vel = ego_.empty() ? Eigen::Vector2d::Zero() :
    ego_.sensorVelocity(output_time_);
  const double range = std::max(1e-3, o.pos.norm());
  const Eigen::Vector2d los = o.pos / range;
  o.closing_speed = -(o.vel - ego_vel).dot(los);
  o.time_to_collision = o.closing_speed > cfg_.relative_motion.radial_speed_threshold ?
    range / o.closing_speed : -1.0;

  const auto & rm = cfg_.relative_motion;
  const double ego_speed = ego_vel.norm();
  const double object_speed = o.vel.norm();
  const double same_direction_cos = std::cos(
    rm.same_direction_max_angle * M_PI / 180.0);
  const bool same_direction = o.is_moving && ego_speed >= rm.min_ego_speed &&
    object_speed >= rm.min_object_speed &&
    o.vel.dot(ego_vel) / (object_speed * ego_speed) >= same_direction_cos;
  if (same_direction) {
    o.relative_motion_state = "same_direction";
  } else if (o.closing_speed > rm.radial_speed_threshold) {
    o.relative_motion_state = "approaching";
  } else if (o.closing_speed < -rm.radial_speed_threshold) {
    o.relative_motion_state = "moving_away";
  } else if (!o.is_moving) {
    o.relative_motion_state = "stationary";
  } else {
    o.relative_motion_state = "crossing";
  }

  // predicted_pos: where the object will be over ground, in the CURRENT radar frame, so it is
  // transformed to the output frame exactly like pos (stationary tracks stay where they are).
  // p_pred = p + h v_over_ground
  o.prediction_horizon = std::max(0.0, rm.prediction_horizon);
  o.predicted_pos = o.is_moving ? Eigen::Vector2d(o.pos + o.prediction_horizon * o.vel) : o.pos;
  // relative to the ego at the horizon (ego moves too): what the corridor test below needs
  Eigen::Vector2d rel_future = snapshot(t, output_time_ + o.prediction_horizon).p;
  if (cameraRecent(t) && t.bearing_delta != 0.0) {
    const double c = std::cos(t.bearing_delta), s = std::sin(t.bearing_delta);
    rel_future = Eigen::Vector2d(c * rel_future.x() - s * rel_future.y(), s * rel_future.x() + c * rel_future.y());
  }
  o.age = static_cast<uint32_t>(t.age);
  o.hits = static_cast<uint32_t>(t.hits);
  o.existence = t.existence;
  o.radar_updated = t.radar_updated;
  o.lidar_updated = t.lidar_updated;
  // camera
  o.camera_associated = cameraRecent(t);
  o.status = o.camera_associated ? "radar+camera" : "radar";
  o.camera_track_id = o.camera_associated ? t.camera_track_id : 0;
  o.assoc_prob = o.camera_associated ? t.assoc_prob : 0.0;
  o.bearing_radar = std::atan2(x[1], x[0]);
  o.bearing_fused = std::atan2(o.pos.y(), o.pos.x());
  o.class_name = bestClass(t, &o.class_confidence);
  if (!t.votes.empty()) {
    double sum = 0;
    for (const auto & kv : t.votes) {sum += kv.second;}
    for (const auto & c : fusion_common::fusedClasses()) {
      const auto it = t.votes.find(c);
      o.class_probs.push_back(it == t.votes.end() ? 0.0 : it->second / sum);
    }
  }
  const double half_w = 0.5 * (o.class_name == "unknown" ? 1.0 : fusion_common::classInfo(o.class_name).width);
  const PathCheck pc = path_.check(o.pos, half_w);
  o.in_path = pc.in_path;
  o.path_distance = pc.distance;
  o.path_lateral = pc.lateral;
  const PathCheck predicted_pc = path_.check(rel_future, half_w);
  o.predicted_in_path = predicted_pc.in_path;
  o.predicted_path_distance = predicted_pc.distance;
  o.predicted_path_lateral = predicted_pc.lateral;
  o.time_to_stop_zone = timeToStopZone(t, half_w);
  o.in_stop_zone = o.time_to_stop_zone == 0.0;
  // image (detector image): fused object centre, radar-only projection, class box
  if (have_camera_model_ && cam_size_.first > 0) {
    const auto uv = camera_.projectRadarPoint(classCentre(t, o.pos), 0.0);
    const auto pr = camera_.projectRadarPoint(classCentre(t, x.head<2>()), 1e4);
    if (uv) {
      o.in_image = true;
      o.u = uv->x();
      o.v = uv->y();
      if (pr) {o.proj_u = pr->x(); o.proj_v = pr->y();}
      o.has_bbox = constructBox(t, o.pos, o.bbox);
    }
  }
  return o;
}

// Every confirmed track.
std::vector<FusedTrack> FusionCore::confirmedTracks() const
{
  std::vector<FusedTrack> out;
  for (const auto & t : tracks_) {
    if (t.confirmed) {out.push_back(toOutput(t));}
  }
  return out;
}

// ------------------------------------------------------------------ stop zone

// L = min_distance + |v_ego| * time_headway
double FusionCore::stopZoneLength() const
{
  const double v = ego_.empty() ? 0.0 : std::abs(ego_.at(output_time_).speed);
  return cfg_.stop_zone.min_distance + v * cfg_.stop_zone.time_headway;
}

// Corridor from the radar to L ahead of it (path arc lengths are measured from base_link).
std::vector<Eigen::Vector2d> FusionCore::stopZone() const
{
  const double s0 = std::max(0.0, cfg_.fusion_frame_mount.x);
  return path_.section(s0, s0 + stopZoneLength());
}

// Sensor frame at last_stamp_ -> sensor frame at output_time_ + k step, ego holding its last
// speed / yaw rate beyond the newest odometry sample (EgoMotion::at holds the last sample).
const FusionCore::FutureFrames & FusionCore::futureFrames() const
{
  if (future_.from == last_stamp_ && future_.to == output_time_) {return future_;}
  const auto & sz = cfg_.stop_zone;
  const int n = static_cast<int>(std::floor(std::max(0.0, sz.horizon) / std::max(0.01, sz.step))) + 1;
  future_ = FutureFrames{last_stamp_, output_time_, {}, {}, {}};
  for (int k = 0; k < n; ++k) {
    const double time = output_time_ + k * sz.step;
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Vector2d tr = Eigen::Vector2d::Zero();
    if (!ego_.empty() && std::abs(time - last_stamp_) > 1e-4) {ego_.frameTransform(last_stamp_, time, R, tr);}
    future_.dt.push_back(time - last_stamp_);
    future_.R.push_back(R);
    future_.t.push_back(tr);
  }
  return future_;
}

// p_k = R_k (p + dt_k v) + t_k in the ego frame at step k, inside the zone if it is in the
// corridor between the radar (s0) and s0 + L. Stationary tracks use v = 0 (their relative
// motion is the ego's), so velocity noise on walls does not create entries.
double FusionCore::timeToStopZone(const Track & t, double half_w) const
{
  const auto & ff = futureFrames();
  const auto & x = t.filter->x();
  const Eigen::Vector2d v = t.moving ? Eigen::Vector2d(x.segment<2>(2)) : Eigen::Vector2d::Zero();
  const double s0 = std::max(0.0, cfg_.fusion_frame_mount.x);
  const double s1 = s0 + stopZoneLength();
  const bool rotate = cameraRecent(t) && t.bearing_delta != 0.0;
  const double c = std::cos(t.bearing_delta), s = std::sin(t.bearing_delta);
  for (std::size_t k = 0; k < ff.R.size(); ++k) {
    Eigen::Vector2d p = ff.R[k] * (x.head<2>() + ff.dt[k] * v) + ff.t[k];
    if (rotate) {p = Eigen::Vector2d(c * p.x() - s * p.y(), s * p.x() + c * p.y());}
    const PathCheck pc = path_.check(p, half_w);
    if (pc.in_path && pc.distance >= s0 && pc.distance <= s1) {return k * cfg_.stop_zone.step;}
  }
  return -1.0;
}

// Final list filters: ghost, existence, in front, range, high static radar-only clutter.
std::vector<FusedTrack> FusionCore::objects() const
{
  std::vector<FusedTrack> out;
  for (const auto & t : tracks_) {
    if (!t.confirmed) {continue;}
    FusedTrack o = toOutput(t);
    if (t.ghost || !t.listed) {continue;}   // existence hysteresis, see runCycle
    if (o.pos.x() <= 0.0 || o.range() > cfg_.output_max_range) {continue;}   // in front, within range
    if (!o.is_moving && !o.camera_associated && o.height > cfg_.output_max_static_height) {continue;}
    out.push_back(std::move(o));
  }
  std::sort(out.begin(), out.end(), [](const FusedTrack & a, const FusedTrack & b) {return a.range() < b.range();});
  return out;
}

}  // namespace radar_fusion
