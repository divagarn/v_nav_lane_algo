#include "radar_fusion/byte_tracker.hpp"

#include <algorithm>

#include "fusion_common/hungarian.hpp"

namespace radar_fusion
{

namespace
{
/**
 * @brief Intersection over union of box a and detection d.
 * @param[in] ax1, ay1, ax2, ay2  corners of box a [px]
 * @param[in] d                   detection box
 * @return IoU = |a n d| / (|a| + |d| - |a n d|)
 */
float boxIou(float ax1, float ay1, float ax2, float ay2, const Detection2D & d)
{
  const float ix = std::max(0.f, std::min(ax2, d.x2) - std::max(ax1, d.x1));
  const float iy = std::max(0.f, std::min(ay2, d.y2) - std::max(ay1, d.y1));
  const float inter = ix * iy;
  const float u = (ax2 - ax1) * (ay2 - ay1) + (d.x2 - d.x1) * (d.y2 - d.y1) - inter;
  return u > 0 ? inter / u : 0.f;
}
}  // namespace

// Track state (cx, cy, w, h) -> corners, then IoU.
float ByteTracker::iou(const Vec8 & x, const Detection2D & d)
{
  return boxIou(x[0] - x[2] / 2, x[1] - x[3] / 2, x[0] + x[2] / 2, x[1] + x[3] / 2, d);
}

// Constant-velocity prediction on (cx, cy, w, h).
void ByteTracker::predict(Track & t, double dt) const
{
  Mat8 F = Mat8::Identity();
  for (int i = 0; i < 4; ++i) {F(i, i + 4) = dt;}
  // x' = F x, F = [I dt I; 0 I];  P' = F P F^T + Q
  t.x = F * t.x;
  t.x[2] = std::max(2.0, t.x[2]);
  t.x[3] = std::max(2.0, t.x[3]);
  // noise proportional to box size (as in SORT/ByteTrack)
  const double sp = 0.05 * t.x[3], sv = 0.6 * t.x[3];
  Mat8 Q = Mat8::Zero();
  for (int i = 0; i < 4; ++i) {Q(i, i) = sp * sp * dt; Q(i + 4, i + 4) = sv * sv * dt;}
  t.P = F * t.P * F.transpose() + Q;
}

// Kalman update with z = (cx, cy, w, h) of the detection.
void ByteTracker::correct(Track & t, const Detection2D & d) const
{
  Eigen::Matrix<double, 4, 1> z((d.x1 + d.x2) / 2, (d.y1 + d.y2) / 2, d.x2 - d.x1, d.y2 - d.y1);
  Eigen::Matrix<double, 4, 8> H = Eigen::Matrix<double, 4, 8>::Zero();
  for (int i = 0; i < 4; ++i) {H(i, i) = 1;}
  const double sm = 0.05 * z[3];
  const Eigen::Matrix4d R = Eigen::Matrix4d::Identity() * sm * sm;
  // S = H P H^T + R,  K = P H^T S^-1,  x <- x + K (z - H x),  P <- (I - K H) P
  const Eigen::Matrix4d S = H * t.P * H.transpose() + R;
  const Eigen::Matrix<double, 8, 4> K = t.P * H.transpose() * S.inverse();
  t.x += K * (z - H * t.x);
  t.P = (Mat8::Identity() - K * H) * t.P;
  t.det = d;
}

// ByteTrack frame: high-score match, low-score match for the remaining tracks, births, deletion.
std::vector<TrackedBox> ByteTracker::update(const std::vector<Detection2D> & dets, double stamp)
{
  if (last_stamp_ >= 0 && stamp < last_stamp_) {reset();}   // time jumped back (bag loop)
  const double dt = last_stamp_ < 0 ? 0.033 : std::max(1e-3, stamp - last_stamp_);
  last_stamp_ = stamp;
  for (auto & t : tracks_) {predict(t, dt); t.matched = false; t.age++;}

  std::vector<int> high, low;
  for (int i = 0; i < static_cast<int>(dets.size()); ++i) {
    if (dets[i].score >= c_.high_thresh) {high.push_back(i);} else if (dets[i].score >= c_.low_thresh) {low.push_back(i);}
  }
  std::vector<char> det_used(dets.size(), 0);
  auto match = [&](const std::vector<int> & cand, float min_iou) {
      std::vector<int> tidx;
      for (int i = 0; i < static_cast<int>(tracks_.size()); ++i) {
        if (!tracks_[i].matched) {tidx.push_back(i);}
      }
      Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(tidx.size(), cand.size(), 1e9);
      for (std::size_t a = 0; a < tidx.size(); ++a) {
        for (std::size_t b = 0; b < cand.size(); ++b) {
          const auto & d = dets[cand[b]];
          if (c_.class_consistent && tracks_[tidx[a]].det.class_id != d.class_id) {continue;}
          const float v = iou(tracks_[tidx[a]].x, d);
          // cost = 1 - IoU (gated by min IoU)
          if (v >= min_iou) {cost(a, b) = 1.0 - v;}
        }
      }
      const auto as = fusion_common::hungarian(cost, 1.0 - min_iou + 1e-6);
      for (std::size_t a = 0; a < tidx.size(); ++a) {
        if (as[a] < 0) {continue;}
        auto & t = tracks_[tidx[a]];
        correct(t, dets[cand[as[a]]]);
        t.matched = true;
        t.hits++;
        t.last_seen = stamp;
        if (t.hits >= c_.min_hits) {t.confirmed = true;}
        det_used[cand[as[a]]] = 1;
      }
    };
  match(high, c_.match_iou);
  match(low, c_.low_match_iou);

  for (int i : high) {
    if (det_used[i] || dets[i].score < c_.new_track_thresh) {continue;}
    const auto & d = dets[i];
    Track t;
    t.id = next_id_++;
    t.x << (d.x1 + d.x2) / 2, (d.y1 + d.y2) / 2, d.x2 - d.x1, d.y2 - d.y1, 0, 0, 0, 0;
    t.P = Mat8::Identity();
    const double h = d.y2 - d.y1;
    for (int k = 0; k < 4; ++k) {t.P(k, k) = (0.1 * h) * (0.1 * h); t.P(k + 4, k + 4) = (h) * (h);}
    t.det = d;
    t.hits = 1;
    t.age = 1;
    t.last_seen = stamp;
    t.matched = true;
    t.confirmed = c_.min_hits <= 1;
    tracks_.push_back(t);
  }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track & t) {
      return stamp - t.last_seen > (t.confirmed ? c_.max_lost_s : 0.0);
    }), tracks_.end());

  std::vector<TrackedBox> out;
  for (const auto & t : tracks_) {
    if (!t.confirmed || !t.matched) {continue;}
    TrackedBox o;
    o.track_id = t.id;
    o.det = t.det;
    o.det.x1 = static_cast<float>(t.x[0] - t.x[2] / 2);
    o.det.y1 = static_cast<float>(t.x[1] - t.x[3] / 2);
    o.det.x2 = static_cast<float>(t.x[0] + t.x[2] / 2);
    o.det.y2 = static_cast<float>(t.x[1] + t.x[3] / 2);
    o.vu = static_cast<float>(t.x[4]);
    o.vv = static_cast<float>(t.x[5]);
    o.age = t.age;
    out.push_back(o);
  }
  return out;
}

}  // namespace radar_fusion
