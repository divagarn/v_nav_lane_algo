#include "radar_fusion/preprocessors.hpp"

#include <cmath>
#include <unordered_map>

namespace radar_fusion
{

namespace
{
/// Simple union-find (disjoint set) with path compression, used by the lidar clustering.
struct DSU
{
  std::vector<int> p;
  /// @param[in] n  number of elements (each starts as its own set)
  explicit DSU(int n) : p(n) {for (int i = 0; i < n; ++i) {p[i] = i;}}
  /// @brief Root of x.
  int f(int x) {return p[x] == x ? x : p[x] = f(p[x]);}
  /// @brief Joins the sets of a and b.
  void u(int a, int b) {p[f(a)] = f(b);}
};

/**
 * @brief Cartesian covariance of a polar measurement.
 * @param[in] r    range [m]
 * @param[in] az   azimuth [rad]
 * @param[in] sr   range std [m]
 * @param[in] saz  azimuth std [rad]
 * @return 2x2 covariance in x/y
 */
Eigen::Matrix2d polarCov(double r, double az, double sr, double saz)
{
  // J * diag(sr^2, saz^2) * J^T for x = r cos az, y = r sin az
  Eigen::Matrix2d J;
  J << std::cos(az), -r * std::sin(az), std::sin(az), r * std::cos(az);
  Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
  S(0, 0) = sr * sr;
  S(1, 1) = saz * saz;
  return J * S * J.transpose();
}
}  // namespace

// Ego-motion compensation of the radial speed.
double RadarPreprocessor::groundDoppler(const RadarDetection & d, const Eigen::Vector2d & vs)
{
  // measured radial speed = (v_obj - v_sensor) . u  ->  v_obj . u = vr + v_sensor . u
  const double r = std::max(1e-3, std::hypot(d.x, d.y));
  return d.radial_speed + (vs.x() * d.x + vs.y() * d.y) / r;
}

// RANSAC: every target with cos(az) >= 0.3 proposes v = -vr / cos(az); the largest inlier set
// is refined by least squares.
bool RadarPreprocessor::estimateSensorSpeed(
  const std::vector<RadarDetection> & d, double & speed, double tol, double min_ratio,
  int min_inliers)
{
  const int n = static_cast<int>(d.size());
  if (n < min_inliers) {return false;}
  int best = 0;
  double best_v = 0;
  for (int i = 0; i < n; ++i) {
    const double c = std::cos(std::atan2(d[i].y, d[i].x));
    if (c < 0.3) {continue;}
    const double v = -d[i].radial_speed / c;
    int inl = 0;
    double sum_num = 0, sum_den = 0;
    for (int k = 0; k < n; ++k) {
      const double ck = std::cos(std::atan2(d[k].y, d[k].x));
      if (std::abs(d[k].radial_speed + v * ck) < tol) {
        ++inl;
        sum_num += -d[k].radial_speed * ck;
        sum_den += ck * ck;
      }
    }
    // least squares on inliers: v = sum(-vr_k c_k) / sum(c_k^2)
    if (inl > best) {best = inl; best_v = sum_den > 0 ? sum_num / sum_den : v;}
  }
  if (best < min_inliers || best < min_ratio * n) {return false;}
  speed = best_v;
  return true;
}

// Height tests above the ground plane with range-proportional elevation slack.
bool RadarPreprocessor::isGroundOrOverhead(const RadarDetection & d, double vg) const
{
  if (!c_.ground_filter) {return false;}
  const double r = std::hypot(d.x, d.y);
  // slack = r * sigma_elevation,  h = z + radar_height
  const double slack = r * c_.elevation_std;
  const double h = d.z + c_.radar_height;   // height above ground
  if (h < -(c_.below_ground_margin + slack)) {return true;}
  if (h > c_.overhead_height + slack) {return true;}
  // no slack here: only weak static returns measured at ground level (keeps standing pedestrians)
  return std::abs(vg) < c_.static_speed && h < c_.ground_band && d.rcs < c_.ground_max_rcs;
}

// Range / SNR / RCS / azimuth / height crop, ground removal, DBSCAN, one measurement per cluster.
std::vector<Measurement> RadarPreprocessor::process(
  const std::vector<RadarDetection> & in, Time stamp, const EgoMotion * ego) const
{
  const Eigen::Vector2d vs = (ego && !ego->empty()) ? ego->sensorVelocity(stamp) : Eigen::Vector2d::Zero();
  std::vector<RadarDetection> d;
  std::vector<double> vg;
  for (const auto & r : in) {
    const double rng = r.range > 0 ? r.range : std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
    const double az = std::atan2(r.y, r.x);
    if (rng < c_.min_range || rng > c_.max_range || r.snr < c_.min_snr || r.rcs < c_.min_rcs ||
      std::abs(az) > c_.max_abs_azimuth || r.z < c_.z_min || r.z > c_.z_max)
    {
      continue;
    }
    const double v = groundDoppler(r, vs);
    if (isGroundOrOverhead(r, v)) {continue;}
    d.push_back(r);
    vg.push_back(v);
  }
  // --- DBSCAN ---
  const int n = static_cast<int>(d.size());
  std::vector<std::vector<int>> nb(n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const double r = std::max(std::hypot(d[i].x, d[i].y), std::hypot(d[j].x, d[j].y));
      const double eps = c_.cluster_eps_xy + c_.cluster_eps_rel * r;
      // neighbours: |p_i - p_j| <= eps_xy + eps_rel * r  and  |vg_i - vg_j| <= eps_doppler
      if (std::hypot(d[i].x - d[j].x, d[i].y - d[j].y) <= eps &&
        std::abs(vg[i] - vg[j]) <= c_.cluster_eps_doppler)
      {
        nb[i].push_back(j);   // includes i itself
      }
    }
  }
  std::vector<int> label(n, -1);
  int nc = 0;
  for (int i = 0; i < n; ++i) {
    if (label[i] >= 0 || static_cast<int>(nb[i].size()) < c_.cluster_min_points) {continue;}
    std::vector<int> queue{i};
    label[i] = nc;
    for (std::size_t q = 0; q < queue.size(); ++q) {
      const int p = queue[q];
      if (static_cast<int>(nb[p].size()) < c_.cluster_min_points) {continue;}   // border point
      for (int k : nb[p]) {
        if (label[k] < 0) {label[k] = nc; queue.push_back(k);}
      }
    }
    ++nc;
  }
  std::unordered_map<int, std::vector<int>> groups;
  for (int i = 0; i < n; ++i) {
    if (label[i] >= 0) {groups[label[i]].push_back(i);}   // unlabelled = noise
  }

  std::vector<Measurement> out;
  for (auto & [root, idx] : groups) {
    (void)root;
    Measurement m;
    m.sensor = SensorType::RADAR;
    m.stamp = stamp;
    // power-weighted centroid (rcs in dBsm -> linear)
    double wsum = 0, z = 0, dop = 0, rcs_max = -1e9;
    Eigen::Vector2d c = Eigen::Vector2d::Zero();
    for (int i : idx) {
      // weight = 10^(rcs / 20) (amplitude), centroid = sum w p / sum w
      const double w = std::pow(10.0, std::clamp(d[i].rcs, -30.0, 40.0) / 20.0);
      c += w * Eigen::Vector2d(d[i].x, d[i].y);
      z += w * d[i].z;
      dop += w * vg[i];
      wsum += w;
      rcs_max = std::max(rcs_max, d[i].rcs);
    }
    c /= wsum;
    z /= wsum;
    dop /= wsum;
    double ext = 0;
    Eigen::Matrix2d spread = Eigen::Matrix2d::Zero();
    for (int i : idx) {
      Eigen::Vector2d e = Eigen::Vector2d(d[i].x, d[i].y) - c;
      ext = std::max(ext, e.norm() * 2);
      spread += e * e.transpose();
    }
    spread /= static_cast<double>(idx.size());
    const double r = c.norm();
    const double az = std::atan2(c.y(), c.x());
    m.pos = c;
    // R = J diag(s_r^2, s_az^2) J^T + spread + glint^2 I
    m.pos_cov = polarCov(r, az, c_.range_std, c_.azimuth_std) + spread +
      Eigen::Matrix2d::Identity() * (c_.glint_std * c_.glint_std);
    m.has_doppler = c_.use_doppler;
    m.doppler = dop;
    m.doppler_var = c_.doppler_std * c_.doppler_std;
    m.z = z;
    m.extent = ext;
    m.num_points = static_cast<int>(idx.size());
    m.rcs = rcs_max;
    out.push_back(m);
  }
  return out;
}

// Crop, 2D voxel averaging, union-find clustering of voxels closer than cluster_eps.
std::vector<Measurement> LidarPreprocessor::process(
  const std::vector<LidarPoint> & pts, Time stamp) const
{
  // voxel downsample (keep one point per voxel), then Euclidean clustering on the grid
  struct Key
  {
    int x, y;
    /// @brief Same voxel.
    bool operator==(const Key & o) const {return x == o.x && y == o.y;}
  };
  struct KH
  {
    /// @brief Hash of the voxel index: (x << 32) ^ y.
    std::size_t operator()(const Key & k) const {return std::hash<long long>()((static_cast<long long>(k.x) << 32) ^ (k.y & 0xffffffff));}
  };
  std::unordered_map<Key, std::pair<Eigen::Vector3d, int>, KH> vox;
  for (const auto & p : pts) {
    const double r = std::hypot(p.x, p.y);
    if (r < c_.min_range || r > c_.max_range || p.z < c_.z_min || p.z > c_.z_max) {continue;}
    Key k{static_cast<int>(std::floor(p.x / c_.voxel)), static_cast<int>(std::floor(p.y / c_.voxel))};
    auto & v = vox[k];
    v.first += Eigen::Vector3d(p.x, p.y, p.z);
    v.second += 1;
  }
  std::vector<Key> keys;
  std::vector<Eigen::Vector3d> cen;
  std::unordered_map<Key, int, KH> index;
  for (auto & [k, v] : vox) {
    index[k] = static_cast<int>(keys.size());
    keys.push_back(k);
    cen.push_back(v.first / v.second);
  }
  DSU dsu(static_cast<int>(keys.size()));
  const int reach = static_cast<int>(std::ceil(c_.cluster_eps / c_.voxel));
  for (std::size_t i = 0; i < keys.size(); ++i) {
    for (int dx = -reach; dx <= reach; ++dx) {
      for (int dy = -reach; dy <= reach; ++dy) {
        auto it = index.find(Key{keys[i].x + dx, keys[i].y + dy});
        if (it != index.end() && (cen[i] - cen[it->second]).head<2>().norm() < c_.cluster_eps) {
          dsu.u(static_cast<int>(i), it->second);
        }
      }
    }
  }
  std::unordered_map<int, std::vector<int>> groups;
  for (std::size_t i = 0; i < keys.size(); ++i) {groups[dsu.f(static_cast<int>(i))].push_back(static_cast<int>(i));}
  std::vector<Measurement> out;
  for (auto & [root, idx] : groups) {
    (void)root;
    int npts = 0;
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (int i : idx) {c += cen[i] * vox[keys[i]].second; npts += vox[keys[i]].second;}
    if (npts < c_.cluster_min_points) {continue;}
    c /= npts;
    Measurement m;
    m.sensor = SensorType::LIDAR;
    m.stamp = stamp;
    m.pos = c.head<2>();
    m.pos_cov = Eigen::Matrix2d::Identity() * c_.pos_std * c_.pos_std;
    m.z = c.z();
    m.num_points = npts;
    double ext = 0;
    for (int i : idx) {ext = std::max(ext, 2 * (cen[i].head<2>() - m.pos).norm());}
    m.extent = ext;
    out.push_back(m);
  }
  return out;
}

}  // namespace radar_fusion
