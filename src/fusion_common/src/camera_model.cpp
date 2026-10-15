#include "fusion_common/camera_model.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace fusion_common
{

namespace
{
/**
 * @brief Opens an OpenCV YAML file for reading.
 * @param[in] path  file path
 * @return open FileStorage
 * @throws std::runtime_error if the file cannot be opened
 */
cv::FileStorage open(const std::string & path)
{
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {throw std::runtime_error("cannot open " + path);}
  return fs;
}

/**
 * @brief Reads intrinsics in either supported format (ChArUco K/D or camera: node).
 * @param[in]  path  intrinsics YAML path
 * @param[out] c     width, height, fx, fy, cx, cy, lens model and D are filled
 * @throws std::runtime_error on missing nodes or invalid values
 */
void readIntrinsics(const std::string & path, CameraCalibration & c)
{
  cv::FileStorage fs = open(path);
  std::string model = "fisheye";
  std::vector<double> d;
  if (!fs["K"].empty()) {   // ChArUco tool output
    cv::Mat K, D;
    fs["K"] >> K;
    fs["D"] >> D;
    K.convertTo(K, CV_64F);
    D.convertTo(D, CV_64F);
    if (K.rows != 3 || K.cols != 3) {throw std::runtime_error("K must be 3x3 in " + path);}
    c.fx = K.at<double>(0, 0); c.cx = K.at<double>(0, 2);
    c.fy = K.at<double>(1, 1); c.cy = K.at<double>(1, 2);
    c.width = static_cast<int>(fs["image_width"]);
    c.height = static_cast<int>(fs["image_height"]);
    if (!fs["lens_model"].empty()) {fs["lens_model"] >> model;}
    d.assign(D.begin<double>(), D.end<double>());
  } else {
    const cv::FileNode cam = fs["camera"];
    if (cam.empty()) {throw std::runtime_error("no 'K' or 'camera' node in " + path);}
    c.width = static_cast<int>(cam["width"]);
    c.height = static_cast<int>(cam["height"]);
    c.fx = cam["fx"]; c.fy = cam["fy"]; c.cx = cam["cx"]; c.cy = cam["cy"];
    if (!cam["distortion_model"].empty()) {cam["distortion_model"] >> model;}
    if (!cam["distortion_coeffs"].empty()) {cam["distortion_coeffs"] >> d;}
  }
  c.lens = model == "fisheye" || model == "equidistant" ? LensModel::FISHEYE : LensModel::PINHOLE;
  c.D.setZero();
  for (std::size_t i = 0; i < d.size() && i < 5; ++i) {c.D[i] = d[i];}
  if (c.fx <= 0 || c.width <= 0) {throw std::runtime_error("invalid intrinsics in " + path);}
}

/**
 * @brief Reads the radar_to_camera extrinsics and folds the perception_apps remap into R.
 * @param[in]  path  extrinsics YAML path
 * @param[out] c     R and t are filled (p_cam = R * p_radar + t)
 * @throws std::runtime_error on missing nodes or wrong sizes
 */
void readExtrinsics(const std::string & path, CameraCalibration & c)
{
  cv::FileStorage fs = open(path);
  const cv::FileNode n = fs["extrinsics"]["radar_to_camera"];
  if (n.empty()) {throw std::runtime_error("missing extrinsics/radar_to_camera in " + path);}
  std::vector<double> r, t;
  n["rotation"] >> r;
  n["translation"] >> t;
  if (r.size() != 9 || t.size() != 3) {throw std::runtime_error("bad rotation/translation in " + path);}
  const Eigen::Matrix3d R_pa = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(r.data());
  // R = R_pa * M with M * (x, y, z) = (-y, -z, x)
  // perception_apps remap (-y, -z, x) of the radar point, folded into R
  Eigen::Matrix3d M;
  M << 0, -1, 0,
    0, 0, -1,
    1, 0, 0;
  c.R = R_pa * M;
  c.t = Eigen::Vector3d(t[0], t[1], t[2]);
}
}  // namespace

// Intrinsics + extrinsics from the two calibration files.
CameraModel CameraModel::load(const std::string & intrinsics_yaml, const std::string & extrinsics_yaml)
{
  CameraCalibration c;
  readIntrinsics(intrinsics_yaml, c);
  readExtrinsics(extrinsics_yaml, c);
  return CameraModel(c);
}

// Intrinsics only, identity extrinsics.
CameraModel CameraModel::loadIntrinsics(const std::string & intrinsics_yaml)
{
  CameraCalibration c;
  readIntrinsics(intrinsics_yaml, c);
  return CameraModel(c);
}

// Scales fx, cx by width ratio and fy, cy by height ratio.
void CameraModel::setImageSize(int width, int height)
{
  if (width <= 0 || height <= 0 || (width == c_.width && height == c_.height)) {return;}
  const double sx = static_cast<double>(width) / c_.width;
  const double sy = static_cast<double>(height) / c_.height;
  // fx' = fx * w'/w, cx' = cx * w'/w (same for y with h'/h)
  c_.fx *= sx; c_.cx *= sx; c_.fy *= sy; c_.cy *= sy;
  c_.width = width; c_.height = height;
}

// K = [fx 0 cx; 0 fy cy; 0 0 1]
cv::Mat CameraModel::K() const
{
  return (cv::Mat_<double>(3, 3) << c_.fx, 0, c_.cx, 0, c_.fy, c_.cy, 0, 0, 1);
}

// Distortion row vector in the length OpenCV expects for the lens model.
cv::Mat CameraModel::D() const
{
  const int n = fisheye() ? 4 : 5;
  cv::Mat d(1, n, CV_64F);
  for (int i = 0; i < n; ++i) {d.at<double>(i) = c_.D[i];}
  return d;
}

// New pinhole camera matrix via OpenCV (balance between valid pixels and full view).
CameraModel CameraModel::rectified(double balance, double fov_scale) const
{
  const cv::Size size(c_.width, c_.height);
  cv::Mat newK;
  if (fisheye()) {
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
      K(), D(), size, cv::Mat::eye(3, 3, CV_64F), newK, balance, size, fov_scale);
  } else {
    newK = cv::getOptimalNewCameraMatrix(K(), D(), size, balance, size);
  }
  CameraCalibration r = c_;
  r.lens = LensModel::PINHOLE;
  r.D.setZero();
  r.fx = newK.at<double>(0, 0); r.fy = newK.at<double>(1, 1);
  r.cx = newK.at<double>(0, 2); r.cy = newK.at<double>(1, 2);
  return CameraModel(r);
}

// Lens projection of an optical-frame point (equidistant fisheye or plumb_bob).
Eigen::Vector2d CameraModel::projectCamPoint(const Eigen::Vector3d & p) const
{
  const auto & D = c_.D;
  if (fisheye()) {
    const double rr = std::hypot(p.x(), p.y());
    // theta = angle from the optical axis
    const double th = std::atan2(rr, p.z());
    const double th2 = th * th;
    // theta_d = theta * (1 + k1 th^2 + k2 th^4 + k3 th^6 + k4 th^8)
    const double thd = th * (1 + D[0] * th2 + D[1] * th2 * th2 + D[2] * th2 * th2 * th2 +
      D[3] * th2 * th2 * th2 * th2);
    const double s = rr > 1e-9 ? thd / rr : 1.0 / std::max(1e-9, p.z());
    // u = fx * (theta_d / r) * x + cx,  v = fy * (theta_d / r) * y + cy
    return {c_.fx * s * p.x() + c_.cx, c_.fy * s * p.y() + c_.cy};
  }
  const double z = std::max(1e-6, p.z());
  // normalised coordinates x = X/Z, y = Y/Z
  const double x = p.x() / z, y = p.y() / z;
  const double r2 = x * x + y * y;
  // radial = 1 + k1 r^2 + k2 r^4 + k3 r^6
  const double radial = 1 + D[0] * r2 + D[1] * r2 * r2 + D[4] * r2 * r2 * r2;
  // tangential: xd = x*radial + 2 p1 x y + p2 (r^2 + 2x^2); yd = y*radial + p1 (r^2 + 2y^2) + 2 p2 x y
  const double xd = x * radial + 2 * D[2] * x * y + D[3] * (r2 + 2 * x * x);
  const double yd = y * radial + D[2] * (r2 + 2 * y * y) + 2 * D[3] * x * y;
  return {c_.fx * xd + c_.cx, c_.fy * yd + c_.cy};
}

// Radar point -> optical frame -> pixel, with field-of-view and image-bounds checks.
std::optional<Eigen::Vector2d> CameraModel::projectRadarPoint(
  const Eigen::Vector3d & p_radar, double margin_px, double max_angle_rad) const
{
  // p_cam = R * p_radar + t
  const Eigen::Vector3d p = c_.R * p_radar + c_.t;
  // off-axis angle = atan2(sqrt(x^2 + y^2), z)
  const double angle = std::atan2(std::hypot(p.x(), p.y()), p.z());
  // a pinhole model cannot image points at 90 deg or more off axis
  if (angle > (fisheye() ? max_angle_rad : std::min(max_angle_rad, 1.4))) {return std::nullopt;}
  Eigen::Vector2d uv = projectCamPoint(p);
  if (uv.x() < -margin_px || uv.y() < -margin_px || uv.x() > c_.width + margin_px ||
    uv.y() > c_.height + margin_px)
  {
    return std::nullopt;
  }
  return uv;
}

// Central differences on each radar axis, step h = 1 mm.
Eigen::Matrix<double, 2, 3> CameraModel::projectionJacobian(const Eigen::Vector3d & p_radar) const
{
  Eigen::Matrix<double, 2, 3> J;
  const double h = 1e-3;
  for (int k = 0; k < 3; ++k) {
    Eigen::Vector3d a = p_radar, b = p_radar;
    a[k] += h;
    b[k] -= h;
    // J_k = (pi(p + h e_k) - pi(p - h e_k)) / (2h)
    J.col(k) = (projectCamPoint(c_.R * a + c_.t) - projectCamPoint(c_.R * b + c_.t)) / (2 * h);
  }
  return J;
}

// Undistorts the pixel to normalised coordinates (x, y, 1) and normalises it.
Eigen::Vector3d CameraModel::pixelToCamRay(const Eigen::Vector2d & uv) const
{
  std::vector<cv::Point2d> in{{uv.x(), uv.y()}}, out;
  if (fisheye()) {
    cv::fisheye::undistortPoints(in, out, K(), D());
  } else {
    cv::undistortPoints(in, out, K(), D());
  }
  return Eigen::Vector3d(out[0].x, out[0].y, 1.0).normalized();
}

// Scales the base model to the image size and rectifies it if requested.
CameraModel cameraForImage(
  const CameraModel & base, int width, int height, bool rectified, double balance, double fov_scale)
{
  CameraModel m = base;
  m.setImageSize(width, height);
  return rectified ? m.rectified(balance, fov_scale) : m;
}

// Remap tables are cached per image size; each frame is one cv::remap.
void ImageRectifier::rectify(const cv::Mat & raw, cv::Mat & out)
{
  if (raw.size() != size_) {
    size_ = raw.size();
    CameraModel in = base_;
    in.setImageSize(size_.width, size_.height);
    out_ = in.rectified(balance_, fov_scale_);
    if (in.fisheye()) {
      cv::fisheye::initUndistortRectifyMap(in.K(), in.D(), cv::Mat::eye(3, 3, CV_64F), out_.K(), size_,
        CV_16SC2, map1_, map2_);
    } else {
      cv::initUndistortRectifyMap(in.K(), in.D(), cv::Mat::eye(3, 3, CV_64F), out_.K(), size_,
        CV_16SC2, map1_, map2_);
    }
  }
  cv::remap(raw, out, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

}  // namespace fusion_common
