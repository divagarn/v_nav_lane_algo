/**
 * @file camera_model.hpp
 * @brief Camera model (fisheye or pinhole) + radar->camera extrinsics.
 *
 * Loaded from the calibration app output (perception_apps OpenCV-YAML format):
 *   - intrinsics: ChArUco tool output (lens_model, image_width, image_height, K, D)
 *                 or camera: {width, height, fx, fy, cx, cy, distortion_model, distortion_coeffs}
 *   - extrinsics: extrinsics: radar_to_camera: {rotation[9], translation[3]}
 *                 p_cam = R_pa * (-y, -z, x)_radar + t   (perception_apps remap convention)
 *
 * Internally the remap is folded into R, so p_cam = R * p_radar + t.
 */
#pragma once

#include <Eigen/Dense>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

namespace fusion_common
{

/// Lens projection model of the camera.
enum class LensModel { FISHEYE, PINHOLE };

/// Raw calibration data: intrinsics, distortion and radar->camera extrinsics.
struct CameraCalibration
{
  int width{0};                    ///< calibrated image width [px]
  int height{0};                   ///< calibrated image height [px]
  double fx{0}, fy{0}, cx{0}, cy{0};   ///< focal lengths and principal point [px]
  LensModel lens{LensModel::FISHEYE};
  /// fisheye: k1..k4 (equidistant, cv::fisheye); pinhole: k1 k2 p1 p2 k3 (plumb_bob)
  Eigen::Matrix<double, 5, 1> D{Eigen::Matrix<double, 5, 1>::Zero()};
  /// p_cam_optical = R * p_radar + t   (radar: x fwd, y left, z up; optical: x right, y down, z fwd)
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t{Eigen::Vector3d::Zero()};   ///< translation [m], see R
};

/**
 * @brief Projection between the radar frame and image pixels for one camera.
 */
class CameraModel
{
public:
  /// @brief Creates an invalid model (valid() == false).
  CameraModel() = default;
  /**
   * @brief Creates a model from an existing calibration.
   * @param[in] c  intrinsics, distortion and extrinsics
   */
  explicit CameraModel(const CameraCalibration & c) : c_(c) {}

  /**
   * @brief Loads the calibration app output (intrinsics + extrinsics files).
   * @param[in] intrinsics_yaml  path of the intrinsics YAML (K/D or camera: format)
   * @param[in] extrinsics_yaml  path of the radar_to_camera extrinsics YAML
   * @return camera model with the perception_apps axis remap folded into R
   * @throws std::runtime_error if a file is missing or malformed
   */
  static CameraModel load(const std::string & intrinsics_yaml, const std::string & extrinsics_yaml);
  /**
   * @brief Loads intrinsics only (identity extrinsics), e.g. for image rectification.
   * @param[in] intrinsics_yaml  path of the intrinsics YAML
   * @return camera model with R = I, t = 0
   * @throws std::runtime_error if the file is missing or malformed
   */
  static CameraModel loadIntrinsics(const std::string & intrinsics_yaml);

  /**
   * @brief Rescales the intrinsics when the incoming image size differs from the calibrated one.
   * @param[in] width   actual image width [px]
   * @param[in] height  actual image height [px]
   */
  void setImageSize(int width, int height);

  /**
   * @brief Undistorted pinhole model of the same camera (D = 0) for rectified images.
   * @param[in] balance    0 = only valid pixels (narrower view), 1 = keep the whole field of view
   * @param[in] fov_scale  > 1 zooms out, < 1 zooms in (divides the new focal length)
   * @return pinhole model with the same extrinsics and the new camera matrix
   */
  CameraModel rectified(double balance, double fov_scale = 1.0) const;

  /**
   * @brief Projects a point given in the radar frame into the image.
   * @param[in] p_radar        point in the radar frame [m]
   * @param[in] margin_px      accept points up to this far outside the image [px]
   * @param[in] max_angle_rad  reject points further than this from the optical axis [rad]
   * @return pixel (u, v), or nullopt if behind the camera / outside image + margin
   */
  std::optional<Eigen::Vector2d> projectRadarPoint(
    const Eigen::Vector3d & p_radar, double margin_px = 0.0, double max_angle_rad = 1.45) const;
  /**
   * @brief Projects a point given in the camera optical frame (no visibility check).
   * @param[in] p_cam  point in the optical frame [m], z > 0
   * @return pixel (u, v) including lens distortion
   */
  Eigen::Vector2d projectCamPoint(const Eigen::Vector3d & p_cam) const;
  /**
   * @brief Numerical Jacobian d(u,v)/d(p_radar) (central differences).
   * @param[in] p_radar  linearisation point in the radar frame [m]
   * @return 2x3 Jacobian [px/m]
   */
  Eigen::Matrix<double, 2, 3> projectionJacobian(const Eigen::Vector3d & p_radar) const;

  /**
   * @brief Unit viewing ray of a pixel in the camera optical frame (undistorted).
   * @param[in] uv  pixel (u, v)
   * @return unit vector in the optical frame
   */
  Eigen::Vector3d pixelToCamRay(const Eigen::Vector2d & uv) const;
  /**
   * @brief Unit viewing ray of a pixel in the radar frame.
   * @param[in] uv  pixel (u, v)
   * @return unit vector in the radar frame
   */
  // ray_radar = R^T * ray_cam
  Eigen::Vector3d pixelToRadarRay(const Eigen::Vector2d & uv) const {return c_.R.transpose() * pixelToCamRay(uv);}
  /**
   * @brief Camera optical centre in the radar frame.
   * @return position [m]
   */
  // p_radar = R^T * (0 - t)
  Eigen::Vector3d cameraPositionInRadar() const {return -c_.R.transpose() * c_.t;}

  /// @brief OpenCV 3x3 camera matrix K (CV_64F).
  cv::Mat K() const;
  /// @brief OpenCV distortion vector: 4 coeffs (fisheye) or 5 (pinhole).
  cv::Mat D() const;
  /// @brief Underlying calibration data.
  const CameraCalibration & calib() const {return c_;}
  /// @brief True when intrinsics have been loaded (fx > 0).
  bool valid() const {return c_.fx > 0;}
  /// @brief True for the equidistant fisheye lens model.
  bool fisheye() const {return c_.lens == LensModel::FISHEYE;}

private:
  CameraCalibration c_;
};

/**
 * @brief Camera model for images of size width x height, rectified (pinhole) or raw.
 *
 * Every node that works on rectified images uses this with the same balance / fov_scale,
 * so detector, fusion and visualization agree on the projection.
 * @param[in] base       calibrated model (raw lens)
 * @param[in] width      image width [px]
 * @param[in] height     image height [px]
 * @param[in] rectified  true if the images are undistorted by ImageRectifier
 * @param[in] balance    rectification balance (see CameraModel::rectified)
 * @param[in] fov_scale  rectification field-of-view scale
 * @return model matching the images
 */
CameraModel cameraForImage(
  const CameraModel & base, int width, int height, bool rectified, double balance, double fov_scale);

/**
 * @brief Undistorts raw images to the rectified pinhole model of cameraForImage().
 */
class ImageRectifier
{
public:
  /**
   * @param[in] base       calibrated model (raw lens)
   * @param[in] balance    rectification balance
   * @param[in] fov_scale  rectification field-of-view scale
   */
  ImageRectifier(const CameraModel & base, double balance, double fov_scale)
  : base_(base), balance_(balance), fov_scale_(fov_scale) {}
  /**
   * @brief Undistorts one image. Builds the remap tables on the first image and when the size changes.
   * @param[in]  raw  distorted input image
   * @param[out] out  rectified image (same size as raw)
   */
  void rectify(const cv::Mat & raw, cv::Mat & out);
  /// @brief Pinhole model of the rectified images (valid after the first rectify()).
  const CameraModel & output() const {return out_;}

private:
  CameraModel base_, out_;
  double balance_, fov_scale_;
  cv::Mat map1_, map2_;
  cv::Size size_;
};

}  // namespace fusion_common
