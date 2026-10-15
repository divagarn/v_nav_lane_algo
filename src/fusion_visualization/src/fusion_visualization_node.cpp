/**
 * @file fusion_visualization_node.cpp
 * @brief Visualization of the fusion output (independent node, subscribe-only).
 */
//   ~/markers           visualization_msgs/MarkerArray  (frame = fusion output), all objects
//   ~/markers_relevant  same, only moving or in-path objects (relevant.require_both: moving and in path)
//       - class-sized 3D boxes of the final object list (in-path objects solid red),
//         labels (id, class, motion state, range, speed), velocity arrows,
//         ego path corridor, radar field of view (blue), output range arc (green), ego marker
//   ~/overlay  sensor_msgs/Image (bgr8), only with overlay.enabled (off by default)
//       - rectified camera image + the fused objects only (radar tracks, "R" or "R+C"):
//         class-sized box at the fused position (red if in the ego path), label, ego path
//       Images are buffered and drawn when the fusion result of that frame (trigger
//       "camera", same stamp) arrives, so boxes and image match despite detector latency.
#include <cmath>
#include <deque>
#include <memory>
#include <set>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "fusion_common/camera_model.hpp"
#include "fusion_common/object_classes.hpp"
#include "perception_msgs/msg/fused_object_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace fusion_visualization
{

using perception_msgs::msg::FusedObject;
using perception_msgs::msg::FusedObjectArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{
/**
 * @brief ROS time to seconds.
 * @param[in] t  ROS time
 * @return sec + 1e-9 nanosec
 */
double toSec(const builtin_interfaces::msg::Time & t) {return t.sec + 1e-9 * t.nanosec;}

/// Colour, components in [0, 1].
struct Rgb {float r, g, b;};
/**
 * @brief Display colour of a fused class.
 * @param[in] c  class name
 * @return colour (grey for unknown)
 */
Rgb classColor(const std::string & c)
{
  if (c == "pedestrian") {return {1.0f, 0.55f, 0.0f};}
  if (c == "bicycle" || c == "motorcycle") {return {0.9f, 0.2f, 0.9f};}
  if (c == "car") {return {0.1f, 0.6f, 1.0f};}
  if (c == "truck" || c == "bus") {return {0.2f, 0.9f, 0.9f};}
  if (c == "animal") {return {0.6f, 0.4f, 0.2f};}
  return {0.7f, 0.7f, 0.7f};
}
/**
 * @brief Colour -> OpenCV BGR scalar (0..255).
 * @param[in] c  colour
 * @return BGR scalar
 */
cv::Scalar toCv(const Rgb & c) {return cv::Scalar(255 * c.b, 255 * c.g, 255 * c.r);}

}  // namespace

/// RViz markers of the final fused list (and the optional image overlay).
class FusionVisualizationNode : public rclcpp::Node
{
public:
  /// @brief Declares the parameters, loads the calibration (overlay only), creates the topics.
  FusionVisualizationNode()
  : Node("fusion_visualization")
  {
    const auto objects_topic = declare_parameter<std::string>("objects_topic", "/radar_fusion/objects");
    const auto image_topic = declare_parameter<std::string>("image_topic", "/object_detection/image_rect");
    const auto intr = declare_parameter<std::string>("camera.intrinsics_file", "");
    const auto extr = declare_parameter<std::string>("camera.extrinsics_file", "");
    rectified_ = declare_parameter<bool>("camera.rectified", true);
    balance_ = declare_parameter<double>("camera.rectify_balance", 0.0);
    fov_scale_ = declare_parameter<double>("camera.rectify_fov_scale", 1.0);
    radar_height_ = declare_parameter<double>("radar_height_above_ground", 0.5);
    radar_frame_ = declare_parameter<std::string>("radar_frame", "umrr");
    max_range_ = declare_parameter<double>("range_ring_m", 40.0);
    radar_fov_ = declare_parameter<double>("radar_fov.max_abs_azimuth", 1.0);
    radar_fov_range_ = declare_parameter<double>("radar_fov.max_range", 80.0);
    relevant_require_both_ = declare_parameter<bool>("relevant.require_both", false);
    overlay_enabled_ = declare_parameter<bool>("overlay.enabled", false);
    overlay_scale_ = declare_parameter<double>("overlay.scale", 1.0);
    show_window_ = declare_parameter<bool>("overlay.show_window", false);
    max_wait_ = declare_parameter<double>("overlay.max_wait_s", 0.5);
    marker_lifetime_ = declare_parameter<double>("marker_lifetime_s", 0.3);
    // RViz (especially over Wi-Fi) cannot take 30 Hz full images + marker rebuilds
    markers_max_rate_ = declare_parameter<double>("markers_max_rate", 10.0);
    overlay_max_rate_ = declare_parameter<double>("overlay.max_rate", 15.0);

    if (!intr.empty() && !extr.empty()) {
      try {
        base_camera_ = fusion_common::CameraModel::load(intr, extr);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "calibration not loaded (%s): raw radar overlay off", e.what());
      }
    }

    markers_pub_ = create_publisher<MarkerArray>("~/markers", 10);
    relevant_pub_ = create_publisher<MarkerArray>("~/markers_relevant", 10);
    overlay_pub_ = create_publisher<sensor_msgs::msg::Image>("~/overlay", rclcpp::QoS(2).reliable());

    tracks_sub_ = create_subscription<FusedObjectArray>(objects_topic, 50,
        [this](FusedObjectArray::ConstSharedPtr m) {onTracks(m);});
    if (overlay_enabled_) {
      // best effort: object_detection publishes ~/image_rect best effort (display must never block
      // detection) and a reliable subscriber does not match a best-effort publisher (no images at all);
      // a best-effort subscriber matches both, a lost overlay frame is harmless
      img_sub_ = create_subscription<sensor_msgs::msg::Image>(image_topic, rclcpp::QoS(5).best_effort(),
          [this](sensor_msgs::msg::Image::ConstSharedPtr m) {onImage(m);});
    }
  }

private:
  /**
   * @brief Marker with common header, namespace, id, type and lifetime.
   * @param[in] frame  frame id
   * @param[in] stamp  stamp
   * @param[in] ns     namespace
   * @param[in] id     id within the namespace
   * @param[in] type   Marker type
   * @return marker (action ADD, identity orientation)
   */
  Marker baseMarker(const std::string & frame, const builtin_interfaces::msg::Time & stamp,
    const std::string & ns, int id, int type) const
  {
    Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
    return m;
  }

  /**
   * @brief Adds box, label and (if moving) velocity arrow of one fused object.
   * @param[in,out] arr    marker array
   * @param[in]     o      fused object
   * @param[in]     frame  frame id
   * @param[in]     stamp  stamp
   * @param[in]     radar_relative  true when the message is still in the radar frame
   */
  void addObject(MarkerArray & arr, const FusedObject & o, const std::string & frame,
    const builtin_interfaces::msg::Time & stamp, bool radar_relative) const
  {
    // red: in the stop zone, orange: predicted to enter it, else class colour
    const bool entering = !o.in_stop_zone && o.time_to_stop_zone >= 0.0f;
    const bool highlight = o.in_stop_zone || entering || o.in_path;
    const auto & ci = fusion_common::classInfo(o.class_name);
    const Rgb col = o.in_stop_zone ? Rgb{1.0f, 0.1f, 0.1f} : entering ? Rgb{1.0f, 0.55f, 0.0f} :
      classColor(o.class_name);
    // In the radar frame the configured ground plane is known. In a global frame, keep
    // the transformed object height rather than inventing a global ground elevation.
    const double marker_z = radar_relative ? -radar_height_ + ci.height / 2 : o.position.z;
    Marker b = baseMarker(frame, stamp, "object_boxes", o.id, Marker::CUBE);
    b.pose.position.x = o.position.x;
    b.pose.position.y = o.position.y;
    b.pose.position.z = marker_z;
    // yaw quaternion q = (0, 0, sin(psi / 2), cos(psi / 2))
    b.pose.orientation.z = std::sin(o.heading / 2);
    b.pose.orientation.w = std::cos(o.heading / 2);
    b.scale.x = ci.length;
    b.scale.y = ci.width;
    b.scale.z = ci.height;
    b.color.r = col.r; b.color.g = col.g; b.color.b = col.b;
    b.color.a = highlight ? 0.85f : 0.4f;
    arr.markers.push_back(b);

    Marker t = baseMarker(frame, stamp, "object_labels", o.id, Marker::TEXT_VIEW_FACING);
    t.pose.position = b.pose.position;
    t.pose.position.z = marker_z + ci.height / 2 + 0.5;
    t.scale.z = 0.45;
    t.color.r = t.color.g = t.color.b = t.color.a = 1.0f;
    const std::string zone = o.in_stop_zone ? " STOP ZONE" :
      entering ? cv::format(" zone in %.1fs", o.time_to_stop_zone) : (o.in_path ? " in path" : "");
    t.text = cv::format("%u %s %s%s\n%.1fm %.1fm/s %s %+.1fm/s", o.id, o.class_name.c_str(),
      o.motion_state.c_str(), zone.c_str(), o.range, std::hypot(o.velocity.x, o.velocity.y),
      o.relative_motion_state.c_str(), o.closing_speed);
    arr.markers.push_back(t);
    if (o.prediction_horizon > 0.0f && o.is_moving) {   // where it will be (over ground)
      Marker pa = baseMarker(frame, stamp, "object_prediction", o.id, Marker::ARROW);
      geometry_msgs::msg::Point q0, q1;
      q0.x = o.position.x; q0.y = o.position.y; q0.z = marker_z + 0.1;
      q1.x = o.predicted_position.x; q1.y = o.predicted_position.y; q1.z = q0.z;
      pa.points = {q0, q1};
      pa.scale.x = 0.08; pa.scale.y = 0.2; pa.scale.z = 0.25;
      pa.color.r = 0.0f; pa.color.g = 0.9f; pa.color.b = 1.0f; pa.color.a = 0.9f;
      arr.markers.push_back(pa);
    }
    if (o.is_moving) {
      Marker a = baseMarker(frame, stamp, "object_velocity", o.id, Marker::ARROW);
      geometry_msgs::msg::Point p0, p1;
      p0.x = o.position.x; p0.y = o.position.y; p0.z = marker_z;
      p1 = p0;
      p1.x += o.velocity.x; p1.y += o.velocity.y;
      a.points = {p0, p1};
      a.scale.x = 0.1; a.scale.y = 0.25; a.scale.z = 0.3;
      a.color.r = 1.0f; a.color.g = 1.0f; a.color.a = 0.9f;
      arr.markers.push_back(a);
    }
  }

  /**
   * @brief True if the object belongs on ~/markers_relevant.
   * @param[in] o  fused object
   * @return moving or in path (relevant.require_both: moving and in path)
   */
  bool relevant(const FusedObject & o) const
  {
    const bool zone = o.in_stop_zone || o.time_to_stop_zone >= 0.0f;
    return relevant_require_both_ ? o.is_moving && o.in_path : o.is_moving || o.in_path || zone;
  }

  /// @brief Rebuilds and publishes the markers of the latest result (at most markers_max_rate):
  ///        ~/markers with all objects, ~/markers_relevant with moving / in-path objects only.
  void publishMarkers()
  {
    if (!tracks_) {return;}
    const double t = toSec(tracks_->header.stamp);
    if (markers_max_rate_ > 0 && std::abs(t - last_markers_t_) < 1.0 / markers_max_rate_) {return;}
    last_markers_t_ = t;
    const auto & frame = tracks_->header.frame_id;
    const auto & stamp = tracks_->header.stamp;
    const bool radar_relative = frame == radar_frame_;
    const bool want_all = markers_pub_->get_subscription_count() > 0;
    const bool want_rel = relevant_pub_->get_subscription_count() > 0;
    if (!want_all && !want_rel) {return;}
    // scene markers (path, field of view, range arc, ego) shared by both topics
    MarkerArray arr;
    // ego path corridor
    for (int side = 0; side < 2; ++side) {
      Marker l = baseMarker(frame, stamp, "ego_path", side, Marker::LINE_STRIP);
      l.scale.x = 0.06;
      l.color.r = 1.0f; l.color.g = 0.85f; l.color.a = 0.8f;
      l.points = side == 0 ? tracks_->path_left : tracks_->path_right;
      if (!l.points.empty()) {arr.markers.push_back(l);}
    }
    // stop zone (corridor ahead of the radar, same frame as the objects)
    if (!tracks_->stop_zone.empty()) {
      Marker z = baseMarker(frame, stamp, "stop_zone", 0, Marker::LINE_STRIP);
      z.scale.x = 0.08;
      z.color.r = 1.0f; z.color.g = 0.3f; z.color.b = 0.1f; z.color.a = 0.9f;
      z.points = tracks_->stop_zone;
      arr.markers.push_back(z);
    }
    // These scene markers are defined about the radar origin. Suppress them for global
    // output rather than incorrectly placing the sensor and vehicle at the map origin.
    if (radar_relative) {
      auto ground = [this](double r, double az) {
          geometry_msgs::msg::Point p;
          p.x = r * std::cos(az); p.y = r * std::sin(az); p.z = -radar_height_;
          return p;
        };
      Marker fov = baseMarker(frame, stamp, "radar_fov", 0, Marker::LINE_STRIP);
      fov.scale.x = 0.08;
      fov.color.r = 0.2f; fov.color.g = 0.7f; fov.color.b = 1.0f; fov.color.a = 0.7f;
      fov.points.push_back(ground(0.0, 0.0));
      const int n = 60;
      for (int k = 0; k <= n; ++k) {
        fov.points.push_back(ground(radar_fov_range_, -radar_fov_ + 2.0 * radar_fov_ * k / n));
      }
      fov.points.push_back(ground(0.0, 0.0));
      arr.markers.push_back(fov);
      Marker ring = baseMarker(frame, stamp, "range_ring", 0, Marker::LINE_STRIP);
      ring.scale.x = 0.08;
      ring.color.g = 1.0f; ring.color.a = 0.6f;
      for (int k = 0; k <= n; ++k) {
        ring.points.push_back(ground(max_range_, -radar_fov_ + 2.0 * radar_fov_ * k / n));
      }
      arr.markers.push_back(ring);
      Marker ego = baseMarker(frame, stamp, "ego", 0, Marker::CUBE);
      ego.pose.position.x = -1.5;
      ego.pose.position.z = -radar_height_ + 0.75;
      ego.scale.x = 3.0; ego.scale.y = 1.4; ego.scale.z = 1.5;
      ego.color.r = ego.color.g = ego.color.b = 0.3f; ego.color.a = 0.8f;
      arr.markers.push_back(ego);
    }
    Marker clear;
    clear.action = Marker::DELETEALL;
    for (const bool all : {true, false}) {
      if (!(all ? want_all : want_rel)) {continue;}
      MarkerArray out;
      out.markers.push_back(clear);
      for (const auto & o : tracks_->objects) {
        if (all || relevant(o)) {addObject(out, o, frame, stamp, radar_relative);}
      }
      out.markers.insert(out.markers.end(), arr.markers.begin(), arr.markers.end());
      (all ? markers_pub_ : relevant_pub_)->publish(out);
    }
  }

  /**
   * @brief New fusion result: markers; for camera-triggered results, pairs with the buffered image.
   *
   * Image and fusion result of the same camera frame (same stamp) are paired whichever
   * arrives first: the small result usually overtakes the 1.5 MB image.
   * @param[in] m  fusion result
   */
  void onTracks(FusedObjectArray::ConstSharedPtr m)
  {
    tracks_ = m;
    publishMarkers();
    if (m->trigger != "camera") {return;}
    have_camera_cycles_ = true;
    const double t = toSec(m->header.stamp);
    for (auto it = images_.begin(); it != images_.end(); ++it) {
      if (std::abs(toSec((*it)->header.stamp) - t) < 1e-4) {
        drawOverlay(**it, m);
        images_.erase(images_.begin(), it + 1);   // older images will never get a result
        return;
      }
    }
    results_.push_back(m);
    while (!results_.empty() && t - toSec(results_.front()->header.stamp) > max_wait_) {results_.pop_front();}
  }

  /**
   * @brief New rectified image (overlay only): pairs with a buffered result of the same stamp.
   * @param[in] m  rectified image
   */
  void onImage(sensor_msgs::msg::Image::ConstSharedPtr m)
  {
    if (overlay_pub_->get_subscription_count() == 0 && !show_window_) {images_.clear(); return;}
    if (!have_camera_cycles_) {   // no fusion camera cycles (yet): draw immediately
      drawOverlay(*m, tracks_);
      return;
    }
    const double t = toSec(m->header.stamp);
    for (auto it = results_.begin(); it != results_.end(); ++it) {
      if (std::abs(toSec((*it)->header.stamp) - t) < 1e-4) {
        drawOverlay(*m, *it);
        results_.erase(results_.begin(), it + 1);
        return;
      }
    }
    images_.push_back(m);
    while (!images_.empty() && t - toSec(images_.front()->header.stamp) > max_wait_) {
      images_.pop_front();   // no fusion result within max_wait
    }
  }

  /**
   * @brief Draws ego path and fused objects on the image and publishes it (at most overlay.max_rate).
   * @param[in] m      rectified camera image
   * @param[in] fused  fusion result of the same frame (may be null)
   */
  void drawOverlay(const sensor_msgs::msg::Image & m, FusedObjectArray::ConstSharedPtr fused)
  {
    const double t = toSec(m.header.stamp);
    if (overlay_max_rate_ > 0 && std::abs(t - last_overlay_t_) < 1.0 / overlay_max_rate_) {return;}
    last_overlay_t_ = t;
    cv::Mat img;
    try {
      img = toBgrCopy(m);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
      return;
    }
    if (base_camera_.valid() && (img.cols != cam_w_ || img.rows != cam_h_)) {
      camera_ = fusion_common::cameraForImage(base_camera_, img.cols, img.rows, rectified_, balance_, fov_scale_);
      cam_w_ = img.cols;
      cam_h_ = img.rows;
    }
    const double fs = std::max(0.4, img.cols / 1920.0 * 0.9);   // font scale

    // ego path corridor on the ground
    if (fused && fused->header.frame_id == radar_frame_ && camera_.valid()) {
      for (const auto * side : {&fused->path_left, &fused->path_right}) {
        std::vector<cv::Point> pts;
        for (const auto & p : *side) {
          const auto uv = camera_.projectRadarPoint(Eigen::Vector3d(p.x, p.y, p.z), 50.0);
          if (uv) {pts.emplace_back(uv->x(), uv->y());}
        }
        if (pts.size() > 1) {cv::polylines(img, pts, false, cv::Scalar(0, 215, 255), 2, cv::LINE_AA);}
      }
    }
    // fused objects (final list = radar tracks): box of the class size at the fused position,
    // solid when a camera box is associated, red when in the ego path
    if (fused) {
      for (const auto & o : fused->objects) {
        if (!o.in_image) {continue;}
        const bool hl = o.in_path, cam = o.camera_associated;
        const cv::Scalar col = hl ? cv::Scalar(0, 0, 255) :
          o.class_name == "unknown" ? cv::Scalar(0, 220, 0) : toCv(classColor(o.class_name));
        const cv::Point c(o.image_u, o.image_v);
        if (o.has_bbox) {
          cv::rectangle(img, cv::Point(o.bbox_x_min, o.bbox_y_min), cv::Point(o.bbox_x_max, o.bbox_y_max), col,
            cam ? 2 : 1, cv::LINE_AA);
        }
        cv::circle(img, c, 3, col, -1, cv::LINE_AA);
        const std::string lbl = cv::format("%u %s%s %s %.1fm %.1fm/s%s", o.id, cam ? "R+C " : "R ",
          o.class_name.c_str(), o.motion_state.c_str(), o.range, std::hypot(o.velocity.x, o.velocity.y),
          hl ? " IN PATH" : "");
        const cv::Point at(o.has_bbox ? o.bbox_x_min : c.x + 6, (o.has_bbox ? o.bbox_y_min : c.y) - 4);
        int base = 0;
        const double lfs = cam || hl ? fs : 0.8 * fs;
        const auto ts = cv::getTextSize(lbl, cv::FONT_HERSHEY_SIMPLEX, lfs, 1, &base);
        cv::rectangle(img, at + cv::Point(0, base), at + cv::Point(ts.width, -ts.height - 2), col, -1);
        cv::putText(img, lbl, at, cv::FONT_HERSHEY_SIMPLEX, lfs, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      }
    }
    if (fused) {
      int nc = 0;
      for (const auto & o : fused->objects) {nc += o.camera_associated;}
      cv::putText(img, cv::format("%zu radar tracks (%d radar+camera), path %s k=%.3f",
        fused->objects.size(), nc, fused->path_source.c_str(), fused->path_curvature), cv::Point(8, 22 * fs / 0.6),
        cv::FONT_HERSHEY_SIMPLEX, 0.9 * fs, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
    if (overlay_scale_ != 1.0) {cv::resize(img, img, cv::Size(), overlay_scale_, overlay_scale_);}
    if (show_window_) {
      cv::imshow("fusion overlay", img);
      cv::waitKey(1);
    }
    sensor_msgs::msg::Image out;
    out.header = m.header;
    out.height = img.rows;
    out.width = img.cols;
    out.encoding = "bgr8";
    out.step = static_cast<uint32_t>(img.step);
    out.data.assign(img.data, img.data + img.total() * img.elemSize());
    overlay_pub_->publish(out);
  }

  /**
   * @brief Image message -> owned BGR8 image.
   * @param[in] m  image (bgr8, rgb8, mono8, bgra8)
   * @return BGR8 copy
   * @throws std::runtime_error for an unsupported encoding
   */
  static cv::Mat toBgrCopy(const sensor_msgs::msg::Image & m)
  {
    auto * d = const_cast<uint8_t *>(m.data.data());
    cv::Mat out;
    if (m.encoding == "bgr8") {return cv::Mat(m.height, m.width, CV_8UC3, d, m.step).clone();}
    if (m.encoding == "rgb8") {cv::cvtColor(cv::Mat(m.height, m.width, CV_8UC3, d, m.step), out, cv::COLOR_RGB2BGR); return out;}
    if (m.encoding == "mono8") {cv::cvtColor(cv::Mat(m.height, m.width, CV_8UC1, d, m.step), out, cv::COLOR_GRAY2BGR); return out;}
    if (m.encoding == "bgra8") {cv::cvtColor(cv::Mat(m.height, m.width, CV_8UC4, d, m.step), out, cv::COLOR_BGRA2BGR); return out;}
    throw std::runtime_error("unsupported image encoding '" + m.encoding + "'");
  }

  fusion_common::CameraModel base_camera_, camera_;
  int cam_w_{0}, cam_h_{0};
  double markers_max_rate_{10.0}, overlay_max_rate_{15.0}, last_markers_t_{-1e18}, last_overlay_t_{-1e18};
  bool rectified_{true};
  std::string radar_frame_{"umrr"};
  double balance_{0.0}, fov_scale_{1.0};
  double radar_height_{0.5}, max_range_{40.0}, radar_fov_{1.0}, radar_fov_range_{80.0}, overlay_scale_{1.0}, marker_lifetime_{0.3}, max_wait_{0.5};
  bool relevant_require_both_{false};
  bool overlay_enabled_{true}, show_window_{false}, have_camera_cycles_{false};
  FusedObjectArray::ConstSharedPtr tracks_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> images_;
  std::deque<FusedObjectArray::ConstSharedPtr> results_;
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_, relevant_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Subscription<FusedObjectArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

}  // namespace fusion_visualization

/// Node entry point.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fusion_visualization::FusionVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
