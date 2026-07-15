#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "ur5e_blade_safety/contour_model.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

using ur5e_blade_safety::computeMinimumDistance;
using ur5e_blade_safety::ContourModel;
using ur5e_blade_safety::DistanceResult;
using ur5e_blade_safety::loadFixedContourModel;
using ur5e_blade_safety::Vec2;
using ur5e_blade_safety::Vec3;

struct DetectionResult
{
  std::vector<Vec3> samples_base;
  DistanceResult min_result;
  bool safe;
};

class BladeContourSafety : public rclcpp::Node
{
public:
  BladeContourSafety()
  : Node("blade_contour_safety")
  {
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("blade_frame", "blade_center_link");
    declare_parameter<std::string>("contour_csv", "data/cavity_contour.csv");
    declare_parameter<double>("csv_scale", 0.001);
    declare_parameter<double>("contour_z", 0.20);
    declare_parameter<double>("safety_margin", 0.003);
    declare_parameter<double>("blade_radius", 0.005);
    declare_parameter<double>("blade_angle_deg", 47.2);
    declare_parameter<double>("blade_sample_spacing", 0.0005);
    declare_parameter<double>("blade_yaw_offset_deg", 0.0);
    declare_parameter<double>("danger_distance", 0.006);
    declare_parameter<double>("base_timer_period_ms", 2.0);
    declare_parameter<double>("idle_publish_period_ms", 50.0);
    declare_parameter<double>("moving_publish_period_ms", 20.0);
    declare_parameter<double>("insert_publish_period_ms", 4.0);
    declare_parameter<double>("danger_publish_period_ms", 2.0);
    declare_parameter<bool>("enable_timing_log", true);
    declare_parameter<double>("timing_log_period_s", 0.5);
    declare_parameter<bool>("publish_markers", true);
    declare_parameter<double>("marker_publish_period_ms", 50.0);
    declare_parameter<double>("boundary_line_width", 0.004);
    declare_parameter<double>("opening_line_width", 0.004);
    declare_parameter<double>("shell_bottom_z", 0.0);
    declare_parameter<double>("shell_height", 0.25);
    declare_parameter<double>("shell_edge_width", 0.003);
    declare_parameter<double>("shell_wall_alpha", 0.18);
    declare_parameter<double>("endpoint_radius", 0.025);

    base_frame_ = get_parameter("base_frame").as_string();
    blade_frame_ = get_parameter("blade_frame").as_string();
    contour_csv_ = get_parameter("contour_csv").as_string();
    contour_z_ = get_parameter("contour_z").as_double();
    safety_margin_ = get_parameter("safety_margin").as_double();
    danger_distance_ = get_parameter("danger_distance").as_double();
    idle_publish_period_ms_ = get_parameter("idle_publish_period_ms").as_double();
    moving_publish_period_ms_ = get_parameter("moving_publish_period_ms").as_double();
    insert_publish_period_ms_ = get_parameter("insert_publish_period_ms").as_double();
    danger_publish_period_ms_ = get_parameter("danger_publish_period_ms").as_double();
    current_detection_period_ms_ = idle_publish_period_ms_;
    enable_timing_log_ = get_parameter("enable_timing_log").as_bool();
    timing_log_period_s_ = get_parameter("timing_log_period_s").as_double();
    publish_markers_ = get_parameter("publish_markers").as_bool();
    marker_publish_period_ms_ = get_parameter("marker_publish_period_ms").as_double();
    boundary_line_width_ = get_parameter("boundary_line_width").as_double();
    opening_line_width_ = get_parameter("opening_line_width").as_double();
    shell_bottom_z_ = get_parameter("shell_bottom_z").as_double();
    shell_height_ = get_parameter("shell_height").as_double();
    shell_edge_width_ = get_parameter("shell_edge_width").as_double();
    shell_wall_alpha_ = get_parameter("shell_wall_alpha").as_double();
    endpoint_radius_ = get_parameter("endpoint_radius").as_double();

    contour_model_ = loadFixedContourModel(
      contour_csv_,
      get_parameter("csv_scale").as_double(),
      contour_z_);
    blade_samples_local_ = makeBladeSamples();

    safe_pub_ = create_publisher<std_msgs::msg::Bool>("blade_contour_safe", 10);
    min_distance_pub_ = create_publisher<std_msgs::msg::Float64>(
      "blade_contour_min_distance", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "blade_contour_safety_markers", 10);
    compute_time_pub_ = create_publisher<std_msgs::msg::Float64>(
      "blade_contour_compute_time_ms", 10);
    detection_period_pub_ = create_publisher<std_msgs::msg::Float64>(
      "blade_contour_detection_period_ms", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto period_ms = get_parameter("base_timer_period_ms").as_double();
    const auto period = std::chrono::duration<double, std::milli>(period_ms);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&BladeContourSafety::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu contour points, %zu wall segments, %zu blade samples. Opening gap: %zu -> %zu.",
      contour_model_.points.size(),
      contour_model_.wall_segments.size(),
      blade_samples_local_.size(),
      contour_model_.opening_gap_index,
      (contour_model_.opening_gap_index + 1) % contour_model_.points.size());
    RCLCPP_INFO(
      get_logger(),
      "Dynamic detection periods: idle=%.2f ms, moving=%.2f ms, insert=%.2f ms, danger=%.2f ms, marker=%.2f ms.",
      idle_publish_period_ms_,
      moving_publish_period_ms_,
      insert_publish_period_ms_,
      danger_publish_period_ms_,
      marker_publish_period_ms_);
  }

private:
  std::vector<Vec2> makeBladeSamples() const
  {
    const double radius = get_parameter("blade_radius").as_double();
    const double angle_rad = degreesToRadians(get_parameter("blade_angle_deg").as_double());
    const double spacing = get_parameter("blade_sample_spacing").as_double();
    const double yaw_offset = degreesToRadians(get_parameter("blade_yaw_offset_deg").as_double());

    std::vector<Vec2> samples;
    samples.push_back(Vec2{0.0, 0.0});

    for (double r = spacing; r <= radius + 1.0e-9; r += spacing) {
      const double theta_step = std::max(spacing / r, degreesToRadians(2.0));
      for (double theta = -0.5 * angle_rad; theta <= 0.5 * angle_rad + 1.0e-9;
        theta += theta_step)
      {
        const double shifted_theta = theta + yaw_offset;
        samples.push_back(Vec2{r * std::cos(shifted_theta), r * std::sin(shifted_theta)});
      }
    }

    return samples;
  }

  static double degreesToRadians(const double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  std::vector<Vec3> transformBladeSamples(
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    const auto & q_msg = transform.transform.rotation;
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    const tf2::Matrix3x3 rotation(q);
    const auto & t = transform.transform.translation;

    std::vector<Vec3> samples;
    samples.reserve(blade_samples_local_.size());

    for (const auto & local : blade_samples_local_) {
      const tf2::Vector3 local_point(local.x, local.y, 0.0);
      const tf2::Vector3 rotated = rotation * local_point;
      samples.push_back(Vec3{
        t.x + rotated.x(),
        t.y + rotated.y(),
        t.z + rotated.z()});
    }

    return samples;
  }

  DetectionResult runCoreDetection(
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    DetectionResult result{
      transformBladeSamples(transform),
      DistanceResult{
        std::numeric_limits<double>::infinity(),
        Vec3{0.0, 0.0, 0.0},
        Vec3{0.0, 0.0, 0.0}},
      true};

    result.min_result = computeMinimumDistance(
      result.samples_base,
      contour_model_.wall_segments);
    result.safe = result.min_result.distance > safety_margin_;
    return result;
  }

  static double elapsedMs(const rclcpp::Time & now, const rclcpp::Time & previous)
  {
    return (now - previous).seconds() * 1000.0;
  }

  void timerCallback()
  {
    const rclcpp::Time callback_time = this->now();
    if (last_detection_time_valid_ &&
      elapsedMs(callback_time, last_detection_time_) < current_detection_period_ms_)
    {
      return;
    }
    last_detection_time_ = callback_time;
    last_detection_time_valid_ = true;

    geometry_msgs::msg::TransformStamped transform;

    try {
      transform = tf_buffer_->lookupTransform(
        base_frame_,
        blade_frame_,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "TF lookup failed from %s to %s: %s",
        base_frame_.c_str(),
        blade_frame_.c_str(),
        ex.what());
      return;
    }

    const auto core_start = std::chrono::steady_clock::now();
    const DetectionResult detection = runCoreDetection(transform);
    const auto core_end = std::chrono::steady_clock::now();
    const double core_ms =
      std::chrono::duration<double, std::milli>(core_end - core_start).count();

    updateDetectionPeriod(detection.safe, detection.min_result.distance);
    publishSafetyState(detection, core_ms);
    logUnsafeTransition(detection);
    maybePublishMarkers(callback_time);
    updateTimingLog(callback_time, core_ms, detection);
  }

  void updateDetectionPeriod(const bool safe, const double min_distance)
  {
    if (!safe || min_distance <= danger_distance_) {
      current_detection_period_ms_ = danger_publish_period_ms_;
      return;
    }

    current_detection_period_ms_ = idle_publish_period_ms_;
  }

  void publishSafetyState(const DetectionResult & detection, const double core_ms)
  {
    std_msgs::msg::Bool safe_msg;
    safe_msg.data = detection.safe;
    safe_pub_->publish(safe_msg);

    std_msgs::msg::Float64 distance_msg;
    distance_msg.data = detection.min_result.distance;
    min_distance_pub_->publish(distance_msg);

    std_msgs::msg::Float64 time_msg;
    time_msg.data = core_ms;
    compute_time_pub_->publish(time_msg);

    std_msgs::msg::Float64 period_msg;
    period_msg.data = current_detection_period_ms_;
    detection_period_pub_->publish(period_msg);
  }

  void logUnsafeTransition(const DetectionResult & detection)
  {
    if (!detection.safe && last_safe_) {
      RCLCPP_WARN(
        get_logger(),
        "UNSAFE triggered: min_distance=%.4f m, safety_margin=%.4f m",
        detection.min_result.distance,
        safety_margin_);
    }

    last_safe_ = detection.safe;
  }

  void maybePublishMarkers(const rclcpp::Time & now)
  {
    if (!publish_markers_) {
      return;
    }

    if (last_marker_publish_time_valid_ &&
      elapsedMs(now, last_marker_publish_time_) < marker_publish_period_ms_)
    {
      return;
    }

    marker_pub_->publish(makeMarkers());
    last_marker_publish_time_ = now;
    last_marker_publish_time_valid_ = true;
  }

  void updateTimingLog(
    const rclcpp::Time & now,
    const double core_ms,
    const DetectionResult & detection)
  {
    ++timing_count_;
    timing_sum_ms_ += core_ms;
    timing_max_ms_ = std::max(timing_max_ms_, core_ms);

    if (!enable_timing_log_) {
      return;
    }

    if (!last_timing_log_time_valid_) {
      last_timing_log_time_ = now;
      last_timing_log_time_valid_ = true;
      return;
    }

    if ((now - last_timing_log_time_).seconds() < timing_log_period_s_) {
      return;
    }

    if (timing_count_ > 0) {
      RCLCPP_INFO(
        get_logger(),
        "contour safety: %s, min_distance=%.4f m, margin=%.4f m, core_avg=%.4f ms, core_max=%.4f ms, samples=%d, current_period=%.2f ms",
        detection.safe ? "SAFE" : "UNSAFE",
        detection.min_result.distance,
        safety_margin_,
        timing_sum_ms_ / timing_count_,
        timing_max_ms_,
        timing_count_,
        current_detection_period_ms_);
    }

    last_timing_log_time_ = now;
    timing_count_ = 0;
    timing_sum_ms_ = 0.0;
    timing_max_ms_ = 0.0;
  }

  static geometry_msgs::msg::Point toPoint(const Vec3 & value)
  {
    geometry_msgs::msg::Point point;
    point.x = value.x;
    point.y = value.y;
    point.z = value.z;
    return point;
  }

  visualization_msgs::msg::Marker makeDeleteAllMarker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
  }

  visualization_msgs::msg::Marker makeBoundaryMarker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "blade_contour_safety";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = boundary_line_width_;
    marker.color.r = 0.0F;
    marker.color.g = 0.35F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;

    for (const auto & segment : contour_model_.wall_segments) {
      marker.points.push_back(toPoint(segment.a));
      marker.points.push_back(toPoint(segment.b));
    }

    return marker;
  }

  Vec3 withZ(const Vec3 & point, const double z) const
  {
    return Vec3{point.x, point.y, z};
  }

  visualization_msgs::msg::Marker makeContourShellWallMarker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "blade_contour_safety";
    marker.id = 10;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0F;
    marker.color.g = 0.55F;
    marker.color.b = 1.0F;
    marker.color.a = static_cast<float>(shell_wall_alpha_);

    const double top_z = shell_bottom_z_ + shell_height_;
    if (top_z <= shell_bottom_z_) {
      return marker;
    }

    for (const auto & segment : contour_model_.wall_segments) {
      const Vec3 a_bottom = withZ(segment.a, shell_bottom_z_);
      const Vec3 b_bottom = withZ(segment.b, shell_bottom_z_);
      const Vec3 a_top = withZ(segment.a, top_z);
      const Vec3 b_top = withZ(segment.b, top_z);

      marker.points.push_back(toPoint(a_bottom));
      marker.points.push_back(toPoint(b_bottom));
      marker.points.push_back(toPoint(b_top));

      marker.points.push_back(toPoint(a_bottom));
      marker.points.push_back(toPoint(b_top));
      marker.points.push_back(toPoint(a_top));
    }

    return marker;
  }

  visualization_msgs::msg::Marker makeContourShellEdgeMarker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "blade_contour_safety";
    marker.id = 11;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = shell_edge_width_;
    marker.color.r = 0.0F;
    marker.color.g = 0.85F;
    marker.color.b = 1.0F;
    marker.color.a = 0.85F;

    const double top_z = shell_bottom_z_ + shell_height_;
    if (top_z <= shell_bottom_z_) {
      return marker;
    }

    for (const auto & segment : contour_model_.wall_segments) {
      const Vec3 a_bottom = withZ(segment.a, shell_bottom_z_);
      const Vec3 b_bottom = withZ(segment.b, shell_bottom_z_);
      const Vec3 a_top = withZ(segment.a, top_z);
      const Vec3 b_top = withZ(segment.b, top_z);

      marker.points.push_back(toPoint(a_bottom));
      marker.points.push_back(toPoint(b_bottom));
      marker.points.push_back(toPoint(a_top));
      marker.points.push_back(toPoint(b_top));
      marker.points.push_back(toPoint(a_bottom));
      marker.points.push_back(toPoint(a_top));
      marker.points.push_back(toPoint(b_bottom));
      marker.points.push_back(toPoint(b_top));
    }

    return marker;
  }

  visualization_msgs::msg::Marker makeOpeningLineMarker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "blade_contour_safety";
    marker.id = 12;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = opening_line_width_;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;

    const auto & points = contour_model_.points;
    const auto gap_index = contour_model_.opening_gap_index;
    marker.points.push_back(toPoint(points[gap_index]));
    marker.points.push_back(toPoint(points[(gap_index + 1) % points.size()]));
    return marker;
  }

  visualization_msgs::msg::Marker makeOpeningEndpointMarker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "blade_contour_safety";
    marker.id = 13;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = endpoint_radius_;
    marker.scale.y = endpoint_radius_;
    marker.scale.z = endpoint_radius_;
    marker.color.r = 1.0F;
    marker.color.g = 0.85F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    const auto & points = contour_model_.points;
    const auto gap_index = contour_model_.opening_gap_index;
    marker.points.push_back(toPoint(points[gap_index]));
    marker.points.push_back(toPoint(points[(gap_index + 1) % points.size()]));
    return marker;
  }

  visualization_msgs::msg::MarkerArray makeMarkers() const
  {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(makeDeleteAllMarker());
    markers.markers.push_back(makeContourShellWallMarker());
    markers.markers.push_back(makeContourShellEdgeMarker());
    markers.markers.push_back(makeBoundaryMarker());
    markers.markers.push_back(makeOpeningLineMarker());
    markers.markers.push_back(makeOpeningEndpointMarker());
    return markers;
  }

  std::string base_frame_;
  std::string blade_frame_;
  std::string contour_csv_;
  double contour_z_;
  double safety_margin_;
  double danger_distance_;
  double idle_publish_period_ms_;
  double moving_publish_period_ms_;
  double insert_publish_period_ms_;
  double danger_publish_period_ms_;
  double current_detection_period_ms_;
  bool enable_timing_log_;
  double timing_log_period_s_;
  bool publish_markers_;
  double marker_publish_period_ms_;
  double boundary_line_width_;
  double opening_line_width_;
  double shell_bottom_z_;
  double shell_height_;
  double shell_edge_width_;
  double shell_wall_alpha_;
  double endpoint_radius_;
  ContourModel contour_model_;
  std::vector<Vec2> blade_samples_local_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safe_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr min_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr compute_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr detection_period_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Time last_detection_time_;
  rclcpp::Time last_marker_publish_time_;
  rclcpp::Time last_timing_log_time_;
  bool last_detection_time_valid_ = false;
  bool last_marker_publish_time_valid_ = false;
  bool last_timing_log_time_valid_ = false;
  bool last_safe_ = true;
  int timing_count_ = 0;
  double timing_sum_ms_ = 0.0;
  double timing_max_ms_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BladeContourSafety>());
  rclcpp::shutdown();
  return 0;
}
