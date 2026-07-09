#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
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
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

struct Vec2
{
  double x;
  double y;
};

struct Vec3
{
  double x;
  double y;
  double z;
};

struct Segment
{
  Vec3 a;
  Vec3 b;
};

struct DistanceResult
{
  double distance;
  Vec3 sample_point;
  Vec3 closest_boundary_point;
};

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
    declare_parameter<double>("sample_radius", 0.006);
    declare_parameter<double>("danger_radius", 0.025);

    base_frame_ = get_parameter("base_frame").as_string();
    blade_frame_ = get_parameter("blade_frame").as_string();
    contour_csv_ = resolvePackagePath(get_parameter("contour_csv").as_string());
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
    sample_radius_ = get_parameter("sample_radius").as_double();
    danger_radius_ = get_parameter("danger_radius").as_double();

    contour_points_ = loadCsv(contour_csv_, get_parameter("csv_scale").as_double());
    forceContourZ(contour_points_);
    gap_index_ = findLargestGap(contour_points_);
    segments_ = makeSegments(contour_points_, gap_index_);
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
      contour_points_.size(),
      segments_.size(),
      blade_samples_local_.size(),
      gap_index_,
      (gap_index_ + 1) % contour_points_.size());
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
  static std::string trim(const std::string & text)
  {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
      return "";
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
  }

  static bool parseCsvLine(const std::string & line, Vec3 & point)
  {
    std::stringstream stream(line);
    std::string item;
    std::vector<double> values;

    while (std::getline(stream, item, ',')) {
      try {
        values.push_back(std::stod(trim(item)));
      } catch (const std::exception &) {
        return false;
      }
    }

    if (values.size() < 2) {
      return false;
    }

    point.x = values[0];
    point.y = values[1];
    point.z = values.size() >= 3 ? values[2] : 0.0;
    return true;
  }

  static std::vector<Vec3> loadCsv(const std::string & path, const double scale)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open contour CSV: " + path);
    }

    std::vector<Vec3> points;
    std::string line;
    while (std::getline(file, line)) {
      if (trim(line).empty()) {
        continue;
      }

      Vec3 point{};
      if (!parseCsvLine(line, point)) {
        continue;
      }

      points.push_back(Vec3{point.x * scale, point.y * scale, point.z * scale});
    }

    if (points.size() < 3) {
      throw std::runtime_error("Contour CSV must contain at least three numeric points.");
    }

    return points;
  }

  void forceContourZ(std::vector<Vec3> & points) const
  {
    for (auto & point : points) {
      point.z = contour_z_;
    }
  }

  static double distance3(const Vec3 & a, const Vec3 & b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static size_t findLargestGap(const std::vector<Vec3> & points)
  {
    double max_distance = -std::numeric_limits<double>::infinity();
    size_t gap_index = 0;

    for (size_t i = 0; i < points.size(); ++i) {
      const size_t j = (i + 1) % points.size();
      const double d = distance3(points[i], points[j]);
      if (d > max_distance) {
        max_distance = d;
        gap_index = i;
      }
    }

    return gap_index;
  }

  static std::vector<Segment> makeSegments(
    const std::vector<Vec3> & points,
    const size_t gap_index)
  {
    std::vector<Segment> segments;
    segments.reserve(points.size() - 1);

    for (size_t i = 0; i < points.size(); ++i) {
      if (i == gap_index) {
        continue;
      }

      const size_t j = (i + 1) % points.size();
      segments.push_back(Segment{points[i], points[j]});
    }

    return segments;
  }

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

  std::string resolvePackagePath(const std::string & path) const
  {
    if (path.empty() || path.front() == '/') {
      return path;
    }

    const std::string package_share =
      ament_index_cpp::get_package_share_directory("ur5e_blade_safety");
    return package_share + "/" + path;
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

  static DistanceResult distancePointToSegment2d(const Vec3 & p, const Segment & segment)
  {
    const double ax = segment.a.x;
    const double ay = segment.a.y;
    const double bx = segment.b.x;
    const double by = segment.b.y;
    const double abx = bx - ax;
    const double aby = by - ay;
    const double apx = p.x - ax;
    const double apy = p.y - ay;
    const double ab_len2 = abx * abx + aby * aby;

    double u = 0.0;
    if (ab_len2 > 1.0e-12) {
      u = std::clamp((apx * abx + apy * aby) / ab_len2, 0.0, 1.0);
    }

    const Vec3 closest{ax + u * abx, ay + u * aby, segment.a.z + u * (segment.b.z - segment.a.z)};
    const double dx = p.x - closest.x;
    const double dy = p.y - closest.y;
    return DistanceResult{std::sqrt(dx * dx + dy * dy), p, closest};
  }

  DistanceResult computeMinimumDistance(const std::vector<Vec3> & samples) const
  {
    DistanceResult best{
      std::numeric_limits<double>::infinity(),
      Vec3{0.0, 0.0, 0.0},
      Vec3{0.0, 0.0, 0.0}};

    for (const auto & sample : samples) {
      for (const auto & segment : segments_) {
        const DistanceResult current = distancePointToSegment2d(sample, segment);
        if (current.distance < best.distance) {
          best = current;
        }
      }
    }

    return best;
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

    result.min_result = computeMinimumDistance(result.samples_base);
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
    maybePublishMarkers(callback_time, detection);
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

  void maybePublishMarkers(
    const rclcpp::Time & now,
    const DetectionResult & detection)
  {
    if (!publish_markers_) {
      return;
    }

    if (last_marker_publish_time_valid_ &&
      elapsedMs(now, last_marker_publish_time_) < marker_publish_period_ms_)
    {
      return;
    }

    marker_pub_->publish(makeMarkers(
      detection.samples_base,
      detection.min_result,
      detection.safe));
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

    for (const auto & segment : segments_) {
      marker.points.push_back(toPoint(segment.a));
      marker.points.push_back(toPoint(segment.b));
    }

    return marker;
  }

  visualization_msgs::msg::Marker makeOpeningMarker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "blade_contour_safety";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;
    marker.color.r = 1.0F;
    marker.color.g = 0.85F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    marker.points.push_back(toPoint(contour_points_[gap_index_]));
    marker.points.push_back(toPoint(contour_points_[(gap_index_ + 1) % contour_points_.size()]));
    return marker;
  }

  visualization_msgs::msg::Marker makeSampleMarker(
    const std::vector<Vec3> & samples,
    const bool safe) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "blade_contour_safety";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = sample_radius_;
    marker.scale.y = sample_radius_;
    marker.scale.z = sample_radius_;
    marker.color.r = safe ? 0.0F : 1.0F;
    marker.color.g = safe ? 0.85F : 0.0F;
    marker.color.b = 0.1F;
    marker.color.a = 0.85F;

    for (const auto & sample : samples) {
      marker.points.push_back(toPoint(sample));
    }

    return marker;
  }

  visualization_msgs::msg::Marker makeDangerMarker(const DistanceResult & min_result) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "blade_contour_safety";
    marker.id = 3;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = toPoint(min_result.closest_boundary_point);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = danger_radius_;
    marker.scale.y = danger_radius_;
    marker.scale.z = danger_radius_;
    marker.color.r = 1.0F;
    marker.color.g = 0.0F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    return marker;
  }

  visualization_msgs::msg::Marker makeTextMarker(
    const DistanceResult & min_result,
    const bool safe) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "blade_contour_safety";
    marker.id = 4;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = min_result.sample_point.x == 0.0 && min_result.sample_point.y == 0.0 ?
      toPoint(contour_points_[gap_index_]) :
      toPoint(min_result.sample_point);
    marker.pose.position.z += 0.08;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.05;
    marker.color.r = safe ? 0.0F : 1.0F;
    marker.color.g = safe ? 0.85F : 0.0F;
    marker.color.b = 0.1F;
    marker.color.a = 1.0F;

    std::ostringstream text;
    text.setf(std::ios::fixed);
    text.precision(4);
    text << (safe ? "SAFE" : "UNSAFE") << " d=" << min_result.distance;
    marker.text = text.str();
    return marker;
  }

  visualization_msgs::msg::MarkerArray makeMarkers(
    const std::vector<Vec3> & samples,
    const DistanceResult & min_result,
    const bool safe) const
  {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(makeDeleteAllMarker());
    markers.markers.push_back(makeSampleMarker(samples, safe));
    markers.markers.push_back(makeDangerMarker(min_result));
    markers.markers.push_back(makeTextMarker(min_result, safe));
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
  double sample_radius_;
  double danger_radius_;
  std::vector<Vec3> contour_points_;
  std::vector<Segment> segments_;
  std::vector<Vec2> blade_samples_local_;
  size_t gap_index_;

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
