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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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

struct AlignmentPlan
{
  Vec3 opening_a;
  Vec3 opening_b;
  Vec3 opening_center;
  Vec2 outward_dir;
  Vec2 insert_dir;
  Vec3 pre_center;
  Vec3 inside_center;
  double yaw;
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

class OpeningAlignmentPlanner : public rclcpp::Node
{
public:
  OpeningAlignmentPlanner()
  : Node("opening_alignment_planner")
  {
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("blade_frame", "blade_center_link");
    declare_parameter<std::string>("contour_csv", "data/cavity_contour.csv");
    declare_parameter<double>("csv_scale", 0.001);
    declare_parameter<double>("pre_offset", 0.03);
    declare_parameter<double>("insert_depth", 0.02);
    declare_parameter<double>("contour_z", 0.20);
    declare_parameter<double>("target_z", 0.20);
    declare_parameter<bool>("tool_z_axis_down", true);
    declare_parameter<std::string>("blade_shaft_axis", "minus_y");
    declare_parameter<std::string>("contour_plane", "xy");
    declare_parameter<double>("safety_margin", 0.003);
    declare_parameter<double>("blade_radius", 0.005);
    declare_parameter<double>("blade_angle_deg", 47.2);
    declare_parameter<double>("blade_sample_spacing", 0.0005);
    declare_parameter<double>("blade_yaw_offset_deg", 0.0);
    declare_parameter<double>("publish_period_ms", 100.0);
    declare_parameter<double>("line_width", 0.004);
    declare_parameter<double>("contour_line_width", 0.004);
    declare_parameter<double>("shell_bottom_z", 0.0);
    declare_parameter<double>("shell_height", 0.25);
    declare_parameter<double>("shell_edge_width", 0.003);
    declare_parameter<double>("shell_wall_alpha", 0.18);
    declare_parameter<double>("endpoint_radius", 0.025);
    declare_parameter<double>("target_radius", 0.025);
    declare_parameter<double>("marker_z", 0.0);
    declare_parameter<double>("text_height", 0.025);
    declare_parameter<double>("text_z_offset", 0.08);
    declare_parameter<double>("status_text_outward_offset", 0.08);
    declare_parameter<double>("current_blade_radius", 0.018);
    declare_parameter<double>("sample_radius", 0.006);
    declare_parameter<double>("danger_radius", 0.025);

    base_frame_ = get_parameter("base_frame").as_string();
    blade_frame_ = get_parameter("blade_frame").as_string();
    contour_csv_ = resolvePackagePath(get_parameter("contour_csv").as_string());

    contour_points_ = loadCsv(contour_csv_, get_parameter("csv_scale").as_double());
    applyContourPlaneZ(contour_points_);
    gap_index_ = findLargestGap(contour_points_);
    segments_ = makeSegments(contour_points_, gap_index_);
    blade_samples_local_ = makeBladeSamples();
    plan_ = makePlan();

    preinsert_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("blade_preinsert_pose", 10);
    insert_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("blade_insert_pose", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "opening_alignment_markers", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto period_ms = get_parameter("publish_period_ms").as_double();
    const auto period = std::chrono::duration<double, std::milli>(period_ms);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&OpeningAlignmentPlanner::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu contour points and %zu blade samples from %s. Opening gap: %zu -> %zu. "
      "M=(%.4f, %.4f), outward=(%.4f, %.4f), insert=(%.4f, %.4f), yaw=%.3f rad.",
      contour_points_.size(),
      blade_samples_local_.size(),
      contour_csv_.c_str(),
      gap_index_,
      (gap_index_ + 1) % contour_points_.size(),
      plan_.opening_center.x,
      plan_.opening_center.y,
      plan_.outward_dir.x,
      plan_.outward_dir.y,
      plan_.insert_dir.x,
      plan_.insert_dir.y,
      plan_.yaw);
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

  void applyContourPlaneZ(std::vector<Vec3> & points) const
  {
    const double contour_z = get_parameter("contour_z").as_double();
    for (auto & point : points) {
      point.z = contour_z;
    }
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

  static double norm2(const Vec2 & value)
  {
    return std::sqrt(value.x * value.x + value.y * value.y);
  }

  static Vec2 normalize2(const Vec2 & value)
  {
    const double length = norm2(value);
    if (length < 1.0e-12) {
      throw std::runtime_error("Cannot normalize a near-zero 2D vector.");
    }
    return Vec2{value.x / length, value.y / length};
  }

  static double dot2(const Vec2 & a, const Vec2 & b)
  {
    return a.x * b.x + a.y * b.y;
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

  static double degreesToRadians(const double degrees)
  {
    return degrees * M_PI / 180.0;
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

  AlignmentPlan makePlan() const
  {
    const Vec3 a = contour_points_[gap_index_];
    const Vec3 b = contour_points_[(gap_index_ + 1) % contour_points_.size()];
    const Vec3 center{
      0.5 * (a.x + b.x),
      0.5 * (a.y + b.y),
      0.5 * (a.z + b.z)};

    const Vec2 tangent = normalize2(Vec2{b.x - a.x, b.y - a.y});
    const Vec2 n1{-tangent.y, tangent.x};
    const Vec2 n2{tangent.y, -tangent.x};
    const Vec2 v_origin = normalize2(Vec2{-center.x, -center.y});
    const Vec2 outward = dot2(n1, v_origin) > dot2(n2, v_origin) ? n1 : n2;
    const Vec2 insert{-outward.x, -outward.y};

    const double pre_offset = get_parameter("pre_offset").as_double();
    const double insert_depth = get_parameter("insert_depth").as_double();
    const double target_z = get_parameter("target_z").as_double();
    const Vec3 pre_center{
      center.x + outward.x * pre_offset,
      center.y + outward.y * pre_offset,
      target_z};
    const Vec3 inside_center{
      center.x - outward.x * insert_depth,
      center.y - outward.y * insert_depth,
      target_z};

    const std::string shaft_axis = get_parameter("blade_shaft_axis").as_string();
    double yaw = std::atan2(insert.y, insert.x);
    if (shaft_axis == "minus_y") {
      yaw += M_PI / 2.0;
    } else if (shaft_axis == "plus_y") {
      yaw -= M_PI / 2.0;
    } else {
      throw std::runtime_error(
              "blade_shaft_axis must be either 'minus_y' or 'plus_y', got: " + shaft_axis);
    }

    return AlignmentPlan{a, b, center, outward, insert, pre_center, inside_center, yaw};
  }

  geometry_msgs::msg::PoseStamped makePose(const Vec3 & position) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = base_frame_;
    pose.header.stamp = now();
    pose.pose.position = toPoint(position);

    const tf2::Quaternion q = makeTargetOrientation();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
  }

  tf2::Quaternion makeTargetOrientation() const
  {
    if (!get_parameter("tool_z_axis_down").as_bool()) {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, plan_.yaw);
      q.normalize();
      return q;
    }

    const std::string shaft_axis = get_parameter("blade_shaft_axis").as_string();
    tf2::Vector3 y_axis;
    if (shaft_axis == "minus_y") {
      y_axis = tf2::Vector3(-plan_.insert_dir.x, -plan_.insert_dir.y, 0.0);
    } else if (shaft_axis == "plus_y") {
      y_axis = tf2::Vector3(plan_.insert_dir.x, plan_.insert_dir.y, 0.0);
    } else {
      throw std::runtime_error(
              "blade_shaft_axis must be either 'minus_y' or 'plus_y', got: " + shaft_axis);
    }

    y_axis.normalize();
    const tf2::Vector3 z_axis(0.0, 0.0, -1.0);
    tf2::Vector3 x_axis = y_axis.cross(z_axis);
    x_axis.normalize();

    const tf2::Matrix3x3 rotation(
      x_axis.x(), y_axis.x(), z_axis.x(),
      x_axis.y(), y_axis.y(), z_axis.y(),
      x_axis.z(), y_axis.z(), z_axis.z());

    tf2::Quaternion q;
    rotation.getRotation(q);
    q.normalize();
    return q;
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

  visualization_msgs::msg::Marker makeOpeningLineMarker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "opening_alignment";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = get_parameter("line_width").as_double();
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    marker.points.push_back(toPoint(plan_.opening_a));
    marker.points.push_back(toPoint(plan_.opening_b));
    return marker;
  }

  visualization_msgs::msg::Marker makeContourBoundaryMarker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "opening_alignment";
    marker.id = 12;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = get_parameter("contour_line_width").as_double();
    marker.color.r = 0.0F;
    marker.color.g = 0.35F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;

    for (size_t i = 0; i < contour_points_.size(); ++i) {
      if (i == gap_index_) {
        continue;
      }

      const size_t j = (i + 1) % contour_points_.size();
      marker.points.push_back(toPoint(contour_points_[i]));
      marker.points.push_back(toPoint(contour_points_[j]));
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
    marker.ns = "opening_alignment";
    marker.id = 13;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0F;
    marker.color.g = 0.55F;
    marker.color.b = 1.0F;
    marker.color.a = static_cast<float>(get_parameter("shell_wall_alpha").as_double());

    const double bottom_z = get_parameter("shell_bottom_z").as_double();
    const double top_z = bottom_z + get_parameter("shell_height").as_double();
    if (top_z <= bottom_z) {
      return marker;
    }

    for (size_t i = 0; i < contour_points_.size(); ++i) {
      if (i == gap_index_) {
        continue;
      }

      const size_t j = (i + 1) % contour_points_.size();
      const Vec3 a_bottom = withZ(contour_points_[i], bottom_z);
      const Vec3 b_bottom = withZ(contour_points_[j], bottom_z);
      const Vec3 a_top = withZ(contour_points_[i], top_z);
      const Vec3 b_top = withZ(contour_points_[j], top_z);

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
    marker.ns = "opening_alignment";
    marker.id = 14;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = get_parameter("shell_edge_width").as_double();
    marker.color.r = 0.0F;
    marker.color.g = 0.85F;
    marker.color.b = 1.0F;
    marker.color.a = 0.85F;

    const double bottom_z = get_parameter("shell_bottom_z").as_double();
    const double top_z = bottom_z + get_parameter("shell_height").as_double();
    if (top_z <= bottom_z) {
      return marker;
    }

    for (size_t i = 0; i < contour_points_.size(); ++i) {
      if (i == gap_index_) {
        continue;
      }

      const size_t j = (i + 1) % contour_points_.size();
      const Vec3 a_bottom = withZ(contour_points_[i], bottom_z);
      const Vec3 b_bottom = withZ(contour_points_[j], bottom_z);
      const Vec3 a_top = withZ(contour_points_[i], top_z);
      const Vec3 b_top = withZ(contour_points_[j], top_z);

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

  visualization_msgs::msg::Marker makeEndpointMarker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "opening_alignment";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    const double radius = get_parameter("endpoint_radius").as_double();
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color.r = 1.0F;
    marker.color.g = 0.85F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    marker.points.push_back(toPoint(plan_.opening_a));
    marker.points.push_back(toPoint(plan_.opening_b));
    return marker;
  }

  visualization_msgs::msg::Marker makeSingleSphereMarker(
    const int id,
    const Vec3 & center,
    const float r,
    const float g,
    const float b) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "opening_alignment";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = toPoint(center);
    marker.pose.orientation.w = 1.0;
    const double radius = get_parameter("target_radius").as_double();
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0F;
    return marker;
  }

  visualization_msgs::msg::Marker makeSampleMarker(
    const int id,
    const std::vector<Vec3> & samples,
    const bool safe) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "opening_alignment";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    const double radius = get_parameter("sample_radius").as_double();
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color.r = safe ? 0.0F : 1.0F;
    marker.color.g = safe ? 0.85F : 0.0F;
    marker.color.b = 0.1F;
    marker.color.a = 0.85F;

    for (const auto & sample : samples) {
      marker.points.push_back(toPoint(sample));
    }

    return marker;
  }

  visualization_msgs::msg::Marker makeDangerMarker(
    const int id,
    const DistanceResult & min_result) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "opening_alignment";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = toPoint(min_result.closest_boundary_point);
    marker.pose.orientation.w = 1.0;
    const double radius = get_parameter("danger_radius").as_double();
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color.r = 1.0F;
    marker.color.g = 0.0F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    return marker;
  }

  visualization_msgs::msg::Marker makeTextMarker(
    const int id,
    const Vec3 & position,
    const std::string & text,
    const float r,
    const float g,
    const float b) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = "opening_alignment";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = toPoint(position);
    marker.pose.position.z += get_parameter("text_z_offset").as_double();
    marker.pose.orientation.w = 1.0;
    marker.scale.z = get_parameter("text_height").as_double();
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0F;
    marker.text = text;
    return marker;
  }

  Vec3 makeSafetyTextPosition() const
  {
    const double offset = get_parameter("status_text_outward_offset").as_double();
    return Vec3{
      plan_.opening_center.x + offset * plan_.outward_dir.x,
      plan_.opening_center.y + offset * plan_.outward_dir.y,
      plan_.opening_center.z};
  }

  bool lookupCurrentBladeTransform(geometry_msgs::msg::TransformStamped & transform)
  {
    try {
      transform = tf_buffer_->lookupTransform(
        base_frame_,
        blade_frame_,
        tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Optional TF lookup failed from %s to %s: %s",
        base_frame_.c_str(),
        blade_frame_.c_str(),
        ex.what());
      return false;
    }
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

  visualization_msgs::msg::MarkerArray makeMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(makeDeleteAllMarker());
    markers.markers.push_back(makeContourShellWallMarker());
    markers.markers.push_back(makeContourShellEdgeMarker());
    markers.markers.push_back(makeContourBoundaryMarker());
    markers.markers.push_back(makeOpeningLineMarker());
    markers.markers.push_back(makeEndpointMarker());
    markers.markers.push_back(makeSingleSphereMarker(2, plan_.opening_center, 1.0F, 0.45F, 0.0F));
    markers.markers.push_back(makeSingleSphereMarker(5, plan_.pre_center, 0.0F, 0.85F, 0.1F));

    return markers;
  }

  void timerCallback()
  {
    preinsert_pub_->publish(makePose(plan_.pre_center));
    insert_pub_->publish(makePose(plan_.inside_center));
    marker_pub_->publish(makeMarkers());
  }

  std::string base_frame_;
  std::string blade_frame_;
  std::string contour_csv_;
  std::vector<Vec3> contour_points_;
  std::vector<Segment> segments_;
  std::vector<Vec2> blade_samples_local_;
  size_t gap_index_;
  AlignmentPlan plan_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr preinsert_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr insert_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpeningAlignmentPlanner>());
  rclcpp::shutdown();
  return 0;
}
