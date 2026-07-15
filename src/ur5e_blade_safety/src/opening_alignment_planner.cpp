#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "ur5e_blade_safety/contour_model.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using ur5e_blade_safety::ContourModel;
using ur5e_blade_safety::loadFixedContourModel;
using ur5e_blade_safety::makeOpeningAlignmentPlan;
using ur5e_blade_safety::OpeningAlignmentPlan;
using ur5e_blade_safety::Vec3;

class OpeningAlignmentPlanner : public rclcpp::Node
{
public:
  OpeningAlignmentPlanner()
  : Node("opening_alignment_planner")
  {
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("contour_csv", "data/cavity_contour.csv");
    declare_parameter<double>("csv_scale", 0.001);
    declare_parameter<double>("pre_offset", 0.03);
    declare_parameter<double>("approach_offset", 0.08);
    declare_parameter<double>("contour_z", 0.20);
    declare_parameter<double>("target_z", 0.20);
    declare_parameter<bool>("tool_z_axis_down", true);
    declare_parameter<std::string>("blade_shaft_axis", "minus_y");
    declare_parameter<double>("publish_period_ms", 100.0);
    declare_parameter<double>("line_width", 0.004);
    declare_parameter<double>("contour_line_width", 0.004);
    declare_parameter<double>("shell_bottom_z", 0.0);
    declare_parameter<double>("shell_height", 0.25);
    declare_parameter<double>("shell_edge_width", 0.003);
    declare_parameter<double>("shell_wall_alpha", 0.18);
    declare_parameter<double>("endpoint_radius", 0.025);
    declare_parameter<double>("target_radius", 0.025);

    base_frame_ = get_parameter("base_frame").as_string();
    contour_csv_ = get_parameter("contour_csv").as_string();
    contour_model_ = loadFixedContourModel(
      contour_csv_,
      get_parameter("csv_scale").as_double(),
      get_parameter("contour_z").as_double());
    plan_ = makeOpeningAlignmentPlan(
      contour_model_,
      get_parameter("pre_offset").as_double(),
      get_parameter("approach_offset").as_double(),
      get_parameter("target_z").as_double(),
      get_parameter("blade_shaft_axis").as_string());

    approach_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("blade_approach_pose", 10);
    preinsert_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("blade_preinsert_pose", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "opening_alignment_markers", 10);

    const auto period_ms = get_parameter("publish_period_ms").as_double();
    const auto period = std::chrono::duration<double, std::milli>(period_ms);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&OpeningAlignmentPlanner::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu contour points from %s. Opening gap: %zu -> %zu. "
      "M=(%.4f, %.4f), outward=(%.4f, %.4f), insert=(%.4f, %.4f), yaw=%.3f rad.",
      contour_model_.points.size(),
      ur5e_blade_safety::resolvePackagePath(contour_csv_).c_str(),
      contour_model_.opening_gap_index,
      (contour_model_.opening_gap_index + 1) % contour_model_.points.size(),
      plan_.opening_center.x,
      plan_.opening_center.y,
      plan_.outward_dir.x,
      plan_.outward_dir.y,
      plan_.insert_dir.x,
      plan_.insert_dir.y,
      plan_.yaw);
  }

private:
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

    for (const auto & segment : contour_model_.wall_segments) {
      const Vec3 a_bottom = withZ(segment.a, bottom_z);
      const Vec3 b_bottom = withZ(segment.b, bottom_z);
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

    for (const auto & segment : contour_model_.wall_segments) {
      const Vec3 a_bottom = withZ(segment.a, bottom_z);
      const Vec3 b_bottom = withZ(segment.b, bottom_z);
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
    markers.markers.push_back(makeSingleSphereMarker(4, plan_.approach_center, 0.3F, 0.65F, 1.0F));
    markers.markers.push_back(makeSingleSphereMarker(5, plan_.pre_center, 0.0F, 0.85F, 0.1F));

    return markers;
  }

  void timerCallback()
  {
    approach_pub_->publish(makePose(plan_.approach_center));
    preinsert_pub_->publish(makePose(plan_.pre_center));
    marker_pub_->publish(makeMarkers());
  }

  std::string base_frame_;
  std::string contour_csv_;
  ContourModel contour_model_;
  OpeningAlignmentPlan plan_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr approach_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr preinsert_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpeningAlignmentPlanner>());
  rclcpp::shutdown();
  return 0;
}
