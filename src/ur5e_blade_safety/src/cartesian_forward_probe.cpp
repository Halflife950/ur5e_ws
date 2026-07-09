#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

namespace
{

tf2::Vector3 axisFromName(const std::string & axis_name)
{
  if (axis_name == "plus_x") {
    return tf2::Vector3(1.0, 0.0, 0.0);
  }
  if (axis_name == "minus_x") {
    return tf2::Vector3(-1.0, 0.0, 0.0);
  }
  if (axis_name == "plus_y") {
    return tf2::Vector3(0.0, 1.0, 0.0);
  }
  if (axis_name == "minus_y") {
    return tf2::Vector3(0.0, -1.0, 0.0);
  }
  if (axis_name == "plus_z") {
    return tf2::Vector3(0.0, 0.0, 1.0);
  }
  if (axis_name == "minus_z") {
    return tf2::Vector3(0.0, 0.0, -1.0);
  }

  throw std::runtime_error(
    "Unsupported blade_forward_axis '" + axis_name +
    "'. Use plus_x, minus_x, plus_y, minus_y, plus_z, or minus_z.");
}

tf2::Vector3 directionInBase(
  const geometry_msgs::msg::TransformStamped & base_to_blade,
  const std::string & blade_forward_axis)
{
  const auto & q_msg = base_to_blade.transform.rotation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  q.normalize();

  const tf2::Matrix3x3 rotation(q);
  tf2::Vector3 direction = rotation * axisFromName(blade_forward_axis);
  if (direction.length2() < 1.0e-12) {
    throw std::runtime_error("Computed blade forward direction is near zero.");
  }
  direction.normalize();
  return direction;
}

double durationToSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

}  // namespace

class CartesianForwardProbe
{
public:
  explicit CartesianForwardProbe(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
    node_->declare_parameter<std::string>("move_group_name", "ur_manipulator");
    node_->declare_parameter<std::string>("base_frame", "base_link");
    node_->declare_parameter<std::string>("end_effector_link", "tool0");
    node_->declare_parameter<std::string>("blade_frame", "blade_center_link");
    node_->declare_parameter<std::string>("blade_forward_axis", "minus_y");
    node_->declare_parameter<double>("cartesian_distance", 0.18);
    node_->declare_parameter<double>("planning_time_s", 10.0);
    node_->declare_parameter<int>("planning_attempts", 5);
    node_->declare_parameter<double>("velocity_scaling", 0.03);
    node_->declare_parameter<double>("acceleration_scaling", 0.03);
    node_->declare_parameter<double>("max_cartesian_speed", 0.02);
    node_->declare_parameter<std::string>("cartesian_speed_link", "blade_center_link");
    node_->declare_parameter<double>("cartesian_eef_step", 0.002);
    node_->declare_parameter<double>("cartesian_jump_threshold", 0.0);
    node_->declare_parameter<double>("min_cartesian_fraction", 0.95);
    node_->declare_parameter<double>("tf_wait_timeout_s", 5.0);
    node_->declare_parameter<double>("start_delay_s", 1.0);
    node_->declare_parameter<bool>("execute_motion", true);
  }

  int run()
  {
    const double start_delay_s = node_->get_parameter("start_delay_s").as_double();
    if (start_delay_s > 0.0) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Waiting %.2f s before Cartesian forward probe.",
        start_delay_s);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(start_delay_s)));
    }

    const std::string move_group_name = node_->get_parameter("move_group_name").as_string();
    const std::string base_frame = node_->get_parameter("base_frame").as_string();
    const std::string end_effector_link = node_->get_parameter("end_effector_link").as_string();
    const std::string blade_frame = node_->get_parameter("blade_frame").as_string();
    const std::string blade_forward_axis =
      node_->get_parameter("blade_forward_axis").as_string();
    const double distance = node_->get_parameter("cartesian_distance").as_double();

    geometry_msgs::msg::TransformStamped base_to_blade;
    try {
      base_to_blade = tf_buffer_.lookupTransform(
        base_frame,
        blade_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(node_->get_parameter("tf_wait_timeout_s").as_double()));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to lookup %s -> %s: %s",
        base_frame.c_str(),
        blade_frame.c_str(),
        ex.what());
      return 1;
    }

    tf2::Vector3 direction;
    try {
      direction = directionInBase(base_to_blade, blade_forward_axis);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
      return 1;
    }

    MoveGroupInterface move_group(node_, move_group_name);
    move_group.setEndEffectorLink(end_effector_link);
    move_group.setPlanningTime(node_->get_parameter("planning_time_s").as_double());
    move_group.setNumPlanningAttempts(node_->get_parameter("planning_attempts").as_int());
    move_group.setMaxVelocityScalingFactor(node_->get_parameter("velocity_scaling").as_double());
    move_group.setMaxAccelerationScalingFactor(
      node_->get_parameter("acceleration_scaling").as_double());
    move_group.setStartStateToCurrentState();

    geometry_msgs::msg::Pose target = move_group.getCurrentPose(end_effector_link).pose;
    target.position.x += direction.x() * distance;
    target.position.y += direction.y() * distance;
    target.position.z += direction.z() * distance;

    RCLCPP_INFO(
      node_->get_logger(),
      "Cartesian forward probe: axis=%s distance=%.3f m direction(base)=(%.4f, %.4f, %.4f), target=(%.4f, %.4f, %.4f).",
      blade_forward_axis.c_str(),
      distance,
      direction.x(),
      direction.y(),
      direction.z(),
      target.position.x,
      target.position.y,
      target.position.z);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group.computeCartesianPath(
      waypoints,
      node_->get_parameter("cartesian_eef_step").as_double(),
      node_->get_parameter("cartesian_jump_threshold").as_double(),
      trajectory);

    RCLCPP_INFO(node_->get_logger(), "Forward probe Cartesian path fraction: %.3f", fraction);

    if (fraction < node_->get_parameter("min_cartesian_fraction").as_double()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Forward probe path incomplete. Required %.3f, got %.3f.",
        node_->get_parameter("min_cartesian_fraction").as_double(),
        fraction);
      return 1;
    }

    if (!applyCartesianSpeedLimit(move_group, trajectory)) {
      return 1;
    }

    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    if (!node_->get_parameter("execute_motion").as_bool()) {
      RCLCPP_WARN(node_->get_logger(), "execute_motion=false, skipping forward probe execution.");
      return 0;
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "Executing forward probe. Safety interlock should cancel this trajectory if /blade_contour_safe becomes false.");
    const auto result = move_group.execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Forward probe execution did not complete successfully. This is expected if the safety interlock cancelled it.");
      return 2;
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "Forward probe reached the full target without being cancelled.");
    return 0;
  }

private:
  bool applyCartesianSpeedLimit(
    MoveGroupInterface & move_group,
    moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    const double max_speed = node_->get_parameter("max_cartesian_speed").as_double();
    if (max_speed <= 0.0) {
      RCLCPP_WARN(node_->get_logger(), "max_cartesian_speed<=0, Cartesian speed limit disabled.");
      return true;
    }

    auto & joint_trajectory = trajectory.joint_trajectory;
    if (joint_trajectory.points.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Forward probe trajectory has no points.");
      return false;
    }

    const std::string link_name = node_->get_parameter("cartesian_speed_link").as_string();
    const auto robot_model = move_group.getRobotModel();
    if (!robot_model) {
      RCLCPP_ERROR(node_->get_logger(), "Unable to get robot model for Cartesian speed limit.");
      return false;
    }

    const moveit::core::LinkModel * link_model = robot_model->getLinkModel(link_name);
    if (!link_model) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Cartesian speed link %s was not found in the robot model.",
        link_name.c_str());
      return false;
    }

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2.0);
    if (!current_state) {
      RCLCPP_ERROR(node_->get_logger(), "Unable to get current robot state for speed limit.");
      return false;
    }

    moveit::core::RobotState state(*current_state);
    state.update();
    Eigen::Vector3d previous_position = state.getGlobalLinkTransform(link_model).translation();
    double previous_limited_time = 0.0;
    double max_observed_speed = 0.0;

    for (auto & point : joint_trajectory.points) {
      if (point.positions.size() != joint_trajectory.joint_names.size()) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Trajectory point has %zu positions for %zu joints.",
          point.positions.size(),
          joint_trajectory.joint_names.size());
        return false;
      }

      for (std::size_t i = 0; i < joint_trajectory.joint_names.size(); ++i) {
        state.setVariablePosition(joint_trajectory.joint_names[i], point.positions[i]);
      }
      state.update();

      const Eigen::Vector3d current_position =
        state.getGlobalLinkTransform(link_model).translation();
      const double segment_distance = (current_position - previous_position).norm();
      const double original_time = durationToSeconds(point.time_from_start);
      const double minimum_time = previous_limited_time + segment_distance / max_speed;
      const double limited_time = std::max(original_time, minimum_time);

      if (limited_time > original_time + 1.0e-9) {
        point.time_from_start = rclcpp::Duration::from_seconds(limited_time);
        point.velocities.clear();
        point.accelerations.clear();
        point.effort.clear();
      }

      const double dt = limited_time - previous_limited_time;
      if (dt > 1.0e-9) {
        max_observed_speed = std::max(max_observed_speed, segment_distance / dt);
      }

      previous_position = current_position;
      previous_limited_time = limited_time;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Applied Cartesian speed limit: link=%s max_speed=%.4f m/s final_duration=%.3f s max_segment_speed=%.4f m/s.",
      link_name.c_str(),
      max_speed,
      previous_limited_time,
      max_observed_speed);
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("cartesian_forward_probe");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  CartesianForwardProbe probe(node);
  const int result = probe.run();

  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  return result;
}
