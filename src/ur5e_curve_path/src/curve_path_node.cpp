#include "ur5e_curve_path/curve_path_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/robot_state/robot_state.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <stdexcept>

#include "ur5e_curve_path/ik_utils.hpp"
#include "ur5e_curve_path/orientation_utils.hpp"
#include "ur5e_curve_path/trajectory_utils.hpp"

using namespace std::chrono_literals;

namespace ur5e_curve_path
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

CurvePathNode::CurvePathNode()
: Node(
    "curve_path_node",
    rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true))
{
  config_ =
    loadCurvePathConfig(*this);

  marker_publisher_ =
    create_publisher<visualization_msgs::msg::Marker>(
    config_.marker_topic,
    rclcpp::QoS(1).transient_local());

  marker_timer_ = create_wall_timer(
    1s,
    std::bind(
      &CurvePathNode::publishPathMarker,
      this));
}


void CurvePathNode::initialize()
{
  loadAndValidatePath();

  move_group_ = std::make_unique<
    moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(),
    config_.move_group_name);

  configureMoveGroupForPlanning();

  moveToStartJointPose();

  // Allow joint states and TF data to update.
  rclcpp::sleep_for(1s);

  path_origin_ =
    move_group_->getCurrentPose(
    config_.end_effector_link).pose;

  verifyDownwardOrientation(
    path_origin_,
    config_.end_effector_link,
    config_.max_tilt_error_deg,
    get_logger());

  RCLCPP_INFO(
    get_logger(),
    "Path origin position: [%.6f, %.6f, %.6f] m",
    path_origin_.position.x,
    path_origin_.position.y,
    path_origin_.position.z);

  RCLCPP_INFO(
    get_logger(),
    "Path origin orientation: "
    "[x=%.6f, y=%.6f, z=%.6f, w=%.6f]",
    path_origin_.orientation.x,
    path_origin_.orientation.y,
    path_origin_.orientation.z,
    path_origin_.orientation.w);

  path_ready_ = true;
  publishPathMarker();

  RCLCPP_INFO(
    get_logger(),
    "The verified %s pose is now the path origin.",
    config_.end_effector_link.c_str());

  validatePathIk();

  RCLCPP_INFO(
    get_logger(),
    "Path geometry and continuous IK validation completed.");

  buildTimedTrajectory();
  enforceTimedTrajectoryLimits();
  validateTimedTrajectory();

  RCLCPP_INFO(
    get_logger(),
    "Timed trajectory validation completed.");

  executeTimedTrajectoryLoop();
}


void CurvePathNode::configureMoveGroupForPlanning()
{
  move_group_->setPlanningTime(
    config_.planning_time_s);

  move_group_->setNumPlanningAttempts(
    config_.planning_attempts);

  move_group_->setMaxVelocityScalingFactor(
    config_.velocity_scaling);

  move_group_->setMaxAccelerationScalingFactor(
    config_.acceleration_scaling);
}


std::string CurvePathNode::resolveCsvPath() const
{
  if (
    !config_.path_csv.empty() &&
    config_.path_csv.front() == '/')
  {
    return config_.path_csv;
  }

  const std::string package_share =
    ament_index_cpp::get_package_share_directory(
    "ur5e_curve_path");

  return
    package_share +
    "/" +
    config_.path_csv;
}


void CurvePathNode::loadAndValidatePath()
{
  const std::string csv_path =
    resolveCsvPath();

  raw_points_ = loadCsv(csv_path);

  validatePathContinuity(
    raw_points_,
    config_.max_gap_mm,
    get_logger());

  RCLCPP_INFO(
    get_logger(),
    "Loaded %zu path points from: %s",
    raw_points_.size(),
    csv_path.c_str());

  RCLCPP_INFO(
    get_logger(),
    "CSV axis mapping: csv %s -> base_link X, "
    "csv %s -> base_link Y, csv %s -> base_link Z.",
    csvAxisName(
      config_.
      axis_mapping.
      base_x_from_csv_axis).c_str(),
    csvAxisName(
      config_.
      axis_mapping.
      base_y_from_csv_axis).c_str(),
    csvAxisName(
      config_.
      axis_mapping.
      base_z_from_csv_axis).c_str());

  calculatePathBounds();
}


void CurvePathNode::calculatePathBounds()
{
  const PathBounds bounds =
    ur5e_curve_path::calculatePathBounds(
      raw_points_,
      config_.axis_mapping);

  RCLCPP_INFO(
    get_logger(),
    "Raw path dimensions: "
    "X %.3f mm, Y %.3f mm, Z %.3f mm",
    bounds.raw_dimensions.x_mm,
    bounds.raw_dimensions.y_mm,
    bounds.raw_dimensions.z_mm);

  RCLCPP_INFO(
    get_logger(),
    "Mapped path dimensions: "
    "base_link X %.3f mm, Y %.3f mm, Z %.3f mm",
    bounds.mapped_dimensions.x_mm,
    bounds.mapped_dimensions.y_mm,
    bounds.mapped_dimensions.z_mm);
}


void CurvePathNode::moveToStartJointPose()
{
  /*
   * Joint values recorded from /joint_states.
   *
   * Joint order is specified explicitly by name to avoid
   * relying on the internal group variable order.
   */
  const std::map<std::string, double> start_joint_positions = {
    {
      "shoulder_pan_joint",
      0.0
    },
    {
      "shoulder_lift_joint",
      -1.2217305
    },
    {
      "elbow_joint",
      1.8849556
    },
    {
      "wrist_1_joint",
      -2.2165682
    },
    {
      "wrist_2_joint",
      -1.5533430
    },
    {
      "wrist_3_joint",
      0.0
    }
  };

  move_group_->setStartStateToCurrentState();

  if (!move_group_->setJointValueTarget(
      start_joint_positions))
  {
    throw std::runtime_error(
      "Failed to set the start joint target.");
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  RCLCPP_INFO(
    get_logger(),
    "Planning motion to the saved start joint pose.");

  const auto planning_result =
    move_group_->plan(plan);

  if (
    planning_result !=
    moveit::core::MoveItErrorCode::SUCCESS)
  {
    throw std::runtime_error(
      "Planning to the start joint pose failed.");
  }

  RCLCPP_INFO(
    get_logger(),
    "Start pose plan succeeded. Executing.");

  const auto execution_result =
    move_group_->execute(plan);

  if (
    execution_result !=
    moveit::core::MoveItErrorCode::SUCCESS)
  {
    throw std::runtime_error(
      "Execution of the start joint pose failed.");
  }

  move_group_->stop();
  move_group_->clearPoseTargets();

  RCLCPP_INFO(
    get_logger(),
    "The robot reached the saved start joint pose.");
}


void CurvePathNode::validatePathIk()
{
  const moveit::core::RobotModelConstPtr robot_model =
    move_group_->getRobotModel();

  if (!robot_model)
  {
    throw std::runtime_error(
      "Unable to obtain the robot model.");
  }

  const moveit::core::JointModelGroup * joint_model_group =
    robot_model->getJointModelGroup(
      config_.move_group_name);

  if (!joint_model_group)
  {
    throw std::runtime_error(
      "Joint model group " +
      config_.move_group_name +
      " was not found.");
  }

  joint_names_ =
    joint_model_group->getVariableNames();

  joint_solutions_.clear();
  joint_solutions_.reserve(
    raw_points_.size());

  moveit::core::RobotStatePtr current_state =
    move_group_->getCurrentState(2.0);

  if (!current_state)
  {
    throw std::runtime_error(
      "Unable to obtain the current robot state.");
  }

  moveit::core::RobotState ik_state =
    *current_state;

  std::vector<double> previous_joint_values;

  ik_state.copyJointGroupPositions(
    joint_model_group,
    previous_joint_values);

  double largest_joint_step = 0.0;
  std::size_t largest_step_point = 0;
  double largest_absolute_yaw_offset = 0.0;
  std::size_t largest_yaw_point = 0;

  RCLCPP_INFO(
    get_logger(),
    "Starting yaw-released continuous IK validation for %zu path points "
    "with %d yaw samples per point.",
    raw_points_.size(),
    config_.yaw_sample_count);

  for (std::size_t i = 0;
       i < raw_points_.size();
       ++i)
  {
    const geometry_msgs::msg::Point target_position =
      transformPathPoint(
        raw_points_[i],
        raw_points_.front(),
        path_origin_.position,
        config_.axis_mapping);

    moveit::core::RobotState best_state =
      ik_state;

    double yaw_offset = 0.0;

    const bool ik_found =
      findBestYawReleasedIk(
        joint_model_group,
        target_position,
        path_origin_.orientation,
        config_.end_effector_link,
        config_.yaw_sample_count,
        config_.ik_timeout_s,
        previous_joint_values,
        ik_state,
        best_state,
        yaw_offset);

    if (!ik_found)
    {
      throw std::runtime_error(
        "IK failed at path point " +
        std::to_string(i) +
        ".");
    }

    ik_state =
      best_state;

    ik_state.update();

    const double absolute_yaw_offset =
      std::abs(yaw_offset);

    if (absolute_yaw_offset > largest_absolute_yaw_offset)
    {
      largest_absolute_yaw_offset =
        absolute_yaw_offset;

      largest_yaw_point =
        i;
    }

    std::vector<double> current_joint_values;

    ik_state.copyJointGroupPositions(
      joint_model_group,
      current_joint_values);

    joint_solutions_.push_back(
      current_joint_values);

    double point_maximum_step = 0.0;

    for (std::size_t joint_index = 0;
         joint_index < current_joint_values.size();
         ++joint_index)
    {
      const double step =
        std::abs(
          shortestAngularDifference(
            previous_joint_values[joint_index],
            current_joint_values[joint_index]));

      point_maximum_step =
        std::max(
          point_maximum_step,
          step);
    }

    if (point_maximum_step > largest_joint_step)
    {
      largest_joint_step =
        point_maximum_step;

      largest_step_point =
        i;
    }

    if (point_maximum_step > config_.max_joint_step_rad)
    {
      throw std::runtime_error(
        "Joint discontinuity detected at path point " +
        std::to_string(i) +
        ". Maximum joint step: " +
        std::to_string(point_maximum_step) +
        " rad.");
    }

    previous_joint_values =
      current_joint_values;

    if (
      i % 20 == 0 ||
      i + 1 == raw_points_.size())
    {
      RCLCPP_INFO(
        get_logger(),
        "IK validation progress: %zu / %zu",
        i + 1,
        raw_points_.size());
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "Continuous IK validation passed.");

  RCLCPP_INFO(
    get_logger(),
    "Largest joint step: %.6f rad at path point %zu.",
    largest_joint_step,
    largest_step_point);

  RCLCPP_INFO(
    get_logger(),
    "Largest selected yaw offset: %.4f deg at path point %zu.",
    largest_absolute_yaw_offset *
    180.0 /
    kPi,
    largest_yaw_point);
}


double CurvePathNode::pathPointDistanceMeters(
  const std::size_t first_index,
  const std::size_t second_index) const
{
  const geometry_msgs::msg::Point first =
    transformPathPoint(
      raw_points_.at(first_index),
      raw_points_.front(),
      path_origin_.position,
      config_.axis_mapping);

  const geometry_msgs::msg::Point second =
    transformPathPoint(
      raw_points_.at(second_index),
      raw_points_.front(),
      path_origin_.position,
      config_.axis_mapping);

  return pointDistanceMeters(
    first,
    second);
}


void CurvePathNode::buildTimedTrajectory()
{
  if (
    joint_solutions_.size() !=
    raw_points_.size())
  {
    throw std::runtime_error(
      "The number of IK solutions does not match "
      "the number of path points.");
  }

  std::vector<double> segment_lengths;
  segment_lengths.reserve(
    raw_points_.size() - 1);

  for (std::size_t i = 1;
       i < raw_points_.size();
       ++i)
  {
    segment_lengths.push_back(
      pathPointDistanceMeters(
        i - 1,
        i));
  }

  const TimedTrajectoryBuildResult result =
    buildTimedJointTrajectory(
      joint_names_,
      joint_solutions_,
      segment_lengths,
      config_.cartesian_speed_mps,
      config_.cartesian_acceleration_mps2);

  timed_trajectory_ =
    result.trajectory;

  RCLCPP_INFO(
    get_logger(),
    "Generated a %s Cartesian speed profile.",
    result.triangular_profile ?
    "triangular" :
    "trapezoidal");

  RCLCPP_INFO(
    get_logger(),
    "Path length: %.4f m",
    result.total_path_length);

  RCLCPP_INFO(
    get_logger(),
    "Peak Cartesian speed: %.4f m/s",
    result.peak_velocity);

  RCLCPP_INFO(
    get_logger(),
    "Cartesian acceleration limit: %.4f m/s^2",
    result.cartesian_acceleration);

  RCLCPP_INFO(
    get_logger(),
    "Trajectory duration: %.4f s",
    result.duration);
}


void CurvePathNode::enforceTimedTrajectoryLimits()
{
  const TrajectoryLimitStats stats =
    calculateTrajectoryLimitStats(
      timed_trajectory_.
      joint_trajectory);

  double time_scale = 1.0;

  if (
    stats.largest_joint_velocity >
    config_.joint_velocity_limit_radps)
  {
    time_scale =
      std::max(
        time_scale,
        stats.largest_joint_velocity /
        config_.joint_velocity_limit_radps);
  }

  if (
    stats.largest_joint_acceleration >
    config_.joint_acceleration_limit_radps2)
  {
    time_scale =
      std::max(
        time_scale,
        std::sqrt(
          stats.largest_joint_acceleration /
          config_.joint_acceleration_limit_radps2));
  }

  if (time_scale <= 1.0)
  {
    return;
  }

  constexpr double margin = 1.05;
  const double final_time_scale =
    time_scale *
    margin;

  RCLCPP_WARN(
    get_logger(),
    "Stretching trajectory time by %.3fx to satisfy joint limits. "
    "Initial max velocity %.5f rad/s on %s at segment %zu; "
    "initial max acceleration %.5f rad/s^2 on %s near segment %zu.",
    final_time_scale,
    stats.largest_joint_velocity,
    joint_names_[
      stats.largest_velocity_joint].c_str(),
    stats.largest_velocity_segment,
    stats.largest_joint_acceleration,
    joint_names_[
      stats.largest_acceleration_joint].c_str(),
    stats.largest_acceleration_segment);

  scaleTrajectoryTiming(
    timed_trajectory_.
    joint_trajectory,
    final_time_scale);
}


void CurvePathNode::validateTimedTrajectory()
{
  const TrajectoryLimitStats stats =
    calculateTrajectoryLimitStats(
      timed_trajectory_.
      joint_trajectory);

  if (
    stats.largest_joint_velocity >
    config_.joint_velocity_limit_radps)
  {
    throw std::runtime_error(
      "Joint velocity limit exceeded on " +
      joint_names_[
        stats.largest_velocity_joint] +
      " at trajectory segment " +
      std::to_string(
        stats.largest_velocity_segment) +
      ". Measured velocity: " +
      std::to_string(
        stats.largest_joint_velocity) +
      " rad/s.");
  }

  if (
    stats.largest_joint_acceleration >
    config_.joint_acceleration_limit_radps2)
  {
    throw std::runtime_error(
      "Joint acceleration limit exceeded on " +
      joint_names_[
        stats.largest_acceleration_joint] +
      " near trajectory segment " +
      std::to_string(
        stats.largest_acceleration_segment) +
      ". Measured acceleration: " +
      std::to_string(
        stats.largest_joint_acceleration) +
      " rad/s^2.");
  }

  RCLCPP_INFO(
    get_logger(),
    "Largest average joint velocity: %.5f rad/s "
    "on %s at segment %zu.",
    stats.largest_joint_velocity,
    joint_names_[
      stats.largest_velocity_joint].c_str(),
    stats.largest_velocity_segment);

  RCLCPP_INFO(
    get_logger(),
    "Largest estimated joint acceleration: %.5f rad/s^2 "
    "on %s near segment %zu.",
    stats.largest_joint_acceleration,
    joint_names_[
      stats.largest_acceleration_joint].c_str(),
    stats.largest_acceleration_segment);
}


void CurvePathNode::executeTimedTrajectoryLoop()
{
  moveit::planning_interface::
    MoveGroupInterface::Plan trajectory_plan;

  trajectory_plan.trajectory_ =
    timed_trajectory_;

  std::size_t loop_index = 0;

  RCLCPP_INFO(
    get_logger(),
    "Starting continuous heart trajectory loop.");

  while (rclcpp::ok())
  {
    ++loop_index;

    RCLCPP_INFO(
      get_logger(),
      "Executing heart trajectory loop %zu.",
      loop_index);

    const auto execution_result =
      move_group_->execute(
        trajectory_plan);

    if (
      execution_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
    {
      throw std::runtime_error(
        "Heart trajectory execution failed at loop " +
        std::to_string(loop_index) +
        ".");
    }

    move_group_->stop();

    RCLCPP_INFO(
      get_logger(),
      "Heart trajectory loop %zu completed.",
      loop_index);

    // Brief pause between loops to avoid immediately
    // reissuing a trajectory goal to the controller.
    rclcpp::sleep_for(
      std::chrono::milliseconds(
        config_.loop_pause_ms));
  }
}


void CurvePathNode::publishPathMarker()
{
  if (!path_ready_)
  {
    return;
  }

  visualization_msgs::msg::Marker marker;

  marker.header.frame_id =
    config_.marker_frame;
  marker.header.stamp = now();

  marker.ns =
    config_.marker_namespace;
  marker.id = 0;

  marker.type =
    visualization_msgs::msg::Marker::LINE_STRIP;

  marker.action =
    visualization_msgs::msg::Marker::ADD;

  marker.pose.orientation.w = 1.0;

  // For LINE_STRIP, scale.x specifies the line width.
  marker.scale.x = 0.003;

  marker.color.r = 1.0;
  marker.color.g = 0.1;
  marker.color.b = 0.1;
  marker.color.a = 1.0;

  marker.points.reserve(
    raw_points_.size());

  for (const auto & point : raw_points_)
  {
    marker.points.push_back(
      transformPathPoint(
        point,
        raw_points_.front(),
        path_origin_.position,
        config_.axis_mapping));
  }

  // No extra closure point is added here.
  // The CSV path must already be continuous and closed.

  marker_publisher_->publish(marker);
}

}  // namespace ur5e_curve_path
