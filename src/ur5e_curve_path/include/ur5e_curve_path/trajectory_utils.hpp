#pragma once

#include <builtin_interfaces/msg/duration.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace ur5e_curve_path
{

struct TrajectoryLimitStats
{
  double largest_joint_velocity{};
  std::size_t largest_velocity_segment{};
  std::size_t largest_velocity_joint{};
  double largest_joint_acceleration{};
  std::size_t largest_acceleration_segment{};
  std::size_t largest_acceleration_joint{};
};

struct TimedTrajectoryBuildResult
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  double total_path_length{};
  double peak_velocity{};
  double cartesian_acceleration{};
  double duration{};
  bool triangular_profile{};
};

builtin_interfaces::msg::Duration durationFromSeconds(
  double seconds);

double durationToSeconds(
  const builtin_interfaces::msg::Duration & duration);

double shortestAngularDifference(
  double from,
  double to);

TrajectoryLimitStats calculateTrajectoryLimitStats(
  const trajectory_msgs::msg::JointTrajectory & trajectory);

void scaleTrajectoryTiming(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  double time_scale);

TimedTrajectoryBuildResult buildTimedJointTrajectory(
  const std::vector<std::string> & joint_names,
  const std::vector<std::vector<double>> & joint_solutions,
  const std::vector<double> & segment_lengths,
  double cartesian_speed,
  double cartesian_acceleration);

double timeAtPathDistance(
  double distance,
  double total_distance,
  double acceleration,
  double peak_velocity,
  double acceleration_distance,
  double acceleration_time,
  double cruise_time,
  bool triangular_profile);

}  // namespace ur5e_curve_path
