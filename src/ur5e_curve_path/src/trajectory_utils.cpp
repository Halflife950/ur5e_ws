#include "ur5e_curve_path/trajectory_utils.hpp"

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace ur5e_curve_path
{
namespace
{
constexpr double kNanosecondsPerSecond = 1.0e9;
}  // namespace

builtin_interfaces::msg::Duration durationFromSeconds(
  const double seconds)
{
  builtin_interfaces::msg::Duration duration;

  const double clamped_seconds =
    std::max(0.0, seconds);

  duration.sec =
    static_cast<std::int32_t>(
      std::floor(clamped_seconds));

  const double fractional_seconds =
    clamped_seconds -
    static_cast<double>(duration.sec);

  duration.nanosec =
    static_cast<std::uint32_t>(
      std::llround(
        fractional_seconds * kNanosecondsPerSecond));

  if (duration.nanosec >= 1000000000U)
  {
    ++duration.sec;
    duration.nanosec -= 1000000000U;
  }

  return duration;
}


double durationToSeconds(
  const builtin_interfaces::msg::Duration & duration)
{
  return
    static_cast<double>(duration.sec) +
    static_cast<double>(duration.nanosec) *
    1.0e-9;
}


double shortestAngularDifference(
  const double from,
  const double to)
{
  return std::atan2(
    std::sin(to - from),
    std::cos(to - from));
}


TrajectoryLimitStats calculateTrajectoryLimitStats(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  const auto & points =
    trajectory.points;

  if (points.size() < 2)
  {
    throw std::runtime_error(
      "The timed trajectory contains fewer than two points.");
  }

  const std::size_t joint_count =
    trajectory.joint_names.size();

  if (joint_count == 0)
  {
    throw std::runtime_error(
      "The joint name list is empty.");
  }

  for (std::size_t i = 0;
       i < points.size();
       ++i)
  {
    if (points[i].positions.size() != joint_count)
    {
      throw std::runtime_error(
        "Trajectory point " +
        std::to_string(i) +
        " has " +
        std::to_string(points[i].positions.size()) +
        " joint positions, but " +
        std::to_string(joint_count) +
        " joint names are configured.");
    }
  }

  std::vector<std::vector<double>>
    segment_velocities;

  std::vector<double>
    segment_durations;

  segment_velocities.reserve(
    points.size() - 1);

  segment_durations.reserve(
    points.size() - 1);

  TrajectoryLimitStats stats;

  for (std::size_t i = 1;
       i < points.size();
       ++i)
  {
    const double previous_time =
      durationToSeconds(
        points[i - 1].time_from_start);

    const double current_time =
      durationToSeconds(
        points[i].time_from_start);

    const double delta_time =
      current_time -
      previous_time;

    if (delta_time <= 0.0)
    {
      throw std::runtime_error(
        "Trajectory timestamps are not strictly increasing "
        "at segment " +
        std::to_string(i - 1) +
        ".");
    }

    segment_durations.push_back(
      delta_time);

    std::vector<double> velocity(
      joint_count,
      0.0);

    for (std::size_t joint_index = 0;
         joint_index < joint_count;
         ++joint_index)
    {
      const double delta_joint =
        shortestAngularDifference(
          points[i - 1].
            positions[joint_index],
          points[i].
            positions[joint_index]);

      velocity[joint_index] =
        delta_joint /
        delta_time;

      const double absolute_velocity =
        std::abs(
          velocity[joint_index]);

      if (
        absolute_velocity >
        stats.largest_joint_velocity)
      {
        stats.largest_joint_velocity =
          absolute_velocity;

        stats.largest_velocity_segment =
          i - 1;

        stats.largest_velocity_joint =
          joint_index;
      }
    }

    segment_velocities.push_back(
      velocity);
  }

  for (std::size_t i = 1;
       i < segment_velocities.size();
       ++i)
  {
    const double acceleration_interval =
      0.5 *
      (
        segment_durations[i - 1] +
        segment_durations[i]
      );

    if (acceleration_interval <= 0.0)
    {
      throw std::runtime_error(
        "Invalid acceleration interval at segment " +
        std::to_string(i) +
        ".");
    }

    for (std::size_t joint_index = 0;
         joint_index < joint_count;
         ++joint_index)
    {
      const double acceleration =
        (
          segment_velocities[i][joint_index] -
          segment_velocities[i - 1][joint_index]
        ) /
        acceleration_interval;

      const double absolute_acceleration =
        std::abs(
          acceleration);

      if (
        absolute_acceleration >
        stats.largest_joint_acceleration)
      {
        stats.largest_joint_acceleration =
          absolute_acceleration;

        stats.largest_acceleration_segment =
          i;

        stats.largest_acceleration_joint =
          joint_index;
      }
    }
  }

  return stats;
}


void scaleTrajectoryTiming(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const double time_scale)
{
  if (time_scale <= 1.0)
  {
    return;
  }

  for (auto & point : trajectory.points)
  {
    const double scaled_time =
      durationToSeconds(
        point.time_from_start) *
      time_scale;

    point.time_from_start =
      durationFromSeconds(
        scaled_time);
  }
}


TimedTrajectoryBuildResult buildTimedJointTrajectory(
  const std::vector<std::string> & joint_names,
  const std::vector<std::vector<double>> & joint_solutions,
  const std::vector<double> & segment_lengths,
  const double cartesian_speed,
  const double cartesian_acceleration)
{
  if (joint_solutions.empty())
  {
    throw std::runtime_error(
      "The IK solution list is empty.");
  }

  if (joint_names.empty())
  {
    throw std::runtime_error(
      "The joint name list is empty.");
  }

  if (
    segment_lengths.size() + 1 !=
    joint_solutions.size())
  {
    throw std::runtime_error(
      "The number of path segments does not match "
      "the number of IK solutions.");
  }

  TimedTrajectoryBuildResult result;

  std::vector<double> cumulative_path_length;

  cumulative_path_length.assign(
    joint_solutions.size(),
    0.0);

  for (std::size_t i = 1;
       i < joint_solutions.size();
       ++i)
  {
    cumulative_path_length[i] =
      cumulative_path_length[i - 1] +
      segment_lengths[i - 1];
  }

  result.total_path_length =
    cumulative_path_length.back();

  if (result.total_path_length <= 1e-6)
  {
    throw std::runtime_error(
      "The transformed path length is too small.");
  }

  const double nominal_acceleration_distance =
    cartesian_speed *
    cartesian_speed /
    (
      2.0 *
      cartesian_acceleration
    );

  result.triangular_profile =
    2.0 * nominal_acceleration_distance >
    result.total_path_length;

  result.peak_velocity =
    cartesian_speed;

  double acceleration_distance =
    nominal_acceleration_distance;

  double acceleration_time =
    result.peak_velocity /
    cartesian_acceleration;

  double cruise_time = 0.0;

  if (result.triangular_profile)
  {
    acceleration_distance =
      0.5 *
      result.total_path_length;

    result.peak_velocity =
      std::sqrt(
        result.total_path_length *
        cartesian_acceleration);

    acceleration_time =
      result.peak_velocity /
      cartesian_acceleration;
  }
  else
  {
    const double cruise_distance =
      result.total_path_length -
      2.0 * acceleration_distance;

    cruise_time =
      cruise_distance /
      result.peak_velocity;
  }

  result.cartesian_acceleration =
    cartesian_acceleration;

  result.duration =
    2.0 * acceleration_time +
    cruise_time;

  result.trajectory =
    moveit_msgs::msg::RobotTrajectory();

  result.
    trajectory.
    joint_trajectory.
    joint_names =
    joint_names;

  auto & trajectory_points =
    result.
    trajectory.
    joint_trajectory.
    points;

  trajectory_points.clear();
  trajectory_points.reserve(
    joint_solutions.size());

  double previous_time = -1.0;

  for (std::size_t i = 0;
       i < joint_solutions.size();
       ++i)
  {
    double point_time =
      timeAtPathDistance(
        cumulative_path_length[i],
        result.total_path_length,
        cartesian_acceleration,
        result.peak_velocity,
        acceleration_distance,
        acceleration_time,
        cruise_time,
        result.triangular_profile);

    if (
      i > 0 &&
      point_time <= previous_time)
    {
      point_time =
        previous_time + 0.001;
    }

    trajectory_msgs::msg::
      JointTrajectoryPoint trajectory_point;

    trajectory_point.positions =
      joint_solutions[i];

    trajectory_point.time_from_start =
      durationFromSeconds(
        point_time);

    trajectory_points.push_back(
      trajectory_point);

    previous_time =
      point_time;
  }

  return result;
}


double timeAtPathDistance(
  const double distance,
  const double total_distance,
  const double acceleration,
  const double peak_velocity,
  const double acceleration_distance,
  const double acceleration_time,
  const double cruise_time,
  const bool triangular_profile)
{
  if (distance <= 0.0)
  {
    return 0.0;
  }

  if (distance >= total_distance)
  {
    return
      2.0 * acceleration_time +
      cruise_time;
  }

  if (distance < acceleration_distance)
  {
    return std::sqrt(
      2.0 * distance /
      acceleration);
  }

  if (
    !triangular_profile &&
    distance <=
    total_distance - acceleration_distance)
  {
    return
      acceleration_time +
      (
        distance -
        acceleration_distance
      ) /
      peak_velocity;
  }

  const double remaining_distance =
    total_distance - distance;

  return
    2.0 * acceleration_time +
    cruise_time -
    std::sqrt(
      2.0 * remaining_distance /
      acceleration);
}

}  // namespace ur5e_curve_path
