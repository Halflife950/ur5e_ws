#include "ur5e_curve_path/ik_utils.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

#include "ur5e_curve_path/trajectory_utils.hpp"

namespace ur5e_curve_path
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Quaternion applyLocalYawOffset(
  const geometry_msgs::msg::Quaternion & base_orientation,
  const double yaw_offset)
{
  tf2::Quaternion base_quaternion;
  tf2::fromMsg(
    base_orientation,
    base_quaternion);
  base_quaternion.normalize();

  tf2::Quaternion yaw_quaternion;
  yaw_quaternion.setRPY(
    0.0,
    0.0,
    yaw_offset);

  tf2::Quaternion result =
    base_quaternion *
    yaw_quaternion;
  result.normalize();

  return tf2::toMsg(result);
}


double jointSpaceDistanceSquared(
  const std::vector<double> & from,
  const std::vector<double> & to)
{
  double distance_squared = 0.0;

  for (std::size_t i = 0;
       i < from.size();
       ++i)
  {
    const double delta =
      shortestAngularDifference(
        from[i],
        to[i]);

    distance_squared +=
      delta *
      delta;
  }

  return distance_squared;
}
}  // namespace


bool findBestYawReleasedIk(
  const moveit::core::JointModelGroup * joint_model_group,
  const geometry_msgs::msg::Point & target_position,
  const geometry_msgs::msg::Quaternion & base_orientation,
  const std::string & end_effector_link,
  const int yaw_sample_count,
  const double ik_timeout_s,
  const std::vector<double> & previous_joint_values,
  const moveit::core::RobotState & seed_state,
  moveit::core::RobotState & best_state,
  double & best_yaw_offset)
{
  bool found_solution = false;
  double best_score = 0.0;

  const double yaw_step =
    2.0 *
    kPi /
    static_cast<double>(yaw_sample_count);

  for (int sample_index = 0;
       sample_index < yaw_sample_count;
       ++sample_index)
  {
    int centered_index = 0;

    if (sample_index > 0)
    {
      centered_index =
        (sample_index + 1) /
        2;

      if (sample_index % 2 == 0)
      {
        centered_index =
          -centered_index;
      }
    }

    const double yaw_offset =
      static_cast<double>(centered_index) *
      yaw_step;

    geometry_msgs::msg::Pose target_pose;
    target_pose.position =
      target_position;
    target_pose.orientation =
      applyLocalYawOffset(
        base_orientation,
        yaw_offset);

    moveit::core::RobotState candidate_state =
      seed_state;

    const bool ik_found =
      candidate_state.setFromIK(
        joint_model_group,
        target_pose,
        end_effector_link,
        ik_timeout_s);

    if (!ik_found)
    {
      continue;
    }

    candidate_state.update();

    if (!candidate_state.satisfiesBounds(
        joint_model_group))
    {
      continue;
    }

    std::vector<double> candidate_joint_values;

    candidate_state.copyJointGroupPositions(
      joint_model_group,
      candidate_joint_values);

    const double score =
      jointSpaceDistanceSquared(
        previous_joint_values,
        candidate_joint_values);

    if (
      !found_solution ||
      score < best_score)
    {
      found_solution = true;
      best_score =
        score;
      best_state =
        candidate_state;
      best_yaw_offset =
        yaw_offset;
    }
  }

  return found_solution;
}

}  // namespace ur5e_curve_path
