#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/robot_state/robot_state.h>

#include <string>
#include <vector>

namespace ur5e_curve_path
{

bool findBestYawReleasedIk(
  const moveit::core::JointModelGroup * joint_model_group,
  const geometry_msgs::msg::Point & target_position,
  const geometry_msgs::msg::Quaternion & base_orientation,
  const std::string & end_effector_link,
  int yaw_sample_count,
  double ik_timeout_s,
  const std::vector<double> & previous_joint_values,
  const moveit::core::RobotState & seed_state,
  moveit::core::RobotState & best_state,
  double & best_yaw_offset);

}  // namespace ur5e_curve_path
