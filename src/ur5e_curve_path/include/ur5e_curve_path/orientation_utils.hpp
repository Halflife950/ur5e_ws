#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/logger.hpp>

#include <string>

namespace ur5e_curve_path
{

double calculateDownwardErrorRad(
  const geometry_msgs::msg::Quaternion & orientation);

void verifyDownwardOrientation(
  const geometry_msgs::msg::Pose & pose,
  const std::string & end_effector_link,
  double max_tilt_error_deg,
  const rclcpp::Logger & logger);

}  // namespace ur5e_curve_path
