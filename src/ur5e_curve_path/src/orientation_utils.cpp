#include "ur5e_curve_path/orientation_utils.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ur5e_curve_path
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double calculateDownwardErrorRad(
  const geometry_msgs::msg::Quaternion & orientation)
{
  tf2::Quaternion quaternion;
  tf2::fromMsg(orientation, quaternion);
  quaternion.normalize();

  const tf2::Matrix3x3 rotation_matrix(quaternion);

  // The third column is the local Z axis expressed in the base frame.
  const tf2::Vector3 tool_z_axis =
    rotation_matrix.getColumn(2);

  const tf2::Vector3 downward_axis(
    0.0,
    0.0,
    -1.0);

  const double dot_product = std::clamp(
    tool_z_axis.normalized().dot(downward_axis),
    -1.0,
    1.0);

  return std::acos(dot_product);
}


void verifyDownwardOrientation(
  const geometry_msgs::msg::Pose & pose,
  const std::string & end_effector_link,
  const double max_tilt_error_deg,
  const rclcpp::Logger & logger)
{
  constexpr double rad_to_deg =
    180.0 / kPi;

  const double downward_error_rad =
    calculateDownwardErrorRad(
      pose.orientation);

  const double downward_error_deg =
    downward_error_rad *
    rad_to_deg;

  RCLCPP_INFO(
    logger,
    "%s downward alignment error: %.4f deg",
    end_effector_link.c_str(),
    downward_error_deg);

  if (downward_error_deg > max_tilt_error_deg)
  {
    throw std::runtime_error(
      "The " +
      end_effector_link +
      " Z axis is not sufficiently aligned "
      "with the downward base_link Z direction. "
      "Measured error: " +
      std::to_string(downward_error_deg) +
      " deg. Maximum allowed error: " +
      std::to_string(max_tilt_error_deg) +
      " deg.");
  }

  RCLCPP_INFO(
    logger,
    "Flange orientation check passed.");
}

}  // namespace ur5e_curve_path
