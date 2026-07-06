#include "ur5e_curve_path/config_utils.hpp"

#include <cmath>
#include <stdexcept>

namespace ur5e_curve_path
{
namespace
{
double getDoubleParameter(
  rclcpp::Node & node,
  const std::string & name,
  const double default_value)
{
  if (!node.has_parameter(name))
  {
    return node.declare_parameter<double>(
      name,
      default_value);
  }

  return node.get_parameter(name).as_double();
}


int getIntParameter(
  rclcpp::Node & node,
  const std::string & name,
  const int default_value)
{
  if (!node.has_parameter(name))
  {
    return node.declare_parameter<int>(
      name,
      default_value);
  }

  return node.get_parameter(name).as_int();
}


std::string getStringParameter(
  rclcpp::Node & node,
  const std::string & name,
  const std::string & default_value)
{
  if (!node.has_parameter(name))
  {
    return node.declare_parameter<std::string>(
      name,
      default_value);
  }

  const rclcpp::Parameter parameter =
    node.get_parameter(name);

  if (
    parameter.get_type() ==
    rclcpp::ParameterType::PARAMETER_STRING)
  {
    return parameter.as_string();
  }

  /*
   * ROS 2 launch parameters pass through YAML parsing.  In YAML 1.1,
   * bare "y" and "n" can be treated as booleans, which is awkward for
   * CSV axis names.  Convert that ambiguity back to axis text here.
   */
  if (
    parameter.get_type() ==
    rclcpp::ParameterType::PARAMETER_BOOL)
  {
    return parameter.as_bool() ?
      "y" :
      "n";
  }

  throw std::runtime_error(
    "Parameter " +
    name +
    " must be a string.");
}


void requirePositive(
  const double value,
  const std::string & parameter_name)
{
  if (!std::isfinite(value) || value <= 0.0)
  {
    throw std::runtime_error(
      "Parameter " +
      parameter_name +
      " must be a positive finite value.");
  }
}


void requireScalingFactor(
  const double value,
  const std::string & parameter_name)
{
  if (!std::isfinite(value) || value <= 0.0 || value > 1.0)
  {
    throw std::runtime_error(
      "Parameter " +
      parameter_name +
      " must be in the range (0, 1].");
  }
}


void validateConfig(
  const CurvePathConfig & config)
{
  if (config.move_group_name.empty())
  {
    throw std::runtime_error(
      "Parameter move_group_name must not be empty.");
  }

  if (config.end_effector_link.empty())
  {
    throw std::runtime_error(
      "Parameter end_effector_link must not be empty.");
  }

  if (config.path_csv.empty())
  {
    throw std::runtime_error(
      "Parameter path_csv must not be empty.");
  }

  requirePositive(
    config.planning_time_s,
    "planning_time_s");

  if (config.planning_attempts <= 0)
  {
    throw std::runtime_error(
      "Parameter planning_attempts must be positive.");
  }

  requireScalingFactor(
    config.velocity_scaling,
    "velocity_scaling");

  requireScalingFactor(
    config.acceleration_scaling,
    "acceleration_scaling");

  requirePositive(
    config.max_gap_mm,
    "max_gap_mm");

  requirePositive(
    config.max_tilt_error_deg,
    "max_tilt_error_deg");

  requirePositive(
    config.ik_timeout_s,
    "ik_timeout_s");

  if (config.yaw_sample_count <= 0)
  {
    throw std::runtime_error(
      "Parameter yaw_sample_count must be positive.");
  }

  requirePositive(
    config.max_joint_step_rad,
    "max_joint_step_rad");

  requirePositive(
    config.cartesian_speed_mps,
    "cartesian_speed_mps");

  requirePositive(
    config.cartesian_acceleration_mps2,
    "cartesian_acceleration_mps2");

  requirePositive(
    config.joint_velocity_limit_radps,
    "joint_velocity_limit_radps");

  requirePositive(
    config.joint_acceleration_limit_radps2,
    "joint_acceleration_limit_radps2");

  if (config.loop_pause_ms < 0)
  {
    throw std::runtime_error(
      "Parameter loop_pause_ms must not be negative.");
  }
}
}  // namespace


CurvePathConfig loadCurvePathConfig(
  rclcpp::Node & node)
{
  CurvePathConfig config;

  config.move_group_name =
    getStringParameter(
      node,
      "move_group_name",
      config.move_group_name);

  config.end_effector_link =
    getStringParameter(
      node,
      "end_effector_link",
      config.end_effector_link);

  config.path_csv =
    getStringParameter(
      node,
      "path_csv",
      config.path_csv);

  config.axis_mapping.base_x_from_csv_axis =
    parseCsvAxis(
      getStringParameter(
        node,
        "base_x_from_csv_axis",
        csvAxisName(
          config.axis_mapping.base_x_from_csv_axis)),
      false,
      "base_x_from_csv_axis");

  config.axis_mapping.base_y_from_csv_axis =
    parseCsvAxis(
      getStringParameter(
        node,
        "base_y_from_csv_axis",
        csvAxisName(
          config.axis_mapping.base_y_from_csv_axis)),
      false,
      "base_y_from_csv_axis");

  config.axis_mapping.base_z_from_csv_axis =
    parseCsvAxis(
      getStringParameter(
        node,
        "base_z_from_csv_axis",
        csvAxisName(
          config.axis_mapping.base_z_from_csv_axis)),
      true,
      "base_z_from_csv_axis");

  config.marker_topic =
    getStringParameter(
      node,
      "marker_topic",
      config.marker_topic);

  config.marker_frame =
    getStringParameter(
      node,
      "marker_frame",
      config.marker_frame);

  config.marker_namespace =
    getStringParameter(
      node,
      "marker_namespace",
      config.marker_namespace);

  config.planning_time_s =
    getDoubleParameter(
      node,
      "planning_time_s",
      config.planning_time_s);

  config.planning_attempts =
    getIntParameter(
      node,
      "planning_attempts",
      config.planning_attempts);

  config.velocity_scaling =
    getDoubleParameter(
      node,
      "velocity_scaling",
      config.velocity_scaling);

  config.acceleration_scaling =
    getDoubleParameter(
      node,
      "acceleration_scaling",
      config.acceleration_scaling);

  config.max_gap_mm =
    getDoubleParameter(
      node,
      "max_gap_mm",
      config.max_gap_mm);

  config.max_tilt_error_deg =
    getDoubleParameter(
      node,
      "max_tilt_error_deg",
      config.max_tilt_error_deg);

  config.ik_timeout_s =
    getDoubleParameter(
      node,
      "ik_timeout_s",
      config.ik_timeout_s);

  config.yaw_sample_count =
    getIntParameter(
      node,
      "yaw_sample_count",
      config.yaw_sample_count);

  config.max_joint_step_rad =
    getDoubleParameter(
      node,
      "max_joint_step_rad",
      config.max_joint_step_rad);

  config.cartesian_speed_mps =
    getDoubleParameter(
      node,
      "cartesian_speed_mps",
      config.cartesian_speed_mps);

  config.cartesian_acceleration_mps2 =
    getDoubleParameter(
      node,
      "cartesian_acceleration_mps2",
      config.cartesian_acceleration_mps2);

  config.joint_velocity_limit_radps =
    getDoubleParameter(
      node,
      "joint_velocity_limit_radps",
      config.joint_velocity_limit_radps);

  config.joint_acceleration_limit_radps2 =
    getDoubleParameter(
      node,
      "joint_acceleration_limit_radps2",
      config.joint_acceleration_limit_radps2);

  config.loop_pause_ms =
    getIntParameter(
      node,
      "loop_pause_ms",
      config.loop_pause_ms);

  validateConfig(config);

  return config;
}

}  // namespace ur5e_curve_path
