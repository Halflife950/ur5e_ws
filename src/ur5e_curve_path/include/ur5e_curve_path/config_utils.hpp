#pragma once

#include <rclcpp/node.hpp>

#include <string>

#include "ur5e_curve_path/path_utils.hpp"

namespace ur5e_curve_path
{

struct CurvePathConfig
{
  std::string move_group_name{"ur_manipulator"};
  std::string end_effector_link{"tool0"};
  std::string path_csv{"paths/heart_path.csv"};
  PathAxisMapping axis_mapping;
  std::string marker_topic{"/heart_path_marker"};
  std::string marker_frame{"base_link"};
  std::string marker_namespace{"heart_path"};
  double planning_time_s{10.0};
  int planning_attempts{10};
  double velocity_scaling{0.15};
  double acceleration_scaling{0.15};
  double max_gap_mm{10.0};
  double max_tilt_error_deg{2.0};
  double ik_timeout_s{0.05};
  int yaw_sample_count{37};
  double max_joint_step_rad{0.35};
  double cartesian_speed_mps{0.08};
  double cartesian_acceleration_mps2{0.16};
  double joint_velocity_limit_radps{2.0};
  double joint_acceleration_limit_radps2{10.0};
  int loop_pause_ms{200};
};

CurvePathConfig loadCurvePathConfig(
  rclcpp::Node & node);

}  // namespace ur5e_curve_path
