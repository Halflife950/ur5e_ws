#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "ur5e_curve_path/config_utils.hpp"
#include "ur5e_curve_path/path_utils.hpp"

namespace ur5e_curve_path
{

class CurvePathNode : public rclcpp::Node
{
public:
  CurvePathNode();

  void initialize();

private:
  void configureMoveGroupForPlanning();

  std::string resolveCsvPath() const;

  void loadAndValidatePath();

  void calculatePathBounds();

  void moveToStartJointPose();

  void validatePathIk();

  double pathPointDistanceMeters(
    std::size_t first_index,
    std::size_t second_index) const;

  void buildTimedTrajectory();

  void enforceTimedTrajectoryLimits();

  void validateTimedTrajectory();

  void executeTimedTrajectoryLoop();

  void publishPathMarker();

  std::unique_ptr<
    moveit::planning_interface::MoveGroupInterface>
  move_group_;

  CurvePathConfig config_;

  rclcpp::Publisher<
    visualization_msgs::msg::Marker>::SharedPtr
  marker_publisher_;

  rclcpp::TimerBase::SharedPtr marker_timer_;

  std::vector<PathPoint> raw_points_;

  geometry_msgs::msg::Pose path_origin_;

  std::vector<std::string> joint_names_;

  std::vector<std::vector<double>>
    joint_solutions_;

  moveit_msgs::msg::RobotTrajectory
    timed_trajectory_;

  bool path_ready_{false};
};

}  // namespace ur5e_curve_path
