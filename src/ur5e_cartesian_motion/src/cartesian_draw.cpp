#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <thread>
#include <chrono>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

std::vector<geometry_msgs::msg::Pose> makeline(
    const geometry_msgs::msg::Pose& start_pose)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;

  const int num_points = 50;
  const double length = 0.10;

  for (int i = 1; i <= num_points; ++i)
  {
    geometry_msgs::msg::Pose p = start_pose;
    p.position.x += length * static_cast<double>(i) / num_points;
    p.orientation = start_pose.orientation;
    waypoints.push_back(p);
  }

  return waypoints;
}

std::vector<geometry_msgs::msg::Pose> makeSquare(
    const geometry_msgs::msg::Pose& start_pose)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;

  const double side = 0.15;
  const int points_per_side = 30;

  geometry_msgs::msg::Pose p = start_pose;

  for (int i = 1; i <= points_per_side; ++i)
  {
    p = start_pose;
    p.position.x += side * static_cast<double>(i) / points_per_side;
    p.orientation = start_pose.orientation;
    waypoints.push_back(p);
  }

  for (int i = 1; i <= points_per_side; ++i)
  {
    p = start_pose;
    p.position.x += side;
    p.position.y += side * static_cast<double>(i) / points_per_side;
    p.orientation = start_pose.orientation;
    waypoints.push_back(p);
  }

  for (int i = 1; i <= points_per_side; ++i)
  {
    p = start_pose;
    p.position.x += side - side * static_cast<double>(i) / points_per_side;
    p.position.y += side;
    p.orientation = start_pose.orientation;
    waypoints.push_back(p);
  }

  for (int i = 1; i <= points_per_side; ++i)
  {
    p = start_pose;
    p.position.y += side - side * static_cast<double>(i) / points_per_side;
    p.orientation = start_pose.orientation;
    waypoints.push_back(p);
  }

  return waypoints;
}

std::vector<geometry_msgs::msg::Pose> makeCircle(
    const geometry_msgs::msg::Pose& start_pose)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;

  const double radius = 0.15;
  const int num_points = 120;

  const double center_x = start_pose.position.x + radius;
  const double center_y = start_pose.position.y;
  const double center_z = start_pose.position.z;

  for (int i = 0; i < num_points; ++i)
  {
    double angle = 2.0 * M_PI * static_cast<double>(i) / num_points;

    geometry_msgs::msg::Pose p = start_pose;
    p.position.x = center_x - radius * std::cos(angle);
    p.position.y = center_y + radius * std::sin(angle);
    p.position.z = center_z;
    p.orientation = start_pose.orientation;

    waypoints.push_back(p);
  }

  return waypoints;
}


std::map<std::string, double> makeResetJointGoal()
{
  std::map<std::string, double> reset_joint_goal;

  reset_joint_goal["shoulder_pan_joint"] = -0.38936225601918295;
  reset_joint_goal["shoulder_lift_joint"] = -1.1857826864196175;
  reset_joint_goal["elbow_joint"] = 2.3252871337063468;
  reset_joint_goal["wrist_1_joint"] = -4.2810733828378655;
  reset_joint_goal["wrist_2_joint"] = 0.3894140386120055;
  reset_joint_goal["wrist_3_joint"] = 0.0015730608588038208;

  return reset_joint_goal;
}

bool moveToResetJointState(
    MoveGroupInterface& move_group,
    const rclcpp::Logger& logger)
{
  std::map<std::string, double> reset_joint_goal = makeResetJointGoal();

  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(reset_joint_goal);

  move_group.setPlanningTime(20.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  MoveGroupInterface::Plan reset_plan;
  bool reset_success = static_cast<bool>(move_group.plan(reset_plan));

  if (!reset_success)
  {
    RCLCPP_ERROR(logger, "Failed to plan reset joint motion.");
    return false;
  }

  RCLCPP_INFO(logger, "Executing reset joint motion...");
  move_group.execute(reset_plan);

  move_group.clearPoseTargets();
  return true;
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("cartesian_draw");

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([executor]() {
    executor->spin();
  });

  node->declare_parameter<std::string>("mode", "line");
  std::string mode = node->get_parameter("mode").as_string();

  RCLCPP_INFO(node->get_logger(), "Selected mode: %s", mode.c_str());

  MoveGroupInterface move_group(node, "ur_manipulator");

  move_group.setPlanningTime(20.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setMaxVelocityScalingFactor(0.10);
  move_group.setMaxAccelerationScalingFactor(0.10);

  rclcpp::sleep_for(std::chrono::seconds(2));

  bool reset_success = moveToResetJointState(move_group, node->get_logger());

  if (!reset_success)
  {
    rclcpp::shutdown();

    if (spin_thread.joinable())
    {
      spin_thread.join();
    }

    return 1;
  }

  rclcpp::sleep_for(std::chrono::seconds(2));

  move_group.setStartStateToCurrentState();

  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
  geometry_msgs::msg::Quaternion fixed_orientation = start_pose.orientation;

  RCLCPP_INFO(
      node->get_logger(),
      "Reset pose position: x=%.6f, y=%.6f, z=%.6f",
      start_pose.position.x,
      start_pose.position.y,
      start_pose.position.z);

  RCLCPP_INFO(
      node->get_logger(),
      "Fixed orientation after reset: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
      fixed_orientation.x,
      fixed_orientation.y,
      fixed_orientation.z,
      fixed_orientation.w);

  start_pose.orientation = fixed_orientation;

  std::vector<geometry_msgs::msg::Pose> waypoints;

  if (mode == "line")
  {
    waypoints = makeline(start_pose);
  }
  else if (mode == "square")
  {
    waypoints = makeSquare(start_pose);
  }
  else if (mode == "circle")
  {
    waypoints = makeCircle(start_pose);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid mode. Please choose 'line', 'square', or 'circle'.");

    rclcpp::shutdown();

    if (spin_thread.joinable())
    {
      spin_thread.join();
    }

    return 1;
  }

  moveit_msgs::msg::RobotTrajectory trajectory;

  const double eef_step = 0.005;
  const double jump_threshold = 0.0;

  double fraction = move_group.computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory);

  RCLCPP_INFO(node->get_logger(), "Cartesian path fraction: %.3f", fraction);

  if (fraction < 0.95)
  {
    RCLCPP_WARN(
        node->get_logger(),
        "Path planning incomplete. Only %.3f of the path was planned.",
        fraction);

    rclcpp::shutdown();

    if (spin_thread.joinable())
    {
      spin_thread.join();
    }

    return 1;
  }

  MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;

  RCLCPP_INFO(node->get_logger(), "Executing trajectory...");
  move_group.execute(plan);

  rclcpp::shutdown();

  if (spin_thread.joinable())
  {
    spin_thread.join();
  }

  return 0;
}
