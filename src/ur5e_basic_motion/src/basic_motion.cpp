#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "ur5e_basic_motion",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {
    executor.spin();
  });

  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

  MoveGroupInterface move_group(node, "ur_manipulator");

  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  geometry_msgs::msg::Pose target_pose;

  target_pose.position.x = 0.35;
  target_pose.position.y = 0.4;
  target_pose.position.z = 0.6;

  double roll = 1.57;
  double pitch = 0.0;
  double yaw = 0.0;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();

  move_group.setPoseTarget(target_pose);

  MoveGroupInterface::Plan plan;

  bool success = static_cast<bool>(move_group.plan(plan));

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Planning succeeded. Executing...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
  }

  rclcpp::shutdown();
  spinner.join();

  return 0;
}