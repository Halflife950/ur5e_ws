#include <memory>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "ur5e_obstacle_motion",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {
    executor.spin();
  });

  using moveit::planning_interface::MoveGroupInterface;
  using moveit::planning_interface::PlanningSceneInterface;

  MoveGroupInterface move_group(node, "ur_manipulator");
  PlanningSceneInterface planning_scene_interface;

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // 1. create a collision object and add it to the world
  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = move_group.getPlanningFrame();
  box.id = "box_obstacle";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.10;  // x length
  primitive.dimensions[1] = 0.10;  // y length
  primitive.dimensions[2] = 0.10;  // z length

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.85;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.1;

  box.primitives.push_back(primitive);
  box.primitive_poses.push_back(box_pose);
  box.operation = box.ADD;

  planning_scene_interface.applyCollisionObject(box);

  RCLCPP_INFO(node->get_logger(), "Added collision object");

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 2. set target pose
  geometry_msgs::msg::Pose target_pose;

  target_pose.position.x = 0.55;
  target_pose.position.y = 0.4;
  target_pose.position.z = 0.33;

  double roll = 0.0;
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

  // 3. plan the trajectory
  MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Planning succeeded, executing...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed");
  }

  rclcpp::shutdown();
  spinner.join();

  return 0;
}
//colcon build --packages-select ur5e_obstacle_motion
//ros2 run ur5e_obstacle_motion obstacle_motion
