#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using namespace std::chrono_literals;

namespace
{

tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  rotation.normalize();

  return tf2::Transform(
    rotation,
    tf2::Vector3(
      pose.position.x,
      pose.position.y,
      pose.position.z));
}

tf2::Transform transformMsgToTransform(const geometry_msgs::msg::Transform & transform)
{
  tf2::Quaternion rotation(
    transform.rotation.x,
    transform.rotation.y,
    transform.rotation.z,
    transform.rotation.w);
  rotation.normalize();

  return tf2::Transform(
    rotation,
    tf2::Vector3(
      transform.translation.x,
      transform.translation.y,
      transform.translation.z));
}

geometry_msgs::msg::Pose transformToPose(const tf2::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();

  const tf2::Quaternion rotation = transform.getRotation();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

}  // namespace

class OpeningAlignmentExecutor
{
public:
  explicit OpeningAlignmentExecutor(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
    node_->declare_parameter<std::string>("move_group_name", "ur_manipulator");
    node_->declare_parameter<std::string>("base_frame", "base_link");
    node_->declare_parameter<std::string>("end_effector_link", "tool0");
    node_->declare_parameter<std::string>("blade_frame", "blade_center_link");
    node_->declare_parameter<std::string>("approach_topic", "/blade_approach_pose");
    node_->declare_parameter<std::string>("preinsert_topic", "/blade_preinsert_pose");
    node_->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    node_->declare_parameter<std::string>(
      "trajectory_action",
      "/joint_trajectory_controller/follow_joint_trajectory");
    node_->declare_parameter<bool>("use_direct_joint_trajectory", true);
    node_->declare_parameter<bool>("use_preferred_ik_seed", true);
    node_->declare_parameter<double>("preferred_shoulder_pan_joint", 0.0);
    node_->declare_parameter<double>("preferred_shoulder_lift_joint", -1.2217305);
    node_->declare_parameter<double>("preferred_elbow_joint", 1.8849556);
    node_->declare_parameter<double>("preferred_wrist_1_joint", -2.2165682);
    node_->declare_parameter<double>("preferred_wrist_2_joint", -1.5533430);
    node_->declare_parameter<double>("preferred_wrist_3_joint", 0.0);
    node_->declare_parameter<double>("ik_timeout_s", 0.2);
    node_->declare_parameter<double>("approach_duration_s", 6.0);
    node_->declare_parameter<double>("preinsert_duration_s", 3.0);
    node_->declare_parameter<double>("joint_state_wait_timeout_s", 5.0);
    node_->declare_parameter<double>("action_server_wait_s", 5.0);
    node_->declare_parameter<double>("planning_time_s", 10.0);
    node_->declare_parameter<int>("planning_attempts", 5);
    node_->declare_parameter<double>("velocity_scaling", 0.05);
    node_->declare_parameter<double>("acceleration_scaling", 0.05);
    node_->declare_parameter<double>("cartesian_eef_step", 0.002);
    node_->declare_parameter<double>("cartesian_jump_threshold", 0.0);
    node_->declare_parameter<double>("min_cartesian_fraction", 0.95);
    node_->declare_parameter<double>("pose_wait_timeout_s", 0.0);
    node_->declare_parameter<double>("tf_wait_timeout_s", 5.0);
    node_->declare_parameter<bool>("execute_motion", true);

    move_group_name_ = node_->get_parameter("move_group_name").as_string();
    base_frame_ = node_->get_parameter("base_frame").as_string();
    end_effector_link_ = node_->get_parameter("end_effector_link").as_string();
    blade_frame_ = node_->get_parameter("blade_frame").as_string();
    approach_topic_ = node_->get_parameter("approach_topic").as_string();
    preinsert_topic_ = node_->get_parameter("preinsert_topic").as_string();
    joint_state_topic_ = node_->get_parameter("joint_state_topic").as_string();
    trajectory_action_ = node_->get_parameter("trajectory_action").as_string();
    use_direct_joint_trajectory_ =
      node_->get_parameter("use_direct_joint_trajectory").as_bool();
    use_preferred_ik_seed_ =
      node_->get_parameter("use_preferred_ik_seed").as_bool();
    execute_motion_ = node_->get_parameter("execute_motion").as_bool();

    approach_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      approach_topic_,
      10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        approach_pose_ = *msg;
        if (!approach_pose_logged_) {
          RCLCPP_INFO(
            node_->get_logger(),
            "Received first approach pose from %s.",
            approach_topic_.c_str());
          approach_pose_logged_ = true;
        }
      });

    preinsert_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      preinsert_topic_,
      10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        preinsert_pose_ = *msg;
        if (!preinsert_pose_logged_) {
          RCLCPP_INFO(
            node_->get_logger(),
            "Received first pre-insert pose from %s.",
            preinsert_topic_.c_str());
          preinsert_pose_logged_ = true;
        }
      });

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_joint_state_ = *msg;
        if (!joint_state_logged_) {
          RCLCPP_INFO(
            node_->get_logger(),
            "Received first joint state from %s.",
            joint_state_topic_.c_str());
          joint_state_logged_ = true;
        }
      });
  }

  int run()
  {
    if (!waitForTargetPoses()) {
      return 1;
    }

    geometry_msgs::msg::TransformStamped tool_to_blade_msg;
    if (!lookupToolToBladeTransform(tool_to_blade_msg)) {
      return 1;
    }

    geometry_msgs::msg::PoseStamped approach_blade;
    geometry_msgs::msg::PoseStamped preinsert_blade;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      approach_blade = *approach_pose_;
      preinsert_blade = *preinsert_pose_;
    }

    const geometry_msgs::msg::Pose approach_tool =
      bladeTargetToToolTarget(approach_blade, tool_to_blade_msg);
    const geometry_msgs::msg::Pose preinsert_tool =
      bladeTargetToToolTarget(preinsert_blade, tool_to_blade_msg);

    RCLCPP_INFO(
      node_->get_logger(),
      "Converted blade targets to %s targets. approach=(%.4f, %.4f, %.4f), pre=(%.4f, %.4f, %.4f)",
      end_effector_link_.c_str(),
      approach_tool.position.x,
      approach_tool.position.y,
      approach_tool.position.z,
      preinsert_tool.position.x,
      preinsert_tool.position.y,
      preinsert_tool.position.z);

    MoveGroupInterface move_group(node_, move_group_name_);
    move_group.setEndEffectorLink(end_effector_link_);
    move_group.setPlanningTime(node_->get_parameter("planning_time_s").as_double());
    move_group.setNumPlanningAttempts(node_->get_parameter("planning_attempts").as_int());
    move_group.setMaxVelocityScalingFactor(node_->get_parameter("velocity_scaling").as_double());
    move_group.setMaxAccelerationScalingFactor(node_->get_parameter("acceleration_scaling").as_double());

    RCLCPP_INFO(
      node_->get_logger(),
      "MoveIt planning frame: %s, end effector link: %s, execute_motion=%s",
      move_group.getPlanningFrame().c_str(),
      move_group.getEndEffectorLink().c_str(),
      execute_motion_ ? "true" : "false");

    if (use_direct_joint_trajectory_) {
      return executeDirectJointTrajectory(move_group, approach_tool, preinsert_tool) ? 0 : 1;
    }

    if (!planAndMaybeExecutePoseTarget(move_group, approach_tool, "approach")) {
      return 1;
    }

    if (!planAndMaybeExecuteCartesianTarget(move_group, preinsert_tool, "approach-to-preinsert")) {
      return 1;
    }

    RCLCPP_INFO(node_->get_logger(), "Opening alignment execution finished at pre-insert.");
    return 0;
  }

private:
  bool waitForTargetPoses()
  {
    const double timeout_s = node_->get_parameter("pose_wait_timeout_s").as_double();
    const rclcpp::Time start = node_->now();

    RCLCPP_INFO(
      node_->get_logger(),
      "Waiting for target poses from %s and %s.",
      approach_topic_.c_str(),
      preinsert_topic_.c_str());

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (approach_pose_.has_value() && preinsert_pose_.has_value()) {
          return true;
        }
      }

      if (timeout_s > 0.0 && (node_->now() - start).seconds() > timeout_s) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Timed out waiting for target poses from %s and %s.",
          approach_topic_.c_str(),
          preinsert_topic_.c_str());
        return false;
      }

      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(50ms);
    }

    return false;
  }

  bool lookupToolToBladeTransform(geometry_msgs::msg::TransformStamped & transform)
  {
    const double timeout_s = node_->get_parameter("tf_wait_timeout_s").as_double();
    try {
      transform = tf_buffer_.lookupTransform(
        end_effector_link_,
        blade_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(timeout_s));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "TF lookup failed from %s to %s: %s",
        end_effector_link_.c_str(),
        blade_frame_.c_str(),
        ex.what());
      return false;
    }
  }

  geometry_msgs::msg::Pose bladeTargetToToolTarget(
    const geometry_msgs::msg::PoseStamped & blade_target,
    const geometry_msgs::msg::TransformStamped & tool_to_blade_msg) const
  {
    if (blade_target.header.frame_id != base_frame_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Expected target frame %s but received %s. Using pose values as-is.",
        base_frame_.c_str(),
        blade_target.header.frame_id.c_str());
    }

    const tf2::Transform base_to_blade_target = poseToTransform(blade_target.pose);
    const tf2::Transform tool_to_blade = transformMsgToTransform(tool_to_blade_msg.transform);
    const tf2::Transform base_to_tool_target =
      base_to_blade_target * tool_to_blade.inverse();

    return transformToPose(base_to_tool_target);
  }

  bool executeDirectJointTrajectory(
    MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & approach_tool,
    const geometry_msgs::msg::Pose & preinsert_tool)
  {
    moveit::core::RobotStatePtr current_state = makeCurrentRobotState(move_group);
    if (!current_state) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to build current robot state for IK.");
      return false;
    }

    const moveit::core::JointModelGroup * joint_model_group =
      current_state->getJointModelGroup(move_group_name_);
    if (!joint_model_group) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Joint model group %s was not found.",
        move_group_name_.c_str());
      return false;
    }

    const double ik_timeout_s = node_->get_parameter("ik_timeout_s").as_double();
    moveit::core::RobotState approach_state =
      makeApproachSeedState(*current_state, joint_model_group);
    if (!approach_state.setFromIK(
        joint_model_group,
        approach_tool,
        end_effector_link_,
        ik_timeout_s))
    {
      RCLCPP_ERROR(node_->get_logger(), "IK failed for approach target.");
      return false;
    }

    moveit::core::RobotState preinsert_state(approach_state);
    if (!preinsert_state.setFromIK(
        joint_model_group,
        preinsert_tool,
        end_effector_link_,
        ik_timeout_s))
    {
      RCLCPP_ERROR(node_->get_logger(), "IK failed for pre-insert target.");
      return false;
    }

    std::vector<double> current_positions;
    std::vector<double> approach_positions;
    std::vector<double> preinsert_positions;
    current_state->copyJointGroupPositions(joint_model_group, current_positions);
    approach_state.copyJointGroupPositions(joint_model_group, approach_positions);
    preinsert_state.copyJointGroupPositions(joint_model_group, preinsert_positions);

    const std::vector<std::string> joint_names =
      joint_model_group->getActiveJointModelNames();

    RCLCPP_INFO(node_->get_logger(), "IK approach joint target:");
    logJointPositions(joint_names, approach_positions);
    RCLCPP_INFO(node_->get_logger(), "IK pre-insert joint target:");
    logJointPositions(joint_names, preinsert_positions);

    if (!execute_motion_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Direct joint trajectory is ready. execute_motion=false, skipping execution.");
      return true;
    }

    auto action_client =
      rclcpp_action::create_client<FollowJointTrajectory>(node_, trajectory_action_);

    const double action_wait_s = node_->get_parameter("action_server_wait_s").as_double();
    if (!action_client->wait_for_action_server(std::chrono::duration<double>(action_wait_s))) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Trajectory action server %s is not available.",
        trajectory_action_.c_str());
      return false;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names;
    goal.trajectory.points.push_back(makeTrajectoryPoint(current_positions, 0.0));
    goal.trajectory.points.push_back(makeTrajectoryPoint(
      approach_positions,
      node_->get_parameter("approach_duration_s").as_double()));
    goal.trajectory.points.push_back(makeTrajectoryPoint(
      preinsert_positions,
      node_->get_parameter("approach_duration_s").as_double() +
        node_->get_parameter("preinsert_duration_s").as_double()));

    RCLCPP_INFO(
      node_->get_logger(),
      "Sending direct joint trajectory to %s with %zu joints and %zu points.",
      trajectory_action_.c_str(),
      goal.trajectory.joint_names.size(),
      goal.trajectory.points.size());

    auto goal_future = action_client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node_, goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to send direct joint trajectory goal.");
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Direct joint trajectory goal was rejected.");
      return false;
    }

    auto result_future = action_client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed while waiting for direct trajectory result.");
      return false;
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      wrapped_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Direct joint trajectory failed. result_code=%d, error_code=%d",
        static_cast<int>(wrapped_result.code),
        wrapped_result.result->error_code);
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Direct joint trajectory execution succeeded.");
    return true;
  }

  moveit::core::RobotState makeApproachSeedState(
    const moveit::core::RobotState & current_state,
    const moveit::core::JointModelGroup * joint_model_group) const
  {
    moveit::core::RobotState seed_state(current_state);
    if (!use_preferred_ik_seed_) {
      RCLCPP_INFO(node_->get_logger(), "Using current joint state as approach IK seed.");
      return seed_state;
    }

    const std::vector<std::pair<std::string, double>> preferred_joints = {
      {"shoulder_pan_joint", node_->get_parameter("preferred_shoulder_pan_joint").as_double()},
      {"shoulder_lift_joint", node_->get_parameter("preferred_shoulder_lift_joint").as_double()},
      {"elbow_joint", node_->get_parameter("preferred_elbow_joint").as_double()},
      {"wrist_1_joint", node_->get_parameter("preferred_wrist_1_joint").as_double()},
      {"wrist_2_joint", node_->get_parameter("preferred_wrist_2_joint").as_double()},
      {"wrist_3_joint", node_->get_parameter("preferred_wrist_3_joint").as_double()},
    };

    for (const auto & preferred_joint : preferred_joints) {
      const std::string & joint_name = preferred_joint.first;
      const double value = preferred_joint.second;
      const std::vector<std::string> & variable_names =
        seed_state.getRobotModel()->getVariableNames();
      if (std::find(variable_names.begin(), variable_names.end(), joint_name) ==
        variable_names.end())
      {
        RCLCPP_WARN(
          node_->get_logger(),
          "Preferred IK seed joint %s is not in the robot model; ignoring it.",
          joint_name.c_str());
        continue;
      }
      seed_state.setVariablePosition(joint_name, value);
    }

    seed_state.enforceBounds(joint_model_group);
    seed_state.update();
    RCLCPP_INFO(node_->get_logger(), "Using preferred curve-path joint pose as approach IK seed.");
    return seed_state;
  }

  moveit::core::RobotStatePtr makeCurrentRobotState(MoveGroupInterface & move_group)
  {
    const auto robot_model = move_group.getRobotModel();
    if (!robot_model) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt robot model is not available.");
      return nullptr;
    }

    const moveit::core::JointModelGroup * joint_model_group =
      robot_model->getJointModelGroup(move_group_name_);
    if (!joint_model_group) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Joint model group %s was not found.",
        move_group_name_.c_str());
      return nullptr;
    }

    sensor_msgs::msg::JointState joint_state;
    if (!waitForJointState(joint_state)) {
      return nullptr;
    }

    auto state = std::make_shared<moveit::core::RobotState>(robot_model);
    state->setToDefaultValues();

    for (const std::string & joint_name : joint_model_group->getActiveJointModelNames()) {
      const auto it = std::find(
        joint_state.name.begin(),
        joint_state.name.end(),
        joint_name);
      if (it == joint_state.name.end()) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Joint state from %s does not contain required joint %s.",
          joint_state_topic_.c_str(),
          joint_name.c_str());
        return nullptr;
      }

      const std::size_t index = static_cast<std::size_t>(
        std::distance(joint_state.name.begin(), it));
      if (index >= joint_state.position.size()) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Joint state position array is missing value for joint %s.",
          joint_name.c_str());
        return nullptr;
      }

      state->setVariablePosition(joint_name, joint_state.position[index]);
    }

    state->update();
    return state;
  }

  bool waitForJointState(sensor_msgs::msg::JointState & joint_state)
  {
    const double timeout_s = node_->get_parameter("joint_state_wait_timeout_s").as_double();
    const auto start = std::chrono::steady_clock::now();

    RCLCPP_INFO(node_->get_logger(), "Waiting for joint state from %s.", joint_state_topic_.c_str());

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_joint_state_.has_value() && !latest_joint_state_->name.empty()) {
          joint_state = *latest_joint_state_;
          return true;
        }
      }

      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (timeout_s > 0.0 && elapsed > timeout_s) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Timed out waiting for joint state from %s.",
          joint_state_topic_.c_str());
        return false;
      }

      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(50ms);
    }

    return false;
  }

  trajectory_msgs::msg::JointTrajectoryPoint makeTrajectoryPoint(
    const std::vector<double> & positions,
    const double time_from_start_s) const
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities.assign(positions.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(time_from_start_s);
    return point;
  }

  void logJointPositions(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions) const
  {
    for (std::size_t i = 0; i < joint_names.size() && i < positions.size(); ++i) {
      RCLCPP_INFO(
        node_->get_logger(),
        "  %s: %.6f",
        joint_names[i].c_str(),
        positions[i]);
    }
  }

  bool planAndMaybeExecutePoseTarget(
    MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & target,
    const std::string & stage)
  {
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target, end_effector_link_);

    MoveGroupInterface::Plan plan;
    const bool planned =
      static_cast<bool>(move_group.plan(plan));
    move_group.clearPoseTargets();

    if (!planned) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to plan %s motion.", stage.c_str());
      return false;
    }

    if (!execute_motion_) {
      RCLCPP_INFO(node_->get_logger(), "%s plan succeeded. execute_motion=false, skipping execution.", stage.c_str());
      return true;
    }

    const auto result = move_group.execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "%s execution failed.", stage.c_str());
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "%s execution succeeded.", stage.c_str());
    return true;
  }

  bool planAndMaybeExecuteCartesianTarget(
    MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & target,
    const std::string & stage)
  {
    move_group.setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group.computeCartesianPath(
      waypoints,
      node_->get_parameter("cartesian_eef_step").as_double(),
      node_->get_parameter("cartesian_jump_threshold").as_double(),
      trajectory);

    RCLCPP_INFO(node_->get_logger(), "%s Cartesian path fraction: %.3f", stage.c_str(), fraction);

    if (fraction < node_->get_parameter("min_cartesian_fraction").as_double()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "%s Cartesian path incomplete. Required %.3f, got %.3f.",
        stage.c_str(),
        node_->get_parameter("min_cartesian_fraction").as_double(),
        fraction);
      return false;
    }

    if (!execute_motion_) {
      RCLCPP_INFO(node_->get_logger(), "%s Cartesian plan succeeded. execute_motion=false, skipping execution.", stage.c_str());
      return true;
    }

    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const auto result = move_group.execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "%s execution failed.", stage.c_str());
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "%s execution succeeded.", stage.c_str());
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> approach_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> preinsert_pose_;
  std::optional<sensor_msgs::msg::JointState> latest_joint_state_;
  bool approach_pose_logged_{false};
  bool preinsert_pose_logged_{false};
  bool joint_state_logged_{false};
  std::string move_group_name_;
  std::string base_frame_;
  std::string end_effector_link_;
  std::string blade_frame_;
  std::string approach_topic_;
  std::string preinsert_topic_;
  std::string joint_state_topic_;
  std::string trajectory_action_;
  bool use_direct_joint_trajectory_;
  bool use_preferred_ik_seed_;
  bool execute_motion_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr approach_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr preinsert_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("opening_alignment_executor");
  OpeningAlignmentExecutor executor(node);
  const int result = executor.run();
  rclcpp::shutdown();
  return result;
}
