#include <chrono>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

namespace
{

struct Vec2
{
  double x;
  double y;
};

std::string trim(const std::string & text)
{
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

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

class OpeningInsertExecutor
{
public:
  explicit OpeningInsertExecutor(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
    node_->declare_parameter<std::string>("move_group_name", "ur_manipulator");
    node_->declare_parameter<std::string>("base_frame", "base_link");
    node_->declare_parameter<std::string>("end_effector_link", "tool0");
    node_->declare_parameter<std::string>("blade_frame", "blade_center_link");
    node_->declare_parameter<std::string>("preinsert_topic", "/blade_preinsert_pose");
    node_->declare_parameter<std::string>("insert_topic", "/blade_insert_pose");
    node_->declare_parameter<std::string>("safety_topic", "/blade_contour_safe");
    node_->declare_parameter<std::string>("min_distance_topic", "/blade_contour_min_distance");
    node_->declare_parameter<std::string>("contour_csv", "data/cavity_contour.csv");
    node_->declare_parameter<double>("csv_scale", 0.001);
    node_->declare_parameter<bool>("reject_preinsert_plan_inside_contour", true);
    node_->declare_parameter<int>("valid_preinsert_plan_attempts", 5);
    node_->declare_parameter<double>("approach_extra_offset", 0.08);
    node_->declare_parameter<double>("planning_time_s", 10.0);
    node_->declare_parameter<int>("planning_attempts", 10);
    node_->declare_parameter<double>("velocity_scaling", 0.05);
    node_->declare_parameter<double>("acceleration_scaling", 0.05);
    node_->declare_parameter<double>("preinsert_ik_timeout_s", 0.2);
    node_->declare_parameter<bool>("use_preferred_ik_seed", true);
    node_->declare_parameter<double>("preferred_shoulder_pan_joint", 0.0);
    node_->declare_parameter<double>("preferred_shoulder_lift_joint", -1.2217305);
    node_->declare_parameter<double>("preferred_elbow_joint", 1.8849556);
    node_->declare_parameter<double>("preferred_wrist_1_joint", -2.2165682);
    node_->declare_parameter<double>("preferred_wrist_2_joint", -1.5533430);
    node_->declare_parameter<double>("preferred_wrist_3_joint", 0.0);
    node_->declare_parameter<double>("cartesian_eef_step", 0.002);
    node_->declare_parameter<double>("cartesian_jump_threshold", 0.0);
    node_->declare_parameter<double>("min_cartesian_fraction", 0.95);
    node_->declare_parameter<double>("pose_wait_timeout_s", 0.0);
    node_->declare_parameter<double>("tf_wait_timeout_s", 5.0);
    node_->declare_parameter<bool>("execute_motion", true);
    node_->declare_parameter<bool>("execute_insert_stage", false);
    node_->declare_parameter<bool>("require_safety_status_before_motion", true);
    node_->declare_parameter<bool>("require_safe_before_approach", true);
    node_->declare_parameter<bool>("require_safe_before_preinsert", true);
    node_->declare_parameter<bool>("require_safe_before_insert", true);
    node_->declare_parameter<bool>("abort_on_unsafe", true);
    node_->declare_parameter<bool>("stop_on_unsafe", true);
    node_->declare_parameter<double>("safety_wait_timeout_s", 3.0);
    node_->declare_parameter<bool>("segment_check_before_execute", true);
    node_->declare_parameter<bool>("segment_check_after_execute", true);
    node_->declare_parameter<bool>("segmented_cartesian_insert", false);
    node_->declare_parameter<double>("insert_segment_length", 0.002);
    node_->declare_parameter<bool>("allow_continue_after_unsafe", false);

    move_group_name_ = node_->get_parameter("move_group_name").as_string();
    base_frame_ = node_->get_parameter("base_frame").as_string();
    end_effector_link_ = node_->get_parameter("end_effector_link").as_string();
    blade_frame_ = node_->get_parameter("blade_frame").as_string();
    preinsert_topic_ = node_->get_parameter("preinsert_topic").as_string();
    insert_topic_ = node_->get_parameter("insert_topic").as_string();
    safety_topic_ = node_->get_parameter("safety_topic").as_string();
    min_distance_topic_ = node_->get_parameter("min_distance_topic").as_string();
    execute_motion_ = node_->get_parameter("execute_motion").as_bool();
    reject_preinsert_plan_inside_contour_ =
      node_->get_parameter("reject_preinsert_plan_inside_contour").as_bool();
    contour_polygon_ = loadContourPolygon(
      resolvePackagePath(node_->get_parameter("contour_csv").as_string()),
      node_->get_parameter("csv_scale").as_double());

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

    insert_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      insert_topic_,
      10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        insert_pose_ = *msg;
        if (!insert_pose_logged_) {
          RCLCPP_INFO(
            node_->get_logger(),
            "Received first insert pose from %s.",
            insert_topic_.c_str());
          insert_pose_logged_ = true;
        }
      });

    safety_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      safety_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        MoveGroupInterface * move_group_to_stop = nullptr;
        double min_distance = std::numeric_limits<double>::quiet_NaN();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          has_safety_status_ = true;
          latest_safe_ = msg->data;
          min_distance = latest_min_distance_;

          if (!latest_safe_ && last_safe_) {
            RCLCPP_WARN(
              node_->get_logger(),
              "UNSAFE received by executor, min_distance=%.4f m",
              min_distance);
          }

          if (!latest_safe_ && node_->get_parameter("stop_on_unsafe").as_bool()) {
            move_group_to_stop = active_move_group_;
          }

          last_safe_ = latest_safe_;
        }

        if (move_group_to_stop) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Stopping active motion because %s reported unsafe, min_distance=%.4f m.",
            safety_topic_.c_str(),
            min_distance);
          move_group_to_stop->stop();
        }
      });

    min_distance_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      min_distance_topic_,
      10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_min_distance_ = msg->data;
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "Loaded %zu contour points for pre-insert forbidden-region checks.",
      contour_polygon_.size());
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

    geometry_msgs::msg::PoseStamped preinsert_blade;
    geometry_msgs::msg::PoseStamped insert_blade;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      preinsert_blade = *preinsert_pose_;
      insert_blade = *insert_pose_;
    }

    const geometry_msgs::msg::Pose preinsert_tool =
      bladeTargetToToolTarget(preinsert_blade, tool_to_blade_msg);
    const geometry_msgs::msg::PoseStamped approach_blade =
      makeApproachBladePose(preinsert_blade, insert_blade);
    const geometry_msgs::msg::Pose approach_tool =
      bladeTargetToToolTarget(approach_blade, tool_to_blade_msg);
    const geometry_msgs::msg::Pose insert_tool =
      bladeTargetToToolTarget(insert_blade, tool_to_blade_msg);

    RCLCPP_INFO(
      node_->get_logger(),
      "Converted blade targets to %s targets. approach=(%.4f, %.4f, %.4f), pre=(%.4f, %.4f, %.4f), insert=(%.4f, %.4f, %.4f)",
      end_effector_link_.c_str(),
      approach_tool.position.x,
      approach_tool.position.y,
      approach_tool.position.z,
      preinsert_tool.position.x,
      preinsert_tool.position.y,
      preinsert_tool.position.z,
      insert_tool.position.x,
      insert_tool.position.y,
      insert_tool.position.z);

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

    if (node_->get_parameter("require_safety_status_before_motion").as_bool() &&
      !waitForSafetyStatus(node_->get_parameter("safety_wait_timeout_s").as_double()))
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "No safety status received from %s, aborting motion.",
        safety_topic_.c_str());
      return 1;
    }

    if (node_->get_parameter("require_safe_before_approach").as_bool() &&
      !checkSafetyOrAbort("before approach", move_group))
    {
      return 1;
    }

    if (!planAndMaybeExecutePreinsertJointTarget(move_group, approach_tool)) {
      return 1;
    }

    if (node_->get_parameter("require_safe_before_preinsert").as_bool() &&
      !checkSafetyOrAbort("before approach-to-preinsert", move_group))
    {
      return 1;
    }

    if (!planAndMaybeExecuteApproachToPreinsert(move_group, preinsert_tool)) {
      return 1;
    }

    if (node_->get_parameter("execute_insert_stage").as_bool()) {
      if (node_->get_parameter("require_safe_before_insert").as_bool() &&
        !checkSafetyOrAbort("before insert", move_group))
      {
        return 1;
      }

      if (!planAndMaybeExecuteInsert(move_group, preinsert_tool, insert_tool)) {
        return 1;
      }
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "Reached pre-insert pose. execute_insert_stage=false, leaving insert motion to an external controller.");
    }

    RCLCPP_INFO(node_->get_logger(), "Opening insert sequence finished.");
    return 0;
  }

private:
  bool waitForSafetyStatus(const double timeout_s)
  {
    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::duration<double>(timeout_s);
    auto last_log_time = start_time - 2s;

    rclcpp::WallRate rate(50.0);
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_safety_status_) {
          RCLCPP_INFO(
            node_->get_logger(),
            "Received safety status from %s: safe=%s min_distance=%.4f m.",
            safety_topic_.c_str(),
            latest_safe_ ? "true" : "false",
            latest_min_distance_);
          return true;
        }
      }

      const auto now = std::chrono::steady_clock::now();
      if (timeout_s > 0.0 && now - start_time >= timeout) {
        break;
      }

      if (now - last_log_time >= 1s) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Waiting for safety status from %s.",
          safety_topic_.c_str());
        last_log_time = now;
      }
      rate.sleep();
    }

    return false;
  }

  bool checkSafetyOrAbort(const std::string & stage, MoveGroupInterface & move_group)
  {
    bool has_status = false;
    bool safe = false;
    double min_distance = std::numeric_limits<double>::quiet_NaN();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_status = has_safety_status_;
      safe = latest_safe_;
      min_distance = latest_min_distance_;
    }

    if (!has_status) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[%s] no safety status received from %s.",
        stage.c_str(),
        safety_topic_.c_str());
      if (node_->get_parameter("stop_on_unsafe").as_bool()) {
        move_group.stop();
      }
      return !node_->get_parameter("abort_on_unsafe").as_bool();
    }

    if (!safe) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[%s] safety check failed, min_distance=%.4f m.",
        stage.c_str(),
        min_distance);
      if (node_->get_parameter("stop_on_unsafe").as_bool()) {
        move_group.stop();
      }
      return node_->get_parameter("allow_continue_after_unsafe").as_bool() ||
        !node_->get_parameter("abort_on_unsafe").as_bool();
    }

    return true;
  }

  void setActiveMoveGroup(MoveGroupInterface * move_group)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_move_group_ = move_group;
  }

  void clearActiveMoveGroup()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_move_group_ = nullptr;
  }

  bool executePlanWithSafety(
    MoveGroupInterface & move_group,
    const MoveGroupInterface::Plan & plan,
    const std::string & stage)
  {
    if (node_->get_parameter("segment_check_before_execute").as_bool() &&
      !checkSafetyOrAbort(stage + " before execute", move_group))
    {
      return false;
    }

    if (!execute_motion_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "execute_motion=false, skipping %s execution.",
        stage.c_str());
    } else {
      RCLCPP_INFO(node_->get_logger(), "Executing %s...", stage.c_str());
      setActiveMoveGroup(&move_group);
      const auto execute_result = move_group.execute(plan);
      clearActiveMoveGroup();
      if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "%s execution failed or timed out.", stage.c_str());
        return false;
      }
    }

    if (node_->get_parameter("segment_check_after_execute").as_bool() &&
      !checkSafetyOrAbort(stage + " after execute", move_group))
    {
      return false;
    }

    return true;
  }

  bool waitForTargetPoses()
  {
    const double timeout_s = node_->get_parameter("pose_wait_timeout_s").as_double();
    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::duration<double>(timeout_s);
    auto last_log_time = start_time - 2s;

    rclcpp::WallRate rate(20.0);
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (preinsert_pose_.has_value() && insert_pose_.has_value()) {
          RCLCPP_INFO(
            node_->get_logger(),
            "Received target poses from %s and %s.",
            preinsert_topic_.c_str(),
            insert_topic_.c_str());
          return true;
        }
      }

      const auto now = std::chrono::steady_clock::now();
      if (timeout_s > 0.0 && now - start_time >= timeout) {
        break;
      }

      if (now - last_log_time >= 1s) {
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_INFO(
          node_->get_logger(),
          "Waiting for target poses: preinsert=%s insert=%s.",
          preinsert_pose_.has_value() ? "received" : "missing",
          insert_pose_.has_value() ? "received" : "missing");
        last_log_time = now;
      }
      rate.sleep();
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "Timed out waiting for %s and %s. Is opening_alignment_planner running?",
      preinsert_topic_.c_str(),
      insert_topic_.c_str());
    return false;
  }

  bool lookupToolToBladeTransform(geometry_msgs::msg::TransformStamped & transform)
  {
    const auto timeout = tf2::durationFromSec(
      node_->get_parameter("tf_wait_timeout_s").as_double());

    try {
      transform = tf_buffer_.lookupTransform(
        end_effector_link_,
        blade_frame_,
        tf2::TimePointZero,
        timeout);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to lookup fixed transform %s -> %s: %s",
        end_effector_link_.c_str(),
        blade_frame_.c_str(),
        ex.what());
      return false;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Fixed transform %s -> %s: xyz=(%.4f, %.4f, %.4f), q=(%.4f, %.4f, %.4f, %.4f)",
      end_effector_link_.c_str(),
      blade_frame_.c_str(),
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z,
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);
    return true;
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

  geometry_msgs::msg::PoseStamped makeApproachBladePose(
    const geometry_msgs::msg::PoseStamped & preinsert_blade,
    const geometry_msgs::msg::PoseStamped & insert_blade) const
  {
    const double dx = preinsert_blade.pose.position.x - insert_blade.pose.position.x;
    const double dy = preinsert_blade.pose.position.y - insert_blade.pose.position.y;
    const double length = std::hypot(dx, dy);
    if (length < 1.0e-9) {
      throw std::runtime_error("Cannot compute approach direction: pre-insert and insert XY overlap.");
    }

    geometry_msgs::msg::PoseStamped approach = preinsert_blade;
    const double extra_offset = node_->get_parameter("approach_extra_offset").as_double();
    approach.pose.position.x += extra_offset * dx / length;
    approach.pose.position.y += extra_offset * dy / length;
    return approach;
  }

  std::string resolvePackagePath(const std::string & path) const
  {
    if (path.empty() || path.front() == '/') {
      return path;
    }

    const std::string package_share =
      ament_index_cpp::get_package_share_directory("ur5e_blade_safety");
    return package_share + "/" + path;
  }

  static bool parseCsvPoint(const std::string & line, Vec2 & point)
  {
    std::stringstream stream(line);
    std::string item;
    std::vector<double> values;

    while (std::getline(stream, item, ',')) {
      try {
        values.push_back(std::stod(trim(item)));
      } catch (const std::exception &) {
        return false;
      }
    }

    if (values.size() < 2) {
      return false;
    }

    point.x = values[0];
    point.y = values[1];
    return true;
  }

  static std::vector<Vec2> loadContourPolygon(const std::string & path, const double scale)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open contour CSV: " + path);
    }

    std::vector<Vec2> points;
    std::string line;
    while (std::getline(file, line)) {
      if (trim(line).empty()) {
        continue;
      }

      Vec2 point{};
      if (!parseCsvPoint(line, point)) {
        continue;
      }

      points.push_back(Vec2{point.x * scale, point.y * scale});
    }

    if (points.size() < 3) {
      throw std::runtime_error("Contour CSV must contain at least three numeric points.");
    }

    return points;
  }

  static bool pointOnSegment2d(const Vec2 & p, const Vec2 & a, const Vec2 & b)
  {
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double apx = p.x - a.x;
    const double apy = p.y - a.y;
    const double cross = abx * apy - aby * apx;
    if (std::abs(cross) > 1.0e-9) {
      return false;
    }

    const double dot = apx * abx + apy * aby;
    if (dot < -1.0e-9) {
      return false;
    }

    const double ab_len2 = abx * abx + aby * aby;
    return dot <= ab_len2 + 1.0e-9;
  }

  static bool pointInsidePolygon2d(const Vec2 & point, const std::vector<Vec2> & polygon)
  {
    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
      const Vec2 & a = polygon[j];
      const Vec2 & b = polygon[i];

      if (pointOnSegment2d(point, a, b)) {
        return true;
      }

      const bool crosses = ((a.y > point.y) != (b.y > point.y)) &&
        (point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x);
      if (crosses) {
        inside = !inside;
      }
    }

    return inside;
  }

  bool validatePreinsertPlanKeepsBladeCenterOutsideContour(
    const MoveGroupInterface::Plan & plan,
    const moveit::core::RobotState & reference_state,
    const moveit::core::RobotModelConstPtr & robot_model) const
  {
    if (!reject_preinsert_plan_inside_contour_) {
      return true;
    }

    const moveit::core::LinkModel * blade_link = robot_model->getLinkModel(blade_frame_);
    if (!blade_link) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Blade frame %s was not found in the MoveIt robot model.",
        blade_frame_.c_str());
      return false;
    }

    const auto & joint_trajectory = plan.trajectory_.joint_trajectory;
    if (joint_trajectory.points.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Pre-insert plan has no trajectory points.");
      return false;
    }

    moveit::core::RobotState state(reference_state);
    for (std::size_t point_index = 0; point_index < joint_trajectory.points.size(); ++point_index) {
      const auto & point = joint_trajectory.points[point_index];
      if (point.positions.size() != joint_trajectory.joint_names.size()) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Pre-insert trajectory point %zu has %zu positions for %zu joints.",
          point_index,
          point.positions.size(),
          joint_trajectory.joint_names.size());
        return false;
      }

      for (std::size_t joint_index = 0; joint_index < joint_trajectory.joint_names.size();
        ++joint_index)
      {
        state.setVariablePosition(
          joint_trajectory.joint_names[joint_index],
          point.positions[joint_index]);
      }
      state.update();

      const auto & blade_tf = state.getGlobalLinkTransform(blade_link);
      const Vec2 blade_center{blade_tf.translation().x(), blade_tf.translation().y()};
      if (pointInsidePolygon2d(blade_center, contour_polygon_)) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Rejected pre-insert plan: %s enters contour prism at trajectory point %zu, xy=(%.4f, %.4f).",
          blade_frame_.c_str(),
          point_index,
          blade_center.x,
          blade_center.y);
        return false;
      }
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Pre-insert plan accepted: %s stayed outside the contour prism for %zu trajectory points.",
      blade_frame_.c_str(),
      joint_trajectory.points.size());
    return true;
  }

  bool planAndMaybeExecutePreinsertJointTarget(
    MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & preinsert_tool)
  {
    const moveit::core::RobotModelConstPtr robot_model = move_group.getRobotModel();
    if (!robot_model) {
      RCLCPP_ERROR(node_->get_logger(), "Unable to get MoveIt robot model.");
      return false;
    }

    const moveit::core::JointModelGroup * joint_model_group =
      robot_model->getJointModelGroup(move_group_name_);
    if (!joint_model_group) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Joint model group %s was not found.",
        move_group_name_.c_str());
      return false;
    }

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2.0);
    if (!current_state) {
      RCLCPP_ERROR(node_->get_logger(), "Unable to get current robot state for IK seed.");
      return false;
    }

    moveit::core::RobotState ik_state = makeIkSeedState(*current_state, joint_model_group);
    const bool ik_found = ik_state.setFromIK(
      joint_model_group,
      preinsert_tool,
      end_effector_link_,
      node_->get_parameter("preinsert_ik_timeout_s").as_double());

    if (!ik_found) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Preferred-seed IK failed for pre-insert %s target. Retrying from current state.",
        end_effector_link_.c_str());
      ik_state = *current_state;
      const bool fallback_ik_found = ik_state.setFromIK(
        joint_model_group,
        preinsert_tool,
        end_effector_link_,
        node_->get_parameter("preinsert_ik_timeout_s").as_double());

      if (!fallback_ik_found) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "IK failed for pre-insert %s target.",
          end_effector_link_.c_str());
        return false;
      }
    }

    ik_state.update();
    if (!ik_state.satisfiesBounds(joint_model_group)) {
      RCLCPP_ERROR(node_->get_logger(), "Pre-insert IK solution violates joint bounds.");
      return false;
    }

    std::vector<double> joint_values;
    ik_state.copyJointGroupPositions(joint_model_group, joint_values);

    const std::vector<std::string> joint_names = joint_model_group->getVariableNames();
    for (std::size_t i = 0; i < joint_names.size() && i < joint_values.size(); ++i) {
      RCLCPP_INFO(
        node_->get_logger(),
        "pre-insert IK: %s = %.6f",
        joint_names[i].c_str(),
        joint_values[i]);
    }

    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();
    if (!move_group.setJointValueTarget(joint_values)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to set pre-insert IK joint target.");
      return false;
    }

    MoveGroupInterface::Plan plan;
    bool accepted_plan = false;
    const int max_valid_plan_attempts =
      std::max(1, static_cast<int>(
        node_->get_parameter("valid_preinsert_plan_attempts").as_int()));
    for (int attempt = 1; attempt <= max_valid_plan_attempts; ++attempt) {
      const bool success = static_cast<bool>(move_group.plan(plan));
      RCLCPP_INFO(
        node_->get_logger(),
        "Pre-insert IK joint plan attempt %d/%d %s.",
        attempt,
        max_valid_plan_attempts,
        success ? "succeeded" : "failed");

      if (!success) {
        continue;
      }

      if (validatePreinsertPlanKeepsBladeCenterOutsideContour(
          plan,
          *current_state,
          robot_model))
      {
        accepted_plan = true;
        break;
      }
    }

    if (!accepted_plan) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to find a pre-insert plan that keeps %s outside the contour prism.",
        blade_frame_.c_str());
      move_group.clearPoseTargets();
      return false;
    }

    if (!executePlanWithSafety(move_group, plan, "approach motion")) {
      move_group.clearPoseTargets();
      return false;
    }

    move_group.clearPoseTargets();
    return true;
  }

  bool planAndMaybeExecuteApproachToPreinsert(
    MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & preinsert_tool)
  {
    move_group.setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(preinsert_tool);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group.computeCartesianPath(
      waypoints,
      node_->get_parameter("cartesian_eef_step").as_double(),
      node_->get_parameter("cartesian_jump_threshold").as_double(),
      trajectory);

    RCLCPP_INFO(
      node_->get_logger(),
      "Approach-to-preinsert Cartesian path fraction: %.3f",
      fraction);

    if (fraction < node_->get_parameter("min_cartesian_fraction").as_double()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Approach-to-preinsert Cartesian path incomplete. Required %.3f, got %.3f.",
        node_->get_parameter("min_cartesian_fraction").as_double(),
        fraction);
      return false;
    }

    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    if (!executePlanWithSafety(move_group, plan, "approach-to-preinsert Cartesian motion")) {
      return false;
    }

    return true;
  }

  moveit::core::RobotState makeIkSeedState(
    const moveit::core::RobotState & current_state,
    const moveit::core::JointModelGroup * joint_model_group) const
  {
    moveit::core::RobotState seed_state = current_state;

    if (!node_->get_parameter("use_preferred_ik_seed").as_bool()) {
      RCLCPP_INFO(node_->get_logger(), "Using current state as pre-insert IK seed.");
      return seed_state;
    }

    const std::map<std::string, double> preferred_joints{
      {"shoulder_pan_joint", node_->get_parameter("preferred_shoulder_pan_joint").as_double()},
      {"shoulder_lift_joint", node_->get_parameter("preferred_shoulder_lift_joint").as_double()},
      {"elbow_joint", node_->get_parameter("preferred_elbow_joint").as_double()},
      {"wrist_1_joint", node_->get_parameter("preferred_wrist_1_joint").as_double()},
      {"wrist_2_joint", node_->get_parameter("preferred_wrist_2_joint").as_double()},
      {"wrist_3_joint", node_->get_parameter("preferred_wrist_3_joint").as_double()},
    };

    const std::vector<std::string> joint_names = joint_model_group->getVariableNames();
    std::vector<double> seed_values;
    seed_state.copyJointGroupPositions(joint_model_group, seed_values);

    for (std::size_t i = 0; i < joint_names.size() && i < seed_values.size(); ++i) {
      const auto preferred = preferred_joints.find(joint_names[i]);
      if (preferred != preferred_joints.end()) {
        seed_values[i] = preferred->second;
      }
    }

    seed_state.setJointGroupPositions(joint_model_group, seed_values);
    seed_state.enforceBounds(joint_model_group);
    seed_state.update();

    RCLCPP_INFO(node_->get_logger(), "Using preferred joint pose as pre-insert IK seed.");
    return seed_state;
  }

  static std::vector<geometry_msgs::msg::Pose> interpolateLinearPoses(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    const double segment_length)
  {
    const double dx = end.position.x - start.position.x;
    const double dy = end.position.y - start.position.y;
    const double dz = end.position.z - start.position.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const std::size_t segment_count = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(std::ceil(distance / std::max(segment_length, 1.0e-6))));

    std::vector<geometry_msgs::msg::Pose> targets;
    targets.reserve(segment_count);
    for (std::size_t i = 1; i <= segment_count; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(segment_count);
      geometry_msgs::msg::Pose target = end;
      target.position.x = start.position.x + t * dx;
      target.position.y = start.position.y + t * dy;
      target.position.z = start.position.z + t * dz;
      targets.push_back(target);
    }
    return targets;
  }

  bool computeAndExecuteCartesianTarget(
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

    RCLCPP_INFO(
      node_->get_logger(),
      "%s Cartesian path fraction: %.3f",
      stage.c_str(),
      fraction);

    if (fraction < node_->get_parameter("min_cartesian_fraction").as_double()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "%s Cartesian path incomplete. Required %.3f, got %.3f.",
        stage.c_str(),
        node_->get_parameter("min_cartesian_fraction").as_double(),
        fraction);
      return false;
    }

    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    return executePlanWithSafety(move_group, plan, stage);
  }

  bool planAndMaybeExecuteInsert(
    MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & preinsert_tool,
    const geometry_msgs::msg::Pose & insert_tool)
  {
    if (!node_->get_parameter("segmented_cartesian_insert").as_bool()) {
      return computeAndExecuteCartesianTarget(
        move_group,
        insert_tool,
        "insert Cartesian motion");
    }

    const double segment_length = node_->get_parameter("insert_segment_length").as_double();
    const std::vector<geometry_msgs::msg::Pose> segment_targets =
      interpolateLinearPoses(preinsert_tool, insert_tool, segment_length);

    RCLCPP_INFO(
      node_->get_logger(),
      "Executing insert as %zu Cartesian segment(s), segment_length=%.4f m.",
      segment_targets.size(),
      segment_length);

    for (std::size_t i = 0; i < segment_targets.size(); ++i) {
      const std::string stage = "insert segment " + std::to_string(i + 1) + "/" +
        std::to_string(segment_targets.size());

      if (!computeAndExecuteCartesianTarget(move_group, segment_targets[i], stage)) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "ABORT: unsafe or execution failure during %s.",
          stage.c_str());
        return false;
      }
    }

    return true;
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> preinsert_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> insert_pose_;
  bool preinsert_pose_logged_{false};
  bool insert_pose_logged_{false};

  std::string move_group_name_;
  std::string base_frame_;
  std::string end_effector_link_;
  std::string blade_frame_;
  std::string preinsert_topic_;
  std::string insert_topic_;
  std::string safety_topic_;
  std::string min_distance_topic_;
  bool execute_motion_;
  bool reject_preinsert_plan_inside_contour_;
  std::vector<Vec2> contour_polygon_;
  bool has_safety_status_{false};
  bool latest_safe_{false};
  bool last_safe_{true};
  double latest_min_distance_{std::numeric_limits<double>::quiet_NaN()};
  MoveGroupInterface * active_move_group_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr preinsert_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr insert_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr min_distance_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("opening_insert_executor");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  OpeningInsertExecutor insert_executor(node);
  const int result = insert_executor.run();

  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return result;
}
