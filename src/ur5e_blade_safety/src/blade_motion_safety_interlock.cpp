#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class BladeMotionSafetyInterlock : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  BladeMotionSafetyInterlock()
  : Node("blade_motion_safety_interlock")
  {
    declare_parameter<std::string>("safety_topic", "/blade_contour_safe");
    declare_parameter<std::string>("min_distance_topic", "/blade_contour_min_distance");
    declare_parameter<std::string>(
      "trajectory_action", "/joint_trajectory_controller/follow_joint_trajectory");
    declare_parameter<bool>("cancel_trajectory_on_unsafe", true);
    declare_parameter<bool>("log_transition_only", true);
    declare_parameter<double>("action_server_wait_s", 0.0);

    safety_topic_ = get_parameter("safety_topic").as_string();
    min_distance_topic_ = get_parameter("min_distance_topic").as_string();
    trajectory_action_ = get_parameter("trajectory_action").as_string();

    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this,
      trajectory_action_);

    safety_sub_ = create_subscription<std_msgs::msg::Bool>(
      safety_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        handleSafetyStatus(msg->data);
      });

    min_distance_sub_ = create_subscription<std_msgs::msg::Float64>(
      min_distance_topic_,
      10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        latest_min_distance_ = msg->data;
      });

    RCLCPP_INFO(
      get_logger(),
      "Blade motion safety interlock armed: safety_topic=%s trajectory_action=%s",
      safety_topic_.c_str(),
      trajectory_action_.c_str());
  }

private:
  void handleSafetyStatus(const bool safe)
  {
    const bool transition_to_unsafe = has_safety_status_ && last_safe_ && !safe;
    const bool should_log =
      !get_parameter("log_transition_only").as_bool() ? !safe : transition_to_unsafe;

    has_safety_status_ = true;
    last_safe_ = safe;

    if (safe) {
      cancel_sent_for_current_unsafe_ = false;
      return;
    }

    if (should_log) {
      RCLCPP_ERROR(
        get_logger(),
        "UNSAFE: cancelling active trajectory goals, min_distance=%.4f m",
        latest_min_distance_);
    }

    if (!get_parameter("cancel_trajectory_on_unsafe").as_bool() ||
      cancel_sent_for_current_unsafe_)
    {
      return;
    }

    const double wait_s = get_parameter("action_server_wait_s").as_double();
    if (wait_s > 0.0 &&
      !trajectory_client_->wait_for_action_server(std::chrono::duration<double>(wait_s)))
    {
      RCLCPP_WARN(
        get_logger(),
        "Trajectory action server %s is not available; cannot cancel active motion.",
        trajectory_action_.c_str());
      return;
    }

    cancel_sent_for_current_unsafe_ = true;
    trajectory_client_->async_cancel_all_goals();
  }

  std::string safety_topic_;
  std::string min_distance_topic_;
  std::string trajectory_action_;
  double latest_min_distance_{std::numeric_limits<double>::quiet_NaN()};
  bool has_safety_status_{false};
  bool last_safe_{true};
  bool cancel_sent_for_current_unsafe_{false};

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr min_distance_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BladeMotionSafetyInterlock>());
  rclcpp::shutdown();
  return 0;
}
