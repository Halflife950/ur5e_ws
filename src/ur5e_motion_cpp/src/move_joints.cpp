#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class UR5eJointMover : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory =
        rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    UR5eJointMover() : Node("ur5e_joint_mover")
    {
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/joint_trajectory_controller/follow_joint_trajectory"
        );

        timer_ = this->create_wall_timer(
            2s,
            std::bind(&UR5eJointMover::send_goal, this)
        );
    }

private:
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_sent_ = false;

    void send_goal()
    {
        if (goal_sent_) {
            return;
        }

        if (!action_client_->wait_for_action_server(3s)) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory action server not available.");
            return;
        }

        goal_sent_ = true;

        FollowJointTrajectory::Goal goal_msg;

        goal_msg.trajectory.joint_names = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;

        point.positions = {
            0.0,
            -1.57,
            1.57,
            -1.57,
            -1.57,
            0.0
        };

        point.velocities = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };

        point.time_from_start = rclcpp::Duration::from_seconds(4.0);

        goal_msg.trajectory.points.push_back(point);

        RCLCPP_INFO(this->get_logger(), "Sending trajectory goal...");

        auto send_goal_options =
            rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle)
            {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal rejected.");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted.");
                }
            };

        send_goal_options.result_callback =
            [this](const GoalHandleFollowJointTrajectory::WrappedResult & result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Motion finished.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Motion failed.");
                }
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR5eJointMover>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}