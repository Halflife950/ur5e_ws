from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ur5e_basic_motion",
            executable="basic_motion",
            output="screen",
            parameters=[
                "/opt/ros/humble/share/ur_moveit_config/config/kinematics.yaml",
                {
                    "use_sim_time": True
                }
            ],
        )
    ])
