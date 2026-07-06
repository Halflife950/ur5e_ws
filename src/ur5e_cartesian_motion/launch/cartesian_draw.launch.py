from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="line",
        description="Motion mode: line, square, or circle"
    )

    cartesian_draw_node = Node(
        package="ur5e_cartesian_motion",
        executable="cartesian_draw",
        output="screen",
        parameters=[
                "/opt/ros/humble/share/ur_moveit_config/config/kinematics.yaml",
                {
                    "mode": LaunchConfiguration("mode"),
                    "use_sim_time": True
                }
        ],
    )

    return LaunchDescription([
        mode_arg,
        cartesian_draw_node
    ])
