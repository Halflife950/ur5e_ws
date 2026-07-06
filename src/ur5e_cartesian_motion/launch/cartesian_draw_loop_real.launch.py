from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    ur_type = LaunchConfiguration("ur_type")
    prefix = LaunchConfiguration("prefix")

    joint_limit_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            ur_type,
            "joint_limits.yaml",
        ]
    )

    kinematics_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            ur_type,
            "default_kinematics.yaml",
        ]
    )

    physical_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            ur_type,
            "physical_parameters.yaml",
        ]
    )

    visual_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            ur_type,
            "visual_parameters.yaml",
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [   
                    FindPackageShare("ur_description"),
                    "urdf",
                    "ur.urdf.xacro",
                ]
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=true",
            " ",
            "safety_pos_margin:=0.15",
            " ",
            "safety_k_position:=20",
            " ",
            "name:=ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_moveit_config"),
                    "srdf",
                    "ur.srdf.xacro",
                ]
            ),
            " ",
            "name:=ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content,
            value_type=str,
        )
    }

    robot_description_kinematics = PathJoinSubstitution(
        [
            FindPackageShare("ur_moveit_config"),
            "config",
            "kinematics.yaml",
        ]
    )

    cartesian_draw_loop_node = Node(
        package="ur5e_cartesian_motion",
        executable="cartesian_draw_loop",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "mode": mode,
                "use_sim_time": False,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="line",
                description="Motion mode: line, square, or circle",
            ),
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur5e",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="",
            ),
            cartesian_draw_loop_node,
        ]
    )
