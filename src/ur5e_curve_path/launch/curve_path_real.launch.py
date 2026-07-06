import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)

    with open(absolute_path, "r", encoding="utf-8") as yaml_file:
        return yaml.safe_load(yaml_file)


def generate_launch_description():
    path_csv_arg = DeclareLaunchArgument(
        "path_csv",
        default_value="paths/heart_path.csv",
        description=(
            "CSV path file. Relative paths are resolved inside "
            "the ur5e_curve_path package share directory."
        ),
    )

    base_x_from_csv_axis_arg = DeclareLaunchArgument(
        "base_x_from_csv_axis",
        default_value="x",
        description="CSV axis mapped to base_link X: x, y, or z.",
    )

    base_y_from_csv_axis_arg = DeclareLaunchArgument(
        "base_y_from_csv_axis",
        default_value="y",
        description="CSV axis mapped to base_link Y: x, y, or z.",
    )

    base_z_from_csv_axis_arg = DeclareLaunchArgument(
        "base_z_from_csv_axis",
        default_value="fixed",
        description="CSV axis mapped to base_link Z: fixed, x, y, or z.",
    )

    ur_description_share = get_package_share_directory(
        "ur_description"
    )

    ur_moveit_share = get_package_share_directory(
        "ur_moveit_config"
    )

    urdf_xacro = os.path.join(
        ur_description_share,
        "urdf",
        "ur.urdf.xacro",
    )

    srdf_xacro = os.path.join(
        ur_moveit_share,
        "srdf",
        "ur.srdf.xacro",
    )

    joint_limits_file = os.path.join(
        ur_description_share,
        "config",
        "ur5e",
        "joint_limits.yaml",
    )

    default_kinematics_file = os.path.join(
        ur_description_share,
        "config",
        "ur5e",
        "default_kinematics.yaml",
    )

    physical_parameters_file = os.path.join(
        ur_description_share,
        "config",
        "ur5e",
        "physical_parameters.yaml",
    )

    visual_parameters_file = os.path.join(
        ur_description_share,
        "config",
        "ur5e",
        "visual_parameters.yaml",
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    urdf_xacro,
                    " name:=ur",
                    " ur_type:=ur5e",
                    " tf_prefix:=",
                    " robot_ip:=0.0.0.0",
                    " use_fake_hardware:=false",
                    " fake_sensor_commands:=false",
                    " headless_mode:=true",
                    " joint_limit_params:=",
                    joint_limits_file,
                    " kinematics_params:=",
                    default_kinematics_file,
                    " physical_params:=",
                    physical_parameters_file,
                    " visual_params:=",
                    visual_parameters_file,
                ]
            ),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(
                [
                    "xacro ",
                    srdf_xacro,
                    " name:=ur",
                    " prefix:=",
                ]
            ),
            value_type=str,
        )
    }

    kinematics_yaml = load_yaml(
        "ur5e_curve_path",
        "config/kinematics.yaml",
    )

    curve_path_node = Node(
        package="ur5e_curve_path",
        executable="curve_path_node",
        name="curve_path_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {
                "robot_description_kinematics":
                    kinematics_yaml,
                "use_sim_time": False,
                "path_csv": ParameterValue(
                    LaunchConfiguration("path_csv"),
                    value_type=str,
                ),
                "base_x_from_csv_axis":
                    ParameterValue(
                        LaunchConfiguration("base_x_from_csv_axis"),
                        value_type=str,
                    ),
                "base_y_from_csv_axis":
                    ParameterValue(
                        LaunchConfiguration("base_y_from_csv_axis"),
                        value_type=str,
                    ),
                "base_z_from_csv_axis":
                    ParameterValue(
                        LaunchConfiguration("base_z_from_csv_axis"),
                        value_type=str,
                    ),
            },
        ],
    )

    return LaunchDescription(
        [
            path_csv_arg,
            base_x_from_csv_axis_arg,
            base_y_from_csv_axis_arg,
            base_z_from_csv_axis_arg,
            curve_path_node,
        ]
    )
