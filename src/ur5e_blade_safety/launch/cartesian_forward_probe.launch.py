from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_tuning_file = PathJoinSubstitution([
        FindPackageShare('ur5e_blade_safety'),
        'config',
        'blade_task_tuning.yaml',
    ])
    tuning_file = LaunchConfiguration('tuning_file')

    kinematics_file = PathJoinSubstitution([
        FindPackageShare('ur_moveit_config'),
        'config',
        'kinematics.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'tuning_file',
            default_value=default_tuning_file,
            description='Central tuning yaml for the Cartesian forward probe.',
        ),
        Node(
            package='ur5e_blade_safety',
            executable='cartesian_forward_probe',
            name='cartesian_forward_probe',
            output='screen',
            parameters=[
                kinematics_file,
                tuning_file,
                {
                    'use_sim_time': True,
                },
            ],
        ),
    ])
