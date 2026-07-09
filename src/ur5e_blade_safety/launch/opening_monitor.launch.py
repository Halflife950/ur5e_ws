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

    return LaunchDescription([
        DeclareLaunchArgument(
            'tuning_file',
            default_value=default_tuning_file,
            description='Central tuning yaml for opening alignment and blade safety nodes.',
        ),
        Node(
            package='ur5e_blade_safety',
            executable='opening_alignment_planner',
            name='opening_alignment_planner',
            output='screen',
            parameters=[
                tuning_file,
                {
                    'use_sim_time': True,
                },
            ],
        ),
        Node(
            package='ur5e_blade_safety',
            executable='blade_contour_safety',
            name='blade_contour_safety',
            output='screen',
            parameters=[
                tuning_file,
                {
                    'use_sim_time': True,
                },
            ],
        ),
        Node(
            package='ur5e_blade_safety',
            executable='blade_motion_safety_interlock',
            name='blade_motion_safety_interlock',
            output='screen',
            parameters=[
                tuning_file,
                {
                    'use_sim_time': True,
                },
            ],
        ),
    ])
