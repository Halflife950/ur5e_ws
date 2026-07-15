from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_alignment = LaunchConfiguration('enable_alignment')
    enable_interlock = LaunchConfiguration('enable_interlock')
    enable_safety_markers = LaunchConfiguration('enable_safety_markers')

    return LaunchDescription([
        DeclareLaunchArgument(
            'tuning_file',
            default_value=default_tuning_file,
            description='Central tuning yaml for opening alignment and blade safety nodes.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time for Gazebo workflows. Set false on the real robot.',
        ),
        DeclareLaunchArgument(
            'enable_alignment',
            default_value='true',
            description='Start the simulation-only opening alignment target publisher.',
        ),
        DeclareLaunchArgument(
            'enable_interlock',
            default_value='true',
            description='Start the trajectory cancel interlock node.',
        ),
        DeclareLaunchArgument(
            'enable_safety_markers',
            default_value='false',
            description='Publish blade_contour_safety_markers for RViz when alignment markers are not running.',
        ),
        Node(
            package='ur5e_blade_safety',
            executable='opening_alignment_planner',
            name='opening_alignment_planner',
            output='screen',
            condition=IfCondition(enable_alignment),
            parameters=[
                tuning_file,
                {
                    'use_sim_time': use_sim_time,
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
                    'use_sim_time': use_sim_time,
                    'publish_markers': enable_safety_markers,
                },
            ],
        ),
        Node(
            package='ur5e_blade_safety',
            executable='blade_motion_safety_interlock',
            name='blade_motion_safety_interlock',
            output='screen',
            condition=IfCondition(enable_interlock),
            parameters=[
                tuning_file,
                {
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
    ])
