"""Show the selected rescue robots; can attach to an already-running arena."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory('edgeai_robot_description'))
    robots = yaml.safe_load((share / 'config/rescue_robots.yaml').read_text())
    actions = [
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('robot_count', default_value='2', choices=['1', '2'],
                              description='Match the robot_count used by the arena launch'),
    ]
    for index, (name, pose) in enumerate(robots.items(), start=1):
        # Wheel odometry starts at zero at each spawn. These initial offsets
        # provide a shared visualization frame; odometry drift still applies.
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            namespace=name, name='spawn_odom_tf', output='screen',
            condition=IfCondition(PythonExpression([
                LaunchConfiguration('robot_count'), f' >= {index}',
            ])),
            parameters=[{'use_sim_time': True}],
            arguments=['--x', str(pose['x']), '--y', str(pose['y']), '--yaw', str(pose['yaw']),
                       '--frame-id', 'world', '--child-frame-id', f'{name}/odom'],
        ))
    actions.append(Node(
        package='rviz2', executable='rviz2', name='rescue_rviz', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        parameters=[{'use_sim_time': True}],
        arguments=['-d', PathJoinSubstitution([str(share), 'rviz', PythonExpression([
            "'rescue_arena_single.rviz' if ", LaunchConfiguration('robot_count'),
            " == 1 else 'rescue_arena.rviz'",
        ])])],
    ))
    return LaunchDescription(actions)
