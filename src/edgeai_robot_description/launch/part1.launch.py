"""Part 1 in the existing final arena, with mapping held inactive until start."""

from pathlib import Path
from uuid import uuid4
import json

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('edgeai_robot_description'))
    slam_share = Path(get_package_share_directory('slam_toolbox'))
    nav_share = Path(get_package_share_directory('nav2_bringup'))
    explorer_share = Path(get_package_share_directory('mrtsp_exploration_ros2'))
    targets = yaml.safe_load((share / 'config/rescue_targets.yaml').read_text())['targets']
    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('gz_args', default_value='-r'),
        DeclareLaunchArgument('gz_partition', default_value=f'edgeai_part1_{uuid4().hex}',
                              description='Gazebo transport partition for this run'),
        DeclareLaunchArgument('output_root', default_value=str(Path.cwd() / 'runs/part1')),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gz_partition')),
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / 'launch/rescue_arena.launch.py')),
                launch_arguments={
                    'robot_count': '1', 'enable_rviz': 'false', 'enable_aruco': 'true',
                    'enable_registry': 'true', 'enable_world_anchor': 'false',
                    'registry_run_enabled_topic': '/part1/run_enabled',
                    'shutdown_on_gazebo_exit': 'false',
                    'gz_args': LaunchConfiguration('gz_args'),
                }.items(),
            ),
        ]),
        GroupAction(actions=[
            PushRosNamespace('robot1'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(slam_share / 'launch/online_async_launch.py')),
                launch_arguments={
                    'slam_params_file': str(share / 'config/part1_slam_toolbox.yaml'),
                    'use_sim_time': 'true', 'autostart': 'false',
                    'use_lifecycle_manager': 'false',
                }.items(),
            ),
            Node(package='edgeai_aruco', executable='part1_explorer',
                 name='mrtsp_explorer', output='screen',
                 parameters=[str(explorer_share / 'config/params.yaml'),
                             {'use_sim_time': True,
                              'map_topic': '/map',
                              'costmap_topic': '/global_costmap/costmap',
                              'local_costmap_topic': '/local_costmap/costmap',
                              'min_frontier_size_cells': 1,
                              'dilation_kernel_radius_cells': 0,
                              'occ_threshold': 99,
                              'navigate_to_pose_action_name': '/navigate_to_pose',
                              'global_frame': 'map',
                              'robot_base_frame': 'robot1/base_footprint'}]),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_share / 'launch/navigation_launch.py')),
            launch_arguments={
                'namespace': '',
                'params_file': str(share / 'config/part1_nav2_params.yaml'),
                'use_sim_time': 'true', 'autostart': 'false',
                'use_composition': 'False',
            }.items(),
        ),
        Node(package='edgeai_aruco', executable='part1_recorder',
             name='part1_recorder', output='screen',
             parameters=[{'use_sim_time': True,
                          'targets_json': ParameterValue(json.dumps({
                              str(target['id']): [name, target['type']]
                              for name, target in targets.items()}), value_type=str),
                          'output_root': LaunchConfiguration('output_root')}]),
        Node(package='rviz2', executable='rviz2', name='part1_rviz',
             output='screen', condition=IfCondition(LaunchConfiguration('enable_rviz')),
             arguments=['-d', str(share / 'rviz/part1.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
