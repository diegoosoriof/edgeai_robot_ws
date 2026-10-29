import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():
    def launch_slam(context):
        robot_ns = LaunchConfiguration('robot_ns').perform(context).strip('/')
        params_file = os.path.join(
            get_package_share_directory('edgeai_robot_description'),
            'config',
            'slam_toolbox_single_robot.yaml',
        )
        with open(params_file, encoding='utf-8') as source:
            params = source.read().replace('ROBOT_NS', robot_ns)

        # The included slam_toolbox launch accepts a file path, not a
        # ParameterFile object, so materialize the namespace-specific YAML.
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', prefix='slam_toolbox_', delete=False,
            encoding='utf-8',
        ) as params_handle:
            params_handle.write(params)
            slam_params_file = params_handle.name

        slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'online_async_launch.py',
                )
            ),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
            }.items(),
        )
        return [GroupAction([PushRosNamespace(robot_ns), 
                             SetRemap(src='/map', dst='map'),
                             SetRemap(src='/map_metadata', dst='map_metadata'),
                             slam])]

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ns',
            default_value='robot1',
            description='Namespace used for the robot and its SLAM frames/topics.',
        ),
        OpaqueFunction(function=launch_slam),
    ])
