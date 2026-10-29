import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_robots(context, single_robot_launch):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    if not 1 <= num_robots <= 10:
        raise ValueError('num_robots must be between 1 and 10')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single_robot_launch),
            launch_arguments={'robot_ns': f'robot{index + 1}'}.items(),
        )
        for index in range(num_robots)
    ]


def generate_launch_description():
    single_robot_launch = os.path.join(
        get_package_share_directory('edgeai_robot_description'),
        'launch',
        'single_robot.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='2',
            description='Number of robots to launch (1-10)',
        ),
        OpaqueFunction(
            function=launch_robots,
            kwargs={'single_robot_launch': single_robot_launch},
        ),
    ])
