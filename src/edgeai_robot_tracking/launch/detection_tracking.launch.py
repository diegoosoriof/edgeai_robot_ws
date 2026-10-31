"""Optional existing simulation plus independent perception; teleop uses a real terminal."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from edgeai_robot_tracking.methods import DETECTION_METHODS, validate_detection_method


def experiment(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    observer = value('observer_ns').strip('/')
    if not observer:
        raise ValueError('observer_ns must name the observing robot')
    method = validate_detection_method(value('detection_method'))
    overrides = {'use_sim_time': value('use_sim_time').lower() == 'true',
                 'detection_method': method}
    if value('start_simulation').lower() == 'true' and not overrides['use_sim_time']:
        raise ValueError('start_simulation=true requires use_sim_time=true')
    # Explicit launch arguments override YAML only when supplied. Otherwise a real
    # camera's config file may override all simulation defaults, including frames.
    frame_defaults = {
        'camera_optical_frame': f'{observer}/camera_optical_frame',
        'depth_optical_frame': f'{observer}/camera_optical_frame',
        'expected_rgb_frame': f'{observer}/base_footprint/rgbd_camera',
        'expected_depth_frame': f'{observer}/base_footprint/rgbd_camera',
        'base_frame': f'{observer}/base_footprint',
        'tracking_frame': f'{observer}/odom',
        'output_frame': f'{observer}/base_footprint',
    }
    arguments = ('rgb_topic', 'depth_topic', 'camera_info_topic', 'depth_camera_info_topic',
                 'debug_image', 'debug_markers', 'debug_cluster', *frame_defaults)
    for name in arguments:
        if value(name):
            overrides[name] = (value(name).lower() == 'true'
                               if name.startswith('debug_') else value(name))
    parameters = [frame_defaults]
    if method == 'geometry':
        parameters.append(value('geometry_config_file'))
    parameters.extend([value('config_file'), overrides])
    return [
        Node(
            package='edgeai_robot_tracking', executable='rgbd_tracker',
            name='rgbd_tracker', namespace=observer, output='screen',
            parameters=parameters,
            condition=IfCondition(LaunchConfiguration('enable_detector')),
        ),
        LogInfo(msg='Keyboard control runs separately in an interactive terminal: '
                'ros2 run edgeai_robot_tracking keyboard_teleop --namespace ' + value('target_ns')),
    ]


def generate_launch_description():
    share = get_package_share_directory('edgeai_robot_tracking')
    description = get_package_share_directory('edgeai_robot_description')
    arguments = {
        'start_simulation': ('false', 'Include the existing multi_robot_sim launcher'),
        'num_robots': ('2', 'Robot count when starting simulation (1-10)'),
        'gui': ('true', 'Start simulation GUI/RViz; false uses headless rendering'),
        'observer_ns': ('robot1', 'Namespace of the camera-equipped observer'),
        'target_ns': ('robot2', 'Keyboard-control namespace only; never a detector identity'),
        'use_sim_time': ('true', 'Use bridged simulation time for perception'),
        'enable_detector': ('true', 'Enable perception independently of simulation/teleop'),
        'detection_method': ('green_seeds', 'Startup detector: green_seeds or geometry'),
        'config_file': (os.path.join(share, 'config', 'tracking.yaml'), 'Detector YAML parameters'),
        'geometry_config_file': (os.path.join(share, 'config', 'geometry.yaml'),
                                 'Geometry-only defaults, loaded before config_file'),
        'rgb_topic': ('', 'Override RGB topic; relative names resolve under observer_ns'),
        'depth_topic': ('', 'Override depth topic'),
        'camera_info_topic': ('', 'Override RGB CameraInfo topic'),
        'depth_camera_info_topic': ('', 'Override depth CameraInfo topic'),
        'output_frame': ('', 'Output position frame; default observer/base_footprint'),
        'tracking_frame': ('', 'Continuous filter frame; default observer/odom'),
        'base_frame': ('', 'Observer floor-reference frame'),
        'camera_optical_frame': ('', 'RGB optical ray frame for reconstruction'),
        'depth_optical_frame': ('', 'Native depth optical ray frame; also used for registration'),
        'expected_rgb_frame': ('', 'Accepted RGB/CameraInfo header alias'),
        'expected_depth_frame': ('', 'Accepted depth/CameraInfo header alias'),
        'debug_image': ('', 'Override method-specific debug images: true/false'),
        'debug_markers': ('', 'Override marker debugging: true/false'),
        'debug_cluster': ('', 'Override selected-cluster markers: true/false'),
    }
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description, 'launch', 'multi_robot_sim.launch.py')),
        launch_arguments={
            'num_robots': LaunchConfiguration('num_robots'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_simulation')),
    )
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=desc,
                                choices=list(DETECTION_METHODS)
                                if name == 'detection_method' else None)
          for name, (default, desc) in arguments.items()),
        simulation,
        OpaqueFunction(function=experiment),
    ])
