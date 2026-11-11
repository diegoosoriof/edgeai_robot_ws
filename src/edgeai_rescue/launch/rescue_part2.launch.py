"""Part 2: Gazebo, shared map, independent AMCL/Nav2, verification and FSM."""

from datetime import datetime, timezone
from pathlib import Path
import tempfile
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from edgeai_rescue.launch_support import resolve_map, robot_parameters
from edgeai_rescue.configuration import load_mission_config
from edgeai_rescue.geometry import shift_pose
from edgeai_rescue.robot_model import describe_robot, drive_center_x


def launch_part2(context, share, description_share):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    config_path = Path(value('config_file')).expanduser().resolve()
    config = load_mission_config(config_path, description_share)
    if set(config['robots']) != {'robot1', 'robot2'}:
        raise ValueError('Part 2 requires robot1 and robot2')
    temporary = tempfile.TemporaryDirectory(prefix='edgeai_part2_')
    temporary_path = Path(temporary.name)
    map_yaml = resolve_map(value('map_yaml'), value('map_image'), temporary_path/'map.yaml')
    run_name = datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%SZ_')+uuid.uuid4().hex[:6]
    evidence = Path(value('evidence_dir')).expanduser().resolve()/run_name
    evidence.mkdir(parents=True, exist_ok=True)
    (evidence/'map_metadata.yaml').write_text(Path(map_yaml).read_text())
    (evidence/'mission_config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    common = {'use_sim_time': True}
    template = yaml.safe_load((share/'config/nav2.yaml').read_text())
    descriptions = {}
    for robot in config['robots']:
        descriptions[robot] = describe_robot(description_share/'urdf/edgeai_robot_real.urdf.xacro', robot)
    footprint = descriptions['robot1'][1]
    control_offset = drive_center_x(description_share/'urdf/edgeai_robot_real.urdf.xacro')
    flat_footprint = [float(coordinate) for point in footprint for coordinate in point]
    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=lambda _: temporary.cleanup() or [])])),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(description_share/'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': value('gz_args')+' '+str(description_share/'worlds/rescue_arena.sdf'),
                              'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[common, {'yaml_filename': map_yaml, 'topic_name': 'map', 'frame_id': 'map'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='map_lifecycle_manager',
             output='screen', parameters=[common, {'autostart': True, 'node_names': ['map_server']}]),
    ]
    for robot, start in config['robots'].items():
        description, _ = descriptions[robot]
        params_path = temporary_path/f'{robot}_nav2.yaml'
        params_path.write_text(yaml.safe_dump(robot_parameters(template, robot, start, footprint, control_offset), sort_keys=False))
        spawn = shift_pose([start['x'], start['y'], start['yaw']], control_offset)
        prefix = f'/{robot}'
        sensor_qos = {f'qos_overrides.{prefix}/scan.publisher.reliability': 'best_effort',
                      f'qos_overrides.{prefix}/scan.publisher.depth': 1,
                      f'qos_overrides.{prefix}/joint_states.publisher.reliability': 'best_effort'}
        actions.extend([
            Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
                 namespace=robot, output='screen', parameters=[common, {'robot_description': description,
                                                                     'frame_prefix': f'{robot}/'}]),
            Node(package='ros_gz_sim', executable='create', namespace=robot, output='screen',
                 arguments=['-name', robot, '-topic', f'{prefix}/robot_description', '-x', str(spawn[0]),
                            '-y', str(spawn[1]), '-z', '.01', '-Y', str(spawn[2])]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', namespace=robot, name='ros_gz_bridge',
                 output='screen', parameters=[sensor_qos], remappings=[(f'{prefix}/tf', '/tf')], arguments=[
                     f'{prefix}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                     f'{prefix}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                     f'{prefix}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                     f'{prefix}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                     f'{prefix}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                 ]),
            Node(package='twist_mux', executable='twist_mux', namespace=robot, name='twist_mux', output='screen',
                 parameters=[str(params_path)], remappings=[('cmd_vel_out', 'mux_cmd_vel')]),
        ])
        for package, executable, name in (
            ('nav2_amcl', 'amcl', 'amcl'),
            ('nav2_controller', 'controller_server', 'controller_server'),
            ('nav2_planner', 'planner_server', 'planner_server'),
            ('nav2_behaviors', 'behavior_server', 'behavior_server'),
            ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
            ('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        ):
            actions.append(Node(package=package, executable=executable, namespace=robot, name=name,
                                output='screen', parameters=[str(params_path)],
                                remappings=[('cmd_vel', 'nav_cmd_vel')] if name in ('controller_server', 'behavior_server') else []))
        actions.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', namespace=robot,
            name='navigation_lifecycle_manager', output='screen', parameters=[common, {
                'autostart': True, 'bond_timeout': 5.,
                'node_names': ['amcl', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'collision_monitor'],
            }]))
    settings = [common, {'config_file': str(config_path), 'evidence_dir': str(evidence),
                         'footprint': flat_footprint, 'control_offset_x': control_offset}]
    actions.extend([
        Node(package='edgeai_rescue', executable='verifier', name='rescue_verifier', output='screen', parameters=settings),
        Node(package='edgeai_rescue', executable='peer_obstacles', name='rescue_peer_obstacles', output='screen', parameters=settings),
        Node(package='edgeai_rescue', executable='mission', name='rescue_mission', output='screen', parameters=settings+[
            {'autostart_mission': value('autostart_mission').lower() == 'true',
             'elapsed_before_part2': float(value('elapsed_before_part2')),
             'initial_scan_angle': float(value('initial_scan_angle'))}]),
        Node(package='rviz2', executable='rviz2', name='rescue_part2_rviz', output='screen', parameters=[common],
             condition=IfCondition(LaunchConfiguration('enable_rviz')),
             arguments=['-d', str(share/'rviz/rescue_part2.rviz')]),
    ])
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('edgeai_rescue'))
    description_share = Path(get_package_share_directory('edgeai_robot_description'))
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', default_value=str(share/'maps/rescue_reference.yaml')),
        DeclareLaunchArgument('map_image', default_value='', description='Optional override for the map image path'),
        DeclareLaunchArgument('config_file', default_value=str(share/'config/mission.yaml')),
        DeclareLaunchArgument('evidence_dir', default_value='runs/part2'),
        DeclareLaunchArgument('autostart_mission', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('elapsed_before_part2', default_value='0.0', description='Seconds already used by Part 1 and transition'),
        DeclareLaunchArgument('initial_scan_angle', default_value='0.0', description='Optional Nav2 collision-checked scan; maximum 0.5 rad'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('gz_args', default_value='-r'),
        DeclareLaunchArgument('gz_partition', default_value=f'edgeai_part2_{uuid.uuid4().hex}',
                              description='Gazebo transport partition for this run'),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gz_partition')),
        OpaqueFunction(function=launch_part2, kwargs={'share': share, 'description_share': description_share}),
    ])
