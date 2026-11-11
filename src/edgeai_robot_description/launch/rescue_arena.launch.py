"""One or two independently controlled robots in the challenge arena."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory('edgeai_robot_description'))
    aruco = Path(get_package_share_directory('edgeai_aruco'))
    gz = Path(get_package_share_directory('ros_gz_sim'))
    targets = yaml.safe_load((share / 'config/rescue_targets.yaml').read_text())
    robots = yaml.safe_load((share / 'config/rescue_robots.yaml').read_text())
    actions = [
        DeclareLaunchArgument('world', default_value='rescue_arena.sdf',
                              description='World filename in this package worlds directory'),
        DeclareLaunchArgument('gz_args', default_value='-r',
                              description='Gazebo flags; use -r -s --headless-rendering without a GUI'),
        DeclareLaunchArgument('shutdown_on_gazebo_exit', default_value='true'),
        DeclareLaunchArgument('enable_aruco', default_value='true'),
        DeclareLaunchArgument('enable_registry', default_value='true'),
        DeclareLaunchArgument('registry_run_enabled_topic', default_value='',
                              description='Optional Bool topic gating remembered detections'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('enable_world_anchor', default_value='true'),
        DeclareLaunchArgument('robot_count', default_value='2', choices=['1', '2'],
                              description='1 spawns robot1; 2 spawns robot1 and robot2'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(share / 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz / 'launch/gz_sim.launch.py')),
            launch_arguments={
                'gz_args': [LaunchConfiguration('gz_args'), ' ',
                            PathJoinSubstitution([str(share), 'worlds', LaunchConfiguration('world')])],
                'on_exit_shutdown': LaunchConfiguration('shutdown_on_gazebo_exit'),
            }.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
    ]
    # Facing north: a conservative 0.34 x 0.44 m footprint fits inside the start.
    for index, (name, pose) in enumerate(robots.items(), start=1):
        x, y, yaw = pose['x'], pose['y'], pose['yaw']
        description = xacro.process_file(
            str(share / 'urdf/edgeai_robot_real.urdf.xacro'),
            mappings={'robot_namespace': name}).toxml()
        prefix = f'/{name}'
        native_frame = f'{name}/base_footprint/rgbd_camera'
        settings = {'use_sim_time': True}
        # Do not queue old RGB-D frames when both cameras stream at once.
        sensor_qos = {}
        for topic in ('camera/image', 'camera/depth_image', 'camera/camera_info', 'camera/points',
                      'scan', 'joint_states'):
            sensor_qos[f'qos_overrides.{prefix}/{topic}.publisher.reliability'] = 'best_effort'
            sensor_qos[f'qos_overrides.{prefix}/{topic}.publisher.depth'] = 1
        actions.append(GroupAction(condition=IfCondition(PythonExpression([
            LaunchConfiguration('robot_count'), f' >= {index}',
        ])), actions=[
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace=name, name='robot_state_publisher', output='screen',
                 parameters=[settings, {'robot_description': description, 'frame_prefix': f'{name}/'}]),
            Node(package='ros_gz_sim', executable='create', namespace=name, output='screen',
                 arguments=['-name', name, '-topic', f'{prefix}/robot_description',
                            '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw)]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', namespace=name,
                 name='ros_gz_bridge', output='screen', arguments=[
                     f'{prefix}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                     f'{prefix}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                     f'{prefix}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                     f'{prefix}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                     f'{prefix}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                     f'{prefix}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                     f'{prefix}/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                     f'{prefix}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                     f'{prefix}/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                 ], remappings=[(f'{prefix}/tf', '/tf')], parameters=[sensor_qos]),
            Node(package='tf2_ros', executable='static_transform_publisher', namespace=name,
                 name='camera_sensor_tf', output='screen', parameters=[settings],
                 arguments=['--frame-id', f'{name}/camera_link', '--child-frame-id', native_frame]),
            Node(package='edgeai_aruco', executable='aruco_detector', namespace=name,
                 name='aruco_detector', output='screen',
                 condition=IfCondition(LaunchConfiguration('enable_aruco')),
                 parameters=[str(aruco / 'config/aruco.yaml'), settings, {
                     'dictionary': targets['dictionary'], 'marker_size': targets['marker_size'],
                     'input_frame': native_frame, 'optical_frame': f'{name}/camera_optical_frame',
                     'tf_child_prefix': f'{name}/aruco_marker_',
                 }]),
            Node(package='edgeai_aruco', executable='aruco_registry', namespace=name,
                 name='aruco_registry', output='screen',
                 condition=IfCondition(AndSubstitution(
                     LaunchConfiguration('enable_aruco'), LaunchConfiguration('enable_registry'))),
                 parameters=[str(aruco / 'config/aruco.yaml'), settings,
                             {'registry_frame': f'{name}/odom',
                              'run_enabled_topic': ParameterValue(
                                  LaunchConfiguration('registry_run_enabled_topic'), value_type=str)}]),
        ]))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/rescue_visualization.launch.py')),
        launch_arguments={
            'enable_rviz': LaunchConfiguration('enable_rviz'),
            'enable_world_anchor': LaunchConfiguration('enable_world_anchor'),
            'robot_count': LaunchConfiguration('robot_count'),
        }.items(),
    ))
    return LaunchDescription(actions)
