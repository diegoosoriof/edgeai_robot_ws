import copy
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LIDAR_COLORS = [
    '0; 220; 80',     # robot1: green
    '40; 130; 255',   # robot2: blue
    '255; 150; 30',   # robot3: orange
    '200; 60; 255',   # robot4: purple
    '255; 50; 100',   # robot5: pink
    '0; 210; 210',    # robot6: cyan
    '255; 210; 0',    # robot7: yellow
    '160; 100; 255',  # robot8: violet
    '255; 100; 50',   # robot9: coral
    '130; 220; 40',   # robot10: lime
]


def robot_spawn_position(index):
    """Return the world position for zero-based robot index."""
    return -2.0 + 0.8 * index, 0.0, 0.01


def create_rviz_config(template_path, num_robots):
    """Create one RViz configuration with a model and scan for each robot."""
    with open(template_path, 'r', encoding='utf-8') as file:
        config = yaml.safe_load(file)

    manager = config['Visualization Manager']
    original_displays = manager['Displays']

    robot_template = next(
        (d for d in original_displays
         if d.get('Class') == 'rviz_default_plugins/RobotModel'),
        None,
    )
    scan_template = next(
        (d for d in original_displays
         if d.get('Class') == 'rviz_default_plugins/LaserScan'),
        None,
    )
    if robot_template is None or scan_template is None:
        raise ValueError(
            'multi_robot.rviz must contain at least one RobotModel '
            'and one LaserScan display to use as templates.'
        )

    # Keep Grid, TF, and other common displays exactly once.
    displays = [
        copy.deepcopy(d) for d in original_displays
        if d.get('Class') not in (
            'rviz_default_plugins/RobotModel',
            'rviz_default_plugins/LaserScan',
        )
    ]

    for index in range(num_robots):
        name = f'robot{index + 1}'

        robot_display = copy.deepcopy(robot_template)
        robot_display['Name'] = f'{name} Model'
        robot_display['Enabled'] = True
        # RViz inserts the slash; a trailing slash would produce robot1//base_link.
        robot_display['TF Prefix'] = name
        robot_display['Visual Enabled'] = True
        robot_display['Alpha'] = 1.0
        description_topic = robot_display.get('Description Topic')
        if isinstance(description_topic, dict):
            description_topic['Value'] = f'/{name}/robot_description'
            description_topic['Durability Policy'] = 'Transient Local'
        else:
            robot_display['Description Topic'] = f'/{name}/robot_description'
        displays.append(robot_display)

        scan_display = copy.deepcopy(scan_template)
        scan_display['Name'] = f'{name} LiDAR'
        scan_display['Enabled'] = True
        scan_topic = scan_display.get('Topic')
        if isinstance(scan_topic, dict):
            scan_topic['Value'] = f'/{name}/scan'
        else:
            scan_display['Topic'] = f'/{name}/scan'
        scan_display['Color Transformer'] = 'FlatColor'
        scan_display['Color'] = LIDAR_COLORS[index % len(LIDAR_COLORS)]
        scan_display['Alpha'] = 1.0
        displays.append(scan_display)
    
    # Configure TF visualization consistently for all robots
    for display in displays:
        if display.get('Class') == 'rviz_default_plugins/TF':
            display['Enabled'] = True
            display['Show Axes'] = False
            display['Show Names'] = False
            display['Show Arrows'] = False

    manager['Displays'] = displays
    manager['Global Options']['Fixed Frame'] = 'world'

    with tempfile.NamedTemporaryFile(
        mode='w', encoding='utf-8', prefix='multi_robot_',
        suffix='.rviz', delete=False,
    ) as file:
        yaml.safe_dump(config, file, sort_keys=False)
        return file.name


def spawn_robots(context, xacro_file, rviz_template):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    if not 1 <= num_robots <= 10:
        raise ValueError('num_robots must be between 1 and 10')

    robot_nodes = []

    for i in range(num_robots):
        robot_name = f'robot{i + 1}'
        x, y, z = robot_spawn_position(i)

        robot_xml = xacro.process_file(
            xacro_file, mappings={'robot_ns': robot_name}
        ).toxml()

        robot_nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'world_tf_{robot_name}',
            arguments=[
                '--x', str(x), '--y', str(y), '--z', '0',
                '--frame-id', 'world',
                '--child-frame-id', f'{robot_name}/odom',
            ],
            output='screen',
        ))

        robot_nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'robot_description': robot_xml,
                'use_sim_time': True,
                'frame_prefix': f'{robot_name}/',
            }],
        ))

        robot_nodes.append(Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_name}',
            arguments=[
                '-name', robot_name,
                '-string', robot_xml,
                '-x', str(x), '-y', str(y), '-z', str(z),
            ],
            output='screen',
        ))

        robot_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{robot_name}',
            output='screen',
            arguments=[
                f'/{robot_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/{robot_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{robot_name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                f'/{robot_name}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/{robot_name}/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/{robot_name}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                f'/{robot_name}/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
        ))

        robot_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'tf_bridge_{robot_name}',
            output='screen',
            arguments=[
                f'/{robot_name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            remappings=[(f'/{robot_name}/tf', '/tf')],
        ))

    # RViz must be launched only once, after the per-robot loop.
    rviz_config = create_rviz_config(rviz_template, num_robots)
    robot_nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    ))
    return robot_nodes


def generate_launch_description():
    package_name = 'edgeai_robot_description'
    pkg_share = get_package_share_directory(package_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_share, 'urdf', 'edgeai_robot_real_multi.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    rviz_template = os.path.join(pkg_share, 'rviz', 'multi_robot.rviz')

    num_robots_arg = DeclareLaunchArgument(
        'num_robots', default_value='2',
        description='Number of robots to spawn (1-10)',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    robots = OpaqueFunction(
        function=spawn_robots,
        kwargs={'xacro_file': xacro_file, 'rviz_template': rviz_template},
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    return LaunchDescription([
        num_robots_arg,
        gazebo,
        clock_bridge,
        robots,
    ])