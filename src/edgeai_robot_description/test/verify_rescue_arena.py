"""Explicit Gazebo smoke test; run with Pixi and install/setup.bash sourced.

Uses an isolated ROS domain and Gazebo partition. Moves and teleports only the
robots launched by this test, and adds a temporary overhead camera.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from google.protobuf.text_format import MessageToString
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--ros-domain-id', type=int, default=86)
    parser.add_argument('--robot-count', type=int, choices=(1, 2), default=2)
    args = parser.parse_args()
    if not 0 <= args.ros_domain_id <= 232:
        parser.error('ROS domain must be between 0 and 232')
    output = args.output_dir or Path(tempfile.mkdtemp(prefix='edgeai_rescue_'))
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id),
                      GZ_PARTITION=f'edgeai_rescue_test_{os.getpid()}',
                      ROS_LOG_DIR=str(output / 'ros_logs'),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      CYCLONEDDS_URI=(Path(get_package_share_directory('edgeai_robot_description'))
                                      / 'config/rescue_simulation_dds.xml').as_uri())
    rclpy.init()
    node = rclpy.create_node('rescue_arena_probe')
    callbacks = ReentrantCallbackGroup()
    sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    transport = GzNode()
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    bridge = CvBridge()
    latest, poses, overhead = {}, {}, []
    detections = {f'robot{index}': [] for index in range(1, args.robot_count + 1)}
    subscriptions, publishers = [], {}
    sensor_types = {'camera/image': Image, 'camera/depth_image': Image,
                    'camera/camera_info': CameraInfo, 'camera/points': PointCloud2,
                    'scan': LaserScan, 'joint_states': JointState, 'odom': Odometry}
    for name in detections:
        for topic, message_type in sensor_types.items():
            subscriptions.append(node.create_subscription(
                message_type, f'/{name}/{topic}',
                lambda message, key=(name, topic): latest.update({key: message}),
                sensor_qos, callback_group=callbacks))
        subscriptions.append(node.create_subscription(
            Detection3DArray, f'/{name}/aruco/detections',
            lambda message, robot=name: detections[robot].append(message), 1,
            callback_group=callbacks))
        publishers[name] = node.create_publisher(Twist, f'/{name}/cmd_vel', 1)

    def on_poses(message):
        poses.update({pose.name: pose for pose in message.pose if pose.name in ('robot1', 'robot2')})

    transport.subscribe(Pose_V, '/world/rescue_arena/pose/info', on_poses)
    transport.subscribe(GzImage, '/arena_test/overhead', lambda message: overhead.append(message))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    def wait_for(predicate, label, timeout=60):
        deadline, progress = time.monotonic() + timeout, time.monotonic() + 15
        while time.monotonic() < deadline:
            time.sleep(.05)
            if predicate():
                return
            if process.poll() is not None:
                raise RuntimeError(f'Launch exited: {process.returncode}; see {output}')
            if time.monotonic() > progress:
                stamps = {robot: latest[(robot, 'odom')].header.stamp.sec
                          for robot in detections if (robot, 'odom') in latest}
                print(f'Waiting for {label}; simulation seconds={stamps}', flush=True)
                progress += 15
        raise AssertionError(f'Timed out: {label}; see {output}')

    def position(name):
        return np.array([poses[name].position.x, poses[name].position.y])

    def request_service(name, message):
        # A separate transport process lets Python subscriber callbacks keep
        # running while a synchronous Gazebo service request waits for its reply.
        result = subprocess.run([
            'gz', 'service', '-s', f'/world/rescue_arena/{name}',
            '--reqtype', message.DESCRIPTOR.full_name, '--reptype', 'gz.msgs.Boolean',
            '--timeout', '15000', '--req', MessageToString(message),
        ], capture_output=True, text=True, timeout=20)
        assert result.returncode == 0 and 'data: true' in result.stdout, (
            name, result.stdout, result.stderr)

    def move_only(name):
        before = {robot: position(robot).copy() for robot in detections}
        command = Twist()
        command.linear.x = .08
        start = latest[(name, 'odom')].header.stamp
        start_time = start.sec + start.nanosec / 1e9

        def advanced():
            publishers[name].publish(command)
            stamp = latest[(name, 'odom')].header.stamp
            return stamp.sec + stamp.nanosec / 1e9 - start_time > 1.5

        try:
            wait_for(advanced, f'{name} independent movement', 40)
        finally:
            publishers[name].publish(Twist())
        wait_for(lambda: abs(latest[(name, 'odom')].twist.twist.linear.x) < .01,
                 f'{name} stop', 20)
        displacement = {robot: float(np.linalg.norm(position(robot) - before[robot]))
                        for robot in detections}
        assert displacement[name] > .06, displacement
        assert all(distance < .025 for robot, distance in displacement.items() if robot != name), displacement
        return displacement

    command = ['ros2', 'launch', 'edgeai_robot_description', 'rescue_arena.launch.py',
               f'robot_count:={args.robot_count}', 'enable_rviz:=false',
               'gz_args:=-r -s --headless-rendering']
    process = None
    try:
        with (output / 'launch.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            wait_for(lambda: len(latest) == len(sensor_types) * args.robot_count
                     and all(robot in poses for robot in detections),
                     'selected robot poses and all independent sensors', 120)
            # Wait for the robots to settle on the floor before checking controls.
            wait_for(lambda: all(latest[(name, 'odom')].header.stamp.nanosec > 150000000
                                 or latest[(name, 'odom')].header.stamp.sec >= 1 for name in detections),
                     'physics settling')
            spawn_positions = {name: position(name).tolist() for name in detections}
            assert set(poses) == set(detections), f'Unexpected robots spawned: {set(poses) - set(detections)}'
            np.testing.assert_allclose(spawn_positions['robot1'], [.45, .45], atol=.02)
            if args.robot_count == 2:
                np.testing.assert_allclose(spawn_positions['robot2'], [1, .45], atol=.02)
            else:
                assert not any(namespace == '/robot2' for _, namespace in node.get_node_names_and_namespaces())
                assert not buffer.can_transform('world', 'robot2/odom', rclpy.time.Time())
            for name in detections:
                camera_frame = f'{name}/base_footprint/rgbd_camera'
                assert latest[(name, 'scan')].header.frame_id == f'{name}/lidar_link'
                for topic in ('camera/image', 'camera/depth_image', 'camera/camera_info', 'camera/points'):
                    assert latest[(name, topic)].header.frame_id == camera_frame
                assert len(latest[(name, 'camera/points')].data) > 0
                odom = latest[(name, 'odom')]
                assert odom.header.frame_id == f'{name}/odom'
                assert odom.child_frame_id == f'{name}/base_footprint'
                wait_for(lambda robot=name: buffer.can_transform(
                    'world', f'{robot}/base_footprint', rclpy.time.Time()),
                    f'{name} shared visualization frame')
                world_pose = buffer.lookup_transform('world', f'{name}/base_footprint', rclpy.time.Time())
                np.testing.assert_allclose([world_pose.transform.translation.x,
                                            world_pose.transform.translation.y],
                                           spawn_positions[name], atol=.02)
                wait_for(lambda robot=name: buffer.can_transform(
                    f'{robot}/odom', f'{robot}/camera_optical_frame', rclpy.time.Time()),
                    f'{name} complete TF tree')
                transform = buffer.lookup_transform(f'{name}/camera_link', camera_frame, rclpy.time.Time())
                np.testing.assert_allclose([transform.transform.translation.x,
                                            transform.transform.translation.y,
                                            transform.transform.translation.z], 0, atol=1e-8)
                assert abs(transform.transform.rotation.w) > .99999
            # Capture the original spawn layout using an actual Gazebo render.
            camera = '''<sdf version="1.9"><model name="arena_test_camera"><static>true</static>
              <pose>2.5 2.5 7 0 1.570796326795 1.570796326795</pose><link name="camera">
              <sensor name="overhead" type="camera"><always_on>true</always_on><update_rate>1</update_rate>
              <topic>/arena_test/overhead</topic><camera><horizontal_fov>0.85</horizontal_fov>
              <image><width>1000</width><height>1000</height><format>R8G8B8</format></image>
              <clip><near>0.1</near><far>15</far></clip></camera></sensor></link></model></sdf>'''
            request_service('create', EntityFactory(sdf=camera))
            wait_for(lambda: bool(overhead), 'top-down render', 30)
            picture = overhead[-1]
            rgb = np.frombuffer(picture.data, dtype=np.uint8).reshape(picture.height, picture.width, 3)
            cv2.imwrite(str(output / 'arena_top_down.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            overhead.clear()
            movement = {name: move_only(name) for name in detections}
            print(f'{args.robot_count} robot(s) have working sensors, TF and physical control.', flush=True)
            # Move only test-owned robots to collision-free marker approach poses.
            approaches = [
                ('robot1', 1.65, 3.50, math.atan2(.6, -.65), 'S1'),
                ('robot2' if args.robot_count == 2 else 'robot1', 4.1, 2.1, -math.pi / 2, 'C1'),
            ]
            config = Path(get_package_share_directory('edgeai_robot_description')) / 'config/rescue_targets.yaml'
            import yaml
            targets = yaml.safe_load(config.read_text())
            verified = {}
            for name, x, y, yaw, target in approaches:
                request = Pose(name=name)
                request.position.x, request.position.y, request.position.z = x, y, .01
                request.orientation.z, request.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                request_service('set_pose', request)
                expected_id = f"{targets['dictionary']}/{targets['targets'][target]['id']}"
                detections[name].clear()

                def observed():
                    return sum(any(item.id == expected_id for item in message.detections)
                               for message in detections[name]) >= 3

                wait_for(observed, f'{name} detecting {target} ({expected_id})', 50)
                observation = next(item for message in reversed(detections[name])
                                   for item in message.detections if item.id == expected_id)
                assert observation.header.frame_id == f'{name}/camera_optical_frame'
                pose = observation.results[0].pose.pose
                expected_distance = math.hypot(.65, .6) - .175 if target == 'S1' else .925
                np.testing.assert_allclose([pose.position.x, pose.position.y, pose.position.z],
                                           [0, -.1, expected_distance], atol=.07)
                cv2.imwrite(str(output / f'{name}_{target}_camera.png'),
                            bridge.imgmsg_to_cv2(latest[(name, 'camera/image')], desired_encoding='bgr8'))
                verified[target] = {'robot': name, 'marker': expected_id,
                                    'optical_position': [pose.position.x, pose.position.y, pose.position.z]}
            summary = {'robot_count': args.robot_count,
                       'spawn_positions': spawn_positions, 'independent_movement_m': movement,
                       'sensor_topics_per_robot': list(sensor_types), 'unique_tf_verified': True,
                       'rendered_markers_detected': verified, 'top_down_render': 'arena_top_down.png'}
            (output / 'result.json').write_text(json.dumps(summary, indent=2) + '\n')
            print(json.dumps(summary, indent=2), flush=True)
            print(f'Artifacts: {output}', flush=True)
    finally:
        for robot in detections:
            if (robot, 'camera/image') in latest:
                cv2.imwrite(str(output / f'{robot}_last_camera.png'),
                            bridge.imgmsg_to_cv2(latest[(robot, 'camera/image')], desired_encoding='bgr8'))
        if process is not None and process.poll() is None:
            os.kill(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        # The Gazebo wrapper can exit while its rendering server is still
        # shutting down. Only reap servers in this probe's exact partition/log.
        identity = {
            f'GZ_PARTITION={os.environ["GZ_PARTITION"]}'.encode(),
            f'ROS_LOG_DIR={os.environ["ROS_LOG_DIR"]}'.encode(),
        }
        for directory in Path('/proc').glob('[0-9]*'):
            try:
                command_line = (directory / 'cmdline').read_bytes().replace(b'\0', b' ')
                if not command_line.startswith(b'gz sim '):
                    continue
                environment = set((directory / 'environ').read_bytes().split(b'\0'))
                if identity <= environment:
                    os.kill(int(directory.name), signal.SIGKILL)
            except (ProcessLookupError, FileNotFoundError, PermissionError):
                continue
        executor.shutdown()
        spin_thread.join(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
