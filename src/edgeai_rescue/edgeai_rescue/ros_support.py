"""Shared ROS conversions and AMCL/TF observations, without ground truth."""

import json
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.srv import SetInitialPose
import numpy as np
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from .configuration import load_mission_config
from .geometry import Pose2D, shift_pose
from .verification import LocalizedPose


LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def yaw(quaternion):
    values = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    if not np.isfinite(values).all() or abs(np.linalg.norm(values)-1.) > .01:
        raise ValueError('Invalid quaternion')
    x, y, z, w = values
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def pose_message(values, stamp, frame='map'):
    message = PoseStamped()
    message.header.stamp, message.header.frame_id = stamp, frame
    message.pose.position.x, message.pose.position.y = map(float, values[:2])
    message.pose.orientation.z = math.sin(values[2]/2)
    message.pose.orientation.w = math.cos(values[2]/2)
    return message


def load_config(node):
    from ament_index_python.packages import get_package_share_directory
    default = str(Path(get_package_share_directory('edgeai_rescue'))/'config/mission.yaml')
    node.declare_parameter('config_file', default)
    return load_mission_config(node.get_parameter('config_file').value)


def steady_timer(node, period, callback):
    return node.create_timer(period, callback, clock=Clock(clock_type=ClockType.STEADY_TIME))


class PoseMonitor:
    def __init__(self, node, config):
        self.node, self.config = node, config
        if not node.has_parameter('control_offset_x'):
            node.declare_parameter('control_offset_x', 0.)
        self.control_offset = node.get_parameter('control_offset_x').value
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node)
        self.poses, self.scans, self.pending = {}, {}, {}
        self.clients, self.subscriptions = {}, []
        self.seed_clients, self.seed_futures = {}, {}
        for robot in config['robots']:
            self.subscriptions.extend([
                node.create_subscription(PoseWithCovarianceStamped, f'/{robot}/amcl_pose',
                                         lambda msg, name=robot: self.poses.update({name: msg}), qos_profile_sensor_data),
                node.create_subscription(LaserScan, f'/{robot}/scan',
                                         lambda msg, name=robot: self.scans.update({name: msg}), qos_profile_sensor_data),
            ])
            self.clients[robot] = node.create_client(Empty, f'/{robot}/request_nomotion_update')
            self.seed_clients[robot] = node.create_client(SetInitialPose, f'/{robot}/set_initial_pose')
        self.timer = steady_timer(node, .8, self.nomotion_updates)

    def nomotion_updates(self):
        for robot, client in self.clients.items():
            seed = self.seed_clients[robot]
            if robot not in self.seed_futures:
                if not seed.service_is_ready() or self.node.get_clock().now().nanoseconds <= 0:
                    continue
                start = self.config['robots'][robot]
                request = SetInitialPose.Request()
                request.pose.header.frame_id = self.config['map_frame']
                request.pose.header.stamp = self.node.get_clock().now().to_msg()
                request.pose.pose.pose = pose_message(
                    shift_pose([start['x'], start['y'], start['yaw']], self.control_offset),
                    request.pose.header.stamp).pose
                request.pose.pose.covariance[0] = .0025
                request.pose.pose.covariance[7] = .0025
                request.pose.pose.covariance[35] = .01
                self.seed_futures[robot] = seed.call_async(request)
                self.node.get_logger().info(f'Initializing {robot} AMCL once with a 5 cm, 0.1 rad prior')
                continue
            future = self.pending.get(robot)
            if client.service_is_ready() and (future is None or future.done()):
                self.pending[robot] = client.call_async(Empty.Request())

    def snapshot(self):
        observations = {}
        for robot in self.config['robots']:
            pose, scan = self.poses.get(robot), self.scans.get(robot)
            if (pose is None or scan is None or pose.header.frame_id != self.config['map_frame']
                    or scan.header.frame_id != f'{robot}/lidar_link'):
                continue
            try:
                tr = self.buffer.lookup_transform(self.config['map_frame'], f'{robot}/base_footprint',
                                                  Time())
                position = tr.transform.translation
                covariance = np.asarray(pose.pose.covariance).reshape(6, 6)
                if not np.isfinite(covariance).all():
                    continue
                # The largest XY covariance eigenvalue also catches correlated uncertainty.
                angle = yaw(tr.transform.rotation)
                planar = covariance[np.ix_([0, 1, 5], [0, 1, 5])]
                jacobian = np.array([[1., 0., self.control_offset*math.sin(angle)],
                                     [0., 1., -self.control_offset*math.cos(angle)]])
                variance = float(np.linalg.eigvalsh(jacobian @ planar @ jacobian.T).max())
                observations[robot] = LocalizedPose(
                    Pose2D(position.x, position.y, angle),
                    seconds(tr.header.stamp), seconds(pose.header.stamp), seconds(scan.header.stamp),
                    variance, float(covariance[5, 5]))
            except (TransformException, ValueError):
                continue
        return observations


class JsonEvidence:
    def __init__(self, directory, name):
        self.path = Path(directory)
        self.path.mkdir(parents=True, exist_ok=True)
        self.file = (self.path/name).open('a', buffering=1)

    def write(self, data):
        self.file.write(json.dumps(data, allow_nan=False, separators=(',', ':'))+'\n')

    def close(self):
        self.file.close()
