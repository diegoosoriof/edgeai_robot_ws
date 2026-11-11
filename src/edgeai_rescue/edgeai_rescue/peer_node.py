"""Publish peer footprint observations and fail-closed localization locks."""

import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

from .geometry import Pose2D, perimeter_points, polygon_at
from .ros_support import LATCHED, load_config, steady_timer


class PeerNode(Node):
    def __init__(self):
        super().__init__('rescue_peer_obstacles')
        self.config = load_config(self)
        self.declare_parameter('footprint', [-.23, -.17, .20, -.17, .20, .17, -.23, .17])
        self.footprint = np.asarray(self.get_parameter('footprint').value).reshape(-1, 2)
        self.status, self.progress = {}, {}
        self.clouds, self.locks, self.stops = {}, {}, {}
        for robot in self.config['robots']:
            self.clouds[robot] = self.create_publisher(PointCloud2, f'/{robot}/peer_obstacles', 1)
            self.locks[robot] = self.create_publisher(Bool, f'/{robot}/localization_stop', 1)
            self.stops[robot] = self.create_publisher(Twist, f'/{robot}/mux_cmd_vel', 1)
        self.verification_sub = self.create_subscription(String, '/rescue/verification',
            lambda msg: setattr(self, 'status', json.loads(msg.data)), LATCHED)
        self.progress_sub = self.create_subscription(String, '/rescue/progress',
            lambda msg: setattr(self, 'progress', json.loads(msg.data)), LATCHED)
        self.timer = steady_timer(self, .1, self.tick)

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        localized = self.status.get('localized', {})
        healthy = (0 <= now-self.status.get('time', -1e6) < .5
                   and all(localized.get(robot, False) for robot in self.config['robots'])
                   and 0 <= now-self.progress.get('time', -1e6) < .5)
        poses = self.status.get('poses', {})
        for robot in self.config['robots']:
            self.locks[robot].publish(Bool(data=not healthy))
            if not healthy:
                self.stops[robot].publish(Twist())
            if not all(name in poses for name in self.config['robots']):
                continue
            other = next(name for name in self.config['robots'] if name != robot)
            peer = poses[other]
            vertices = polygon_at(Pose2D(peer['x'], peer['y'], peer['yaw']), self.footprint, padding=.02)
            points = perimeter_points(vertices)
            xyz = np.column_stack([points, np.full(len(points), .1)]).astype('<f4')
            message = PointCloud2()
            message.header.frame_id = self.config['map_frame']
            stamp = min(poses[robot]['tf_stamp'], peer['tf_stamp'])
            message.header.stamp.sec = int(stamp)
            message.header.stamp.nanosec = int((stamp-int(stamp))*1e9)
            message.height, message.width, message.point_step, message.row_step = 1, len(xyz), 12, 12*len(xyz)
            message.fields = [PointField(name=axis, offset=offset, datatype=PointField.FLOAT32, count=1)
                              for axis, offset in (('x', 0), ('y', 4), ('z', 8))]
            message.is_dense, message.data = True, xyz.tobytes()
            self.clouds[robot].publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PeerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
