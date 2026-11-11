"""Independent map-frame rescue verification and demonstration evidence."""

import json
import math
from pathlib import Path

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .ros_support import JsonEvidence, LATCHED, PoseMonitor, load_config, steady_timer
from .verification import RescueVerifier


class VerifierNode(Node):
    def __init__(self):
        super().__init__('rescue_verifier')
        self.config = load_config(self)
        self.declare_parameter('evidence_dir', 'rescue_evidence')
        self.evidence = JsonEvidence(self.get_parameter('evidence_dir').value, 'verification.jsonl')
        self.monitor = PoseMonitor(self, self.config)
        self.verifier = RescueVerifier(self.config)
        self.run_id, self.state = '', 'INITIALIZING'
        self.last_sample, self.last_log = None, None
        self.ready_counts = {robot: 0 for robot in self.config['robots']}
        self.announced = set()
        self.status_pub = self.create_publisher(String, '/rescue/verification', LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, '/rescue/markers', LATCHED)
        self.target_sub = self.create_subscription(String, '/rescue/active_target', self.activate, LATCHED)
        self.state_sub = self.create_subscription(String, '/rescue/state',
                                                  lambda msg: setattr(self, 'state', msg.data), LATCHED)
        self.timer = steady_timer(self, .1, self.tick)

    def activate(self, message):
        request = json.loads(message.data)
        if request['run_id'] != self.run_id:
            self.verifier.reset()
            self.run_id, self.announced = request['run_id'], set()
        self.verifier.activate(request['target'])

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if now <= 0:
            return
        poses = self.monitor.snapshot()
        result = self.verifier.sample(now, poses)
        if self.last_sample is not None and now < self.last_sample:
            self.ready_counts = {robot: 0 for robot in self.config['robots']}
            self.announced.clear()
        for robot, valid in result['localized'].items():
            if now != self.last_sample:
                self.ready_counts[robot] = self.ready_counts[robot]+1 if valid else 0
            result['localized'][robot] = valid and self.ready_counts[robot] >= self.config['localization']['ready_samples']
        result['run_id'] = self.run_id
        result['poses'] = {robot: {'x': item.pose.x, 'y': item.pose.y, 'yaw': item.pose.yaw,
                                   'tf_stamp': item.tf_stamp, 'xy_variance': item.xy_variance,
                                   'yaw_variance': item.yaw_variance}
                           for robot, item in poses.items() if item.pose.finite}
        self.status_pub.publish(String(data=json.dumps(result, allow_nan=False)))
        self.marker_pub.publish(self.markers(result))
        if now != self.last_sample and self.run_id:
            self.evidence.write(result)
        for target, check in result['targets'].items():
            if check['completed'] and target not in self.announced:
                self.announced.add(target)
                self.get_logger().info(f"sim={now:.3f} {target} completed: distances={check['distances']}, "
                                       f"continuous_hold={check['held_seconds']:.3f}s")
                summary = {'run_id': self.run_id, 'source': 'AMCL map->base_footprint TF; no Gazebo ground truth',
                           'time': now, 'targets': result['targets']}
                (self.evidence.path/'verification_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        if self.last_log is None or now < self.last_log or now-self.last_log >= 1.:
            self.last_log = now
            current = result['targets'].get(self.verifier.active)
            if current:
                self.get_logger().info(f"sim={now:.3f} {self.verifier.active}: distances={current['distances']}, "
                                       f"hold={current['held_seconds']:.2f}s, eligible={current['eligible']}")
        self.last_sample = now

    def markers(self, result):
        array = MarkerArray()
        for index, (name, target) in enumerate(self.config['targets'].items()):
            check = result['targets'][name]
            circle = Marker()
            circle.header.frame_id = self.config['map_frame']
            circle.ns, circle.id, circle.type = 'rescue_zones', index, Marker.LINE_STRIP
            circle.pose.orientation.w = 1.
            circle.scale.x = .025
            circle.color.a = .9
            if check['completed']:
                circle.color.g = 1.
            elif name == 'S1':
                circle.color.r = 1.
            else:
                circle.color.b = 1.
                circle.color.g = .5
            x, y = target['center']
            circle.points = [Point(x=x+target['radius']*math.cos(theta),
                                   y=y+target['radius']*math.sin(theta), z=.02)
                             for theta in (step*2*math.pi/80 for step in range(81))]
            array.markers.append(circle)
            center = Marker()
            center.header.frame_id = self.config['map_frame']
            center.ns, center.id, center.type = 'rescue_centers', index, Marker.SPHERE
            center.pose.position.x, center.pose.position.y, center.pose.position.z = x, y, .06
            center.pose.orientation.w = 1.
            center.scale.x = center.scale.y = center.scale.z = .08
            center.color = circle.color
            array.markers.append(center)
            label = Marker()
            label.header.frame_id = self.config['map_frame']
            label.ns, label.id, label.type = 'rescue_labels', index, Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, .7
            label.pose.orientation.w = 1.
            label.scale.z, label.color.r, label.color.g, label.color.b, label.color.a = .13, 1., 1., 1., 1.
            label.text = f"{name}: {'COMPLETE' if check['completed'] else f'{check["held_seconds"]:.1f}/10 s'}"
            array.markers.append(label)
        text = Marker()
        text.header.frame_id = self.config['map_frame']
        text.ns, text.id, text.type = 'mission_state', 0, Marker.TEXT_VIEW_FACING
        text.pose.position.x, text.pose.position.y, text.pose.position.z = 2.5, 5.35, .4
        text.pose.orientation.w = 1.
        text.scale.z, text.color.r, text.color.g, text.color.b, text.color.a = .18, 1., 1., 1., 1.
        text.text = f'{self.state}\nS1: {result["targets"]["S1"]["held_seconds"]:.1f}s  C1: {result["targets"]["C1"]["held_seconds"]:.1f}s'
        array.markers.append(text)
        return array


def main(args=None):
    rclpy.init(args=args)
    node = VerifierNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.evidence.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
