"""Save calibrated camera detections and annotated images for a fresh Part 1 run."""

from datetime import datetime, timezone
import json
from pathlib import Path
import uuid

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection3DArray


TARGETS = {1: ('S1', 'single_rescue'), 2: ('C1', 'cooperative_rescue')}


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Part1Recorder(Node):
    def __init__(self):
        super().__init__('part1_recorder')
        self.declare_parameter('output_root', 'runs/part1')
        self.root = Path(self.get_parameter('output_root').value).expanduser().resolve()
        self.bridge = CvBridge()
        self.run_dir = None
        self.started_ns = None
        self.started_wall = None
        self.images = {}
        self.pending_images = set()
        self.saved_images = set()
        self.summary = {}
        self.count = 0
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enabled_pub = self.create_publisher(Bool, '/part1/run_enabled', latch)
        self.summary_pub = self.create_publisher(String, '/part1/summary', latch)
        self.target_pub = self.create_publisher(String, '/part1/target', 10)
        self.create_service(Trigger, '/part1/start', self.start)
        self.create_subscription(Detection3DArray, '/robot1/aruco/detections', self.detect, 10)
        self.create_subscription(Image, '/robot1/aruco/debug_image', self.image,
                                 qos_profile_sensor_data)
        self.create_timer(2.0, self.write_summary)
        self.get_logger().info('Ready for /part1/start; no run data recorded yet')

    def start(self, request, response):
        if self.run_dir is not None:
            response.success = False
            response.message = 'Already started; restart the launch for a clean second run'
            return response
        self.root.mkdir(parents=True, exist_ok=True)
        label = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.run_dir = self.root / f'{label}_{uuid.uuid4().hex[:8]}'
        self.run_dir.mkdir()
        (self.run_dir / 'images').mkdir()
        (self.root / 'latest_run.txt').write_text(str(self.run_dir) + '\n')
        self.started_ns = self.get_clock().now().nanoseconds
        self.started_wall = datetime.now(timezone.utc).isoformat()
        self.images.clear()
        self.pending_images.clear()
        self.saved_images.clear()
        self.summary.clear()
        self.count = 0
        self.write_summary()
        self.enabled_pub.publish(Bool(data=True))
        response.success = True
        response.message = str(self.run_dir)
        self.get_logger().info(f'Part 1 started: {self.run_dir}')
        return response

    def detect(self, message):
        if self.run_dir is None:
            return
        for detection in message.detections:
            if not detection.results or not message.header.frame_id:
                continue
            try:
                marker_id = int(detection.results[0].hypothesis.class_id)
            except ValueError:
                continue
            if marker_id not in TARGETS:
                continue
            name, kind = TARGETS[marker_id]
            point = detection.results[0].pose.pose.position
            record = {
                'marker_id': marker_id, 'target': name, 'target_type': kind,
                'position_m': {'x': point.x, 'y': point.y, 'z': point.z},
                'frame_id': message.header.frame_id,
                'timestamp': {'sec': message.header.stamp.sec,
                              'nanosec': message.header.stamp.nanosec},
                'sim_time_s': stamp_ns(message.header.stamp) / 1e9,
                'source': 'camera solvePnP from calibrated RGB image',
            }
            with (self.run_dir / 'detections.jsonl').open('a') as stream:
                stream.write(json.dumps(record, sort_keys=True) + '\n')
            self.target_pub.publish(String(data=json.dumps(record, sort_keys=True)))
            self.count += 1
            current = self.summary.setdefault(name, {
                'marker_id': marker_id, 'target_type': kind,
                'first_detection': record, 'count': 0,
            })
            current['count'] += 1
            current['last_detection'] = record
            if name not in self.saved_images:
                key = stamp_ns(message.header.stamp)
                self.pending_images.add((name, key))
                if key in self.images:
                    self.save_image(name, key, self.images[key])
        self.write_summary()

    def image(self, message):
        if self.run_dir is None:
            return
        key = stamp_ns(message.header.stamp)
        self.images[key] = message
        if len(self.images) > 20:
            self.images.pop(next(iter(self.images)))
        for name, requested in list(self.pending_images):
            if key == requested and name not in self.saved_images:
                self.save_image(name, key, message)

    def save_image(self, name, key, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            path = self.run_dir / 'images' / f'{name}_{key}.png'
            if cv2.imwrite(str(path), image):
                self.saved_images.add(name)
                self.pending_images = {item for item in self.pending_images if item[0] != name}
                self.summary[name]['annotated_image'] = str(path.relative_to(self.run_dir))
                self.write_summary()
        except Exception as error:
            self.get_logger().warning(f'Could not save {name} image: {error}')

    def write_summary(self):
        if self.run_dir is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        report = {
            'started_wall_utc': self.started_wall,
            'started_sim_time_s': self.started_ns / 1e9,
            'elapsed_sim_time_s': max(0.0, (now_ns - self.started_ns) / 1e9),
            'total_detections': self.count,
            'targets': self.summary,
            'map_path': str(self.run_dir / 'map.yaml'),
        }
        content = json.dumps(report, indent=2, sort_keys=True)
        (self.run_dir / 'timing_summary.json').write_text(content + '\n')
        if rclpy.ok():
            self.summary_pub.publish(String(data=content))


def main(args=None):
    rclpy.init(args=args)
    node = Part1Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
