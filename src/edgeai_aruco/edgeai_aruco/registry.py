"""Remember uniquely identified markers in a fixed TF frame for one session."""

from collections import Counter, deque
from copy import deepcopy
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose, do_transform_pose_with_covariance_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from .detection import positive_float


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def transform_detection(detection, transform):
    """Transform pose, covariance, and box; keep the observation timestamp."""
    result = deepcopy(detection)
    result.header.frame_id = transform.header.frame_id
    result.bbox.center = do_transform_pose(detection.bbox.center, transform)
    for hypothesis in result.results:
        stamped_pose = PoseWithCovarianceStamped(header=detection.header, pose=hypothesis.pose)
        hypothesis.pose = do_transform_pose_with_covariance_stamped(stamped_pose, transform).pose
    return result


def registry_visualization(snapshot):
    """Persistent RViz panels and labels, keyed by the registry's stable IDs."""
    if not snapshot.detections:
        return MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
    markers = []
    for detection in snapshot.detections:
        # Latest TF keeps late subscribers independent of old observation times.
        # The actual last-seen times remain in /aruco/known_markers.
        header = Header(frame_id=snapshot.header.frame_id)
        panel = Marker(
            header=header, ns=detection.id, id=0, type=Marker.CUBE,
            action=Marker.ADD, pose=deepcopy(detection.bbox.center), frame_locked=True,
        )
        panel.scale.x = detection.bbox.size.x
        panel.scale.y = detection.bbox.size.y
        panel.scale.z = 0.005
        panel.color.r, panel.color.g, panel.color.b, panel.color.a = 0.1, 0.9, 0.4, 0.8
        label = Marker(
            header=header, ns=detection.id, id=1, type=Marker.TEXT_VIEW_FACING,
            action=Marker.ADD, text=detection.id, frame_locked=True,
        )
        label.pose.position = deepcopy(detection.bbox.center.position)
        label.pose.position.z += panel.scale.y / 2 + 0.1
        label.pose.orientation.w = 1.0
        label.scale.z = 0.08
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        # Default zero lifetime keeps both objects visible while unobserved.
        markers.extend((panel, label))
    return MarkerArray(markers=markers)


class MarkerRegistry:
    def __init__(self, frame):
        self.frame = frame
        self.markers = {}

    def update(self, detection):
        if not detection.id or not detection.results or detection.header.frame_id != self.frame:
            raise ValueError('Registry entries need an ID, a pose, and the configured fixed frame')
        previous = self.markers.get(detection.id)
        if previous is not None and stamp_ns(detection.header.stamp) <= stamp_ns(previous.header.stamp):
            return False
        self.markers[detection.id] = deepcopy(detection)
        return True

    def snapshot(self, stamp):
        # Per-marker header stamps remain last_seen, even while invisible.
        return Detection3DArray(
            header=Header(stamp=stamp, frame_id=self.frame),
            detections=[deepcopy(self.markers[key]) for key in sorted(self.markers)],
        )


class MarkerRegistryNode(Node):
    def __init__(self, **kwargs):
        super().__init__('aruco_registry', **kwargs)
        defaults = {
            'detections_topic': 'aruco/detections',
            'known_markers_topic': 'aruco/known_markers',
            'visualization_topic': 'aruco/known_markers_visualization',
            'registry_frame': 'odom',
            'transform_timeout': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        settings = {name: self.get_parameter(name).value for name in defaults}
        if not settings['registry_frame'] or settings['registry_frame'].startswith('/'):
            raise ValueError('registry_frame must name a fixed TF frame without a leading slash')
        self.transform_timeout = positive_float(settings['transform_timeout'], 'transform_timeout')
        self.registry = MarkerRegistry(settings['registry_frame'])
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Bound retries when image observations arrive just before their TF.
        self.pending = deque(maxlen=30)
        self.publisher = self.create_publisher(
            Detection3DArray, settings['known_markers_topic'],
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.visualization_publisher = self.create_publisher(
            MarkerArray, settings['visualization_topic'],
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.subscription = self.create_subscription(
            Detection3DArray, settings['detections_topic'], self.on_detections, 1)
        self.last_clock = self.get_clock().now().nanoseconds
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.retry_timer = self.create_timer(0.05, self.retry_pending, clock=self.steady_clock)
        self.publish_snapshot()
        self.get_logger().info(
            f'Remembering markers in {self.registry.frame} on {self.publisher.topic_name}')

    def warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def check_clock(self):
        now = self.get_clock().now().nanoseconds
        if now < self.last_clock:
            # A simulation/ROS time reset starts a new session in the fixed frame.
            self.registry.markers.clear()
            self.pending.clear()
            self.publish_snapshot()
        self.last_clock = now

    def on_detections(self, message):
        self.check_clock()
        if message.detections:
            if not message.header.frame_id or stamp_ns(message.header.stamp) <= 0:
                self.warn('Skipping observations without a source frame and nonzero image timestamp')
                return
            if not self.store_observations(message):
                if len(self.pending) == self.pending.maxlen:
                    self.warn('TF retry queue full; dropping its oldest observation')
                self.pending.append((time.monotonic() + self.transform_timeout, message))
        self.publish_snapshot()

    def store_observations(self, message):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.registry.frame, message.header.frame_id, Time.from_msg(message.header.stamp))
        except TransformException:
            return False
        counts = Counter(detection.id for detection in message.detections)
        for detection in message.detections:
            if (not detection.id or not detection.results or counts[detection.id] != 1
                    or detection.header != message.header):
                self.warn('Skipping ambiguous marker IDs or observations with inconsistent headers')
                continue
            self.registry.update(transform_detection(detection, transform))
        return True

    def retry_pending(self):
        self.check_clock()
        pending = self.pending
        self.pending = deque(maxlen=pending.maxlen)
        updated = False
        for deadline, message in pending:
            if time.monotonic() >= deadline:
                self.warn(f'Dropping observation: timestamped TF to {self.registry.frame} unavailable')
            elif self.store_observations(message):
                updated = True
            else:
                self.pending.append((deadline, message))
        if updated:
            self.publish_snapshot()

    def publish_snapshot(self):
        snapshot = self.registry.snapshot(self.get_clock().now().to_msg())
        self.publisher.publish(snapshot)
        self.visualization_publisher.publish(registry_visualization(snapshot))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MarkerRegistryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
