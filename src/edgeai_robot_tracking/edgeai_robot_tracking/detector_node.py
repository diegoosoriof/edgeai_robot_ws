"""ROS adapters for calibrated RGB-D detection and observer-motion-aware tracking."""

from collections import deque
from dataclasses import asdict
import json
import math
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PointStamped
import message_filters
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .camera import Calibration, depth_metres, register_depth, transform_matrix, transform_points
from .detection import DetectionConfig, detect
from .geometry_detection import GeometryConfig, detect_geometry
from .methods import validate_detection_method
from .tracking import Estimate, Measurement, SingleTargetTracker, TrackerConfig


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class RGBDTracker(Node):
    def __init__(self, **kwargs):
        super().__init__('rgbd_tracker', **kwargs)
        defaults = {
            'detection_method': 'green_seeds',
            'rgb_topic': 'camera/image', 'depth_topic': 'camera/depth_image',
            'camera_info_topic': 'camera/camera_info',
            'depth_camera_info_topic': 'camera/camera_info',
            'camera_optical_frame': 'robot1/camera_optical_frame',
            'depth_optical_frame': 'robot1/camera_optical_frame',
            # Explicit adapter for this model's Gazebo headers; not a TF relabelling.
            # Image/depth rays are reconstructed in the actual ROS optical frame.
            'expected_rgb_frame': 'robot1/base_footprint/rgbd_camera',
            'expected_depth_frame': 'robot1/base_footprint/rgbd_camera',
            'base_frame': 'robot1/base_footprint', 'tracking_frame': 'robot1/odom',
            'output_frame': 'robot1/base_footprint', 'depth_registered': True,
            'uint16_depth_scale': 0.001, 'min_depth': 0.10, 'max_depth': 9.5,
            'sync_slop': 0.025, 'sync_queue': 10, 'max_message_age': 0.5,
            'camera_info_timeout': 2.0, 'processing_hz': 10.0, 'output_delay': 0.06,
            'debug_image': True, 'debug_markers': True, 'debug_cluster': True,
        }
        for prefix, config in (('detection', DetectionConfig()), ('geometry', GeometryConfig()),
                               ('tracking', TrackerConfig())):
            for key, value in asdict(config).items():
                defaults[f'{prefix}.{key}'] = list(value) if isinstance(value, tuple) else value
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.params = {key: self.get_parameter(key).value for key in defaults}
        self.method = validate_detection_method(self.params['detection_method'])
        self.detection_config = DetectionConfig(**{
            key: self.params[f'detection.{key}'] for key in asdict(DetectionConfig())})
        self.geometry_config = GeometryConfig(**{
            key: self.params[f'geometry.{key}'] for key in asdict(GeometryConfig())})
        if self.method == 'geometry':
            self.geometry_config.validate()
        self.tracker = SingleTargetTracker(TrackerConfig(**{
            key: self.params[f'tracking.{key}'] for key in asdict(TrackerConfig())}))
        self._validate_parameters()
        self.bridge = CvBridge()
        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf, self)
        self.calibration = {}
        self.debug_rgb = deque(maxlen=self.params['sync_queue'])
        self.pending = None
        self.last_processed = -math.inf
        self.last_published = -math.inf
        self.last_clock = None
        self.input_counts = {'rgb': 0, 'depth': 0, 'synchronized': 0}
        self.candidate_count = 0
        self.rejections = {}
        self.processing_ms = 0.0
        self.candidate_details = []
        self.problem = ('waiting_for_depth_camera_info_and_tf' if self.method == 'geometry'
                        else 'waiting_for_rgb_depth_camera_info_and_tf')
        self.position_pub = self.create_publisher(PointStamped, 'tracking/position', 10)
        self.continuous_pub = self.create_publisher(
            PointStamped, 'tracking/position_continuous', 10)
        self.status_pub = self.create_publisher(DiagnosticArray, 'tracking/status', 10)
        self.debug_pub = self.create_publisher(Image, 'tracking/debug_image', 1)
        self.depth_debug_pub = None
        if self.method == 'geometry' and self.params['debug_image']:
            self.depth_debug_pub = self.create_publisher(Image, 'tracking/debug_depth_image', 1)
        self.mask_pub = (self.create_publisher(Image, 'tracking/green_mask', 1)
                         if self.method == 'green_seeds' else None)
        self.marker_pub = self.create_publisher(MarkerArray, 'tracking/markers', 1)
        self.info_subs = []
        for key, topic_key in (('rgb', 'camera_info_topic'), ('depth', 'depth_camera_info_topic')):
            if self.method == 'geometry' and key == 'rgb' and not self.params['debug_image']:
                continue
            self.info_subs.append(self.create_subscription(
                CameraInfo, self.params[topic_key],
                lambda msg, kind=key: self.calibration.__setitem__(kind, msg),
                qos_profile_sensor_data))
        self.sync = None
        self.rgb_sub = None
        if self.method == 'geometry':
            self.depth_sub = self.create_subscription(
                Image, self.params['depth_topic'], self._depth_received, qos_profile_sensor_data)
            if self.params['debug_image']:
                # Optional display input only; depth processing never waits for RGB.
                self.rgb_sub = self.create_subscription(
                    Image, self.params['rgb_topic'], self._debug_rgb_received,
                    qos_profile_sensor_data)
        else:
            self.rgb_sub = message_filters.Subscriber(
                self, Image, self.params['rgb_topic'], qos_profile=qos_profile_sensor_data)
            self.depth_sub = message_filters.Subscriber(
                self, Image, self.params['depth_topic'], qos_profile=qos_profile_sensor_data)
            self.rgb_sub.registerCallback(lambda msg: self._count('rgb'))
            self.depth_sub.registerCallback(lambda msg: self._count('depth'))
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.rgb_sub, self.depth_sub], self.params['sync_queue'], self.params['sync_slop'])
            self.sync.registerCallback(self._synchronized)
        self.timer = self.create_timer(1.0 / self.params['processing_hz'], self._tick,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"Method {self.method}: tracking visible surfaces in {self.params['tracking_frame']}; "
            f"output {self.params['output_frame']}. No target pose or identity inputs.")

    def _validate_parameters(self):
        positive = ('processing_hz', 'sync_slop', 'sync_queue', 'max_message_age',
                    'camera_info_timeout', 'uint16_depth_scale', 'tracking.timeout',
                    'tracking.confirmation_hits', 'tracking.measurement_std',
                    'tracking.acceleration_std', 'tracking.association_distance',
                    'tracking.max_speed', 'tracking.mahalanobis_gate',
                    'detection.cluster_stride', 'detection.region_radius',
                    'detection.connect_distance', 'detection.min_seed_depths',
                    'detection.seed_depth_band', 'detection.min_visible_width',
                    'detection.min_visible_height', 'detection.max_visible_width',
                    'detection.max_visible_diagonal',
                    'detection.min_cluster_points', 'detection.max_cluster_pixels',
                    'detection.max_candidates', 'detection.min_green_area')
        for key in positive:
            if self.method == 'geometry' and key.startswith('detection.'):
                continue
            if not math.isfinite(self.params[key]) or self.params[key] <= 0:
                raise ValueError(f'{key} must be positive')
        if not 0 < self.params['min_depth'] < self.params['max_depth']:
            raise ValueError('Require 0 < min_depth < max_depth')
        if not 0 <= self.params['output_delay'] < self.params['tracking.timeout']:
            raise ValueError('output_delay must be nonnegative and shorter than tracking.timeout')
        frames = ('base_frame', 'tracking_frame', 'output_frame',
                  'depth_optical_frame', 'expected_depth_frame')
        if self.method == 'green_seeds':
            frames += ('camera_optical_frame', 'expected_rgb_frame')
        for key in frames:
            if not self.params[key] or self.params[key].startswith('/'):
                raise ValueError(f'{key} must be a nonempty TF frame without a leading slash')
        if self.method == 'geometry':
            return  # Inactive colour-detector settings have no role in geometry mode.
        cfg = self.detection_config
        if (len(cfg.hsv_lower) != 3 or len(cfg.hsv_upper) != 3
                or any(not 0 <= lo <= hi <= cap for lo, hi, cap
                       in zip(cfg.hsv_lower, cfg.hsv_upper, (179, 255, 255)))):
            raise ValueError('Invalid OpenCV HSV limits')
        if (not 0 < cfg.min_seed_fraction <= 1 or not 0 < cfg.duplicate_overlap <= 1
                or cfg.floor_clearance < 0 or cfg.seed_min_height > cfg.seed_max_height
                or cfg.seed_max_height > cfg.max_height or cfg.morphology_size < 0):
            raise ValueError('Invalid detection geometry/fraction parameters')

    def _count(self, key):
        self.input_counts[key] += 1

    def _synchronized(self, rgb, depth):
        self._count('synchronized')
        # Keep the newest pair only. Processing and TF waiting never block subscriptions.
        if (self.pending is None
                or seconds(rgb.header.stamp) > seconds(self.pending[0].header.stamp)):
            self.pending = (rgb, depth)

    def _depth_received(self, depth):
        self._count('depth')
        if (self.pending is None
                or seconds(depth.header.stamp) > seconds(self.pending[1].header.stamp)):
            self.pending = (None, depth)

    def _debug_rgb_received(self, rgb):
        self._count('rgb')
        self.debug_rgb.append(rgb)

    def _camera(self, kind, image):
        info = self.calibration.get(kind)
        if info is None:
            raise ValueError(f'Waiting for {kind} CameraInfo')
        optical = self.params['camera_optical_frame' if kind == 'rgb' else 'depth_optical_frame']
        expected = self.params[f'expected_{kind}_frame']
        if image.header.frame_id not in (expected, optical):
            raise ValueError(f'Unexpected {kind} image frame: {image.header.frame_id}')
        if info.header.frame_id not in (expected, optical):
            raise ValueError(f'Unexpected {kind} CameraInfo frame: {info.header.frame_id}')
        if (info.width, info.height) != (image.width, image.height):
            raise ValueError(f'{kind} image/CameraInfo dimensions differ')
        if (info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset
                or info.roi.width not in (0, info.width)
                or info.roi.height not in (0, info.height)):
            raise ValueError('Use CameraInfo for the current image grid; ROI/binning unsupported')
        if info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
            raise ValueError(f'Unsupported distortion model: {info.distortion_model}')
        info_age = abs(seconds(image.header.stamp) - seconds(info.header.stamp))
        if seconds(info.header.stamp) and info_age > self.params['camera_info_timeout']:
            raise ValueError(f'Stale {kind} CameraInfo')
        return Calibration(info.width, info.height, info.k, info.d)

    def _lookup(self, target, source, stamp):
        if target == source:
            return np.eye(4)
        return transform_matrix(self.tf.lookup_transform(target, source, stamp).transform)

    def _process(self, rgb, depth_msg):
        started = time.perf_counter()
        if self.method == 'geometry':
            self._process_geometry(depth_msg, started)
            return
        rgb_camera, depth_camera = self._camera('rgb', rgb), self._camera('depth', depth_msg)
        stamp = Time.from_msg(rgb.header.stamp)
        optical = self.params['camera_optical_frame']
        camera_to_base = self._lookup(self.params['base_frame'], optical, stamp)
        camera_to_tracking = self._lookup(self.params['tracking_frame'], optical, stamp)
        base_to_tracking = camera_to_tracking @ np.linalg.inv(camera_to_base)
        if base_to_tracking[2, 2] < math.cos(math.radians(15)):
            raise ValueError('Observer is tilted: flat-floor mounting model is invalid')
        bgr = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
        depth = depth_metres(
            self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'),
            depth_msg.encoding, self.params['min_depth'], self.params['max_depth'],
            self.params['uint16_depth_scale'])
        if self.params['depth_registered']:
            if (rgb.width != depth_msg.width or rgb.height != depth_msg.height
                    or not np.allclose(rgb_camera.k, depth_camera.k)
                    or rgb_camera.d.shape != depth_camera.d.shape
                    or not np.allclose(rgb_camera.d, depth_camera.d)
                    or rgb.header.frame_id != depth_msg.header.frame_id
                    or optical != self.params['depth_optical_frame']):
                raise ValueError('Registered depth requires a shared pixel grid/calibration/frame; '
                                 'otherwise configure depth_registered=false and depth calibration')
        else:
            extrinsic = self.tf.lookup_transform_full(
                optical, stamp, self.params['depth_optical_frame'],
                Time.from_msg(depth_msg.header.stamp), self.params['tracking_frame'])
            depth = register_depth(depth, depth_camera, rgb_camera,
                                   transform_matrix(extrinsic.transform))
        candidates, mask, self.rejections = detect(
            bgr, depth, rgb_camera, camera_to_base, self.detection_config)
        self._finish_detection(candidates, base_to_tracking, rgb.header, started,
                               bgr, mask)

    def _process_geometry(self, depth_msg, started):
        # Native depth calibration/frame/timestamp; no RGB or registration path.
        camera = self._camera('depth', depth_msg)
        stamp = Time.from_msg(depth_msg.header.stamp)
        optical = self.params['depth_optical_frame']
        camera_to_base = self._lookup(self.params['base_frame'], optical, stamp)
        camera_to_tracking = self._lookup(self.params['tracking_frame'], optical, stamp)
        base_to_tracking = camera_to_tracking @ np.linalg.inv(camera_to_base)
        if base_to_tracking[2, 2] < math.cos(math.radians(15)):
            raise ValueError('Observer is tilted: flat-floor mounting model is invalid')
        depth = depth_metres(
            self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'),
            depth_msg.encoding, self.params['min_depth'], self.params['max_depth'],
            self.params['uint16_depth_scale'])
        candidates, _, self.rejections = detect_geometry(
            depth, camera, camera_to_base, self.geometry_config)
        self._finish_detection(candidates, base_to_tracking, depth_msg.header, started,
                               depth=depth)

    def _finish_detection(self, candidates, base_to_tracking, header, started,
                          bgr=None, mask=None, depth=None):
        stamp = Time.from_msg(header.stamp)
        self.candidate_count = len(candidates)
        self.candidate_details = [dict(index=i, quality=c.quality, **c.metrics)
                                  for i, c in enumerate(candidates)]
        measurements = [Measurement(transform_points(c.position, base_to_tracking),
                                    c.quality, c.extent, i) for i, c in enumerate(candidates)]
        estimate = self.tracker.update(seconds(header.stamp), measurements)
        self.problem = ''
        self.last_processed = seconds(header.stamp)
        self.processing_ms = (time.perf_counter() - started) * 1000
        selected = (candidates[estimate.selected_index]
                    if estimate.selected_index is not None else None)
        self._publish(estimate, stamp, selected)
        if self.params['debug_image']:
            if self.method == 'geometry':
                self._geometry_debug_image(depth, candidates, selected, estimate, header)
            else:
                self._debug_image(bgr, mask, candidates, selected, estimate, header)

    def _tick(self):
        now = self.get_clock().now()
        now_seconds = now.nanoseconds * 1e-9
        if self.last_clock is not None and now_seconds < self.last_clock - 1e-6:
            self.tracker.reset()
            self.pending = None
            self.debug_rgb.clear()
            self.last_processed = self.last_published = -math.inf
            if self.sync is not None:
                for queue in self.sync.queues:
                    queue.clear()
            self.problem = 'simulation_clock_reset'
        self.last_clock = now_seconds
        if self.pending is not None:
            rgb, depth = self.pending
            image = depth if self.method == 'geometry' else rgb
            age = now_seconds - seconds(image.header.stamp)
            if age > self.params['max_message_age'] or age < -self.params['sync_slop']:
                self.pending = None
                self.problem = 'stale_or_future_images'
            elif seconds(image.header.stamp) <= self.last_processed:
                self.pending = None
            else:
                try:
                    self._process(rgb, depth)
                    self.pending = None
                    return
                except TransformException as error:
                    # Keep the pair until timestamped TF arrives, bounded by max_message_age.
                    self.problem = f'waiting_for_timestamped_tf: {error}'
                except (ValueError, CvBridgeError, cv2.error) as error:
                    self.problem = str(error)
                    self.pending = None
        # Non-mutating prediction lets a delayed sensor measurement still update its own time.
        stamp_seconds = max(self.last_processed, now_seconds - self.params['output_delay'])
        if not math.isfinite(stamp_seconds) or stamp_seconds < 0:
            stamp_seconds = now_seconds
        stamp = Time(seconds=stamp_seconds, clock_type=ClockType.ROS_TIME)
        estimate = self.tracker.estimate(stamp_seconds)
        self._publish(estimate, stamp)

    def _publish(self, estimate, stamp, selected=None):
        valid = estimate.position is not None
        output = None
        if valid:
            try:
                output = transform_points(estimate.position, self._lookup(
                    self.params['output_frame'], self.params['tracking_frame'], stamp))
            except (TransformException, ValueError) as error:
                valid = False
                self.problem = f'output_tf_unavailable: {error}'
        # Also enforce expiry at the current clock, not just at a delayed output stamp.
        if (self.tracker.last_measurement is not None
                and self.get_clock().now().nanoseconds * 1e-9
                - self.tracker.last_measurement >= self.tracker.config.timeout):
            valid = False
            estimate = Estimate('lost', reason='timeout', track_id=estimate.track_id)
        if valid and stamp.nanoseconds * 1e-9 > self.last_published:
            point = PointStamped()
            point.header.stamp = stamp.to_msg()
            point.header.frame_id = self.params['output_frame']
            point.point = Point(x=float(output[0]), y=float(output[1]), z=float(output[2]))
            self.position_pub.publish(point)
            point.header.frame_id = self.params['tracking_frame']
            point.point = Point(x=float(estimate.position[0]), y=float(estimate.position[1]),
                                z=float(estimate.position[2]))
            self.continuous_pub.publish(point)
            self.last_published = stamp.nanoseconds * 1e-9
        self._status(estimate, stamp, valid)
        if self.params['debug_markers']:
            self._markers(estimate, stamp, output if valid else None, selected if valid else None)

    def _status(self, estimate, stamp, valid):
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = stamp.to_msg()
        status = DiagnosticStatus()
        status.name = f'{self.get_namespace()}/robot_surface_tracker'
        status.hardware_id = 'rgbd'
        status.level = (DiagnosticStatus.OK if valid and estimate.state == 'measured'
                        else DiagnosticStatus.WARN)
        status.message = estimate.state
        values = {
            'detection_method': self.method,
            'processing_duration_ms': f'{self.processing_ms:.3f}',
            'candidate_details': json.dumps(self.candidate_details, sort_keys=True),
            'valid': str(valid).lower(), 'state': estimate.state,
            'reason': self.problem or estimate.reason, 'track_id': str(estimate.track_id),
            'quality_score': f'{estimate.confidence:.3f}',
            'measurement_age_s': f'{estimate.measurement_age:.3f}',
            'last_processed_image_s': f'{self.last_processed:.6f}',
            'candidate_count': str(self.candidate_count),
            'rejections': json.dumps(self.rejections, sort_keys=True),
            'input_counts': json.dumps(self.input_counts, sort_keys=True),
            'tracking_frame': self.params['tracking_frame'],
            'output_frame': self.params['output_frame'],
            'position_semantics': 'visible_surface_centroid_not_base_center',
        }
        if estimate.covariance is not None:
            values['tracking_std_xyz_m'] = str(np.sqrt(np.diag(estimate.covariance)).tolist())
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        diagnostics.status = [status]
        self.status_pub.publish(diagnostics)

    def _debug_image(self, bgr, mask, candidates, selected, estimate, header):
        image = bgr.copy()
        image[mask > 0] = (0.5 * image[mask > 0] + np.array([0, 127, 0])).astype(np.uint8)
        for candidate in candidates:
            x1, y1, x2, y2 = map(int, candidate.bbox)
            color = (0, 255, 0) if candidate is selected else (0, 200, 255)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 1)
        text = f'{self.method} | {estimate.state} | {len(candidates)} candidates | surface position'
        cv2.putText(image, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        debug = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        debug.header = header
        self.debug_pub.publish(debug)
        green = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        green.header = header
        self.mask_pub.publish(green)

    def _geometry_rgb_view(self, candidates, depth_header):
        """Optional RGB display, projecting native-depth surfaces into its own grid.

        RGB/calibration/TF failure affects this view only. Candidates, qualities,
        filtering and position timestamps have already been computed from depth.
        """
        if not self.debug_rgb:
            return None
        rgb = min(self.debug_rgb,
                  key=lambda msg: abs(seconds(msg.header.stamp) - seconds(depth_header.stamp)))
        if abs(seconds(rgb.header.stamp) - seconds(depth_header.stamp)) > self.params['sync_slop']:
            return None
        try:
            calibration = self._camera('rgb', rgb)
            extrinsic = self.tf.lookup_transform_full(
                self.params['camera_optical_frame'], Time.from_msg(rgb.header.stamp),
                self.params['base_frame'], Time.from_msg(depth_header.stamp),
                self.params['tracking_frame'])
            base_to_rgb = transform_matrix(extrinsic.transform)
            image = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8').copy()
            boxes = []
            for index, candidate in enumerate(candidates):
                points = transform_points(candidate.points, base_to_rgb)
                points = points[np.isfinite(points).all(axis=1) & (points[:, 2] > 0)]
                if not len(points):
                    continue
                pixels = calibration.project(points)
                pixels = pixels[np.isfinite(pixels).all(axis=1)
                                & (pixels[:, 0] >= 0) & (pixels[:, 0] < rgb.width)
                                & (pixels[:, 1] >= 0) & (pixels[:, 1] < rgb.height)]
                if len(pixels):
                    low = np.floor(pixels.min(axis=0)).astype(int)
                    high = np.minimum(np.ceil(pixels.max(axis=0)).astype(int),
                                      [rgb.width - 1, rgb.height - 1])
                    boxes.append((index, candidate, (*low, *high)))
            return image, boxes, rgb.header
        except (TransformException, ValueError, CvBridgeError, cv2.error):
            return None

    def _geometry_debug_image(self, depth, candidates, selected, estimate, header):
        def publish_view(image, boxes, image_header, background, publisher):
            for index, candidate, bbox in boxes:
                x1, y1, x2, y2 = map(int, bbox)
                color = (0, 255, 0) if candidate is selected else (0, 200, 255)
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 1)
                cv2.putText(image, f'#{index} score={candidate.quality:.2f}',
                            (x1, max(12, y1 - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            text = (f'geometry | {background} | {estimate.state} | {len(candidates)} candidates | '
                    f'{self.processing_ms:.1f} ms')
            cv2.putText(image, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 255), 1)
            debug = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            debug.header = image_header
            publisher.publish(debug)

        # Stable topic meanings: never replace an RGB frame with false-colour depth.
        # Depth debug remains available independently, including when RGB stops.
        valid = np.isfinite(depth)
        scaled = np.zeros(depth.shape, np.uint8)
        scaled[valid] = (255 * (1 - np.clip(depth[valid] / self.params['max_depth'], 0, 1)))
        image = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
        image[~valid] = 0
        publish_view(image, [(i, c, c.bbox) for i, c in enumerate(candidates)],
                     header, 'depth', self.depth_debug_pub)
        view = self._geometry_rgb_view(candidates, header)
        if view is not None:
            rgb, boxes, rgb_header = view
            publish_view(rgb, boxes, rgb_header, 'RGB', self.debug_pub)

    def _markers(self, estimate, stamp, output, selected):
        clear = Marker(action=Marker.DELETEALL)
        markers = [clear]
        if output is not None:
            marker = Marker()
            marker.header.frame_id = self.params['output_frame']
            marker.header.stamp = stamp.to_msg()
            marker.ns, marker.id, marker.type = 'surface_estimate', 0, Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=float(output[0]), y=float(output[1]), z=float(output[2]))
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.10
            marker.color.a = 0.9
            marker.color.r = 1.0 if estimate.state == 'predicted' else 0.1
            marker.color.g = 0.8
            marker.lifetime = Duration(seconds=0.3).to_msg()
            markers.append(marker)
        if selected is not None and self.params['debug_cluster']:
            cluster = Marker()
            cluster.header.frame_id = self.params['base_frame']
            cluster.header.stamp = stamp.to_msg()
            cluster.ns, cluster.id, cluster.type = 'selected_surface', 1, Marker.POINTS
            cluster.action = Marker.ADD
            cluster.pose.orientation.w = 1.0
            cluster.scale.x = cluster.scale.y = 0.012
            cluster.color.b, cluster.color.g, cluster.color.a = 1.0, 0.7, 0.8
            cluster.lifetime = Duration(seconds=0.3).to_msg()
            cluster.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                              for p in selected.points[::max(1, len(selected.points) // 1500)]]
            markers.append(cluster)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RGBDTracker()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass  # launch may forward a second interrupt during cleanup
        try:
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
