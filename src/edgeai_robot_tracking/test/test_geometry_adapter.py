"""Exercise the real geometry adapter without DDS or any RGB dependency."""

from types import MethodType, SimpleNamespace

from cv_bridge import CvBridge
import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo

from edgeai_robot_tracking.camera import depth_metres
from edgeai_robot_tracking.detector_node import RGBDTracker
from edgeai_robot_tracking.geometry_detection import GeometryConfig
from edgeai_robot_tracking.tracking import Measurement, SingleTargetTracker, TrackerConfig
from test_geometry_detection import geometry_scene


def test_geometry_native_depth_timestamps_and_results_ignore_rgb(monkeypatch):
    image, depth, camera, transform = geometry_scene()
    bridge = CvBridge()
    depth_msg = bridge.cv2_to_imgmsg(depth, encoding='32FC1')
    depth_msg.header.frame_id = 'depth_header_alias'
    depth_msg.header.stamp.sec = 12
    depth_msg.header.stamp.nanosec = 345
    info = CameraInfo(width=camera.width, height=camera.height,
                      k=camera.k.ravel().tolist(), distortion_model='plumb_bob')
    info.header = depth_msg.header
    calls, results = [], []

    def lookup(target, source, stamp):
        calls.append((target, source, stamp.nanoseconds))
        return transform

    def finish(candidates, base_to_tracking, header, started, **kwargs):
        results.append((candidates, header, kwargs['depth']))

    node = SimpleNamespace(
        method='geometry', bridge=bridge, geometry_config=GeometryConfig(),
        calibration={'depth': info}, rejections={}, _lookup=lookup, _finish_detection=finish,
        params={'depth_optical_frame': 'native_depth_optical',
                'expected_depth_frame': 'depth_header_alias', 'camera_info_timeout': 2.,
                'base_frame': 'base', 'tracking_frame': 'odom',
                'min_depth': .1, 'max_depth': 9.5, 'uint16_depth_scale': .001,
                # Intentionally incompatible RGB setup: geometry must ignore it.
                'depth_registered': True, 'camera_optical_frame': 'different_rgb_frame'})
    node._camera = MethodType(RGBDTracker._camera, node)
    node._process_geometry = MethodType(RGBDTracker._process_geometry, node)

    def forbidden(*args, **kwargs):
        raise AssertionError('Geometry called a colour/registration path')

    monkeypatch.setattr('edgeai_robot_tracking.detector_node.detect', forbidden)
    monkeypatch.setattr('edgeai_robot_tracking.detector_node.register_depth', forbidden)
    for color in (None, [0, 255, 0], [0, 0, 255], [140, 140, 140]):
        rgb = None
        if color is not None:
            image[:] = color
            # Different pixel grid, timestamp and no RGB calibration available.
            rgb = bridge.cv2_to_imgmsg(image[:60, :80], encoding='bgr8')
            rgb.header.stamp.sec = 900
        RGBDTracker._process(node, rgb, depth_msg)
    assert len(results) == 4
    expected = results[0][0]
    assert len(expected) == 1
    for candidates, header, prepared_depth in results:
        assert header.stamp == depth_msg.header.stamp
        assert len(candidates) == len(expected)
        for candidate, reference in zip(candidates, expected):
            np.testing.assert_array_equal(candidate.points, reference.points)
            assert candidate.quality == reference.quality
            assert candidate.metrics == reference.metrics
        np.testing.assert_array_equal(prepared_depth, depth_metres(depth, '32FC1'))
    assert all(source == 'native_depth_optical' and stamp == 12000000345
               for _, source, stamp in calls)


def test_depth_only_clock_rewind_resets_tracker_and_pending_input_without_synchronizer():
    tracker = SingleTargetTracker(TrackerConfig(confirmation_hits=1))
    measurement = Measurement(np.array([1., 0., .08]), .9, np.array([.2, .3, .1]))
    assert tracker.update(10., [measurement]).state == 'measured'
    published = []
    node = SimpleNamespace(
        method='geometry', tracker=tracker, last_clock=10., pending=(None, object()),
        sync=None, debug_rgb=[], last_processed=10., last_published=10.,
        params={'output_delay': .06},
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=5.)),
        _publish=lambda estimate, stamp: published.append(estimate))
    RGBDTracker._tick(node)
    assert tracker.x is None and node.pending is None
    assert node.last_processed == node.last_published == -float('inf')
    assert node.problem == 'simulation_clock_reset'
    assert published[-1].state == 'lost'
