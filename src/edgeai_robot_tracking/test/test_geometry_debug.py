"""Verify stable RGB/depth topics, original colours and independent image grids."""

from collections import deque
from types import MethodType, SimpleNamespace

from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from tf2_ros import TransformException

from edgeai_robot_tracking.detector_node import RGBDTracker
from edgeai_robot_tracking.geometry_detection import GeometryConfig, detect_geometry
from edgeai_robot_tracking.tracking import Estimate
from test_geometry_detection import geometry_scene


def debug_scene():
    _, depth, camera, camera_to_base = geometry_scene()
    candidates, _, _ = detect_geometry(depth, camera, camera_to_base, GeometryConfig())
    assert len(candidates) == 1
    bridge = CvBridge()
    # RGB has twice the native depth resolution and a 2 cm optical-x offset.
    colours = np.full((240, 320, 3), [17, 93, 181], np.uint8)
    rgb = bridge.cv2_to_imgmsg(colours, encoding='bgr8')
    rgb.header = Header(stamp=Time(seconds=12.01).to_msg(), frame_id='rgb_header')
    depth_header = Header(stamp=Time(seconds=12.).to_msg(), frame_id='depth_header')
    k = camera.k.copy()
    k[:2] *= 2
    info = CameraInfo(width=320, height=240, k=k.ravel().tolist(),
                      distortion_model='plumb_bob')
    info.header = rgb.header
    extrinsic = TransformStamped()
    q = extrinsic.transform.rotation
    q.x, q.y, q.z, q.w = .5, -.5, .5, .5
    t = extrinsic.transform.translation
    t.x, t.y, t.z = .02, .1, -.175
    lookups, published, depth_published = [], [], []

    def lookup(*args):
        lookups.append(args)
        return extrinsic

    node = SimpleNamespace(
        debug_rgb=deque([rgb]), calibration={'rgb': info}, bridge=bridge,
        tf=SimpleNamespace(lookup_transform_full=lookup), processing_ms=1.,
        params={'camera_optical_frame': 'rgb_optical', 'expected_rgb_frame': 'rgb_header',
                'base_frame': 'base', 'tracking_frame': 'odom', 'sync_slop': .025,
                'camera_info_timeout': 2., 'max_depth': 9.5},
        debug_pub=SimpleNamespace(publish=published.append),
        depth_debug_pub=SimpleNamespace(publish=depth_published.append))
    node._camera = MethodType(RGBDTracker._camera, node)
    node._geometry_rgb_view = MethodType(RGBDTracker._geometry_rgb_view, node)
    return node, depth, depth_header, candidates, rgb, lookups, published


@pytest.mark.parametrize('selected', [True, False])
def test_rgb_debug_preserves_colours_and_projects_native_surface_boxes(selected):
    node, depth, header, candidates, rgb, lookups, published = debug_scene()
    reference_points = candidates[0].points.copy()
    reference_quality = candidates[0].quality
    view, boxes, image_header = node._geometry_rgb_view(candidates, header)
    assert image_header == rgb.header
    assert view.shape == (240, 320, 3)
    # Native x bounds [60, 98] scale by 2 and shift by fx_rgb * 0.02 / 1.5 = 4.8 px.
    x1, y1, x2, y2 = boxes[0][2]
    assert (x1, x2) == (124, 201)
    np.testing.assert_allclose([y1, y2], [108, 136], atol=1)
    chosen = candidates[0] if selected else None
    RGBDTracker._geometry_debug_image(
        node, depth, candidates, chosen, Estimate('measured'), header)
    output = published[-1]
    pixels = node.bridge.imgmsg_to_cv2(output, desired_encoding='bgr8')
    np.testing.assert_array_equal(pixels[220, 300], [17, 93, 181])
    np.testing.assert_array_equal(pixels[y1 + 5, x1], [0, 255, 0] if selected else [0, 200, 255])
    assert output.header == rgb.header
    assert lookups[-1][0] == 'rgb_optical' and lookups[-1][2] == 'base'
    assert lookups[-1][1].nanoseconds == 12010000000
    assert lookups[-1][3].nanoseconds == 12000000000
    np.testing.assert_array_equal(candidates[0].points, reference_points)
    assert candidates[0].quality == reference_quality
    # Drawing must never mutate the cached source image.
    np.testing.assert_array_equal(node.bridge.imgmsg_to_cv2(rgb)[y1 + 5, x1], [17, 93, 181])


@pytest.mark.parametrize('failure', ['missing', 'stale', 'calibration', 'encoding', 'tf'])
def test_optional_rgb_failures_never_send_depth_to_the_rgb_topic(failure):
    node, depth, header, candidates, rgb, _, published = debug_scene()
    depth_output = []
    node.depth_debug_pub.publish = depth_output.append
    if failure == 'missing':
        node.debug_rgb.clear()
    elif failure == 'stale':
        rgb.header.stamp.sec += 1
    elif failure == 'calibration':
        node.calibration.clear()
    elif failure == 'encoding':
        rgb.encoding = 'not_an_image_encoding'
    else:
        def unavailable(*args):
            raise TransformException('RGB display transform unavailable')
        node.tf.lookup_transform_full = unavailable
    RGBDTracker._geometry_debug_image(
        node, depth, candidates, candidates[0], Estimate('measured'), header)
    assert not published
    assert depth_output[-1].header == header
    assert depth_output[-1].width == depth.shape[1]
    assert depth_output[-1].height == depth.shape[0]
    assert depth_output[-1].encoding == 'bgr8'


def test_intermittent_rgb_and_tf_do_not_alternate_rgb_topic_background():
    node, depth, header, candidates, rgb, _, published = debug_scene()
    depth_output = []
    node.depth_debug_pub.publish = depth_output.append
    available_tf = node.tf.lookup_transform_full

    def missing_tf(*args):
        raise TransformException('Intermittent RGB display TF')

    for state in ('rgb', 'late_rgb', 'rgb', 'missing_tf', 'missing_rgb', 'rgb'):
        node.debug_rgb.clear()
        node.tf.lookup_transform_full = missing_tf if state == 'missing_tf' else available_tf
        rgb.header.stamp = Time(seconds=12.1 if state == 'late_rgb' else 12.01).to_msg()
        if state != 'missing_rgb':
            node.debug_rgb.append(rgb)
        RGBDTracker._geometry_debug_image(
            node, depth, candidates, candidates[0], Estimate('measured'), header)
    assert len(depth_output) == 6
    assert len(published) == 3
    for msg in published:
        assert (msg.width, msg.height) == (320, 240)
        pixels = node.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        np.testing.assert_array_equal(pixels[220, 300], [17, 93, 181])


def test_display_uses_closest_rgb_timestamp_from_bounded_cache():
    node, _, header, candidates, rgb, _, _ = debug_scene()
    distant = node.bridge.cv2_to_imgmsg(np.zeros((240, 320, 3), np.uint8), encoding='bgr8')
    distant.header = Header(stamp=Time(seconds=12.5).to_msg(), frame_id='rgb_header')
    node.debug_rgb.append(distant)
    assert node._geometry_rgb_view(candidates, header)[2] == rgb.header
