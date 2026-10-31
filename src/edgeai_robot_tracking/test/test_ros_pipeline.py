"""ROS transport smoke test using synthetic RGB-D and timestamped moving-observer TF.

Run explicitly with RUN_ROS_TESTS=1 in an isolated ROS_DOMAIN_ID; no Gazebo required.
"""

import os
import time

from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped, TransformStamped
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from edgeai_robot_tracking.detector_node import RGBDTracker
from test_detection import scene


@pytest.mark.skipif(os.environ.get('RUN_ROS_TESTS') != '1', reason='opt-in DDS integration test')
@pytest.mark.parametrize('method,rgb_debug', [
    ('green_seeds', True), ('geometry', False), ('geometry', True),
])
def test_ros_transport_moving_observer_and_input_loss(method, rgb_debug):
    context = Context()
    rclpy.init(context=context)
    detector = source = executor = None
    try:
        overrides = [
            Parameter('detection_method', value=method),
            Parameter('tracking.timeout', value=0.4),
            Parameter('output_delay', value=0.05),
            Parameter('processing_hz', value=25.),
        ]
        if method == 'geometry' and not rgb_debug:
            overrides += [Parameter('camera_optical_frame', value=''),
                          Parameter('expected_rgb_frame', value=''),
                          Parameter('detection.hsv_lower', value=[-1, -1, -1])]
        detector = RGBDTracker(context=context, namespace='robot1', parameter_overrides=overrides)
        source = Node('synthetic_rgbd', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(detector)
        executor.add_node(source)
        rgb_pub = source.create_publisher(Image, '/robot1/camera/image', 10)
        depth_pub = source.create_publisher(Image, '/robot1/camera/depth_image', 10)
        info_pub = source.create_publisher(CameraInfo, '/robot1/camera/camera_info', 10)
        received = {'status': [], 'positions': [], 'continuous': [], 'debug': [],
                    'depth_debug': [], 'markers': []}
        received_masks = []
        mask_subscription = source.create_subscription(
            Image, '/robot1/tracking/green_mask', received_masks.append, 10)
        assert detector.method == method
        if method == 'geometry':
            assert detector.rgb_sub is not None  # Optional display input, not synchronization.
            assert detector.sync is None and detector.mask_pub is None
        subscriptions = [source.create_subscription(kind, topic, received[key].append, 10)
                         for kind, topic, key in (
                             (DiagnosticArray, '/robot1/tracking/status', 'status'),
                             (PointStamped, '/robot1/tracking/position', 'positions'),
                             (PointStamped, '/robot1/tracking/position_continuous', 'continuous'),
                             (Image, '/robot1/tracking/debug_image', 'debug'),
                             (Image, '/robot1/tracking/debug_depth_image', 'depth_debug'),
                             (MarkerArray, '/robot1/tracking/markers', 'markers'))]
        static = StaticTransformBroadcaster(source)
        transform = TransformStamped()
        transform.header.frame_id = 'robot1/base_footprint'
        transform.child_frame_id = 'robot1/camera_optical_frame'
        transform.transform.translation.x = .175
        transform.transform.translation.z = .1
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = -.5, .5, -.5, .5
        static.sendTransform(transform)
        moving = TransformBroadcaster(source)
        bridge = CvBridge()
        start = time.monotonic()
        last_image_stamp = None
        while time.monotonic() - start < 1.8:
            stamp = source.get_clock().now().to_msg()
            displacement = min(.15, (time.monotonic() - start) * .1)
            odom = TransformStamped()
            odom.header.frame_id = 'robot1/odom'
            odom.header.stamp = stamp
            odom.child_frame_id = 'robot1/base_footprint'
            odom.transform.translation.x = displacement
            odom.transform.rotation.w = 1.
            moving.sendTransform(odom)
            image, depth, calibration, _ = scene(distance=1.5 - displacement)
            if method == 'geometry' and rgb_debug:
                image[:] = [17, 93, 181]
            rgb = bridge.cv2_to_imgmsg(image, encoding='bgr8')
            depth_msg = bridge.cv2_to_imgmsg(depth, encoding='32FC1')
            info = CameraInfo(width=calibration.width, height=calibration.height,
                              k=calibration.k.ravel().tolist(), distortion_model='plumb_bob')
            for msg in (rgb, depth_msg, info):
                msg.header.stamp = stamp
                msg.header.frame_id = 'robot1/base_footprint/rgbd_camera'
            if rgb_debug:
                rgb_pub.publish(rgb)
            info_pub.publish(info)
            # Stop depth first: unmatched RGB must not keep an old track valid.
            if time.monotonic() - start < 1.0:
                depth_pub.publish(depth_msg)
                last_image_stamp = stamp.sec + stamp.nanosec * 1e-9
            for _ in range(15):
                executor.spin_once(timeout_sec=0.002)
        states = [msg.status[0].message for msg in received['status']]
        assert 'measured' in states, [(s.status[0].message, s.status[0].values)
                                      for s in received['status'][-3:]]
        assert 'predicted' in states
        assert states[-1] == 'lost'
        assert received['positions'] and received['continuous']
        if method == 'geometry':
            assert not received_masks
            if rgb_debug:
                assert detector.input_counts['rgb'] > 0
                assert received['debug']
                assert all(np.array_equal(
                    bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')[100, 140], [17, 93, 181])
                    for msg in received['debug'])
            else:
                assert detector.input_counts['rgb'] == 0
                assert not received['debug']
            assert received['depth_debug']
            assert all(m.header.frame_id == 'robot1/base_footprint/rgbd_camera'
                       for m in received['depth_debug'])
        else:
            assert received['debug'] and not received['depth_debug']
            assert received_masks
        assert all(dict((v.key, v.value) for v in m.status[0].values)['detection_method'] == method
                   for m in received['status'])
        positions = received['positions']
        assert all(p.header.frame_id == 'robot1/base_footprint' for p in positions)
        assert all(p.header.frame_id == 'robot1/odom' for p in received['continuous'])
        continuous_x = [p.point.x for p in received['continuous']]
        np.testing.assert_allclose(continuous_x, 1.675, atol=.06)
        assert positions[0].point.x - positions[-1].point.x > .07
        assert all(p.header.stamp.sec + p.header.stamp.nanosec * 1e-9
                   < last_image_stamp + .4 for p in positions)
        assert received['markers'][-1].markers[0].action == Marker.DELETEALL
        assert subscriptions and mask_subscription  # Keep handles alive through publications.
    finally:
        if executor is not None:
            executor.shutdown()
        if detector is not None:
            detector.destroy_node()
        if source is not None:
            source.destroy_node()
        context.try_shutdown()
