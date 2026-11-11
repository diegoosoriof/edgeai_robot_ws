"""Part 1 must not remember target observations captured before Start."""

from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock

from std_msgs.msg import Bool, Header
from vision_msgs.msg import Detection3D, Detection3DArray
from visualization_msgs.msg import Marker

from edgeai_aruco.registry import MarkerRegistry, MarkerRegistryNode, registry_visualization


def test_start_clears_remembered_markers_and_pending_observations():
    node = SimpleNamespace(
        run_enabled=False, run_started_ns=0, registry=MarkerRegistry('robot1/odom'),
        pending=deque([object()]), publish_snapshot=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)),
    )
    node.registry.markers['old'] = object()
    MarkerRegistryNode.on_run_enabled(node, Bool(data=True))
    assert node.run_enabled and node.run_started_ns == 10_000_000_000
    assert not node.registry.markers and not node.pending
    node.publish_snapshot.assert_called_once()
    # A repeated latched True must not erase detections from the current run.
    node.registry.markers['current'] = object()
    MarkerRegistryNode.on_run_enabled(node, Bool(data=True))
    assert 'current' in node.registry.markers


def test_disabled_and_delayed_prestart_observations_are_ignored():
    node = SimpleNamespace(run_enabled=False, run_started_ns=10_000_000_000,
                           check_clock=Mock(), publish_snapshot=Mock())
    message = Detection3DArray()
    message.header.stamp.sec = 9
    MarkerRegistryNode.on_detections(node, message)
    node.run_enabled = True
    MarkerRegistryNode.on_detections(node, message)
    node.check_clock.assert_not_called()
    node.publish_snapshot.assert_not_called()
    message.header.stamp.sec = 10
    MarkerRegistryNode.on_detections(node, message)
    node.check_clock.assert_called_once()
    node.publish_snapshot.assert_called_once()


def test_saved_panels_have_persistent_black_labels():
    message = Detection3DArray(header=Header(frame_id='robot1/odom'),
                               detections=[Detection3D(id='DICT_4X4_50:1')])
    panel, label = registry_visualization(message).markers
    assert panel.type == Marker.CUBE and label.type == Marker.TEXT_VIEW_FACING
    assert label.text == message.detections[0].id
    assert (label.color.r, label.color.g, label.color.b, label.color.a) == (0., 0., 0., 1.)
    assert panel.frame_locked and label.frame_locked
    assert panel.lifetime.sec == label.lifetime.sec == 0
