from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import pytest
import yaml

from edgeai_robot_tracking.geometry_detection import GeometryConfig, detect_geometry, metric_labels
from edgeai_robot_tracking.tracking import Measurement, SingleTargetTracker
from test_detection import scene


def geometry_scene():
    image, depth, camera, transform = scene()
    image[:] = 140  # No green or other colour evidence exists in this scene.
    return image, depth, camera, transform


def detect_scene(depth, camera, transform, config=None):
    return detect_geometry(depth, camera, transform, config or GeometryConfig())


def test_geometry_acquires_without_colour_and_reports_real_geometric_support():
    _, depth, camera, transform = geometry_scene()
    candidates, mask, rejected = detect_scene(depth, camera, transform)
    assert len(candidates) == 1, rejected
    candidate = candidates[0]
    assert mask is None
    assert candidate.green_pixels is None and candidate.depth_inlier_fraction is None
    assert candidate.quality >= GeometryConfig().min_quality
    assert candidate.metrics['chassis_fraction'] > .5
    assert candidate.metrics['point_count'] == len(candidate.points)
    np.testing.assert_allclose(candidate.position[0], 1.675, atol=.02)


def test_floor_only_and_tall_obstacle_with_robot_sized_lower_crop_rejected():
    _, depth, camera, transform = geometry_scene()
    floor = np.full_like(depth, np.nan)
    for row in range(61, 120):
        floor[row] = .1 * 180 / (row - 60)
    assert not detect_scene(floor, camera, transform)[0]
    depth[20:70, 60:100] = 1.5
    candidates, _, rejected = detect_scene(depth, camera, transform)
    assert not candidates
    assert rejected['tall_component'] == 1
    # If this object's top is removed BEFORE clustering, the remaining lower band
    # looks plausible. The detector must retain the upper points and reject it.
    v, u = np.mgrid[:120, :160]
    points = camera.unproject(np.column_stack((u.ravel(), v.ravel())), depth.ravel())
    heights = (points @ transform[:3, :3].T + transform[:3, 3])[:, 2].reshape(depth.shape)
    cropped = depth.copy()
    cropped[heights > GeometryConfig().max_height] = np.nan
    assert len(detect_scene(cropped, camera, transform)[0]) == 1


@pytest.mark.parametrize('region,reason', [
    ((slice(54, 70), slice(10, 150)), 'footprint_dimensions'),
    ((slice(46, 53), slice(60, 100)), 'height_dimensions'),
])
def test_large_low_and_floating_obstacles_rejected(region, reason):
    _, depth, camera, transform = geometry_scene()
    depth[:] = np.nan
    depth[region] = 1.5
    candidates, _, rejected = detect_scene(depth, camera, transform)
    assert not candidates and rejected[reason] == 1


def test_configurable_observer_structure_box_excludes_native_depth_points():
    _, depth, camera, transform = geometry_scene()
    depth[:] = np.nan
    depth[54:70, 60:100] = .15
    config = replace(GeometryConfig(), min_forward=.05, self_max_forward=.4)
    candidates, _, rejected = detect_scene(depth, camera, transform, config)
    assert not candidates and rejected['self_points'] > 0


def test_geometry_yaml_matches_builtin_defaults_without_shared_overrides():
    path = Path(__file__).resolve().parents[1] / 'config/geometry.yaml'
    params = yaml.safe_load(path.read_text())['/**']['ros__parameters']
    assert set(params) == {'geometry'}
    assert params['geometry'] == asdict(GeometryConfig())


def test_partial_wheel_surface_is_allowed_but_distant_obstacle_is_not_joined():
    _, depth, camera, transform = geometry_scene()
    depth[:] = np.nan
    depth[60:68, 62:70] = 1.5
    candidates, _, rejected = detect_scene(depth, camera, transform)
    assert len(candidates) == 1, rejected
    assert candidates[0].metrics['footprint_length_m'] < .10
    depth[15:70, 108:125] = 1.9  # A nearby tall obstacle with a depth gap.
    candidates, _, rejected = detect_scene(depth, camera, transform)
    assert len(candidates) == 1
    assert candidates[0].bbox[2] < 100
    assert rejected['tall_component'] == 1


def test_disconnected_robot_sized_surfaces_remain_ambiguous():
    _, depth, camera, transform = geometry_scene()
    depth[:] = np.nan
    depth[54:70, 30:60] = 1.5
    depth[54:70, 100:130] = 1.5
    candidates, _, rejected = detect_scene(depth, camera, transform)
    assert len(candidates) == 2, rejected
    measurements = [Measurement(c.position, c.quality, c.extent, i)
                    for i, c in enumerate(candidates)]
    tracker = SingleTargetTracker()
    assert tracker.update(1., measurements).reason == 'ambiguous_initialization'
    assert tracker.x is None


def test_search_crop_does_not_turn_a_large_wall_into_a_small_object():
    _, depth, camera, transform = geometry_scene()
    depth[:] = np.nan
    depth[54:70, 10:150] = 1.5
    config = replace(GeometryConfig(), max_lateral=.15, boundary_margin=0.)
    candidates, _, rejected = detect_scene(depth, camera, transform, config)
    assert not candidates
    assert rejected['search_or_image_boundary'] == 1


def test_footprint_validation_is_invariant_to_base_axis_rotation():
    _, depth, camera, transform = geometry_scene()
    expected = detect_scene(depth, camera, transform)[0][0]
    # Rotate around z and translate to keep the object in the broad search volume.
    turn = np.eye(4)
    angle = .8
    turn[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    turn[0, 3] = .5
    actual = detect_scene(depth, camera, turn @ transform)[0][0]
    assert actual.quality == pytest.approx(expected.quality, abs=1e-6)
    assert actual.metrics['footprint_length_m'] == pytest.approx(
        expected.metrics['footprint_length_m'], abs=1e-6)


def test_explicit_processing_limits_and_config_validation():
    _, depth, camera, transform = geometry_scene()
    for config, key in (
        (replace(GeometryConfig(), max_sampled_points=10), 'sample_limit'),
        (replace(GeometryConfig(), max_component_points=10), 'component_point_limit'),
    ):
        candidates, _, rejected = detect_scene(depth, camera, transform, config)
        assert not candidates
        assert rejected[key] > 0
    depth[:] = np.nan
    depth[54:70, 30:60] = 1.5
    depth[54:70, 100:130] = 1.5
    candidates, _, rejected = detect_scene(
        depth, camera, transform, replace(GeometryConfig(), max_components=1))
    assert not candidates and rejected['component_limit'] == 2
    for config in (replace(GeometryConfig(), cluster_stride=0),
                   replace(GeometryConfig(), cluster_stride=2.),
                   replace(GeometryConfig(), max_height=float('nan')),
                   replace(GeometryConfig(), min_quality=1.1),
                   replace(GeometryConfig(), max_forward=.1)):
        with pytest.raises(ValueError, match='geometry'):
            config.validate()


def test_metric_graph_does_not_connect_diagonal_pixels_or_depth_jumps():
    points = np.zeros((3, 3, 3))
    allowed = np.eye(3, dtype=bool)
    labels = metric_labels(points, allowed, .065)
    assert len(np.unique(labels[allowed])) == 3
    allowed[:] = True
    points[:, 2, 0] = .5
    labels = metric_labels(points, allowed, .065)
    assert len(np.unique(labels)) == 2
