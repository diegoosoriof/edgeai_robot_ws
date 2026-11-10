"""Geometry and installed-asset generation checks for the challenge arena."""

from itertools import product
from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import xacro
import yaml

from edgeai_aruco.detection import Detector


PACKAGE = Path(__file__).resolve().parents[1]


def world():
    return ET.parse(PACKAGE / 'worlds/rescue_arena.sdf').getroot().find('world')


def bounds(model):
    center = np.array(list(map(float, model.findtext('pose').split()))[:2])
    size = np.array(list(map(float, model.findtext('link/collision/geometry/box/size').split()))[:2])
    return center - size / 2, center + size / 2


def test_arena_dimensions_and_connected_walls():
    models = {model.get('name'): model for model in world().findall('model')}
    expected = {
        'south_wall': ([0, 0], [5, .1]), 'north_wall': ([0, 4.9], [5, 5]),
        'west_wall': ([0, .1], [.1, 4.9]), 'east_wall': ([4.9, .1], [5, 4.9]),
        'W1': ([0, 3.05], [2.2, 3.15]), 'W1b': ([2.15, 3.1], [2.25, 3.6]),
        'W2': ([3.35, 0], [3.45, 2.8]),
        'O2': ([1.9, 1.5], [2.5, 2.5]), 'O3': ([4, 3.3], [4.8, 3.9]),
    }
    colliding = {name for name, model in models.items()
                 if model.find('link/collision/geometry/box') is not None}
    assert colliding == set(expected)  # No extra obstacles.
    for name, extent in expected.items():
        model = models[name]
        np.testing.assert_allclose(bounds(model), extent)
        assert model.findtext('static') == 'true'
        assert model.findtext('link/collision/geometry/box/size') == model.findtext(
            'link/visual/geometry/box/size')
        assert float(model.findtext('link/collision/geometry/box/size').split()[2]) == .5
    for first, second in (('W1', 'west_wall'), ('W1', 'W1b'), ('W2', 'south_wall')):
        lo1, hi1 = bounds(models[first])
        lo2, hi2 = bounds(models[second])
        assert np.all(np.minimum(hi1, hi2) >= np.maximum(lo1, lo2))


def test_targets_and_noncolliding_overlays():
    arena = world()
    includes = {item.findtext('name'): item for item in arena.findall('include')}
    for name, center in (('S1', [1, 4.1, 0]), ('C1', [4.1, 1, 0])):
        np.testing.assert_allclose(list(map(float, includes[name].findtext('pose').split()))[:3], center)
        assert includes[name].findtext('uri') == f'model://rescue_target_{name}'
    for name, radius in (('S1', .5), ('C1', .7)):
        overlay = arena.find(f"model[@name='{name}_zone']")
        assert overlay.findall('.//collision') == []
        assert float(overlay.findtext('link/visual/geometry/cylinder/radius')) == radius
    start = arena.find("model[@name='start_area']")
    assert start.findall('.//collision') == []
    np.testing.assert_allclose(list(map(float, start.findtext('pose').split()))[:2], [.7, .5])
    assert start.findtext('link/visual/geometry/box/size') == '1.4 1.0 0.001'


def rotation(rpy):
    r, p, y = rpy
    cr, cp, cy = np.cos(rpy)
    sr, sp, sy = np.sin(rpy)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        transform[:3, :3] = rotation(np.fromstring(element.get('rpy', '0 0 0'), sep=' '))
    return transform


def test_robot_footprints_and_namespace_isolation():
    robots = yaml.safe_load((PACKAGE / 'config/rescue_robots.yaml').read_text())
    footprints = []
    for name, pose in robots.items():
        x, y, yaw = pose['x'], pose['y'], pose['yaw']
        robot = ET.fromstring(xacro.process_file(str(PACKAGE / 'urdf/edgeai_robot_real.urdf.xacro'),
                                                mappings={'robot_namespace': name}).toxml())
        assert robot.findtext('.//frame_id') == f'{name}/odom'
        assert robot.findtext('.//child_frame_id') == f'{name}/base_footprint'
        assert robot.findtext('.//gz_frame_id') == f'{name}/lidar_link'
        assert all(topic.text.startswith(f'/{name}/') for topic in robot.findall('.//topic'))
        spawn = np.eye(4)
        spawn[:3, :3] = rotation([0, 0, yaw])
        spawn[:2, 3] = [x, y]
        transforms = {'base_footprint': spawn}
        pending = list(robot.findall('joint'))
        while pending:
            ready = [joint for joint in pending if joint.find('parent').get('link') in transforms]
            assert ready
            for joint in ready:
                transforms[joint.find('child').get('link')] = (
                    transforms[joint.find('parent').get('link')] @ origin(joint.find('origin')))
                pending.remove(joint)
        points = []
        for link in robot.findall('link'):
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry.find('box') is not None:
                    half = np.fromstring(geometry.find('box').get('size'), sep=' ') / 2
                elif geometry.find('cylinder') is not None:
                    cylinder = geometry.find('cylinder')
                    half = [float(cylinder.get('radius'))] * 2 + [float(cylinder.get('length')) / 2]
                else:
                    raise AssertionError('Unhandled robot collision geometry')
                transform = transforms[link.get('name')] @ origin(collision.find('origin'))
                for signs in product((-1, 1), repeat=3):
                    points.append((transform @ np.r_[np.array(half) * signs, 1])[:2])
        lo, hi = np.min(points, axis=0), np.max(points, axis=0)
        # Start region, perimeter walls and an extra 1 cm clearance.
        assert np.all(lo > [.11, .11]) and np.all(hi < [1.4, 1])
        footprints.append((lo, hi))
    (lo1, hi1), (lo2, hi2) = footprints
    assert np.any((hi1 < lo2) | (hi2 < lo1))


def test_stands_reproduce_valid_distinct_markers(tmp_path):
    generator = runpy.run_path(str(PACKAGE / 'scripts/generate_rescue_targets.py'))
    config = yaml.safe_load((PACKAGE / 'config/rescue_targets.yaml').read_text())
    generator['generate_targets'](PACKAGE / 'config/rescue_targets.yaml', tmp_path)
    ids = []
    for name in ('S1', 'C1'):
        directory = tmp_path / f'rescue_target_{name}'
        for filename in ('model.sdf', 'model.config', 'marker.png'):
            assert (directory / filename).read_bytes() == (PACKAGE / 'models' / directory.name / filename).read_bytes()
        marker_ids = Detector(config['dictionary'], config['marker_size']).detect(
            cv2.imread(str(directory / 'marker.png')))[1].ravel().tolist()
        assert marker_ids == [config['targets'][name]['id']]
        ids.extend(marker_ids)
        model = ET.parse(directory / 'model.sdf').getroot().find('model')
        assert model.findall('.//collision')
        marker = model.find("link[@name='marker']")
        assert marker.findall('collision') == []
        np.testing.assert_allclose(np.fromstring(marker.findtext('pose'), sep=' ')[:3], [0, 0, .20])
    assert len(set(ids)) == 2
