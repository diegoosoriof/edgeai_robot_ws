import numpy as np
import pytest

from edgeai_robot_tracking.camera import Calibration, depth_metres, register_depth


def test_depth_units_and_invalid_values():
    raw = np.array([[1000, 0, 65535, 500]], dtype=np.uint16)
    depth = depth_metres(raw, '16UC1', max_depth=10.)
    np.testing.assert_allclose(depth[0, [0, 3]], [1., .5])
    assert np.isnan(depth[0, [1, 2]]).all()
    raw_float = np.array([[1., np.nan, np.inf, -np.inf, 0., -1., 12.]], dtype=np.float32)
    depth = depth_metres(raw_float, '32FC1', max_depth=10.)
    assert depth[0, 0] == 1.
    assert np.isnan(depth[0, 1:]).all()
    assert raw_float[0, 4] == 0.  # caller's buffer is not mutated
    with pytest.raises(ValueError, match='Unsupported depth'):
        depth_metres(raw, 'mono16')


def test_unprojection_and_registered_identity():
    camera = Calibration(5, 3, [[100, 0, 2], [0, 100, 1], [0, 0, 1]], [])
    points = camera.unproject([[2, 1], [3, 2]], np.array([2., 3.]))
    np.testing.assert_allclose(points, [[0., 0., 2.], [.03, .03, 3.]])
    np.testing.assert_allclose(camera.project(points), [[2, 1], [3, 2]])
    depth = np.full((3, 5), np.nan, np.float32)
    depth[1, 2] = 2.
    registered = register_depth(depth, camera, camera, np.eye(4))
    np.testing.assert_allclose(registered, depth, equal_nan=True)


def test_registration_uses_extrinsic_and_nearest_surface():
    camera = Calibration(5, 3, [[100, 0, 2], [0, 100, 1], [0, 0, 1]], [])
    depth = np.full((3, 5), np.nan, np.float32)
    depth[1, 1] = 1.  # shifts two pixels right
    depth[1, 2] = 2.  # shifts one pixel right, same destination
    extrinsic = np.eye(4)
    extrinsic[0, 3] = .02
    registered = register_depth(depth, camera, camera, extrinsic)
    assert registered[1, 3] == 1.  # nearest surface wins
    assert np.isnan(registered[1, 1:3]).all()


def test_distortion_roundtrip():
    camera = Calibration(100, 80, [[80, 0, 50], [0, 80, 40], [0, 0, 1]],
                         [.05, -.01, .001, .002, 0.])
    pixels = np.array([[30., 20.], [65., 50.]])
    points = camera.unproject(pixels, np.array([1., 2.]))
    np.testing.assert_allclose(camera.project(points), pixels, atol=1e-5)
