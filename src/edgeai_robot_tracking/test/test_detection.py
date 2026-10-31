import numpy as np

from edgeai_robot_tracking.camera import Calibration
from edgeai_robot_tracking.detection import DetectionConfig, detect


def scene(distance=1.5, two_wheels=False):
    camera = Calibration(160, 120, [[180, 0, 80], [0, 180, 60], [0, 0, 1]], [])
    camera_to_base = np.array([[0., 0., 1., .175], [-1., 0., 0., 0.],
                               [0., -1., 0., .1], [0., 0., 0., 1.]])
    image = np.full((120, 160, 3), 140, np.uint8)
    depth = np.full((120, 160), 3., np.float32)
    # Flat floor at base z=0, not a constant-depth surrogate.
    for row in range(61, 120):
        depth[row, :] = .1 * 180 / (row - 60)
    depth[54:70, 60:100] = distance
    image[60:69, 60:67] = [30, 170, 25]
    if two_wheels:
        image[60:69, 93:100] = [30, 170, 25]
    return image, depth, camera, camera_to_base


def test_one_green_wheel_seeds_gray_chassis_and_removes_floor():
    image, depth, camera, transform = scene()
    candidates, mask, _ = detect(image, depth, camera, transform, DetectionConfig())
    assert len(candidates) == 1
    candidate = candidates[0]
    assert len(candidate.points) > candidate.green_pixels
    assert candidate.bbox[2] > 90  # gray chassis is included beyond green seed
    assert (candidate.points[:, 2] > .012).all()
    np.testing.assert_allclose(candidate.position[0], 1.675, atol=.02)
    assert mask.sum() > 0


def test_two_wheel_seeds_merge_into_one_surface():
    candidates, _, _ = detect(*scene(two_wheels=True), DetectionConfig())
    assert len(candidates) == 1


def test_neighboring_obstacle_separated_by_depth_jump():
    image, depth, camera, transform = scene()
    depth[54:70, 100:125] = 1.75
    candidates, _, _ = detect(image, depth, camera, transform, DetectionConfig())
    assert len(candidates) == 1
    assert candidates[0].bbox[2] < 100
    assert candidates[0].extent[0] < .01


def test_high_green_distractor_and_floor_green_rejected():
    image, depth, camera, transform = scene()
    image[:] = 140
    image[8:17, 60:70] = [30, 170, 25]
    depth[8:17, 60:70] = 1.5
    image[95:106, 60:70] = [30, 170, 25]
    candidates, _, rejected = detect(image, depth, camera, transform, DetectionConfig())
    assert not candidates
    assert rejected['seed_height'] == 2


def test_green_wall_does_not_become_a_bounded_robot():
    image, depth, camera, transform = scene()
    image[:] = 140
    image[57:70, 5:155] = [30, 170, 25]
    depth[57:70, 5:155] = 1.5
    candidates, _, rejected = detect(image, depth, camera, transform, DetectionConfig())
    assert not candidates
    assert rejected.get('visible_dimensions', 0) + rejected.get('region_boundary', 0) > 0


def test_invalid_depth_and_partial_wheel_only_surface():
    image, depth, camera, transform = scene()
    depth[:] = np.nan
    candidates, _, _ = detect(image, depth, camera, transform, DetectionConfig())
    assert not candidates
    image[:] = 140
    image[60:68, 62:70] = [30, 170, 25]
    depth[60:68, 62:70] = 1.5
    candidates, _, _ = detect(image, depth, camera, transform, DetectionConfig())
    assert len(candidates) == 1  # no full-body or both-wheels requirement


def test_tall_narrow_obstacle_is_not_accepted_after_height_clipping():
    image, depth, camera, transform = scene()
    image[:] = 140
    image[60:69, 62:69] = [30, 170, 25]
    depth[10:70, 60:100] = 1.5
    candidates, _, rejected = detect(image, depth, camera, transform, DetectionConfig())
    assert not candidates
    assert rejected['height_boundary'] == 1
