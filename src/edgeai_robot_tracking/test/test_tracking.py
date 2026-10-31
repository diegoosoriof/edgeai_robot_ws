import numpy as np

from edgeai_robot_tracking.camera import transform_points
from edgeai_robot_tracking.tracking import Measurement, SingleTargetTracker, TrackerConfig


def measurement(x=1., y=0., index=0):
    return Measurement(np.array([x, y, .08]), .9, np.array([.2, .3, .1]), index)


def confirmed_tracker():
    tracker = SingleTargetTracker()
    assert tracker.update(10., [measurement()]).state == 'initializing'
    assert tracker.update(10.1, [measurement()]).state == 'measured'
    return tracker


def test_missed_frames_predict_then_expire_without_more_input():
    tracker = confirmed_tracker()
    estimate = tracker.update(10.3, [])
    assert estimate.state == 'predicted'
    assert estimate.confidence < .9
    assert tracker.estimate(11.09).position is not None
    assert tracker.estimate(11.1).position is None
    assert tracker.estimate(11.2).state == 'lost'
    old_id = tracker.track_id
    assert tracker.update(11.3, [measurement(2.)]).state == 'initializing'
    assert tracker.track_id == old_id + 1
    assert tracker.update(11.4, [measurement(2.)]).state == 'measured'


def test_outliers_and_ambiguous_candidates_do_not_refresh_expiry():
    tracker = confirmed_tracker()
    assert tracker.update(10.2, [measurement(5.)]).state == 'predicted'
    estimate = tracker.update(10.3, [measurement(1., -.01), measurement(1., .01, 1)])
    assert estimate.reason == 'ambiguous_association'
    assert estimate.state == 'predicted'
    assert tracker.last_measurement == 10.1
    assert tracker.estimate(11.2).position is None
    fresh = SingleTargetTracker()
    assert fresh.update(1., [measurement(), measurement(2.)]).position is None
    assert fresh.x is None


def test_irregular_timestamps_estimate_velocity_and_reacquire_after_short_occlusion():
    tracker = SingleTargetTracker(TrackerConfig(confirmation_hits=1))
    for t in (1., 1.08, 1.21, 1.4, 1.7, 1.92, 2.1):
        assert tracker.update(t, [measurement(1. + .2 * t)]).state == 'measured'
    predicted = tracker.estimate(2.4)
    np.testing.assert_allclose(predicted.position[0], 1.48, atol=.04)
    assert tracker.update(2.5, [measurement(1.5)]).state == 'measured'
    assert np.linalg.eigvalsh(tracker.p).min() > 0


def test_delayed_frames_and_predictions_do_not_rewind_or_extend_track():
    tracker = confirmed_tracker()
    tracker.estimate(10.8)  # a query must not mutate posterior time
    assert tracker.stamp == 10.1
    assert tracker.update(10.2, [measurement(1.01)]).state == 'measured'
    tracker.update(10.15, [measurement(1.01)])
    assert tracker.last_measurement == 10.2
    assert tracker.estimate(11.3).state == 'lost'


def test_stationary_surface_remains_stationary_when_observer_moves_and_rotates():
    tracker = SingleTargetTracker(TrackerConfig(confirmation_hits=1))
    target = np.array([2., .5, .08])
    for i in range(12):
        yaw = i * .04
        base_to_odom = np.eye(4)
        base_to_odom[:3, :3] = [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1],
        ]
        base_to_odom[0, 3] = i * .03
        relative = transform_points(target, np.linalg.inv(base_to_odom))
        continuous = transform_points(relative, base_to_odom)
        m = Measurement(continuous, .9, np.array([.2, .3, .1]))
        estimate = tracker.update(1. + i * .1, [m])
    np.testing.assert_allclose(estimate.position, target, atol=1e-8)
    np.testing.assert_allclose(tracker.x[3:], 0., atol=1e-8)
