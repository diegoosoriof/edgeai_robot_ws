"""Midterm distance and simultaneous, continuous simulation-time hold rules."""

from pathlib import Path

from edgeai_rescue.configuration import load_mission_config
from edgeai_rescue.geometry import Pose2D
from edgeai_rescue.verification import ContinuousHold, LocalizedPose, RescueVerifier


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = load_mission_config(PACKAGE / 'config/mission.yaml',
                             PACKAGE.parent / 'edgeai_robot_description')


def observation(x, y, time):
    return LocalizedPose(Pose2D(x, y), time, time, time, .001, .001)


def sample(verifier, time, positions):
    return verifier.sample(time, {robot: observation(*point, time)
                                  for robot, point in positions.items()})


def test_s1_requires_ten_seconds_and_resets_on_leaving_the_radius():
    verifier = RescueVerifier(CONFIG)
    verifier.activate('S1')
    positions = {'robot2': (1.4, 4.1)}
    for tick in range(20, 39):
        result = sample(verifier, tick / 2, positions)
    assert not result['targets']['S1']['completed']
    assert result['targets']['S1']['held_seconds'] == 9.
    # One sample outside the radius invalidates the entire unfinished hold.
    result = sample(verifier, 19.5, {'robot2': (1.501, 4.1)})
    assert result['targets']['S1']['held_seconds'] == 0.
    for tick in range(40, 60):
        result = sample(verifier, tick / 2, positions)
    assert not result['targets']['S1']['completed']
    result = sample(verifier, 30., positions)
    assert result['targets']['S1']['completed']
    assert result['targets']['S1']['held_seconds'] == 10.


def test_c1_clock_starts_only_when_both_robots_are_inside_simultaneously():
    verifier = RescueVerifier(CONFIG)
    verifier.activate('C1')
    first = {'robot1': (3.8, 1.4), 'robot2': (4.5, 2.0)}
    both = {'robot1': (3.8, 1.4), 'robot2': (4.5, 1.4)}
    for tick in range(20, 40):
        result = sample(verifier, tick / 2, first)
    assert result['targets']['C1']['held_seconds'] == 0.
    for tick in range(40, 60):
        result = sample(verifier, tick / 2, both)
    assert not result['targets']['C1']['completed']
    # Losing either robot's observation also resets the joint timer.
    result = sample(verifier, 30., {'robot1': both['robot1']})
    assert not result['targets']['C1']['eligible']
    assert result['targets']['C1']['held_seconds'] == 0.
    for tick in range(61, 82):
        result = sample(verifier, tick / 2, both)
    assert result['targets']['C1']['completed']
    assert result['targets']['C1']['held_seconds'] == 10.


def test_pause_gap_and_backward_clock_do_not_create_hold_time():
    hold = ContinuousHold(10., max_gap=.5)
    hold.update(10., True)
    hold.update(10.5, True)
    for _ in range(20):
        hold.update(10.5, True)
    assert hold.elapsed == .5
    hold.update(12., True)
    assert hold.elapsed == 0.
    hold.update(1., True)
    assert hold.elapsed == 0. and not hold.completed


def test_radius_boundary_is_inclusive():
    verifier = RescueVerifier(CONFIG)
    verifier.activate('S1')
    assert sample(verifier, 10., {'robot2': (1.5, 4.1)})['targets']['S1']['eligible']


def test_clock_reset_clears_previously_completed_rescues():
    verifier = RescueVerifier(CONFIG)
    verifier.activate('S1')
    positions = {'robot2': (1.4, 4.1)}
    for tick in range(20, 41):
        result = sample(verifier, tick / 2, positions)
    assert result['targets']['S1']['completed']
    result = sample(verifier, 1., positions)
    assert result['epoch'] == 1
    assert not result['targets']['S1']['completed']
