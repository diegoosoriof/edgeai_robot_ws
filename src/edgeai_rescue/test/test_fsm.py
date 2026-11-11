from copy import deepcopy
from pathlib import Path

import pytest

from edgeai_rescue.configuration import load_mission_config
from edgeai_rescue.fsm import Inputs, MissionFSM, State


CONFIG = load_mission_config(Path(__file__).parents[1]/'config/mission.yaml',
                             Path(__file__).parents[2]/'edgeai_robot_description')


def ready(**kwargs):
    return Inputs(ready=True, localized=True, **kwargs)


def at_s1_verification():
    fsm = MissionFSM(deepcopy(CONFIG))
    fsm.step(10, ready())
    assert fsm.state == State.SELECT_INITIAL_GOALS
    fsm.step(10.1, ready(selection=True))
    fsm.step(11, ready(navigation={'robot2': 'SUCCEEDED'}))
    assert fsm.state == State.VERIFY_S1_POSITION
    return fsm


def test_both_robots_depart_together_and_rescues_require_verification():
    fsm = MissionFSM(deepcopy(CONFIG))
    fsm.step(10, ready())
    assert fsm.selection_goals == {'robot2': 'S1', 'robot1': 'C1'}
    effects = fsm.step(10.1, ready(selection=True))
    assert ('navigate', 'robot2', 'S1') in effects and ('navigate', 'robot1', 'C1') in effects
    assert fsm.moving_robots == {'robot2', 'robot1'}
    fsm.step(11, ready(navigation={'robot2': 'SUCCEEDED', 'robot1': 'ACTIVE'}))
    assert fsm.state == State.VERIFY_S1_POSITION and fsm.moving_robots == {'robot1'}
    effects = fsm.step(11.1, ready(verification={'targets': {'S1': {'eligible': True, 'completed': False}}}))
    assert all(kind == 'state' for kind, _, _ in effects)
    assert fsm.state == State.HOLD_S1 and fsm.moving_robots == {'robot1'}
    fsm.step(15, ready(navigation={'robot1': 'SUCCEEDED', 'robot2': 'SUCCEEDED'},
                       verification={'targets': {'S1': {'eligible': True}}}))
    assert fsm.state == State.HOLD_S1 and not fsm.moving_robots
    fsm.step(21.1, ready(verification={'targets': {'S1': {'eligible': True, 'completed': True}}}))
    assert fsm.state == State.S1_COMPLETED
    effects = fsm.step(21.2, ready())
    assert fsm.s1_completed and fsm.state == State.BACK_UP_FROM_S1
    assert ('depart_s1', 'robot2', '') in effects and ('select', 'C1', '') not in effects
    effects = fsm.step(22, ready(navigation={'robot2': 'SUCCEEDED'}))
    assert ('select', 'C1', '') in effects and fsm.selection_goals == {'robot2': 'C1'}
    effects = fsm.step(22.1, ready(selection=True))
    assert ('navigate', 'robot2', 'C1') in effects and ('navigate', 'robot1', 'C1') not in effects
    fsm.step(25, ready(navigation={'robot2': 'SUCCEEDED'}))
    fsm.step(28, ready())
    assert fsm.state == State.VERIFY_C1_POSITIONS and not fsm.moving_robots
    fsm.step(28.1, ready(verification={'targets': {'C1': {'eligible': True, 'completed': False}}}))
    assert fsm.state == State.HOLD_C1
    fsm.step(38.1, ready(verification={'targets': {'C1': {'eligible': True, 'completed': True}}}))
    assert fsm.state == State.C1_COMPLETED
    fsm.step(38.2, ready())
    assert fsm.state == State.MISSION_COMPLETED and fsm.c1_completed
    assert fsm.moving_robots == set()


def test_nav2_success_alone_is_never_rescue_success():
    fsm = at_s1_verification()
    fsm.step(11.1, ready(navigation={'robot2': 'SUCCEEDED'}))
    assert fsm.state == State.VERIFY_S1_POSITION
    assert not fsm.s1_completed


def test_position_loss_returns_to_verification():
    fsm = at_s1_verification()
    fsm.step(11.1, ready(verification={'targets': {'S1': {'eligible': True}}}))
    fsm.step(12, ready(verification={'targets': {'S1': {'eligible': False, 'completed': False}}}))
    assert fsm.state == State.VERIFY_S1_POSITION and not fsm.s1_completed


def test_readiness_is_required_and_autostart_can_be_disabled():
    fsm = MissionFSM(deepcopy(CONFIG), autostart=False)
    assert fsm.step(10, ready()) == [] and fsm.state == State.INITIALIZING
    fsm.requested = True
    fsm.step(10, Inputs(ready=True, localized=False))
    assert fsm.state == State.WAIT_FOR_LOCALIZATION
    assert not fsm.moving_robots


def test_recovery_is_bounded_and_cancels_navigation():
    fsm = at_s1_verification()
    for index in range(2):
        now = 20+index*10
        effects = fsm.step(now, ready())
        assert fsm.state == State.RECOVERING
        assert ('cancel_all', '', '') in effects and not fsm.moving_robots
        fsm.step(now+2.1, ready(recovered=True))
        fsm.step(now+2.2, ready())
        assert fsm.state == State.SELECT_INITIAL_GOALS
        fsm.step(now+2.3, ready(selection=True))
        fsm.step(now+2.4, ready(navigation={'robot2': 'SUCCEEDED'}))
    fsm.step(50, ready())
    assert fsm.state == State.FAILED


def test_navigation_failure_and_timeout_have_bounded_recovery():
    for failed_status in ('FAILED', 'REJECTED', 'CANCELED'):
        fsm = MissionFSM(deepcopy(CONFIG))
        fsm.step(10, ready())
        fsm.step(10.1, ready(selection=True))
        fsm.step(11, ready(navigation={'robot2': failed_status}))
        assert fsm.state == State.RECOVERING
    fsm = MissionFSM(deepcopy(CONFIG))
    fsm.step(10, ready())
    fsm.step(10.1, ready(selection=True))
    fsm.step(161, ready(navigation={'robot2': 'ACTIVE'}))
    assert fsm.state == State.RECOVERING


def test_clock_reset_aborts_and_clears_completion():
    fsm = at_s1_verification()
    fsm.s1_completed = True
    effects = fsm.step(1, ready())
    assert fsm.state == State.FAILED and not fsm.s1_completed
    assert ('cancel_all', '', '') in effects


def test_combined_budget_includes_part1_and_transition():
    config = deepcopy(CONFIG)
    config['timing']['elapsed_before_part2'] = 779.
    fsm = MissionFSM(config)
    fsm.step(100, Inputs())
    fsm.step(101, ready())
    assert fsm.state == State.FAILED


def test_optional_scan_is_sequential_and_not_mandatory():
    config = deepcopy(CONFIG)
    config['initial_scan_angle'] = .35
    fsm = MissionFSM(config)
    effects = fsm.step(10, ready())
    assert fsm.state == State.INITIAL_LOCALIZATION_SCAN and ('scan', 'robot1', '') in effects
    effects = fsm.step(11, ready(navigation={'robot1': 'SUCCEEDED'}))
    assert ('scan', 'robot2', '') in effects
    fsm.step(12, ready(navigation={'robot1': 'SUCCEEDED', 'robot2': 'SUCCEEDED'}))
    assert fsm.state == State.WAIT_FOR_LOCALIZATION


def test_robot1_keeps_driving_after_s1_and_robot2_waits():
    fsm = at_s1_verification()
    complete_s1 = {'targets': {'S1': {'eligible': True, 'completed': True}}}
    fsm.step(21, ready(verification=complete_s1))
    fsm.step(21.1, ready())
    assert fsm.state == State.BACK_UP_FROM_S1 and fsm.moving_robots == {'robot1', 'robot2'}
    fsm.step(23, ready(navigation={'robot2': 'SUCCEEDED'}))
    assert fsm.selection_goals == {'robot2': 'C1'} and fsm.moving_robots == {'robot1'}
    fsm.step(23.1, ready(selection=True))
    fsm.step(27, ready(navigation={'robot2': 'SUCCEEDED'}))  # NavigateToPose result
    assert fsm.state == State.WAIT_FOR_C1_PARTNER
    assert fsm.moving_robots == {'robot1'} and fsm.active_target == ''
    fsm.step(30, ready(navigation={'robot1': 'SUCCEEDED'}))
    assert fsm.state == State.VERIFY_C1_POSITIONS and fsm.active_target == 'C1'
    assert not fsm.moving_robots


def test_robot1_failure_during_s1_hold_cancels_and_retries_both():
    fsm = at_s1_verification()
    fsm.step(11.1, ready(verification={'targets': {'S1': {'eligible': True}}}))
    effects = fsm.step(12, ready(navigation={'robot1': 'FAILED'}))
    assert fsm.state == State.RECOVERING and not fsm.moving_robots
    assert ('cancel_all', '', '') in effects
    fsm.step(14.1, ready(recovered=True))
    fsm.step(14.2, ready())
    assert fsm.selection_goals == {'robot2': 'S1', 'robot1': 'C1'}


def test_failed_s1_backup_recovers_before_planning_c1():
    fsm = at_s1_verification()
    hold_complete = {'targets': {'S1': {'eligible': True, 'completed': True}}}
    fsm.step(21.1, ready(verification=hold_complete))
    fsm.step(21.2, ready())
    effects = fsm.step(21.3, ready(navigation={'robot2': 'FAILED'}))
    assert fsm.state == State.RECOVERING and fsm.s1_completed and not fsm.departed_s1
    assert ('select', 'C1', '') not in effects
    fsm.step(23.4, ready(recovered=True))
    effects = fsm.step(23.5, ready())
    assert fsm.state == State.BACK_UP_FROM_S1 and ('depart_s1', 'robot2', '') in effects


def test_blocked_c1_route_can_wait_for_the_partner_without_canceling():
    fsm = at_s1_verification()
    fsm.step(21, ready(verification={'targets': {'S1': {'eligible': True, 'completed': True}}}))
    fsm.step(21.1, ready())
    fsm.step(21.9, ready(navigation={'robot2': 'SUCCEEDED'}))
    effects = fsm.step(22, ready(selection=False))
    assert fsm.state == State.WAIT_FOR_C1_ROUTE and fsm.moving_robots == {'robot1'}
    assert ('cancel_all', '', '') not in effects
    effects = fsm.step(27, ready(navigation={'robot1': 'SUCCEEDED'}))
    assert ('select', 'C1', '') in effects and fsm.selection_goals == {'robot2': 'C1'}
