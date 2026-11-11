"""Mission-level FSM. ROS actions/services are effects, never blocking calls."""

from dataclasses import dataclass, field
from enum import Enum, auto


class State(Enum):
    INITIALIZING = auto()
    WAIT_FOR_LOCALIZATION = auto()
    INITIAL_LOCALIZATION_SCAN = auto()
    SELECT_S1_GOAL = auto()
    NAVIGATE_TO_S1 = auto()
    VERIFY_S1_POSITION = auto()
    HOLD_S1 = auto()
    S1_COMPLETED = auto()
    SELECT_C1_GOALS = auto()
    NAVIGATE_TO_C1 = auto()
    VERIFY_C1_POSITIONS = auto()
    HOLD_C1 = auto()
    C1_COMPLETED = auto()
    MISSION_COMPLETED = auto()
    RECOVERING = auto()
    FAILED = auto()


@dataclass
class Inputs:
    ready: bool = False
    localized: bool = False
    selection: object = None  # None pending, True valid, False invalid/unreachable.
    navigation: dict = field(default_factory=dict)
    verification: dict = field(default_factory=dict)
    recovered: bool = False


class MissionFSM:
    def __init__(self, config, autostart=True):
        self.config = config
        self.timing = config['timing']
        self.requested = autostart
        self.state = State.INITIALIZING
        self.state_time = self.start_time = self.last_time = None
        self.reason = 'waiting for start'
        self.s1_completed = self.c1_completed = False
        self.recoveries = {'S1': 0, 'C1': 0}
        self.active_target = ''
        self.c1_second_sent = False
        self.departed_s1 = False
        self.scan_index = 0
        self.scan_done = config.get('initial_scan_angle', 0.) == 0
        self.effects = []

    @property
    def moving_robots(self):
        if self.state == State.NAVIGATE_TO_S1:
            return {'robot1'}
        if self.state == State.NAVIGATE_TO_C1:
            return {'robot2' if self.c1_second_sent else 'robot1'}
        if self.state == State.INITIAL_LOCALIZATION_SCAN:
            return {('robot1', 'robot2')[self.scan_index]}
        return set()

    @property
    def phase(self):
        return 'C1' if self.s1_completed else 'S1'

    def transition(self, state, now, reason):
        self.state, self.state_time, self.reason = state, now, reason
        self.effects.append(('state', state.name, reason))

    def fail(self, now, reason):
        self.active_target = ''
        self.transition(State.FAILED, now, reason)
        self.effects.append(('cancel_all', '', ''))

    def recover(self, now, reason):
        if self.recoveries[self.phase] >= self.timing['max_recoveries']:
            self.fail(now, f'Recovery limit: {reason}')
            return
        self.recoveries[self.phase] += 1
        self.active_target = ''
        self.transition(State.RECOVERING, now, reason)
        self.effects.extend([('cancel_all', '', ''), ('clear_costmaps', '', '')])

    def step(self, now, inputs):
        self.effects = []
        if self.last_time is not None and now < self.last_time:
            self.s1_completed = self.c1_completed = False
            self.fail(now, 'ROS clock moved backwards; start a new mission')
        self.last_time = now
        if self.state in (State.FAILED, State.MISSION_COMPLETED):
            return self.effects
        if self.start_time is None:
            if not self.requested or now <= 0:
                return self.effects
            self.start_time = now
            self.transition(State.WAIT_FOR_LOCALIZATION, now, 'mission started')
        if (now-self.start_time+self.timing['elapsed_before_part2']
                >= self.timing['combined_budget']):
            self.fail(now, 'Combined Part 1/Part 2 time budget exhausted')
            return self.effects
        elapsed = now-self.state_time
        status = inputs.navigation
        failed = any(value in ('FAILED', 'CANCELED', 'REJECTED') for value in status.values())
        if self.state == State.WAIT_FOR_LOCALIZATION:
            if inputs.ready and inputs.localized:
                if not self.scan_done:
                    self.transition(State.INITIAL_LOCALIZATION_SCAN, now, 'optional collision-checked scan')
                    self.effects.append(('scan', 'robot1', ''))
                else:
                    target = self.phase
                    self.transition(State.SELECT_C1_GOALS if target == 'C1' else State.SELECT_S1_GOAL,
                                    now, 'AMCL and navigation ready')
                    self.effects.append(('select', target, ''))
            elif elapsed > self.timing['localization_timeout']:
                self.recover(now, 'Localization/navigation readiness timeout')
        elif self.state == State.INITIAL_LOCALIZATION_SCAN:
            robot = ('robot1', 'robot2')[self.scan_index]
            if failed or elapsed > self.timing['navigation_timeout']:
                self.recover(now, 'Optional localization scan failed')
            elif status.get(robot) == 'SUCCEEDED':
                if self.scan_index == 0:
                    self.scan_index = 1
                    self.effects.append(('scan', 'robot2', ''))
                else:
                    self.scan_done = True
                    self.transition(State.WAIT_FOR_LOCALIZATION, now, 'scan completed; recheck localization')
        elif self.state in (State.SELECT_S1_GOAL, State.SELECT_C1_GOALS):
            if inputs.selection is True:
                target = self.phase
                self.c1_second_sent = False
                self.transition(State.NAVIGATE_TO_S1 if target == 'S1' else State.NAVIGATE_TO_C1,
                                now, 'footprints and Nav2 reachability validated')
                self.effects.append(('depart_s1', 'robot1', '') if target == 'C1' and not self.departed_s1
                                    else ('navigate', 'robot1', target))
            elif inputs.selection is False or elapsed > self.timing['selection_timeout']:
                self.recover(now, 'Goal selection/path validation failed')
        elif self.state in (State.NAVIGATE_TO_S1, State.NAVIGATE_TO_C1):
            if not inputs.localized or failed or elapsed > self.timing['navigation_timeout']:
                self.recover(now, 'Navigation failed, timed out, or localization became invalid')
            elif self.state == State.NAVIGATE_TO_S1 and status.get('robot1') == 'SUCCEEDED':
                self.active_target = 'S1'
                self.transition(State.VERIFY_S1_POSITION, now, 'verify distance independently of Nav2')
            elif self.state == State.NAVIGATE_TO_C1:
                if status.get('robot1') == 'SUCCEEDED' and not self.departed_s1:
                    self.departed_s1 = True
                    self.effects.append(('navigate', 'robot1', 'C1'))
                elif status.get('robot1') == 'SUCCEEDED' and not self.c1_second_sent:
                    self.c1_second_sent = True
                    self.effects.append(('navigate', 'robot2', 'C1'))
                elif all(status.get(robot) == 'SUCCEEDED' for robot in ('robot1', 'robot2')):
                    self.active_target = 'C1'
                    self.transition(State.VERIFY_C1_POSITIONS, now, 'verify both actual map-frame distances')
        elif self.state in (State.VERIFY_S1_POSITION, State.VERIFY_C1_POSITIONS, State.HOLD_S1, State.HOLD_C1):
            target = self.phase
            check = inputs.verification.get('targets', {}).get(target, {})
            eligible = inputs.localized and check.get('eligible', False)
            holding = self.state in (State.HOLD_S1, State.HOLD_C1)
            if eligible and check.get('completed', False):
                self.transition(State.S1_COMPLETED if target == 'S1' else State.C1_COMPLETED,
                                now, 'independent verifier confirmed continuous 10-second hold')
            elif eligible and not holding:
                self.transition(State.HOLD_S1 if target == 'S1' else State.HOLD_C1, now, 'inside rescue radius')
            elif not eligible and holding:
                self.transition(State.VERIFY_S1_POSITION if target == 'S1' else State.VERIFY_C1_POSITIONS,
                                now, 'hold reset: distance, TF or localization invalid')
            elif not eligible and elapsed > self.timing['position_timeout']:
                self.recover(now, 'Position verification timeout')
        elif self.state == State.S1_COMPLETED:
            self.s1_completed, self.active_target = True, ''
            self.transition(State.SELECT_C1_GOALS, now, 'single rescue completed')
            self.effects.append(('select', 'C1', ''))
        elif self.state == State.C1_COMPLETED:
            self.c1_completed, self.active_target = True, ''
            self.transition(State.MISSION_COMPLETED, now, 'both rescue conditions satisfied')
        elif self.state == State.RECOVERING:
            if inputs.recovered and elapsed >= self.timing['recovery_wait']:
                self.transition(State.WAIT_FOR_LOCALIZATION, now, 'bounded recovery completed')
            elif elapsed > self.timing['recovery_timeout']:
                self.fail(now, 'Recovery services/cancellation timed out')
        return self.effects
