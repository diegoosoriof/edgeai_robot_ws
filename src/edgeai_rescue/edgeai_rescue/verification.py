"""Rescue conditions, independent of navigation actions and ROS transport."""

from dataclasses import dataclass
import math

from .geometry import Pose2D


@dataclass
class LocalizedPose:
    pose: Pose2D
    tf_stamp: float
    pose_stamp: float
    scan_stamp: float
    xy_variance: float
    yaw_variance: float

    def valid(self, now, settings):
        fields = (self.tf_stamp, self.pose_stamp, self.scan_stamp,
                  self.xy_variance, self.yaw_variance, now)
        if not self.pose.finite or not all(math.isfinite(value) for value in fields):
            return False
        for stamp, limit in ((self.tf_stamp, settings['max_tf_age']),
                             (self.pose_stamp, settings['max_pose_age']),
                             (self.scan_stamp, settings['max_scan_age'])):
            if stamp <= 0 or now-stamp > limit or stamp-now > .5:
                return False
        # AMCL's floating point covariance calculation can produce tiny negative roundoff.
        return (-1e-10 <= self.xy_variance <= settings['max_xy_variance']
                and -1e-10 <= self.yaw_variance <= settings['max_yaw_variance'])


class ContinuousHold:
    def __init__(self, duration, max_gap=.5):
        if duration <= 0 or max_gap <= 0:
            raise ValueError('Hold duration and observation gap must be positive')
        self.duration, self.max_gap = duration, max_gap
        self.reset()

    def reset(self):
        self.since = self.last = None
        self.elapsed = 0.0
        self.completed = False

    def update(self, now, eligible):
        jump = self.last is not None and (now < self.last or now-self.last > self.max_gap)
        if jump or not eligible or not math.isfinite(now):
            self.reset()
        self.last = now
        if not eligible or not math.isfinite(now):
            return False
        if self.since is None:
            self.since = now
        self.elapsed = max(0., now-self.since)
        self.completed = self.elapsed >= self.duration
        return self.completed


class RescueVerifier:
    def __init__(self, config):
        self.config = config
        self.holds = {name: ContinuousHold(target['hold_seconds'], config['localization']['max_observation_gap'])
                      for name, target in config['targets'].items()}
        self.active = ''
        self.last_time = None
        self.epoch = 0

    def activate(self, target):
        if target not in ('', *self.holds):
            raise ValueError(f'Unknown target {target}')
        if target != self.active:
            if target and not self.holds[target].completed:
                self.holds[target].reset()
            self.active = target

    def reset(self):
        for hold in self.holds.values():
            hold.reset()
        self.active = ''
        self.epoch += 1

    def sample(self, now, poses):
        if self.last_time is not None and now < self.last_time:
            self.reset()
        self.last_time = now
        valid = {robot: robot in poses and poses[robot].valid(now, self.config['localization'])
                 for robot in self.config['robots']}
        result = {'time': now, 'epoch': self.epoch, 'active_target': self.active,
                  'localized': valid, 'targets': {}}
        for name, target in self.config['targets'].items():
            distances = {}
            for robot in target['robots']:
                pose = poses.get(robot)
                distances[robot] = (math.hypot(pose.pose.x-target['center'][0], pose.pose.y-target['center'][1])
                                    if pose and pose.pose.finite else None)
            eligible = all(valid[robot] and distances[robot] is not None
                           and distances[robot] <= target['radius'] for robot in target['robots'])
            hold = self.holds[name]
            if self.active == name and not hold.completed:
                hold.update(now, eligible)
            result['targets'][name] = {'eligible': eligible, 'distances': distances,
                                      'held_seconds': hold.elapsed, 'completed': hold.completed,
                                      'hold_started': hold.since,
                                      'completed_at': hold.last if hold.completed else None}
        return result
