"""Timestamped single-target constant-velocity filter in one continuous frame."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class TrackerConfig:
    timeout: float = 1.0
    confirmation_hits: int = 2
    acceleration_std: float = 1.0
    measurement_std: float = 0.07
    association_distance: float = 0.7
    max_speed: float = 1.0
    jump_allowance: float = 0.20
    mahalanobis_gate: float = 11.35
    ambiguity_margin: float = 1.0


@dataclass
class Measurement:
    position: np.ndarray
    quality: float
    extent: np.ndarray
    index: int = 0


@dataclass
class Estimate:
    state: str
    position: object = None
    covariance: object = None
    confidence: float = 0.0
    measurement_age: float = math.inf
    selected_index: object = None
    reason: str = ''
    track_id: int = 0


class SingleTargetTracker:
    def __init__(self, config=None):
        self.config = config or TrackerConfig()
        self.track_id = 0
        self.reset()

    def reset(self):
        self.x = None
        self.p = None
        self.stamp = None
        self.last_measurement = None
        self.last_input = None
        self.hits = 0
        self.quality = 0.0
        self.extent = None
        self.reason = 'waiting_for_detection'

    def _predict(self, stamp):
        dt = max(0., stamp - self.stamp)
        f = np.eye(6)
        f[:3, 3:] = np.eye(3) * dt
        g = np.vstack((np.eye(3) * dt**2 / 2, np.eye(3) * dt))
        q = self.config.acceleration_std ** 2 * (g @ g.T)
        return f @ self.x, f @ self.p @ f.T + q

    def estimate(self, stamp, measured=False, selected_index=None):
        if self.x is None:
            return Estimate('lost', reason=self.reason, track_id=self.track_id)
        age = stamp - self.last_measurement
        if age < -1e-6 or age >= self.config.timeout:
            return Estimate('lost', measurement_age=age, reason='timeout_or_clock_reset',
                            track_id=self.track_id)
        if self.hits < self.config.confirmation_hits:
            return Estimate('initializing', measurement_age=age, reason='confirming_detection',
                            track_id=self.track_id)
        x, p = self._predict(stamp)
        return Estimate('measured' if measured else 'predicted', x[:3], p[:3, :3],
                        self.quality * math.exp(-age / self.config.timeout), age,
                        selected_index, self.reason, self.track_id)

    def update(self, stamp, measurements):
        if not np.isfinite(stamp):
            raise ValueError('Non-finite measurement timestamp')
        if self.last_input is not None and stamp <= self.last_input:
            # Delayed/duplicate frames must not rewind the posterior or refresh expiry.
            return self.estimate(self.last_input)
        self.last_input = stamp
        if self.x is not None and stamp - self.last_measurement >= self.config.timeout:
            self.reset()
            self.last_input = stamp
        measurements = [m for m in measurements
                        if np.asarray(m.position).shape == (3,)
                        and np.isfinite(m.position).all() and np.isfinite(m.quality)]
        if self.x is None:
            if len(measurements) != 1:
                self.reason = 'ambiguous_initialization' if measurements else 'no_detection'
                return self.estimate(stamp)
            m = measurements[0]
            self.track_id += 1
            self.x = np.r_[m.position, np.zeros(3)]
            self.p = np.diag([self.config.measurement_std**2 * 4] * 3 + [0.5] * 3)
            self.stamp = self.last_measurement = stamp
            self.hits = 1
            self.quality, self.extent = m.quality, m.extent.copy()
            self.reason = 'new_track'
            return self.estimate(stamp, True, m.index)
        predicted, covariance = self._predict(stamp)
        r = np.eye(3) * self.config.measurement_std**2
        innovation_cov = covariance[:3, :3] + r
        eligible = []
        for m in measurements:
            residual = m.position - predicted[:3]
            distance = np.linalg.norm(residual)
            mahal = float(residual @ np.linalg.solve(innovation_cov, residual))
            elapsed = stamp - self.last_measurement
            travel = np.linalg.norm(m.position - self.x[:3])
            if (distance > self.config.association_distance
                    or travel > self.config.jump_allowance + self.config.max_speed * elapsed
                    or mahal > self.config.mahalanobis_gate):
                continue
            geometry_cost = np.linalg.norm(m.extent - self.extent) / 0.3
            cost = mahal + 0.3 * geometry_cost + 0.5 * (1 - m.quality)
            eligible.append((cost, m))
        eligible.sort(key=lambda item: item[0])
        if not eligible or (len(eligible) > 1
                            and eligible[1][0] - eligible[0][0] < self.config.ambiguity_margin):
            self.reason = 'ambiguous_association' if len(eligible) > 1 else 'no_gated_detection'
            return self.estimate(stamp)
        m = eligible[0][1]
        gain = np.linalg.solve(innovation_cov, covariance[:3, :]).T
        self.x = predicted + gain @ (m.position - predicted[:3])
        h = np.zeros((3, 6))
        h[:, :3] = np.eye(3)
        correction = np.eye(6) - gain @ h
        # Joseph form preserves positive semidefinite covariance under repeated updates.
        self.p = correction @ covariance @ correction.T + gain @ r @ gain.T
        self.stamp = self.last_measurement = stamp
        self.hits += 1
        self.quality = 0.6 * self.quality + 0.4 * m.quality
        self.extent = 0.6 * self.extent + 0.4 * m.extent
        self.reason = 'associated_detection'
        return self.estimate(stamp, True, m.index)
