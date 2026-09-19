"""Minimum-jerk joint interpolation, used for named-pose moves."""
from __future__ import annotations

import numpy as np


def minimum_jerk(s: float) -> tuple[float, float]:
    """Position and velocity scaling of a minimum-jerk profile at s in [0, 1].

    10s^3 - 15s^4 + 6s^5 has zero velocity AND zero acceleration at both ends,
    so a pose change does not start with a torque step. That matters here
    because a step into a 6 N.m continuous limit is exactly what trips the I2t
    integrator for no useful reason.
    """
    s = min(max(s, 0.0), 1.0)
    pos = s * s * s * (10.0 + s * (-15.0 + 6.0 * s))
    vel = 30.0 * s * s * (1.0 + s * (-2.0 + s))
    return pos, vel


class JointTrajectory:
    """Time-parameterised move from one joint vector to another."""

    def __init__(self, start: np.ndarray, goal: np.ndarray, duration_s: float) -> None:
        self.start = np.asarray(start, dtype=float).copy()
        self.goal = np.asarray(goal, dtype=float).copy()
        self.duration = max(float(duration_s), 1e-3)
        self.t = 0.0

    @classmethod
    def with_speed_limit(cls, start: np.ndarray, goal: np.ndarray,
                         max_velocity: float, min_duration_s: float = 0.5) -> "JointTrajectory":
        """Duration long enough that the peak of the minimum-jerk profile stays
        under `max_velocity`. The peak of the velocity scaling is 1.875."""
        span = float(np.max(np.abs(np.asarray(goal) - np.asarray(start))))
        need = 1.875 * span / max(max_velocity, 1e-6)
        return cls(start, goal, max(need, min_duration_s))

    @property
    def done(self) -> bool:
        return self.t >= self.duration

    @property
    def progress(self) -> float:
        return min(self.t / self.duration, 1.0)

    def step(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        self.t = min(self.t + dt, self.duration)
        p, v = minimum_jerk(self.t / self.duration)
        delta = self.goal - self.start
        return self.start + delta * p, delta * v / self.duration
