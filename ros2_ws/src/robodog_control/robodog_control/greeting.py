"""A bounded, nonblocking front-leg greeting for the control loop."""
from __future__ import annotations

import numpy as np

from .trajectory import JointTrajectory


class GreetingMotion:
    """Quasi-static front-left wave with a three-foot stability margin.

    Joint offsets are relative to the calibrated stand pose.  The first move
    shifts the body rearward and rightward while all four feet remain down.
    This puts the base origin well inside the FR/RL/RR support triangle before
    FL is raised.  The reverse sequence plants FL before centring the body.
    """

    _SHIFT_OFFSETS = np.array([
        0.309087, -0.327377, 0.416214,   # FL
        0.000000, -0.143609, 0.021779,   # FR
        0.309087, -0.327377, 0.416214,   # RL
        -0.309087, -0.021859, 0.496860,  # RR
    ])
    _LOW_LIFT_OFFSET = np.array([0.153315, 0.033181, -0.435291])
    _LIFT_OFFSET = np.array([0.167422, 0.077489, -0.552212])
    _WAVE_OUT_OFFSET = np.array([0.290507, 0.062641, -0.512139])
    _WAVE_IN_OFFSET = np.array([0.054541, 0.123835, -0.685774])

    def __init__(self, start: np.ndarray, stand: np.ndarray,
                 lower: np.ndarray, upper: np.ndarray) -> None:
        start = np.asarray(start, dtype=float)
        stand = np.asarray(stand, dtype=float)
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        if not all(v.shape == (12,) for v in (start, stand, lower, upper)):
            raise ValueError("greeting joint vectors must each contain 12 values")

        shifted = stand + self._SHIFT_OFFSETS
        low_lift = shifted.copy()
        low_lift[:3] = stand[:3] + self._LOW_LIFT_OFFSET
        lifted = shifted.copy()
        lifted[:3] = stand[:3] + self._LIFT_OFFSET
        wave_out = shifted.copy()
        wave_out[:3] = stand[:3] + self._WAVE_OUT_OFFSET
        wave_in = shifted.copy()
        wave_in[:3] = stand[:3] + self._WAVE_IN_OFFSET
        raw = (
            (0.60, stand),        # settle before shifting the support polygon
            (1.35, shifted),
            (1.45, low_lift),
            (0.45, lifted),
            (0.50, wave_out),
            (0.50, wave_in),
            (0.50, wave_out),
            (0.50, lifted),
            (0.45, low_lift),
            (1.45, shifted),      # plant FL before moving the body back
            (1.35, stand),
            (0.60, stand),
        )
        self.duration = sum(duration for duration, _goal in raw)
        self._segments: list[JointTrajectory] = []
        previous = np.clip(start, lower, upper)
        for duration, goal in raw:
            bounded = np.clip(goal, lower, upper)
            self._segments.append(JointTrajectory(previous, bounded, duration))
            previous = bounded
        self._index = 0
        self._elapsed = 0.0
        self._last_q = previous.copy()

    @property
    def done(self) -> bool:
        return self._index >= len(self._segments)

    @property
    def progress(self) -> float:
        return min(self._elapsed / self.duration, 1.0)

    def update(self, dt: float) -> tuple[np.ndarray, np.ndarray, bool]:
        if self.done:
            return self._last_q.copy(), np.zeros_like(self._last_q), True
        step = max(float(dt), 0.0)
        self._elapsed += step
        q, qd = self._segments[self._index].step(step)
        self._last_q = q.copy()
        if self._segments[self._index].done:
            self._index += 1
        return q, qd, self.done
