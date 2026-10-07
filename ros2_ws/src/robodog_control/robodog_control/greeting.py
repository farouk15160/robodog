"""A bounded, nonblocking front-leg greeting for the control loop."""
from __future__ import annotations

import numpy as np

from .trajectory import JointTrajectory


class GreetingMotion:
    """Smooth keyframe sequence that waves FL while the other feet stay planted."""

    duration = 5.25

    def __init__(self, start: np.ndarray, stand: np.ndarray,
                 lower: np.ndarray, upper: np.ndarray) -> None:
        start = np.asarray(start, dtype=float)
        stand = np.asarray(stand, dtype=float)
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        if not all(v.shape == (12,) for v in (start, stand, lower, upper)):
            raise ValueError("greeting joint vectors must each contain 12 values")

        lifted = stand.copy()
        lifted[1] = 0.45
        lifted[2] = -2.05
        wave_left = lifted.copy()
        wave_left[0] = 0.24
        wave_right = lifted.copy()
        wave_right[0] = -0.24
        raw = (
            (0.75, stand),       # settle before unloading a foot
            (0.75, lifted),
            (0.45, wave_left),
            (0.45, wave_right),
            (0.45, wave_left),
            (0.45, wave_right),
            (0.45, lifted),
            (0.75, stand),
            (0.75, stand),       # settle again before ordinary motion resumes
        )
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
