"""
The camera boundary.

Same pattern as the joint hardware boundary in robodog_hardware: everything
above this line -- the ROS publishers, the point-cloud builder, the GUI
placeholders -- talks only to `CameraBackend`. Two implementations:

    SimCameraBackend    renders from the MuJoCo scene
    NuwaHP60CBackend    the real Yahboom NUWA HP60C

Both read the same config/nuwa_hp60c.yaml, so the simulated camera produces
frames with the real device's resolution, field of view and range limits.
Swapping to hardware is a launch argument.

Intrinsics are part of the BACKEND contract, not of the node: a real camera
reports its own factory or calibration intrinsics, and the simulated one
derives them from the configured field of view. Consumers read
`intrinsics()` either way and never compute fx themselves.
"""
from __future__ import annotations

import abc
import math
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)
    distortion_model: str = "plumb_bob"

    @classmethod
    def from_fov(cls, width: int, height: int, hfov_deg: float, vfov_deg: float,
                 **kw) -> "Intrinsics":
        fx = width / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
        fy = height / (2.0 * math.tan(math.radians(vfov_deg) / 2.0))
        return cls(width, height, fx, fy, width / 2.0, height / 2.0, **kw)

    @property
    def K(self) -> list[float]:
        return [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]

    @property
    def P(self) -> list[float]:
        return [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]


@dataclass
class Frame:
    """One synchronised capture. Either field may be None if that stream is off."""
    color: np.ndarray | None = None      # (h, w, 3) uint8, RGB
    depth: np.ndarray | None = None      # (h, w) float32, METRES, NaN = invalid
    stamp: float = 0.0


class CameraBackendError(RuntimeError):
    pass


class CameraBackend(abc.ABC):
    name: str = "abstract"
    is_simulation: bool = True

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config

    @abc.abstractmethod
    def configure(self) -> None:
        ...

    def shutdown(self) -> None:
        ...

    @abc.abstractmethod
    def capture(self) -> Frame:
        """Latest frame. Must not block for longer than one frame period."""

    @abc.abstractmethod
    def intrinsics(self) -> tuple[Intrinsics, Intrinsics]:
        """(colour, depth) intrinsics."""

    def stats(self) -> dict[str, Any]:
        return {}
