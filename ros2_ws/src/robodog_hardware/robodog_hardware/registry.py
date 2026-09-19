"""
Backend factory.

The only place in the codebase that knows which backend implementations exist.
The control node takes a name from a parameter, so switching from simulation to
the real robot is a launch argument, not a code change:

    ros2 launch robodog_bringup robot.launch.py backend:=mujoco
    ros2 launch robodog_bringup robot.launch.py backend:=robstride02_can

Imports are deferred so that a missing optional dependency (mujoco, python-can)
only breaks the backend that needs it.
"""
from __future__ import annotations

from typing import Any, Callable

from .backend import BackendError, JointBackend


def _kinematic(cfg):
    from .backends.kinematic import KinematicBackend
    return KinematicBackend(cfg)


def _mujoco(cfg):
    from .backends.mujoco_backend import MujocoBackend
    return MujocoBackend(cfg)


def _robstride(cfg):
    from .backends.robstride_can import RobStride02Backend
    return RobStride02Backend(cfg)


BACKENDS: dict[str, Callable[[dict], JointBackend]] = {
    "kinematic": _kinematic,
    "mujoco": _mujoco,
    "robstride02_can": _robstride,
}


def available() -> list[str]:
    return sorted(BACKENDS)


def create_backend(name: str, config: dict[str, Any] | None = None) -> JointBackend:
    try:
        factory = BACKENDS[name]
    except KeyError:
        raise BackendError(f"unknown backend '{name}'; available: {available()}") from None
    return factory(config or {})
