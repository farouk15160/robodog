"""Camera backend factory -- the only place that knows which exist."""
from __future__ import annotations

from typing import Any, Callable

from .backend import CameraBackend, CameraBackendError


def _sim(cfg):
    from .backends.sim_camera import SimCameraBackend
    return SimCameraBackend(cfg)


def _nuwa(cfg):
    from .backends.nuwa_hp60c import NuwaHP60CBackend
    return NuwaHP60CBackend(cfg)


BACKENDS: dict[str, Callable[[dict], CameraBackend]] = {
    "sim": _sim,
    "nuwa_hp60c": _nuwa,
}


def available() -> list[str]:
    return sorted(BACKENDS)


def create_camera(name: str, config: dict[str, Any]) -> CameraBackend:
    try:
        return BACKENDS[name](config)
    except KeyError:
        raise CameraBackendError(f"unknown camera backend '{name}'; have {available()}") from None
