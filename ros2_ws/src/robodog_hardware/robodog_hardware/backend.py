"""
The hardware boundary.

Every component above this line -- the control loop, the gait generator, the
safety monitor, the GUI -- talks only to `JointBackend`. Three implementations
exist:

    KinematicBackend   ideal integrator, no contact. Runs anywhere, used for
                       controller and GUI development and in CI.
    MujocoBackend      full rigid-body dynamics and contact.
    RobStride02Backend real actuators over CAN.

Adding the real robot is therefore a backend selection, not an architectural
change: `robodog_control/control_node.py` never imports a backend directly, it
asks `robodog_hardware.registry.create_backend(name, config)`.

Timing contract
---------------
`read()` and `write()` are called once per control cycle, in that order, from a
single thread. `step()` exists only so a simulation backend can advance its own
integrator; the real backend implements it as a no-op, which is what makes the
same control loop drive both.
"""
from __future__ import annotations

import abc
from typing import Any

from .types import JOINT_NAMES, BaseState, JointCommand, JointState


class BackendError(RuntimeError):
    """Unrecoverable backend fault. The control node treats this as an e-stop."""


class JointBackend(abc.ABC):
    #: human-readable backend id, published in SimulationState.backend
    name: str = "abstract"
    #: False for the real robot; the GUI uses this to hide the simulation panel
    is_simulation: bool = True

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = config or {}
        self.joint_names = list(JOINT_NAMES)

    # ---------------- lifecycle ----------------
    @abc.abstractmethod
    def configure(self) -> None:
        """Open devices / load the model. May raise BackendError."""

    def shutdown(self) -> None:
        """Release resources. Must be safe to call twice and after a failure."""

    # ---------------- actuator power ----------------
    @abc.abstractmethod
    def enable(self, mask: list[bool] | None = None) -> None:
        """Energise the selected joints (None = all)."""

    @abc.abstractmethod
    def disable(self, mask: list[bool] | None = None) -> None:
        """De-energise. Must leave the joints back-driveable, not braked."""

    # ---------------- control cycle ----------------
    @abc.abstractmethod
    def read(self) -> JointState:
        """Latest measurement of all joints."""

    @abc.abstractmethod
    def write(self, cmd: JointCommand) -> None:
        """Apply a command to all joints. Already safety-clamped by the caller."""

    def step(self, dt: float) -> None:
        """Advance a simulation backend by `dt`. No-op on real hardware."""

    # ---------------- floating base ----------------
    def base_state(self) -> BaseState | None:
        """Ground-truth base state, if the backend has one.

        Returns None on the real robot: there is no sensor for absolute base
        pose, so `robodog_control/state_estimator` fuses the IMU with the leg
        kinematics instead. Consumers must handle None rather than assume it.
        """
        return None

    # ---------------- diagnostics ----------------
    def stats(self) -> dict[str, Any]:
        """Backend-specific counters for SimulationState / diagnostics."""
        return {}
