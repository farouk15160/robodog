"""
Value types crossing the hardware boundary.

Everything is a fixed-length numpy array indexed by joint, in the canonical
joint order. No backend is allowed to reorder, rename or rescale: the canonical
convention from robot_parameters.yaml holds on both sides of the boundary, and
any per-motor sign or offset is applied inside the real backend only.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

import numpy as np

LEGS = ("FL", "FR", "RL", "RR")
KINDS = ("haa", "hfe", "kfe")
JOINT_NAMES: tuple[str, ...] = tuple(f"{leg}_{k}_joint" for leg in LEGS for k in KINDS)
NJ = len(JOINT_NAMES)
JOINT_INDEX = {n: i for i, n in enumerate(JOINT_NAMES)}


class ControlMode(IntEnum):
    """Mirrors robodog_msgs/JointCommand.MODE_*."""
    IDLE = 0
    POSITION = 1
    VELOCITY = 2
    TORQUE = 3
    IMPEDANCE = 4


class Fault(IntFlag):
    """Mirrors robodog_msgs/JointTelemetry.FAULT_*."""
    NONE = 0
    POSITION_LIMIT = 1
    VELOCITY_LIMIT = 2
    TORQUE_LIMIT = 4
    OVERTEMPERATURE = 8
    COMMUNICATION = 16
    ENCODER = 32
    UNDERVOLTAGE = 64
    OVERCURRENT = 128
    NOT_ENABLED = 256
    WATCHDOG = 512


def _z(v: float = 0.0) -> np.ndarray:
    return np.full(NJ, v, dtype=np.float64)


@dataclass
class JointState:
    """One sample of every joint, as reported by a backend."""
    position: np.ndarray = field(default_factory=_z)      # [rad]
    velocity: np.ndarray = field(default_factory=_z)      # [rad/s]
    effort: np.ndarray = field(default_factory=_z)        # [N.m] output-referred
    temperature: np.ndarray = field(default_factory=lambda: _z(20.0))   # [degC]
    enabled: np.ndarray = field(default_factory=lambda: np.zeros(NJ, bool))
    faults: np.ndarray = field(default_factory=lambda: np.zeros(NJ, np.uint16))
    stamp: float = 0.0                                    # backend clock [s]

    def copy(self) -> "JointState":
        return JointState(self.position.copy(), self.velocity.copy(), self.effort.copy(),
                          self.temperature.copy(), self.enabled.copy(), self.faults.copy(),
                          self.stamp)


@dataclass
class JointCommand:
    """One command for every joint. The (position, velocity, effort, kp, kd)
    tuple is the ROBSTRIDE02 motion-control frame verbatim; `mode` only selects
    which of those the control layer populated, it is not sent to the motor."""
    mode: np.ndarray = field(default_factory=lambda: np.zeros(NJ, np.uint8))
    position: np.ndarray = field(default_factory=_z)
    velocity: np.ndarray = field(default_factory=_z)
    effort: np.ndarray = field(default_factory=_z)
    kp: np.ndarray = field(default_factory=_z)
    kd: np.ndarray = field(default_factory=_z)

    def copy(self) -> "JointCommand":
        return JointCommand(self.mode.copy(), self.position.copy(), self.velocity.copy(),
                            self.effort.copy(), self.kp.copy(), self.kd.copy())

    def zero_output(self) -> "JointCommand":
        """Torque-free command: what the safety layer substitutes on e-stop."""
        return JointCommand(mode=np.full(NJ, int(ControlMode.IDLE), np.uint8))


@dataclass
class BaseState:
    """Floating-base state. A simulation backend knows this exactly; the real
    robot does not, and the state estimator produces it from the IMU and the
    leg kinematics. `ground_truth` says which of the two you are looking at."""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0, 0, 1]))  # xyzw
    linear_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    linear_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    foot_contact: np.ndarray = field(default_factory=lambda: np.zeros(4, bool))
    foot_force: np.ndarray = field(default_factory=lambda: np.zeros(4))
    ground_truth: bool = False
