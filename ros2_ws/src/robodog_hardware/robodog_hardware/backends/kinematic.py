"""
Ideal-joint backend: no physics engine, no contact, no gravity.

Each joint is an independent second-order system driven by the same impedance
law the real actuator runs:

    tau = kp * (q* - q) + kd * (qd* - qd) + tau_ff        (clamped to peak)
    qdd = tau / J_eff  -  damping * qd  -  friction * sign(qd)

J_eff is the reflected rotor inertia plus a lumped link inertia, so the response
time is in the right order of magnitude rather than arbitrary.

Why this exists: it makes the entire stack above the hardware boundary --
control modes, trajectories, gait phase logic, safety clamping, the GUI, the
web protocol -- runnable and testable with no simulator installed and no
hardware. The base is pinned at the nominal stance height; anything that
depends on ground reaction forces needs the MuJoCo backend.
"""
from __future__ import annotations

import time
from typing import Any

import numpy as np

from ..backend import JointBackend
from ..thermal import ThermalModel
from ..types import NJ, BaseState, ControlMode, JointCommand, JointState


class KinematicBackend(JointBackend):
    name = "kinematic"
    is_simulation = True

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        c = self.config
        self.tau_max = float(c.get("peak_torque_nm", 17.0))
        self.damping = float(c.get("damping_nms_per_rad", 0.05))
        self.friction = float(c.get("coulomb_friction_nm", 0.12))
        # reflected rotor inertia dominates; add a nominal link contribution
        self.inertia = float(c.get("armature_kgm2", 4.805e-3)) + float(c.get("link_inertia_kgm2", 2.0e-3))
        self.vel_max = float(c.get("no_load_speed_rad_s", 42.935))
        self.base_height = float(c.get("base_height_m", 0.3198))
        self._q0 = np.asarray(c.get("initial_position", np.zeros(NJ)), dtype=float).copy()

        self.q = self._q0.copy()
        self.qd = np.zeros(NJ)
        self.tau = np.zeros(NJ)
        self.on = np.zeros(NJ, bool)
        self.cmd = JointCommand()
        self.cmd.position[:] = self._q0
        self.thermal = ThermalModel(
            NJ, torque_constant=float(c.get("torque_constant_nm_per_arms", 1.22)),
            phase_resistance=float(c.get("phase_resistance_ohm", 0.29)),
            r_th=float(c.get("thermal_resistance_k_per_w", 2.84)),
            c_th=float(c.get("thermal_capacitance_j_per_k", 190.0)),
            ambient_c=float(c.get("ambient_temp_c", 20.0)))
        self.sim_time = 0.0
        self.steps = 0
        self._t0 = time.monotonic()

    # ---------------- lifecycle ----------------
    def configure(self) -> None:
        self.q[:] = self._q0
        self.qd[:] = 0.0
        self.thermal.reset()

    def enable(self, mask: list[bool] | None = None) -> None:
        self.on[:] = True if mask is None else np.asarray(mask, bool)

    def disable(self, mask: list[bool] | None = None) -> None:
        if mask is None:
            self.on[:] = False
        else:
            self.on[np.asarray(mask, bool)] = False

    # ---------------- control cycle ----------------
    def read(self) -> JointState:
        return JointState(position=self.q.copy(), velocity=self.qd.copy(),
                          effort=self.tau.copy(), temperature=self.thermal.temperature.copy(),
                          enabled=self.on.copy(), faults=np.zeros(NJ, np.uint16),
                          stamp=self.sim_time)

    def write(self, cmd: JointCommand) -> None:
        self.cmd = cmd.copy()

    def step(self, dt: float) -> None:
        c = self.cmd
        tau = c.kp * (c.position - self.q) + c.kd * (c.velocity - self.qd) + c.effort
        tau = np.where(c.mode == int(ControlMode.IDLE), 0.0, tau)
        tau = np.where(self.on, tau, 0.0)
        self.tau = np.clip(tau, -self.tau_max, self.tau_max)

        net = self.tau - self.damping * self.qd - self.friction * np.tanh(self.qd / 0.01)
        self.qd = np.clip(self.qd + net / self.inertia * dt, -self.vel_max, self.vel_max)
        self.q += self.qd * dt

        self.thermal.update(self.tau, dt)
        self.sim_time += dt
        self.steps += 1

    # ---------------- floating base ----------------
    def base_state(self) -> BaseState:
        s = BaseState(ground_truth=True)
        s.position[2] = self.base_height
        s.linear_acceleration[2] = 9.81
        s.foot_contact[:] = True          # pinned base: treat all feet as loaded
        return s

    def stats(self) -> dict[str, Any]:
        wall = time.monotonic() - self._t0
        return {"backend": self.name, "sim_time": self.sim_time, "wall_time": wall,
                "steps": self.steps, "realtime_factor": self.sim_time / wall if wall > 0 else 0.0}
