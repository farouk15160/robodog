"""
MuJoCo backend: full rigid-body dynamics, contact and friction.

Fidelity choices that matter
----------------------------
1. The impedance law is evaluated at the PHYSICS rate, not the control rate.
   The real RS02 closes its position/velocity loop in firmware in the tens of
   kHz; holding a 500 Hz zero-order hold on kp*(q*-q) would make the simulated
   joints noticeably softer and less stable than the hardware, and would tune
   gains that are wrong on the robot. `step()` therefore re-evaluates the law
   at every substep from the current state and only the SETPOINT is held.

2. `armature` in the model carries the reflected rotor inertia (4.8e-3 kg.m2,
   larger than the calf's own inertia about the knee). Without it the legs
   accelerate roughly four times too easily.

3. Torque saturation is applied here as well as in the safety monitor, so a
   controller bug cannot produce forces the actuator could never generate.

What is NOT modelled: gearbox backlash, belt compliance in the knee drive,
current-loop bandwidth, CAN latency and jitter. Latency is injected separately
by the `command_delay_steps` option so that controllers are not tuned against
an unrealistically prompt plant.
"""
from __future__ import annotations

import collections
import time
from typing import Any

import numpy as np

from ..backend import BackendError, JointBackend
from ..thermal import ThermalModel
from ..types import NJ, BaseState, ControlMode, JointCommand, JointState

FOOT_ORDER = ("FL", "FR", "RL", "RR")


class MujocoBackend(JointBackend):
    name = "mujoco"
    is_simulation = True

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        c = self.config
        self.model_path = c.get("model_path")
        if not self.model_path:
            raise BackendError("MujocoBackend requires config['model_path']")
        self.tau_max = float(c.get("peak_torque_nm", 17.0))
        self.vel_max = float(c.get("no_load_speed_rad_s", 42.935))
        self.keyframe = c.get("keyframe", "stand")
        self.launch_viewer = bool(c.get("viewer", False))
        # CAN round trip + firmware latency, in physics steps. 2 steps at
        # 0.5 ms = 1 ms, which is the order of a 1 Mbit/s bus at 500 Hz.
        self.delay_steps = int(c.get("command_delay_steps", 2))

        self.m = self.d = self._viewer = None
        self.cmd = JointCommand()
        self._queue: collections.deque[JointCommand] = collections.deque()
        self.on = np.zeros(NJ, bool)
        self.tau = np.zeros(NJ)
        self.steps = 0
        self._t0 = time.monotonic()
        self.thermal = ThermalModel(
            NJ, torque_constant=float(c.get("torque_constant_nm_per_arms", 1.22)),
            phase_resistance=float(c.get("phase_resistance_ohm", 0.29)),
            r_th=float(c.get("thermal_resistance_k_per_w", 2.84)),
            c_th=float(c.get("thermal_capacitance_j_per_k", 190.0)),
            ambient_c=float(c.get("ambient_temp_c", 20.0)))

    # ---------------- lifecycle ----------------
    def configure(self) -> None:
        try:
            import mujoco
        except ImportError as e:                       # pragma: no cover
            raise BackendError(
                "mujoco is not installed. `pip install mujoco`, or run with "
                "backend:=kinematic which needs no simulator.") from e
        self._mj = mujoco
        try:
            self.m = mujoco.MjModel.from_xml_path(str(self.model_path))
        except Exception as e:
            raise BackendError(f"could not load MuJoCo model {self.model_path}: {e}") from e
        self.d = mujoco.MjData(self.m)

        # Resolve every index once; a missing name is a model/URDF mismatch and
        # must fail loudly at start-up rather than silently drive nothing.
        def _id(objtype, name):
            i = mujoco.mj_name2id(self.m, objtype, name)
            if i < 0:
                raise BackendError(f"{objtype.name} '{name}' not found in {self.model_path}")
            return i

        T = mujoco.mjtObj
        self.jid = np.array([_id(T.mjOBJ_JOINT, n) for n in self.joint_names])
        self.qadr = self.m.jnt_qposadr[self.jid]
        self.vadr = self.m.jnt_dofadr[self.jid]
        self.aid = np.array([_id(T.mjOBJ_ACTUATOR, n) for n in self.joint_names])
        self.touch_id = np.array([_id(T.mjOBJ_SENSOR, f"{f}_touch") for f in FOOT_ORDER])
        self.touch_adr = self.m.sensor_adr[self.touch_id]
        self.base_bid = _id(T.mjOBJ_BODY, "base_link")
        self.free_jid = _id(T.mjOBJ_JOINT, "base_free")
        self.free_qadr = self.m.jnt_qposadr[self.free_jid]
        self.free_vadr = self.m.jnt_dofadr[self.free_jid]
        for s in ("imu_accel", "imu_gyro"):
            setattr(self, f"{s}_adr", self.m.sensor_adr[_id(T.mjOBJ_SENSOR, s)])

        self.reset()
        if self.launch_viewer:                          # pragma: no cover
            import mujoco.viewer
            self._viewer = mujoco.viewer.launch_passive(self.m, self.d)

    def reset(self) -> None:
        self._mj.mj_resetData(self.m, self.d)
        if self.keyframe is not None:
            kid = self._mj.mj_name2id(self.m, self._mj.mjtObj.mjOBJ_KEY, str(self.keyframe))
            if kid >= 0:
                self._mj.mj_resetDataKeyframe(self.m, self.d, kid)
        self._mj.mj_forward(self.m, self.d)
        self.cmd = JointCommand()
        self.cmd.position[:] = self.d.qpos[self.qadr]
        self._queue.clear()
        self.thermal.reset()
        self.steps = 0

    def shutdown(self) -> None:
        if self._viewer is not None:                    # pragma: no cover
            self._viewer.close()
            self._viewer = None

    def enable(self, mask: list[bool] | None = None) -> None:
        self.on[:] = True if mask is None else np.asarray(mask, bool)

    def disable(self, mask: list[bool] | None = None) -> None:
        if mask is None:
            self.on[:] = False
        else:
            self.on[np.asarray(mask, bool)] = False

    # ---------------- control cycle ----------------
    def read(self) -> JointState:
        return JointState(position=self.d.qpos[self.qadr].copy(),
                          velocity=self.d.qvel[self.vadr].copy(),
                          effort=self.tau.copy(),
                          temperature=self.thermal.temperature.copy(),
                          enabled=self.on.copy(), faults=np.zeros(NJ, np.uint16),
                          stamp=float(self.d.time))

    def write(self, cmd: JointCommand) -> None:
        self._queue.append(cmd.copy())
        while len(self._queue) > max(1, self.delay_steps):
            self.cmd = self._queue.popleft()

    def _apply(self) -> None:
        """Evaluate the actuator impedance law against the CURRENT state."""
        c = self.cmd
        q = self.d.qpos[self.qadr]
        qd = self.d.qvel[self.vadr]
        tau = c.kp * (c.position - q) + c.kd * (c.velocity - qd) + c.effort
        tau = np.where(c.mode == int(ControlMode.IDLE), 0.0, tau)
        tau = np.where(self.on, tau, 0.0)
        # Torque falls off as the no-load speed is approached, as a real BLDC
        # running out of voltage headroom does.
        headroom = np.clip(1.0 - np.abs(qd) / self.vel_max, 0.0, 1.0)
        tau = np.clip(tau, -self.tau_max, self.tau_max) * np.where(np.sign(tau) == np.sign(qd), headroom, 1.0)
        self.tau = tau
        self.d.ctrl[self.aid] = tau

    def step(self, dt: float) -> None:
        n = max(1, int(round(dt / self.m.opt.timestep)))
        for _ in range(n):
            self._apply()
            self._mj.mj_step(self.m, self.d)
            self.steps += 1
        self.thermal.update(self.tau, dt)
        if self._viewer is not None and self._viewer.is_running():   # pragma: no cover
            self._viewer.sync()

    # ---------------- floating base ----------------
    def base_state(self) -> BaseState:
        qp = self.d.qpos[self.free_qadr:self.free_qadr + 7]
        qv = self.d.qvel[self.free_vadr:self.free_vadr + 6]
        forces = self.d.sensordata[self.touch_adr]
        return BaseState(
            position=qp[0:3].copy(),
            orientation=np.array([qp[4], qp[5], qp[6], qp[3]]),   # wxyz -> xyzw
            linear_velocity=qv[0:3].copy(),
            angular_velocity=self.d.sensordata[self.imu_gyro_adr:self.imu_gyro_adr + 3].copy(),
            linear_acceleration=self.d.sensordata[self.imu_accel_adr:self.imu_accel_adr + 3].copy(),
            foot_contact=forces > 1.0,
            foot_force=forces.copy(),
            ground_truth=True)

    def stats(self) -> dict[str, Any]:
        wall = time.monotonic() - self._t0
        st = float(self.d.time) if self.d is not None else 0.0
        return {"backend": self.name, "sim_time": st, "wall_time": wall, "steps": self.steps,
                "timestep": float(self.m.opt.timestep) if self.m is not None else 0.0,
                "realtime_factor": st / wall if wall > 0 else 0.0,
                "world": str(self.model_path)}
