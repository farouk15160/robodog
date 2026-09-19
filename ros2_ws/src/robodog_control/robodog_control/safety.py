"""
Safety monitor: the last thing between a controller and the actuators.

Design rules
------------
1. It runs INSIDE the control loop, on every cycle, in both simulation and on
   the real robot. A safety layer that is only active on hardware has never
   been tested by the time it matters.
2. It is a pure function of (measurement, request, state) -> (allowed command,
   status). No ROS, no I/O, no threads. That is what makes the whole envelope
   testable, and the tests in test_safety.py are the specification.
3. It CLAMPS rather than rejects. A gait cycle that asks for slightly too much
   torque should be quietly limited, not dropped: dropping a cycle mid-stride
   is more dangerous than saturating one.
4. Faults that can destroy hardware LATCH. Clearing requires the condition to
   have gone away AND an explicit operator action.

Two-tier limits
---------------
    hard   URDF / datasheet maxima (17 N.m, 42.9 rad/s, joint end stops)
    soft   operational limits from robstride02.yaml -> operational_limits
           (6 N.m continuous, 20 rad/s, end stops inset by 0.05 rad)

The soft tier is what is enforced continuously. The hard tier is what the
actuator can physically deliver, and is allowed only for `peak_torque_duration_s`
under the I2t integrator below.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from robodog_hardware.types import NJ, ControlMode, Fault, JointCommand, JointState


@dataclass
class SafetyLimits:
    position_lower: np.ndarray
    position_upper: np.ndarray
    velocity_max: float = 20.0
    continuous_torque: float = 6.0
    peak_torque: float = 17.0
    peak_duration_s: float = 2.0
    position_margin: float = 0.05
    temp_warn_c: float = 70.0
    temp_derate_c: float = 85.0
    temp_fault_c: float = 100.0
    command_timeout_s: float = 0.1
    max_position_step_rad: float = 0.05    # per cycle, rate limit on setpoints

    @classmethod
    def from_config(cls, params: dict, rs02: dict, joint_names: list[str]) -> "SafetyLimits":
        jl, ol = params["joint_limits"], rs02["operational_limits"]
        kinds = [n.split("_")[1] for n in joint_names]
        return cls(
            position_lower=np.array([jl[k]["lower"] + ol["position_margin_rad"] for k in kinds]),
            position_upper=np.array([jl[k]["upper"] - ol["position_margin_rad"] for k in kinds]),
            velocity_max=ol["velocity_rad_s"],
            continuous_torque=ol["continuous_torque_nm"],
            peak_torque=ol["peak_torque_nm"],
            peak_duration_s=rs02["performance"]["peak_torque_duration_s"],
            position_margin=ol["position_margin_rad"],
            temp_warn_c=ol["temperature_warn_c"],
            temp_derate_c=ol["temperature_derate_c"],
            temp_fault_c=ol["temperature_fault_c"])


@dataclass
class SafetyReport:
    faults: np.ndarray = field(default_factory=lambda: np.zeros(NJ, np.uint16))
    clamped: int = 0
    messages: list[str] = field(default_factory=list)
    torque_utilisation: np.ndarray = field(default_factory=lambda: np.zeros(NJ))
    estop: bool = False
    latched: bool = False
    source: str = ""
    command_age_s: float = 0.0


class SafetyMonitor:
    def __init__(self, limits: SafetyLimits, joint_names: list[str], rate_hz: float = 400.0) -> None:
        self.lim = limits
        self.names = joint_names
        self.dt = 1.0 / rate_hz
        # I2t integrator: accumulates time spent above the continuous rating,
        # weighted by how far above. Budget = peak_duration_s at full peak.
        self._i2t = np.zeros(NJ)
        self._i2t_budget = limits.peak_duration_s
        self._latched = False
        self._latch_faults = np.zeros(NJ, np.uint16)
        self._source = ""
        self._clamp_events = 0
        self._last_cmd = np.zeros(NJ)
        self._have_last = False
        self._messages: list[str] = []

    # ---------------- e-stop ----------------
    @property
    def latched(self) -> bool:
        return self._latched

    def engage_estop(self, source: str = "service") -> None:
        self._latched = True
        self._source = source
        self._note(f"E-STOP engaged ({source})")

    def clear_estop(self, measured: JointState | None = None) -> tuple[bool, str]:
        """Refuse to clear while a hardware-protective fault is still true."""
        if measured is not None:
            blocking = self._hard_faults(measured)
            if blocking.any():
                names = [self.names[i] for i in np.flatnonzero(blocking)]
                return False, f"cannot clear, active faults on {', '.join(names)}"
        self._latched = False
        self._latch_faults[:] = 0
        self._source = ""
        self._i2t[:] = 0.0
        self._note("E-STOP cleared")
        return True, "cleared"

    def _hard_faults(self, m: JointState) -> np.ndarray:
        over_temp = m.temperature >= self.lim.temp_fault_c
        motor = (m.faults & (int(Fault.OVERCURRENT) | int(Fault.UNDERVOLTAGE)
                             | int(Fault.ENCODER) | int(Fault.COMMUNICATION))) != 0
        return over_temp | motor

    def _note(self, msg: str) -> None:
        self._messages.insert(0, msg)
        del self._messages[16:]

    # ---------------- main gate ----------------
    def apply(self, measured: JointState, request: JointCommand,
              command_age_s: float = 0.0) -> tuple[JointCommand, SafetyReport]:
        lim = self.lim
        rep = SafetyReport(command_age_s=command_age_s)
        cmd = request.copy()
        faults = np.zeros(NJ, np.uint16)

        # ---- 1. hardware faults reported by the actuator -------------------
        faults |= measured.faults

        # ---- 2. thermal ----------------------------------------------------
        t = measured.temperature
        faults |= np.where(t >= lim.temp_warn_c, int(Fault.OVERTEMPERATURE), 0).astype(np.uint16)
        # Linear derate between warn and fault: full torque at derate_c, zero at
        # fault_c. Cutting output abruptly at a threshold would drop the robot.
        span = max(lim.temp_fault_c - lim.temp_derate_c, 1e-6)
        thermal_scale = np.clip((lim.temp_fault_c - t) / span, 0.0, 1.0)

        # ---- 3. watchdog ---------------------------------------------------
        if command_age_s > lim.command_timeout_s:
            faults |= np.uint16(Fault.WATCHDOG)
            self._note(f"command stale by {command_age_s*1e3:.0f} ms, holding position")
            cmd = self._hold(measured)

        # ---- 4. position: clamp the setpoint, rate-limit the change --------
        # An IDLE joint has no meaningful position setpoint, so track the
        # measurement instead. Otherwise the rate limiter would remember the
        # placeholder zero and, on the next enable, ramp from a position the
        # robot was never at -- demanding full torque for the first few cycles.
        idle = cmd.mode == int(ControlMode.IDLE)
        cmd.position = np.where(idle, measured.position, cmd.position)
        lo, hi = lim.position_lower, lim.position_upper
        want = np.clip(cmd.position, lo, hi)
        if not np.allclose(want, cmd.position, atol=1e-12):
            faults |= np.where(cmd.position != want, int(Fault.POSITION_LIMIT), 0).astype(np.uint16)
            self._clamp_events += 1
            rep.clamped += 1
        if not self._have_last:
            # Seed the limiter from the MEASUREMENT, not from the first
            # setpoint. Letting cycle one through unlimited would leave exactly
            # one unguarded step, which is the cycle most likely to contain a
            # large jump (start-up, or a controller handing over).
            self._last_cmd = measured.position.copy()
            self._have_last = True
        step = np.clip(want - self._last_cmd, -lim.max_position_step_rad, lim.max_position_step_rad)
        want = np.where(idle, want, self._last_cmd + step)
        self._last_cmd = want.copy()
        cmd.position = want
        # measured position past the HARD limit is a mechanical problem
        faults |= np.where((measured.position < lo - lim.position_margin)
                           | (measured.position > hi + lim.position_margin),
                           int(Fault.POSITION_LIMIT), 0).astype(np.uint16)

        # ---- 5. velocity ---------------------------------------------------
        cmd.velocity = np.clip(cmd.velocity, -lim.velocity_max, lim.velocity_max)
        faults |= np.where(np.abs(measured.velocity) > lim.velocity_max * 1.1,
                           int(Fault.VELOCITY_LIMIT), 0).astype(np.uint16)

        # ---- 6. torque: predict what the impedance law will produce --------
        # The actuator computes tau = kp*(q*-q) + kd*(qd*-qd) + tau_ff in
        # firmware, so limiting only `effort` would not limit the torque. We
        # predict the result and scale the WHOLE triple, preserving the
        # commanded impedance direction rather than distorting it.
        tau_pred = (cmd.kp * (cmd.position - measured.position)
                    + cmd.kd * (cmd.velocity - measured.velocity) + cmd.effort)
        allowed = self._torque_budget(measured) * thermal_scale
        mag = np.abs(tau_pred)
        scale = np.where(mag > allowed, allowed / np.maximum(mag, 1e-9), 1.0)
        if np.any(scale < 1.0):
            faults |= np.where(scale < 1.0, int(Fault.TORQUE_LIMIT), 0).astype(np.uint16)
            self._clamp_events += 1
            rep.clamped += int(np.sum(scale < 1.0))
        cmd.kp = cmd.kp * scale
        cmd.kd = cmd.kd * scale
        cmd.effort = cmd.effort * scale
        rep.torque_utilisation = np.abs(tau_pred * scale) / lim.continuous_torque

        # ---- 7. I2t accounting ---------------------------------------------
        self._update_i2t(np.abs(tau_pred * scale))

        # ---- 8. latch and gate ---------------------------------------------
        hard = self._hard_faults(measured)
        if hard.any() and not self._latched:
            names = [self.names[i] for i in np.flatnonzero(hard)]
            self.engage_estop(f"fault: {', '.join(names)}")
            self._latch_faults = faults.copy()
        if self._latched:
            cmd = JointCommand()                       # mode IDLE everywhere
            faults |= self._latch_faults

        rep.faults = faults
        rep.estop = self._latched
        rep.latched = self._latched
        rep.source = self._source
        rep.messages = list(self._messages)
        return cmd, rep

    # ---------------- helpers ----------------
    def _hold(self, m: JointState) -> JointCommand:
        """Safe fallback: hold the measured position with moderate impedance.
        Not zero torque -- going limp mid-stance drops the robot."""
        c = JointCommand()
        c.mode[:] = int(ControlMode.IMPEDANCE)
        c.position[:] = m.position
        c.kp[:] = 40.0
        c.kd[:] = 2.0
        return c

    def _torque_budget(self, m: JointState) -> np.ndarray:
        """Continuous rating normally; the peak rating while I2t budget remains."""
        head = np.clip(1.0 - self._i2t / max(self._i2t_budget, 1e-9), 0.0, 1.0)
        return self.lim.continuous_torque + head * (self.lim.peak_torque - self.lim.continuous_torque)

    def _update_i2t(self, tau: np.ndarray) -> None:
        """Charge the integrator above the continuous rating, discharge below.

        Charging is scaled by (tau/continuous)^2 because heating goes as current
        squared, so 2x the continuous torque exhausts the budget 4x faster.
        """
        over = tau > self.lim.continuous_torque
        excess = (tau / max(self.lim.continuous_torque, 1e-9)) ** 2 - 1.0
        self._i2t += np.where(over, np.maximum(excess, 0.0) * self.dt, -self.dt / 4.0)
        np.clip(self._i2t, 0.0, self._i2t_budget * 2.0, out=self._i2t)
        exhausted = self._i2t >= self._i2t_budget
        if exhausted.any():
            names = [self.names[i] for i in np.flatnonzero(exhausted)]
            self._note(f"I2t budget exhausted on {', '.join(names)}, limiting to continuous")

    @property
    def clamp_events(self) -> int:
        return self._clamp_events

    @property
    def i2t(self) -> np.ndarray:
        return self._i2t.copy()
