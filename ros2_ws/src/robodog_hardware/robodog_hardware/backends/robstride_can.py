"""
RobStride backends: 12 real actuators over CAN, with model-specific codecs.

STATUS: complete in structure, UNVALIDATED against hardware. Everything that
can be written without a motor on the bench is written; everything that cannot
is marked `HW-CHECK`. Nothing above the hardware boundary changes when this
becomes the active backend -- that is the point of the whole layer.

Bus topology
------------
12 motors x (1 command + 1 feedback) x ~130 bits/frame at 500 Hz is 1.56 Mbit/s,
which does not fit on one 1 Mbit/s bus. The default configuration therefore
splits the robot across two buses (front / rear, 6 motors each, ~78% load).
Running one bus is possible at 250 Hz; `config/robstride_bus.yaml` carries the
choice and `configure()` refuses a combination that cannot fit.

Threading
---------
A reader thread per bus drains feedback into a per-motor slot. `read()` copies
the slots; it never blocks on the bus. `write()` is synchronous and sends one
frame per motor. If a motor's feedback goes stale beyond `feedback_timeout_s`,
its FAULT_COMMUNICATION bit is raised and the safety monitor stops the robot --
a silently stale joint is the failure mode most likely to break a leg.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any

import numpy as np

from ..backend import BackendError, JointBackend
from ..protocol import robstride02 as rs
from ..protocol import robstride06
from ..transmission import validate_transmission
from ..thermal import ThermalModel
from ..types import NJ, ControlMode, Fault, JointCommand, JointState

log = logging.getLogger(__name__)


class RobStride02Backend(JointBackend):
    name = "robstride02_can"
    is_simulation = False
    motor_model = "ROBSTRIDE02"
    codec = rs

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        c = self.config
        if c.get("motor_model", self.motor_model) != self.motor_model:
            raise BackendError(f"{self.name} cannot drive {c['motor_model']}; select the matching CAN backend")
        self.ratio, self.efficiency = validate_transmission(
            c.get("transmission_ratio", 1.0), c.get("transmission_efficiency", 1.0), NJ)
        self.torque_gain = self.ratio * self.efficiency
        self.host_id = int(c.get("host_id", 0xFD))
        self.bus_specs: list[dict] = list(c.get("buses", []))
        self.joint_map: dict[str, dict] = dict(c.get("joints", {}))
        self.feedback_timeout = float(c.get("feedback_timeout_s", 0.05))
        self.control_rate = float(c.get("control_rate_hz", 500.0))
        self.tau_max = float(c.get("peak_torque_nm", self.codec.T_MAX))

        missing = [n for n in self.joint_names if n not in self.joint_map]
        if missing:
            raise BackendError(f"no CAN mapping for joints: {missing}")

        # Per-joint hardware mapping. `direction` and `offset` are the ONLY
        # calibration parameters; the external transmission also changes units.
        self.bus_of = np.array([self.joint_map[n]["bus"] for n in self.joint_names])
        self.motor_id = np.array([self.joint_map[n]["motor_id"] for n in self.joint_names])
        self.direction = np.array([float(self.joint_map[n].get("direction", 1)) for n in self.joint_names])
        self.offset = np.array([float(self.joint_map[n].get("offset_rad", 0.0)) for n in self.joint_names])

        self._buses: list[Any] = []
        self._threads: list[threading.Thread] = []
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._fb = [None] * NJ                      # latest rs.Feedback per joint
        self._fb_time = np.zeros(NJ)
        self._fault_latched = np.zeros(NJ, np.uint16)
        self._fault_feedback_bytes = [bytes(4)] * NJ
        self._fault_warning_bytes = [bytes(4)] * NJ
        self._rx_count = np.zeros(NJ, dtype=np.int64)
        self._tx_count = 0
        self._rx_errors = 0
        self.on = np.zeros(NJ, bool)
        # index lookup: (bus, motor_id) -> joint index
        self._slot = {(int(b), int(m)): i for i, (b, m) in enumerate(zip(self.bus_of, self.motor_id))}

        self.thermal = ThermalModel(
            NJ, torque_constant=float(c.get("torque_constant_nm_per_arms", 1.22)),
            phase_resistance=float(c.get("phase_resistance_ohm", 0.29)),
            r_th=float(c.get("thermal_resistance_k_per_w", 2.84)),
            c_th=float(c.get("thermal_capacitance_j_per_k", 190.0)),
            ambient_c=float(c.get("ambient_temp_c", 20.0)))

    # ---------------- bus budget ----------------
    def check_bus_budget(self) -> list[str]:
        """Frames/s each bus must carry versus what 1 Mbit/s can deliver.
        An extended-ID 8-byte frame is 64 + 64 = 128 bits before stuffing;
        150 bits is a safe worst case with stuffing and interframe space."""
        warnings = []
        bits_per_frame = 150
        for spec in self.bus_specs:
            b = int(spec["id"])
            n = int(np.sum(self.bus_of == b))
            load = n * 2 * bits_per_frame * self.control_rate / float(spec.get("bitrate", 1_000_000))
            if load > 0.80:
                warnings.append(f"bus {b}: {n} motors at {self.control_rate:.0f} Hz "
                                f"= {load*100:.0f}% load (limit 80%)")
        return warnings

    # ---------------- lifecycle ----------------
    def configure(self) -> None:
        over = self.check_bus_budget()
        if over:
            raise BackendError("CAN bus over budget: " + "; ".join(over) +
                               ". Reduce control_rate_hz or add a bus.")
        try:
            import can
        except ImportError as e:                                   # pragma: no cover
            raise BackendError("python-can is not installed (`pip install python-can`)") from e

        for spec in self.bus_specs:                                # pragma: no cover
            try:
                bus = can.Bus(channel=spec["channel"],
                              interface=spec.get("interface", "socketcan"),
                              bitrate=int(spec.get("bitrate", 1_000_000)))
            except Exception as e:
                raise BackendError(f"cannot open CAN bus {spec}: {e}") from e
            self._buses.append(bus)
            t = threading.Thread(target=self._rx_loop, args=(int(spec["id"]), bus),
                                 name=f"{self.name}-rx-{spec['id']}", daemon=True)
            t.start()
            self._threads.append(t)
        log.info("opened %d CAN bus(es) for %d joints", len(self._buses), NJ)

    def shutdown(self) -> None:                                    # pragma: no cover
        self._stop.set()
        try:
            self.disable()
        except Exception:
            log.exception("disable() failed during shutdown")
        for t in self._threads:
            t.join(timeout=0.5)
        for b in self._buses:
            try:
                b.shutdown()
            except Exception:
                log.exception("bus shutdown failed")
        self._buses.clear()
        self._threads.clear()

    # ---------------- rx ----------------
    def _rx_loop(self, bus_id: int, bus) -> None:
        while not self._stop.is_set():
            msg = bus.recv(timeout=0.05)
            if msg is None:
                continue
            comm, field, target = self.codec.split_id(msg.arbitration_id)
            if comm not in (rs.CommType.FEEDBACK, rs.CommType.FAULT_FEEDBACK):
                continue
            if target != self.host_id:
                continue
            idx = self._slot.get((bus_id, field & 0xFF))
            if idx is None:
                continue
            if comm == rs.CommType.FAULT_FEEDBACK:
                self._receive_fault_frame(idx, msg)
                continue
            try:
                fb = self.codec.decode_feedback(msg.arbitration_id, bytes(msg.data))
            except ValueError:
                self._rx_errors += 1
                continue
            with self._lock:
                self._fb[idx] = fb
                self._fb_time[idx] = time.monotonic()
                self._rx_count[idx] += 1

    def _receive_fault_frame(self, idx: int, msg) -> None:
        decoder = getattr(self.codec, "decode_fault_feedback", None)
        if decoder is None:
            return  # RS02 behavior unchanged; no unverified RS06 decoding on another model.
        try:
            event = decoder(msg.arbitration_id, bytes(msg.data))
        except ValueError:
            with self._lock:
                self._rx_errors += 1
                self._fault_latched[idx] |= int(Fault.COMMUNICATION)
            return
        with self._lock:
            # Type21 does not refresh the position/torque timestamp. A live fault
            # stream must not disguise missing joint-state feedback.
            self._fault_feedback_bytes[idx] = bytes(
                old | new for old, new in zip(self._fault_feedback_bytes[idx], event.fault_bytes))
            self._fault_warning_bytes[idx] = event.warning_bytes
            if any(event.fault_bytes):
                # COMMUNICATION is the contract's generic hard-fault fallback;
                # it means an unclassified motor protection event here, NOT an
                # inferred current measurement. Exact type21 endian is unverified.
                self._fault_latched[idx] |= int(Fault.COMMUNICATION)
            self._rx_count[idx] += 1

    # ---------------- power ----------------
    def _send(self, idx: int, can_id: int, data: bytes) -> None:   # pragma: no cover
        import can
        self._buses[int(self.bus_of[idx])].send(
            can.Message(arbitration_id=can_id, data=data, is_extended_id=True))
        self._tx_count += 1

    def enable(self, mask: list[bool] | None = None) -> None:
        sel = np.ones(NJ, bool) if mask is None else np.asarray(mask, bool)
        for i in np.flatnonzero(sel):
            self._send(i, *self.codec.encode_enable(int(self.motor_id[i]), self.host_id))
            self.on[i] = True

    def disable(self, mask: list[bool] | None = None) -> None:
        sel = np.ones(NJ, bool) if mask is None else np.asarray(mask, bool)
        for i in np.flatnonzero(sel):
            self._send(i, *self.codec.encode_stop(int(self.motor_id[i]), self.host_id))
            self.on[i] = False

    def clear_faults(self) -> None:
        with self._lock:
            previous_flags = self._fault_latched.copy()
            previous_bytes = list(self._fault_feedback_bytes)
            self._fault_latched = np.zeros(NJ, np.uint16)
            self._fault_feedback_bytes = [bytes(4)] * NJ
        # Clear before transmit so a concurrently arriving fresh fault cannot be
        # erased after the command. Restore history if any transmit fails.
        try:
            for i in range(NJ):
                self._send(i, *self.codec.encode_stop(int(self.motor_id[i]), self.host_id, clear_fault=True))
        except Exception:
            with self._lock:
                self._fault_latched |= previous_flags
                self._fault_feedback_bytes = [bytes(a | b for a, b in zip(old, new))
                                              for old, new in zip(previous_bytes, self._fault_feedback_bytes)]
            raise
        with self._lock:
            # A read() call has a fresh host timestamp even when it returns a
            # cached motor frame. Require new normal feedback from EVERY motor
            # after the final clear transmission before reporting it healthy.
            # Do not clear fault latches here: fresh type21 events may already
            # have arrived while the stop/clear frames were being transmitted.
            self._fb_time = np.zeros(NJ)

    # ---------------- control cycle ----------------
    def read(self) -> JointState:
        st = JointState()
        now = time.monotonic()
        with self._lock:
            fbs, ts, = list(self._fb), self._fb_time.copy()
            st.faults = self._fault_latched.copy()
        for i, fb in enumerate(fbs):
            if fb is None or (now - ts[i]) > self.feedback_timeout:
                st.faults[i] |= int(Fault.COMMUNICATION)
                if fb is None:
                    continue
            # motor coordinates -> canonical joint coordinates
            st.position[i] = (fb.position - self.offset[i]) * self.direction[i] / self.ratio[i]
            st.velocity[i] = fb.velocity * self.direction[i] / self.ratio[i]
            st.effort[i] = fb.torque * self.direction[i] * self.torque_gain[i]
            st.temperature[i] = fb.temperature
            st.enabled[i] = fb.mode == rs.MotorMode.RUNNING
            st.faults[i] |= _motor_faults_to_flags(fb.faults)
            if not st.enabled[i]:
                st.faults[i] |= int(Fault.NOT_ENABLED)
        st.stamp = now
        # Observer alongside the reported temperature; the safety monitor takes
        # the maximum of the two. See thermal.py for why.
        self.thermal.update(st.effort / self.torque_gain, 1.0 / self.control_rate)
        st.temperature = np.maximum(st.temperature, self.thermal.temperature)
        return st

    def write(self, cmd: JointCommand) -> None:
        idle = cmd.mode == int(ControlMode.IDLE)
        pos = cmd.position * self.direction * self.ratio + self.offset
        vel = cmd.velocity * self.direction * self.ratio
        tau = np.clip(cmd.effort / self.torque_gain, -self.tau_max, self.tau_max) * self.direction
        kp = np.where(idle, 0.0, cmd.kp / (self.ratio * self.torque_gain))
        kd = np.where(idle, 0.0, cmd.kd / (self.ratio * self.torque_gain))
        tau = np.where(idle, 0.0, tau)
        for i in range(NJ):
            can_id, data = self.codec.encode_motion_control(
                int(self.motor_id[i]), float(pos[i]), float(vel[i]),
                float(tau[i]), float(kp[i]), float(kd[i]))
            self._send(i, can_id, data)

    def base_state(self):
        """None by contract: the real robot has no absolute base-pose sensor.
        robodog_control/state_estimator fuses the IMU with leg kinematics."""
        return None

    def stats(self) -> dict[str, Any]:
        with self._lock:
            fault_bytes = list(self._fault_feedback_bytes)
            warning_bytes = list(self._fault_warning_bytes)
        return {"backend": self.name, "tx_frames": int(self._tx_count),
                "rx_frames": int(self._rx_count.sum()), "rx_errors": int(self._rx_errors),
                "buses": len(self._buses),
                "fault_feedback_bytes": {self.joint_names[i]: value.hex()
                                         for i, value in enumerate(fault_bytes) if any(value)},
                "fault_warning_bytes": {self.joint_names[i]: value.hex()
                                        for i, value in enumerate(warning_bytes) if any(value)},
                "stale_joints": [self.joint_names[i] for i in range(NJ)
                                 if self._fb[i] is None
                                 or (time.monotonic() - self._fb_time[i]) > self.feedback_timeout]}


class RobStride06Backend(RobStride02Backend):
    name = "robstride06_can"
    motor_model = "ROBSTRIDE06"
    codec = robstride06

    def __init__(self, config=None):
        super().__init__(config)
        required = ("transmission_ratio", "transmission_efficiency", "peak_torque_nm",
                    "torque_constant_nm_per_arms", "phase_resistance_ohm",
                    "thermal_resistance_k_per_w", "thermal_capacitance_j_per_k")
        missing = [key for key in required if key not in self.config]
        if missing:
            raise BackendError("RS06 requires full actuator configuration from backend_config: "
                               + ", ".join(missing))
        self.calibrated = np.asarray([
            self.joint_map[name].get("calibrated") is True for name in self.joint_names])
        self._commissioning_index = None

    def enable(self, mask=None, *, commissioning_joint=None):
        selected = np.ones(NJ, bool) if mask is None else np.asarray(mask, bool)
        if selected.shape != (NJ,):
            raise BackendError("enable mask must have one entry per joint")
        if self._commissioning_index is not None and np.any(
                selected & (np.arange(NJ) != self._commissioning_index)):
            raise BackendError("commissioning may enable exactly one joint at a time")
        invalid = selected & (~np.isin(self.direction, [-1, 1]) | ~np.isfinite(self.offset))
        if invalid.any():
            raise BackendError("joint calibration requires direction +/-1 and a finite motor offset")
        override = None
        if commissioning_joint is not None:
            indices = np.flatnonzero(selected)
            if (len(indices) != 1 or self.joint_names[indices[0]] != commissioning_joint
                    or np.any(self.on & ~selected)):
                raise BackendError("commissioning may enable exactly one joint at a time")
            override = int(indices[0])
        elif np.any(selected & ~self.calibrated):
            names = [self.joint_names[i] for i in np.flatnonzero(selected & ~self.calibrated)]
            raise BackendError("refusing to enable uncalibrated joints: " + ", ".join(names))
        # Validate the whole selection before sending any ENABLE frame.
        super().enable(selected)
        self._commissioning_index = override

    def disable(self, mask=None):
        super().disable(mask)
        if self._commissioning_index is not None and not self.on[self._commissioning_index]:
            self._commissioning_index = None

    def write(self, cmd):
        active = cmd.mode != int(ControlMode.IDLE)
        allowed = self.calibrated.copy()
        if self._commissioning_index is not None:
            index = self._commissioning_index
            allowed = np.arange(NJ) == index
            if (active[index] and (cmd.mode[index] != int(ControlMode.TORQUE)
                    or not np.isfinite(cmd.effort[index]) or abs(cmd.effort[index]) > 1.0
                    or cmd.kp[index] != 0 or cmd.kd[index] != 0)):
                raise BackendError("commissioning permits only a torque probe up to 1 Nm")
        if np.any(active & ~allowed):
            raise BackendError("refusing motion on uncalibrated joints outside single-joint commissioning")
        super().write(cmd)


def _motor_faults_to_flags(bits: int) -> int:
    """Map the RS02 fault field onto robodog_msgs FAULT_* flags."""
    out = 0
    if bits & (1 << rs.FaultBit.UNDERVOLTAGE):
        out |= int(Fault.UNDERVOLTAGE)
    if bits & (1 << rs.FaultBit.OVERCURRENT):
        out |= int(Fault.OVERCURRENT)
    if bits & (1 << rs.FaultBit.OVERTEMPERATURE):
        out |= int(Fault.OVERTEMPERATURE)
    if bits & ((1 << rs.FaultBit.MAGNETIC_ENCODING) | (1 << rs.FaultBit.HALL_ENCODING)
               | (1 << rs.FaultBit.UNCALIBRATED)):
        out |= int(Fault.ENCODER)
    return out
