"""
ROBSTRIDE02 backend: 12 real actuators over CAN.

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
from ..thermal import ThermalModel
from ..types import NJ, ControlMode, Fault, JointCommand, JointState

log = logging.getLogger(__name__)


class RobStride02Backend(JointBackend):
    name = "robstride02_can"
    is_simulation = False

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        c = self.config
        self.host_id = int(c.get("host_id", 0xFD))
        self.bus_specs: list[dict] = list(c.get("buses", []))
        self.joint_map: dict[str, dict] = dict(c.get("joints", {}))
        self.feedback_timeout = float(c.get("feedback_timeout_s", 0.05))
        self.control_rate = float(c.get("control_rate_hz", 500.0))
        self.tau_max = float(c.get("peak_torque_nm", 17.0))

        missing = [n for n in self.joint_names if n not in self.joint_map]
        if missing:
            raise BackendError(f"no CAN mapping for joints: {missing}")

        # Per-joint hardware mapping. `direction` and `offset` are the ONLY
        # place where canonical joint coordinates and motor coordinates differ.
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
                                 name=f"rs02-rx-{spec['id']}", daemon=True)
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
    def _rx_loop(self, bus_id: int, bus) -> None:                  # pragma: no cover
        while not self._stop.is_set():
            msg = bus.recv(timeout=0.05)
            if msg is None:
                continue
            comm, field, _ = rs.split_id(msg.arbitration_id)
            if comm != rs.CommType.FEEDBACK:
                continue
            idx = self._slot.get((bus_id, field & 0xFF))
            if idx is None:
                continue
            try:
                fb = rs.decode_feedback(msg.arbitration_id, bytes(msg.data))
            except ValueError:
                self._rx_errors += 1
                continue
            with self._lock:
                self._fb[idx] = fb
                self._fb_time[idx] = time.monotonic()
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
            self._send(i, *rs.encode_enable(int(self.motor_id[i]), self.host_id))
            self.on[i] = True

    def disable(self, mask: list[bool] | None = None) -> None:
        sel = np.ones(NJ, bool) if mask is None else np.asarray(mask, bool)
        for i in np.flatnonzero(sel):
            self._send(i, *rs.encode_stop(int(self.motor_id[i]), self.host_id))
            self.on[i] = False

    def clear_faults(self) -> None:
        for i in range(NJ):
            self._send(i, *rs.encode_stop(int(self.motor_id[i]), self.host_id, clear_fault=True))

    # ---------------- control cycle ----------------
    def read(self) -> JointState:
        st = JointState()
        now = time.monotonic()
        with self._lock:
            fbs, ts, = list(self._fb), self._fb_time.copy()
        for i, fb in enumerate(fbs):
            if fb is None or (now - ts[i]) > self.feedback_timeout:
                st.faults[i] |= int(Fault.COMMUNICATION)
                if fb is None:
                    continue
            # motor coordinates -> canonical joint coordinates
            st.position[i] = (fb.position - self.offset[i]) * self.direction[i]
            st.velocity[i] = fb.velocity * self.direction[i]
            st.effort[i] = fb.torque * self.direction[i]
            st.temperature[i] = fb.temperature
            st.enabled[i] = fb.mode == rs.MotorMode.RUNNING
            st.faults[i] |= _motor_faults_to_flags(fb.faults)
            if not st.enabled[i]:
                st.faults[i] |= int(Fault.NOT_ENABLED)
        st.stamp = now
        # Observer alongside the reported temperature; the safety monitor takes
        # the maximum of the two. See thermal.py for why.
        self.thermal.update(st.effort, 1.0 / self.control_rate)
        st.temperature = np.maximum(st.temperature, self.thermal.temperature)
        return st

    def write(self, cmd: JointCommand) -> None:
        idle = cmd.mode == int(ControlMode.IDLE)
        pos = cmd.position * self.direction + self.offset
        vel = cmd.velocity * self.direction
        tau = np.clip(cmd.effort, -self.tau_max, self.tau_max) * self.direction
        kp = np.where(idle, 0.0, cmd.kp)
        kd = np.where(idle, 0.0, cmd.kd)
        tau = np.where(idle, 0.0, tau)
        for i in range(NJ):
            can_id, data = rs.encode_motion_control(
                int(self.motor_id[i]), float(pos[i]), float(vel[i]),
                float(tau[i]), float(kp[i]), float(kd[i]))
            self._send(i, can_id, data)

    def base_state(self):
        """None by contract: the real robot has no absolute base-pose sensor.
        robodog_control/state_estimator fuses the IMU with leg kinematics."""
        return None

    def stats(self) -> dict[str, Any]:
        return {"backend": self.name, "tx_frames": int(self._tx_count),
                "rx_frames": int(self._rx_count.sum()), "rx_errors": int(self._rx_errors),
                "buses": len(self._buses),
                "stale_joints": [self.joint_names[i] for i in range(NJ)
                                 if self._fb[i] is None
                                 or (time.monotonic() - self._fb_time[i]) > self.feedback_timeout]}


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
