"""
ROBSTRIDE02 CAN frame codec.

This module is pure: it turns numbers into (can_id, data) and back, and has no
I/O, no threading and no ROS. That is deliberate -- it is the one piece of the
real-hardware path that can be fully tested before any motor is powered, and
the one piece most likely to need a change when the firmware revision differs.

Frame format (extended 29-bit identifiers, classic CAN 2.0B at 1 Mbit/s)
-----------------------------------------------------------------------
    bits 28..24   communication type
    bits 23..8    16-bit data field, meaning depends on the type
    bits  7..0    target id

All 16-bit payload values are BIG-ENDIAN and are linear maps of a physical
range onto 0..65535.

!! VERIFY BEFORE FIRST POWERED RUN !!
The layout below matches the RS02 manual this project was specified against.
RobStride have shipped more than one identifier layout across firmware
revisions. Confirm against the units actually delivered by:
    1. powering one motor alone on the bus,
    2. sending GET_DEVICE_ID and checking the reply,
    3. sending ENABLE and confirming FEEDBACK frames appear at the expected id,
    4. commanding kp=kd=0, torque=0.1 N.m and checking the sign of the motion.
If the layout differs, only this file changes.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

# --------------------------------------------------------------------------- #
# identifier layout
# --------------------------------------------------------------------------- #
COMM_TYPE_SHIFT = 24
COMM_TYPE_MASK = 0x1F
DATA_SHIFT = 8
DATA_MASK = 0xFFFF
TARGET_MASK = 0xFF
EXT_ID_MASK = 0x1FFFFFFF


class CommType(IntEnum):
    GET_DEVICE_ID = 0
    MOTION_CONTROL = 1
    FEEDBACK = 2
    ENABLE = 3
    STOP = 4
    SET_MECHANICAL_ZERO = 6
    SET_CAN_ID = 7
    READ_PARAM = 17          # 0x11
    WRITE_PARAM = 18         # 0x12
    FAULT_FEEDBACK = 21      # 0x15


# --------------------------------------------------------------------------- #
# physical ranges -- must match robstride02.yaml -> can.scaling
# --------------------------------------------------------------------------- #
P_MIN, P_MAX = -12.566, 12.566      # [rad]    +/- 4*pi
V_MIN, V_MAX = -44.0, 44.0          # [rad/s]
T_MIN, T_MAX = -17.0, 17.0          # [N.m]
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
TEMP_SCALE_C = 0.1                  # feedback temperature LSB


class FaultBit(IntEnum):
    """Bits of the fault field in the FEEDBACK identifier (bits 16..21)."""
    UNDERVOLTAGE = 0
    OVERCURRENT = 1
    OVERTEMPERATURE = 2
    MAGNETIC_ENCODING = 3
    HALL_ENCODING = 4
    UNCALIBRATED = 5


class MotorMode(IntEnum):
    """Bits 22..23 of the FEEDBACK identifier."""
    RESET = 0
    CALIBRATING = 1
    RUNNING = 2


# --------------------------------------------------------------------------- #
# fixed-point helpers
# --------------------------------------------------------------------------- #
def to_uint16(value: float, lo: float, hi: float) -> int:
    """Clamp `value` into [lo, hi] and map linearly onto 0..65535.

    Clamping (rather than raising) is intentional: a command that arrives one
    LSB outside the range because of floating point must not drop a control
    cycle. The safety monitor upstream is what actually enforces limits.
    """
    span = hi - lo
    v = lo if value < lo else hi if value > hi else value
    return int(round((v - lo) * 65535.0 / span)) & 0xFFFF


def from_uint16(raw: int, lo: float, hi: float) -> float:
    return (raw & 0xFFFF) * (hi - lo) / 65535.0 + lo


def make_id(comm: CommType, data: int, target: int) -> int:
    return (((int(comm) & COMM_TYPE_MASK) << COMM_TYPE_SHIFT)
            | ((data & DATA_MASK) << DATA_SHIFT)
            | (target & TARGET_MASK))


def split_id(can_id: int) -> tuple[int, int, int]:
    can_id &= EXT_ID_MASK
    return ((can_id >> COMM_TYPE_SHIFT) & COMM_TYPE_MASK,
            (can_id >> DATA_SHIFT) & DATA_MASK,
            can_id & TARGET_MASK)


# --------------------------------------------------------------------------- #
# encoders
# --------------------------------------------------------------------------- #
def encode_motion_control(motor_id: int, position: float, velocity: float,
                          torque: float, kp: float, kd: float) -> tuple[int, bytes]:
    """The impedance command. The feed-forward torque rides in the IDENTIFIER,
    not the payload -- the 8 data bytes are already full."""
    can_id = make_id(CommType.MOTION_CONTROL, to_uint16(torque, T_MIN, T_MAX), motor_id)
    data = struct.pack(">HHHH",
                       to_uint16(position, P_MIN, P_MAX),
                       to_uint16(velocity, V_MIN, V_MAX),
                       to_uint16(kp, KP_MIN, KP_MAX),
                       to_uint16(kd, KD_MIN, KD_MAX))
    return can_id, data


def encode_enable(motor_id: int, host_id: int) -> tuple[int, bytes]:
    return make_id(CommType.ENABLE, host_id, motor_id), bytes(8)


def encode_stop(motor_id: int, host_id: int, clear_fault: bool = False) -> tuple[int, bytes]:
    """Disable output. `clear_fault` also resets a latched motor-side fault."""
    return (make_id(CommType.STOP, host_id, motor_id),
            bytes([1 if clear_fault else 0]) + bytes(7))


def encode_set_mechanical_zero(motor_id: int, host_id: int) -> tuple[int, bytes]:
    """Define the CURRENT shaft position as zero. Destructive: it invalidates
    every stored joint offset, so the driver refuses it unless explicitly asked
    for by the calibration tool."""
    return make_id(CommType.SET_MECHANICAL_ZERO, host_id, motor_id), bytes([1]) + bytes(7)


def encode_set_can_id(motor_id: int, host_id: int, new_id: int) -> tuple[int, bytes]:
    return make_id(CommType.SET_CAN_ID, (new_id << 8) | (host_id & 0xFF), motor_id), bytes(8)


def encode_read_param(motor_id: int, host_id: int, index: int) -> tuple[int, bytes]:
    return (make_id(CommType.READ_PARAM, host_id, motor_id),
            struct.pack("<H", index & 0xFFFF) + bytes(6))


def encode_write_param(motor_id: int, host_id: int, index: int,
                       value: float, as_int: bool = False) -> tuple[int, bytes]:
    payload = struct.pack("<i", int(value)) if as_int else struct.pack("<f", float(value))
    return (make_id(CommType.WRITE_PARAM, host_id, motor_id),
            struct.pack("<H", index & 0xFFFF) + bytes(2) + payload)


def encode_get_device_id(motor_id: int, host_id: int) -> tuple[int, bytes]:
    return make_id(CommType.GET_DEVICE_ID, host_id, motor_id), bytes(8)


# --------------------------------------------------------------------------- #
# decoder
# --------------------------------------------------------------------------- #
@dataclass
class Feedback:
    motor_id: int
    position: float        # [rad]
    velocity: float        # [rad/s]
    torque: float          # [N.m]
    temperature: float     # [degC]
    faults: int            # bitmask of FaultBit
    mode: int              # MotorMode

    @property
    def healthy(self) -> bool:
        return self.faults == 0 and self.mode == MotorMode.RUNNING


def decode_feedback(can_id: int, data: bytes) -> Feedback:
    """Decode a type-2 frame. Raises ValueError on anything else, so a caller
    cannot silently mistake a fault frame for a measurement."""
    comm, field, _target = split_id(can_id)
    if comm != CommType.FEEDBACK:
        raise ValueError(f"expected FEEDBACK (2), got comm type {comm}")
    if len(data) != 8:
        raise ValueError(f"expected 8 data bytes, got {len(data)}")
    pos, vel, tor, temp = struct.unpack(">HHHH", data)
    # data field: bits 0..7 motor id, bits 8..13 faults, bits 14..15 mode
    return Feedback(
        motor_id=field & 0xFF,
        position=from_uint16(pos, P_MIN, P_MAX),
        velocity=from_uint16(vel, V_MIN, V_MAX),
        torque=from_uint16(tor, T_MIN, T_MAX),
        temperature=temp * TEMP_SCALE_C,
        faults=(field >> 8) & 0x3F,
        mode=(field >> 14) & 0x03,
    )


def encode_feedback(fb: Feedback, host_id: int) -> tuple[int, bytes]:
    """Inverse of `decode_feedback`. Used by the loopback test double in
    test/test_protocol.py, and by the CAN-level simulator in
    backends/robstride_can.py when it runs without a physical bus."""
    field = ((fb.motor_id & 0xFF) | ((fb.faults & 0x3F) << 8) | ((fb.mode & 0x3) << 14))
    data = struct.pack(">HHHH",
                       to_uint16(fb.position, P_MIN, P_MAX),
                       to_uint16(fb.velocity, V_MIN, V_MAX),
                       to_uint16(fb.torque, T_MIN, T_MAX),
                       int(round(fb.temperature / TEMP_SCALE_C)) & 0xFFFF)
    return make_id(CommType.FEEDBACK, field, host_id), data


# --------------------------------------------------------------------------- #
# resolution notes
# --------------------------------------------------------------------------- #
#   position : 25.13 rad / 65535 = 3.834e-4 rad  -> matches the 14-bit output
#              encoder exactly, so the bus does not lose sensor resolution.
#   velocity : 88 rad/s / 65535  = 1.343e-3 rad/s
#   torque   : 34 N.m  / 65535   = 5.188e-4 N.m
POSITION_LSB_RAD = (P_MAX - P_MIN) / 65535.0
VELOCITY_LSB_RAD_S = (V_MAX - V_MIN) / 65535.0
TORQUE_LSB_NM = (T_MAX - T_MIN) / 65535.0
