"""RS06 private-protocol extended-CAN codec, manual 260713 pp42,57-58.

Frame layout is shared with RS02; physical ranges emphatically are not.
Position uses the manufacturer's C sample endpoints +/-12.57 rad.
"""
from dataclasses import dataclass

from . import robstride02 as _layout
from .robstride02 import (CommType, FaultBit, MotorMode, Feedback, from_uint16,
                         to_uint16, make_id, split_id, encode_enable, encode_stop,
                         encode_get_device_id, encode_set_mechanical_zero,
                         encode_read_param, encode_write_param)

P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -50.0, 50.0
T_MIN, T_MAX = -36.0, 36.0
KP_MIN, KP_MAX = 0.0, 5000.0
KD_MIN, KD_MAX = 0.0, 100.0
TEMP_SCALE_C = 0.1


def encode_motion_control(motor_id, position, velocity, torque, kp, kd):
    return _layout.encode_motion_control(
        motor_id, position, velocity, torque, kp, kd,
        ranges=(P_MIN, P_MAX, V_MIN, V_MAX, T_MIN, T_MAX,
                KP_MIN, KP_MAX, KD_MIN, KD_MAX))


def decode_feedback(can_id, data):
    return _layout.decode_feedback(can_id, data,
                                   ranges=(P_MIN, P_MAX, V_MIN, V_MAX, T_MIN, T_MAX))


@dataclass(frozen=True)
class FaultFeedback:
    motor_id: int
    fault_bytes: bytes
    warning_bytes: bytes


def decode_fault_feedback(can_id: int, data: bytes) -> FaultFeedback:
    """Private type21, manual260713 p46: fault bytes0..3, warnings4..7.

    This table does not explicitly establish word byte order. Preserve raw
    fields instead of guessing exact fault meanings. Any nonzero fault field
    triggers the backend's protective fallback independently of endianness.
    Warning-only packets are not fault packets. Verify an installed firmware
    trace before assigning individual type21 bits to named fault causes.
    """
    comm, field, _ = split_id(can_id)
    if comm != CommType.FAULT_FEEDBACK:
        raise ValueError(f"expected FAULT_FEEDBACK (21), got comm type {comm}")
    if len(data) != 8:
        raise ValueError(f"expected 8 data bytes, got {len(data)}")
    return FaultFeedback(field & 0xFF, bytes(data[:4]), bytes(data[4:]))
