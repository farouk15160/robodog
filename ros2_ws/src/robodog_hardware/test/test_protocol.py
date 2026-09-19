"""
ROBSTRIDE02 codec tests.

These run with no hardware and no simulator, and they are the only pre-hardware
evidence that the CAN layer is right. Each one encodes a property taken
directly from the datasheet, so a firmware revision that changes the layout
fails here rather than on the bench.
"""
import struct

import pytest

from robodog_hardware.protocol import robstride02 as rs


# --------------------------------------------------------------------------- #
# identifier layout
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("comm,data,target", [
    (rs.CommType.MOTION_CONTROL, 0x1234, 0x05),
    (rs.CommType.FEEDBACK, 0xFFFF, 0xFF),
    (rs.CommType.ENABLE, 0x00FD, 0x01),
    (rs.CommType.WRITE_PARAM, 0x0000, 0x00),
])
def test_identifier_round_trip(comm, data, target):
    assert rs.split_id(rs.make_id(comm, data, target)) == (int(comm), data, target)


def test_identifier_fits_29_bits():
    assert rs.make_id(rs.CommType.FAULT_FEEDBACK, 0xFFFF, 0xFF) <= rs.EXT_ID_MASK


def test_comm_type_occupies_the_top_five_bits():
    assert rs.make_id(rs.CommType.WRITE_PARAM, 0, 0) >> 24 == 18


# --------------------------------------------------------------------------- #
# fixed-point scaling
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("lo,hi", [(rs.P_MIN, rs.P_MAX), (rs.V_MIN, rs.V_MAX),
                                   (rs.T_MIN, rs.T_MAX), (rs.KP_MIN, rs.KP_MAX),
                                   (rs.KD_MIN, rs.KD_MAX)])
def test_scaling_endpoints_are_exact(lo, hi):
    assert rs.to_uint16(lo, lo, hi) == 0
    assert rs.to_uint16(hi, lo, hi) == 0xFFFF
    assert rs.from_uint16(0, lo, hi) == pytest.approx(lo)
    assert rs.from_uint16(0xFFFF, lo, hi) == pytest.approx(hi)


@pytest.mark.parametrize("value", [-12.0, -1.0, 0.0, 0.3333, 7.5, 12.5])
def test_position_round_trip_within_one_lsb(value):
    back = rs.from_uint16(rs.to_uint16(value, rs.P_MIN, rs.P_MAX), rs.P_MIN, rs.P_MAX)
    assert abs(back - value) <= rs.POSITION_LSB_RAD


def test_out_of_range_clamps_rather_than_wrapping():
    """A value one LSB outside the range must saturate. Wrapping would turn a
    small overshoot into a full-scale command -- the worst possible failure."""
    assert rs.to_uint16(1e9, rs.T_MIN, rs.T_MAX) == 0xFFFF
    assert rs.to_uint16(-1e9, rs.T_MIN, rs.T_MAX) == 0
    assert rs.from_uint16(rs.to_uint16(100.0, rs.T_MIN, rs.T_MAX),
                          rs.T_MIN, rs.T_MAX) == pytest.approx(rs.T_MAX)


def test_position_lsb_matches_the_14_bit_output_encoder():
    """25.13 rad / 65535 must not be coarser than 2*pi / 16384, otherwise the
    bus throws away sensor resolution the actuator paid for."""
    assert rs.POSITION_LSB_RAD == pytest.approx(3.835e-4, rel=1e-3)


# --------------------------------------------------------------------------- #
# motion control frame
# --------------------------------------------------------------------------- #
def test_motion_control_carries_torque_in_the_identifier():
    """The 8 payload bytes are full (angle, velocity, kp, kd), so torque must
    ride in the identifier's data field."""
    cid, data = rs.encode_motion_control(7, 0.0, 0.0, 5.0, 0.0, 0.0)
    comm, field, target = rs.split_id(cid)
    assert comm == rs.CommType.MOTION_CONTROL
    assert target == 7
    assert rs.from_uint16(field, rs.T_MIN, rs.T_MAX) == pytest.approx(5.0, abs=rs.TORQUE_LSB_NM)
    assert len(data) == 8


def test_motion_control_payload_is_big_endian_in_documented_order():
    cid, data = rs.encode_motion_control(1, 1.0, 2.0, 0.0, 100.0, 2.0)
    pos, vel, kp, kd = struct.unpack(">HHHH", data)
    assert rs.from_uint16(pos, rs.P_MIN, rs.P_MAX) == pytest.approx(1.0, abs=1e-3)
    assert rs.from_uint16(vel, rs.V_MIN, rs.V_MAX) == pytest.approx(2.0, abs=1e-2)
    assert rs.from_uint16(kp, rs.KP_MIN, rs.KP_MAX) == pytest.approx(100.0, abs=1e-1)
    assert rs.from_uint16(kd, rs.KD_MIN, rs.KD_MAX) == pytest.approx(2.0, abs=1e-3)


def test_zero_gain_command_encodes_exactly_zero():
    """Torque-only mode must not leak stiffness through rounding."""
    _, data = rs.encode_motion_control(1, 3.0, 4.0, 0.0, 0.0, 0.0)
    _, _, kp, kd = struct.unpack(">HHHH", data)
    assert (kp, kd) == (0, 0)


# --------------------------------------------------------------------------- #
# feedback frame
# --------------------------------------------------------------------------- #
def test_feedback_round_trip():
    fb = rs.Feedback(motor_id=9, position=-1.25, velocity=3.5, torque=-2.75,
                     temperature=47.3, faults=0, mode=rs.MotorMode.RUNNING)
    got = rs.decode_feedback(*rs.encode_feedback(fb, host_id=0xFD))
    assert got.motor_id == 9
    assert got.position == pytest.approx(-1.25, abs=rs.POSITION_LSB_RAD)
    assert got.velocity == pytest.approx(3.5, abs=rs.VELOCITY_LSB_RAD_S)
    assert got.torque == pytest.approx(-2.75, abs=rs.TORQUE_LSB_NM)
    assert got.temperature == pytest.approx(47.3, abs=0.05)
    assert got.healthy


def test_feedback_reports_faults_and_mode():
    fb = rs.Feedback(3, 0, 0, 0, 30.0,
                     faults=(1 << rs.FaultBit.OVERTEMPERATURE) | (1 << rs.FaultBit.UNDERVOLTAGE),
                     mode=rs.MotorMode.RESET)
    got = rs.decode_feedback(*rs.encode_feedback(fb, 0xFD))
    assert got.faults & (1 << rs.FaultBit.OVERTEMPERATURE)
    assert got.faults & (1 << rs.FaultBit.UNDERVOLTAGE)
    assert not got.healthy


def test_decode_rejects_a_non_feedback_frame():
    """A fault frame must never be mistaken for a measurement."""
    cid, data = rs.encode_motion_control(1, 0, 0, 0, 0, 0)
    with pytest.raises(ValueError):
        rs.decode_feedback(cid, data)


def test_decode_rejects_a_short_frame():
    cid, _ = rs.encode_feedback(rs.Feedback(1, 0, 0, 0, 20.0, 0, 2), 0xFD)
    with pytest.raises(ValueError):
        rs.decode_feedback(cid, b"\x00" * 4)


# --------------------------------------------------------------------------- #
# management frames
# --------------------------------------------------------------------------- #
def test_stop_can_clear_a_latched_motor_fault():
    _, plain = rs.encode_stop(1, 0xFD)
    _, clear = rs.encode_stop(1, 0xFD, clear_fault=True)
    assert plain[0] == 0 and clear[0] == 1


def test_write_param_encodes_float_little_endian():
    _, data = rs.encode_write_param(1, 0xFD, index=0x7005, value=1.5)
    assert struct.unpack("<H", data[0:2])[0] == 0x7005
    assert struct.unpack("<f", data[4:8])[0] == pytest.approx(1.5)


def test_set_mechanical_zero_is_distinct_from_stop():
    """These two must never collide: one is routine, the other destroys every
    stored joint offset."""
    z, _ = rs.encode_set_mechanical_zero(1, 0xFD)
    s, _ = rs.encode_stop(1, 0xFD)
    assert rs.split_id(z)[0] != rs.split_id(s)[0]
