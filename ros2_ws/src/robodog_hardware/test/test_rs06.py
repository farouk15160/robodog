"""Public actuator boundary: RS06 wire units and knee load reflection."""
from pathlib import Path
import struct
import time

import numpy as np
import pytest
import yaml

from robodog_hardware.registry import create_backend
from robodog_hardware.transmission import backend_config, joint_limits
from robodog_hardware.types import JOINT_NAMES, JointCommand, ControlMode
from robodog_hardware.backend import BackendError


@pytest.fixture
def spec():
    path = Path(__file__).parents[2] / "robodog_description/config/robstride06.yaml"
    return yaml.safe_load(path.read_text())


def test_knee_ratings_include_efficiency_and_square_inertia(spec):
    limits = joint_limits(spec, JOINT_NAMES)
    assert limits["continuous_torque_nm"][:3] == pytest.approx([8, 8, 15.2])
    assert limits["peak_torque_nm"][:3] == pytest.approx([36, 36, 68.4])
    assert limits["velocity_rad_s"][:3] == pytest.approx([20, 20, 10])
    assert limits["armature_kgm2"][2] == pytest.approx(4 * limits["armature_kgm2"][0])
    assert limits["armature_kgm2"][:3] == pytest.approx([.012, .012, .048])


def test_equal_motor_load_produces_equal_simulated_heating(spec):
    b = create_backend("kinematic", backend_config(spec, JOINT_NAMES))
    b.configure()
    b.enable()
    cmd = JointCommand()
    cmd.mode[:] = int(ControlMode.TORQUE)
    cmd.effort[:] = [8, 8, 15.2] * 4
    b.write(cmd)
    b.step(0.0025)
    assert b.read().temperature == pytest.approx(np.repeat(b.read().temperature[0], 12))
    assert b.tau_max[:3] == pytest.approx([36, 36, 68.4])
    assert b.vel_max[2] == pytest.approx(b.vel_max[0] / 2)


def test_latest_datasheet_stall_rating_does_not_inherit_rotating_rating(spec):
    assert spec["performance"]["rated_torque_nm"] == 11
    assert spec["performance"]["stall_continuous_torque_nm"] == 8
    assert spec["performance"]["peak_torque_duration_s"] == 1
    conditions = spec["performance"]["vendor_curve"]
    assert conditions["heat_sink_dimensions_mm"] == [200, 200]
    assert conditions["stall_peak_duration_s"] == 1
    assert conditions["rotating_peak_duration_s"] == 4


def test_datasheet_electrical_values_drive_copper_loss_without_claiming_thermal_calibration(spec):
    backend = create_backend("kinematic", backend_config(spec, JOINT_NAMES))
    # Independent September17 spec: line resistance0.23ohm, output Kt1.1Nm/Arms.
    # Equivalent-wye conversion Rphase=Rline/2 is an explicit model inference.
    assert spec["electrical"]["line_resistance_ohm"] == pytest.approx(.23)
    expected_copper_w = 3 * (8 / 1.1) ** 2 * (.23 / 2)
    assert backend.thermal.copper_loss(np.array([8]))[0] == pytest.approx(expected_copper_w)
    assert spec["thermal"]["calibrated"] is False
    assert "rotor_inertia_kgm2" not in spec["mechanical"]


def test_rs06_codec_uses_vendor_ranges():
    from robodog_hardware.protocol import robstride06 as rs
    can_id, payload = rs.encode_motion_control(3, 12.57, 50, 36, 5000, 100)
    assert payload == b"\xff" * 8
    assert rs.split_id(can_id) == (1, 65535, 3)
    fb = rs.decode_feedback(rs.make_id(rs.CommType.FEEDBACK, 3 | (2 << 14), 0xFD),
                            struct.pack(">HHHH", 65535, 65535, 65535, 730))
    assert (fb.position, fb.velocity, fb.torque, fb.temperature) == (12.57, 50, 36, 73)


def test_can_commands_and_feedback_are_in_joint_coordinates(spec):
    from robodog_hardware.protocol import robstride06 as rs
    bus = yaml.safe_load((Path(__file__).parents[1] / "config/robstride_bus.yaml").read_text())
    bus = {**bus, "joints": {name: {**entry, "calibrated": True}
                             for name, entry in bus["joints"].items()}}
    b = create_backend("robstride06_can", {**bus, **backend_config(spec, JOINT_NAMES)})
    frames = []
    b._send = lambda i, can_id, data: frames.append((i, can_id, data))
    cmd = JointCommand()
    cmd.mode[:] = int(ControlMode.IMPEDANCE)
    cmd.position[:] = 0.5
    cmd.velocity[:] = 1
    cmd.effort[:] = 19
    cmd.kp[:] = 38
    cmd.kd[:] = 3.8
    b.write(cmd)
    _, ident, data = frames[2]
    p, v, kp, kd = struct.unpack(">HHHH", data)
    assert rs.from_uint16(p, rs.P_MIN, rs.P_MAX) == pytest.approx(1, abs=.001)
    assert rs.from_uint16(v, rs.V_MIN, rs.V_MAX) == pytest.approx(2, abs=.002)
    assert rs.from_uint16(rs.split_id(ident)[1], rs.T_MIN, rs.T_MAX) == pytest.approx(10, abs=.002)
    assert rs.from_uint16(kp, rs.KP_MIN, rs.KP_MAX) == pytest.approx(10, abs=.08)
    assert rs.from_uint16(kd, rs.KD_MIN, rs.KD_MAX) == pytest.approx(1, abs=.002)
    b._fb = [rs.Feedback(1, 1, 2, 10, 25, 0, 2)] * 12
    b._fb_time[:] = time.monotonic()
    state = b.read()
    assert (state.position[2], state.velocity[2], state.effort[2]) == pytest.approx((.5, 1, 19))


def test_rs02_alias_cannot_encode_rs06_configuration(spec):
    bus = yaml.safe_load((Path(__file__).parents[1] / "config/robstride_bus.yaml").read_text())
    with pytest.raises(BackendError, match="ROBSTRIDE06"):
        create_backend("robstride02_can", {**bus, **backend_config(spec, JOINT_NAMES)})


def test_rs06_refuses_incomplete_motor_configuration():
    bus = yaml.safe_load((Path(__file__).parents[1] / "config/robstride_bus.yaml").read_text())
    with pytest.raises(BackendError, match="actuator configuration"):
        create_backend("robstride06_can", bus)


def test_nominal_series_pack_voltage_reduces_speed(spec):
    cfg = backend_config(spec, JOINT_NAMES)
    assert cfg["no_load_speed_rad_s"] == pytest.approx(50.2654825 * 44.4 / 48)


@pytest.mark.parametrize("ratio,efficiency", [(0, 1), (2, 0), (2, 1.01), (float('nan'), 1)])
def test_invalid_transmissions_fail_before_backend_runs(ratio, efficiency):
    with pytest.raises(ValueError, match="transmission"):
        create_backend("kinematic", {"transmission_ratio": ratio, "transmission_efficiency": efficiency})


@pytest.fixture
def offline_can(spec):
    bus = yaml.safe_load((Path(__file__).parents[1] / "config/robstride_bus.yaml").read_text())
    config = {**bus, **backend_config(spec, JOINT_NAMES)}
    return config


def capture_frames(config):
    backend = create_backend("robstride06_can", config)
    frames = []
    backend._send = lambda *frame: frames.append(frame)
    return backend, frames


def test_shipped_uncalibrated_mapping_cannot_enable(offline_can):
    backend, frames = capture_frames(offline_can)
    with pytest.raises(BackendError, match="uncalibrated"):
        backend.enable()
    assert frames == []
    assert not backend.on.any()


def test_only_calibrated_subset_can_enable_atomically(offline_can):
    mapping = {**offline_can["joints"], JOINT_NAMES[0]: {
        **offline_can["joints"][JOINT_NAMES[0]], "calibrated": True}}
    backend, frames = capture_frames({**offline_can, "joints": mapping})
    with pytest.raises(BackendError, match="uncalibrated"):
        backend.enable([True, True] + [False] * 10)
    assert frames == []
    backend.enable([True] + [False] * 11)
    assert [frame[0] for frame in frames] == [0]


def test_commissioning_override_is_explicit_and_single_joint(offline_can):
    backend, frames = capture_frames(offline_can)
    with pytest.raises(BackendError, match="one joint"):
        backend.enable(commissioning_joint=JOINT_NAMES[0])
    assert frames == []
    backend.enable([True] + [False] * 11, commissioning_joint=JOINT_NAMES[0])
    assert [frame[0] for frame in frames] == [0]
    with pytest.raises(BackendError, match="one joint"):
        backend.enable([False, True] + [False] * 10, commissioning_joint=JOINT_NAMES[1])


def test_commissioning_override_does_not_persist_after_disable(offline_can):
    backend, frames = capture_frames(offline_can)
    mask = [True] + [False] * 11
    backend.enable(mask, commissioning_joint=JOINT_NAMES[0])
    backend.disable(mask)
    frames.clear()
    with pytest.raises(BackendError, match="uncalibrated"):
        backend.enable(mask)
    assert frames == []


def test_uncalibrated_motion_cannot_bypass_enable_gate(offline_can):
    backend, frames = capture_frames(offline_can)
    command = JointCommand()
    command.mode[0] = int(ControlMode.TORQUE)
    command.effort[0] = .5
    with pytest.raises(BackendError, match="uncalibrated"):
        backend.write(command)
    assert frames == []


def test_commissioning_cannot_be_combined_with_normal_calibrated_enable(offline_can):
    mapping = {**offline_can["joints"], JOINT_NAMES[1]: {
        **offline_can["joints"][JOINT_NAMES[1]], "calibrated": True}}
    backend, frames = capture_frames({**offline_can, "joints": mapping})
    backend.enable([True] + [False] * 11, commissioning_joint=JOINT_NAMES[0])
    frames.clear()
    with pytest.raises(BackendError, match="one joint"):
        backend.enable([False, True] + [False] * 10)
    assert frames == []


def test_commissioning_rejects_impedance_or_excessive_probe(offline_can):
    backend, frames = capture_frames(offline_can)
    backend.enable([True] + [False] * 11, commissioning_joint=JOINT_NAMES[0])
    frames.clear()
    command = JointCommand()
    command.mode[0] = int(ControlMode.IMPEDANCE)
    command.kp[0] = 1
    with pytest.raises(BackendError, match="torque probe"):
        backend.write(command)
    assert frames == []
    command.mode[0] = int(ControlMode.TORQUE)
    command.kp[0] = 0
    command.effort[0] = 1.01
    with pytest.raises(BackendError, match="torque probe"):
        backend.write(command)
    assert frames == []
