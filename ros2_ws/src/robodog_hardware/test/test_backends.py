"""Backend-boundary tests: the contract every backend must satisfy, the
thermal model, and the parts of the CAN backend that work without hardware."""
import os

import numpy as np
import pytest
import yaml

from robodog_hardware.backend import BackendError, JointBackend
from robodog_hardware.registry import available, create_backend
from robodog_hardware.thermal import ThermalModel
from robodog_hardware.types import NJ, ControlMode, Fault, JointCommand, JointState

CFG = os.path.join(os.path.dirname(__file__), "..", "config", "robstride_bus.yaml")


# --------------------------------------------------------------------------- #
# the boundary contract
# --------------------------------------------------------------------------- #
def test_registry_lists_all_three_backends():
    assert set(available()) == {"kinematic", "mujoco", "robstride02_can"}


def test_unknown_backend_fails_loudly():
    with pytest.raises(BackendError, match="unknown backend"):
        create_backend("does_not_exist")


def test_joint_order_is_canonical_and_fixed():
    from robodog_hardware.types import JOINT_NAMES
    assert len(JOINT_NAMES) == 12
    assert JOINT_NAMES[0] == "FL_haa_joint"
    assert JOINT_NAMES[-1] == "RR_kfe_joint"
    assert [n.split("_")[0] for n in JOINT_NAMES[:3]] == ["FL"] * 3


@pytest.fixture
def kin():
    b = create_backend("kinematic", {"base_height_m": 0.32})
    b.configure()
    b.enable()
    return b


def test_backend_satisfies_the_abc(kin):
    assert isinstance(kin, JointBackend)
    assert kin.is_simulation
    st = kin.read()
    assert isinstance(st, JointState)
    for a in (st.position, st.velocity, st.effort, st.temperature):
        assert a.shape == (NJ,)


def test_read_returns_a_snapshot_not_a_live_view(kin):
    """A caller that holds a JointState across a step must not see it mutate."""
    st = kin.read()
    before = st.position.copy()
    cmd = JointCommand()
    cmd.mode[:] = int(ControlMode.IMPEDANCE)
    cmd.position[:] = 1.0
    cmd.kp[:] = 60.0
    kin.write(cmd)
    for _ in range(100):
        kin.step(0.002)
    assert np.array_equal(st.position, before)


# --------------------------------------------------------------------------- #
# closed-loop behaviour
# --------------------------------------------------------------------------- #
def test_impedance_command_converges_to_the_setpoint(kin):
    target = np.linspace(-0.4, 0.4, NJ)
    cmd = JointCommand()
    cmd.mode[:] = int(ControlMode.IMPEDANCE)
    cmd.position[:] = target
    cmd.kp[:] = 120.0
    cmd.kd[:] = 3.0
    for _ in range(4000):                       # 8 s at 500 Hz
        kin.write(cmd)
        kin.step(0.002)
    assert kin.read().position == pytest.approx(target, abs=0.02)


def test_disabled_joints_produce_no_torque(kin):
    kin.disable()
    cmd = JointCommand()
    cmd.mode[:] = int(ControlMode.IMPEDANCE)
    cmd.position[:] = 1.0
    cmd.kp[:] = 200.0
    kin.write(cmd)
    for _ in range(50):
        kin.step(0.002)
    assert np.allclose(kin.read().effort, 0.0)


def test_idle_mode_produces_no_torque_even_with_gains(kin):
    cmd = JointCommand()
    cmd.mode[:] = int(ControlMode.IDLE)
    cmd.position[:] = 1.0
    cmd.kp[:] = 500.0
    kin.write(cmd)
    for _ in range(50):
        kin.step(0.002)
    assert np.allclose(kin.read().effort, 0.0)


def test_torque_is_clamped_to_the_actuator_peak(kin):
    cmd = JointCommand()
    cmd.mode[:] = int(ControlMode.IMPEDANCE)
    cmd.position[:] = 50.0                      # absurd error
    cmd.kp[:] = 500.0
    kin.write(cmd)
    kin.step(0.002)
    assert np.all(np.abs(kin.read().effort) <= 17.0 + 1e-9)


def test_simulation_backend_reports_ground_truth_base_state(kin):
    bs = kin.base_state()
    assert bs is not None and bs.ground_truth


# --------------------------------------------------------------------------- #
# thermal model
# --------------------------------------------------------------------------- #
def test_rated_torque_settles_near_the_calibration_point():
    """robstride02.yaml sets R_th so that continuous rated torque (6 N.m)
    settles around 80 C from 20 C ambient. If that drifts, the temperature
    limits in safety.yaml no longer mean what the docs say."""
    t = ThermalModel(1, torque_constant=1.22, phase_resistance=0.29, r_th=2.84,
                     c_th=190.0, ambient_c=20.0)
    assert t.steady_state(6.0) == pytest.approx(80.0, abs=2.0)


def test_temperature_rises_under_load_and_decays_when_free():
    t = ThermalModel(2, torque_constant=1.22, phase_resistance=0.29, r_th=2.84, c_th=190.0)
    tau = np.array([6.0, 0.0])
    for _ in range(6000):
        t.update(tau, 0.01)                     # 60 s
    hot, cold = t.temperature.copy()
    assert hot > 21.0 and cold == pytest.approx(20.0)
    for _ in range(60000):
        t.update(np.zeros(2), 0.01)             # 600 s free
    assert t.temperature[0] < hot


def test_time_to_limit_is_infinite_below_the_steady_state():
    t = ThermalModel(1, torque_constant=1.22, phase_resistance=0.29, r_th=2.84, c_th=190.0)
    assert np.isinf(t.time_to_limit(np.array([2.0]), 100.0)[0])
    assert np.isfinite(t.time_to_limit(np.array([15.0]), 100.0)[0])


def test_copper_loss_matches_the_datasheet_at_rated_current():
    """6 N.m / 1.22 = 4.92 Arms; 3 * I^2 * 0.29 should be ~21 W."""
    t = ThermalModel(1, torque_constant=1.22, phase_resistance=0.29, r_th=2.84, c_th=190.0)
    assert float(t.copper_loss(np.array([6.0]))[0]) == pytest.approx(21.0, abs=0.5)


# --------------------------------------------------------------------------- #
# CAN backend: everything testable without a bus
# --------------------------------------------------------------------------- #
@pytest.fixture
def bus_cfg():
    with open(CFG) as f:
        return yaml.safe_load(f)


def test_bus_config_maps_every_joint(bus_cfg):
    from robodog_hardware.types import JOINT_NAMES
    assert set(bus_cfg["joints"]) == set(JOINT_NAMES)


def test_motor_ids_are_unique_per_bus(bus_cfg):
    seen = set()
    for name, j in bus_cfg["joints"].items():
        key = (j["bus"], j["motor_id"])
        assert key not in seen, f"{name} collides on {key}"
        seen.add(key)


def test_can_backend_rejects_a_missing_joint_mapping(bus_cfg):
    bus_cfg["joints"].pop("RR_kfe_joint")
    with pytest.raises(BackendError, match="no CAN mapping"):
        create_backend("robstride02_can", bus_cfg)


def test_shipped_bus_configuration_fits_within_budget(bus_cfg):
    """The configuration we actually ship must pass its own budget check."""
    assert create_backend("robstride02_can", bus_cfg).check_bus_budget() == []


def test_single_bus_is_rejected(bus_cfg):
    """12 motors on one 1 Mbit/s bus at 400 Hz is 1.44 Mbit/s. The backend must
    refuse rather than silently drop frames."""
    for j in bus_cfg["joints"].values():
        j["bus"] = 0
    bus_cfg["buses"] = [bus_cfg["buses"][0]]
    assert create_backend("robstride02_can", bus_cfg).check_bus_budget()


def test_two_buses_at_500hz_is_rejected_for_lack_of_margin(bus_cfg):
    """90% load leaves nothing for error frames or retransmission."""
    bus_cfg["control_rate_hz"] = 500
    assert create_backend("robstride02_can", bus_cfg).check_bus_budget()


def test_real_backend_declares_itself_not_a_simulation(bus_cfg):
    assert create_backend("robstride02_can", bus_cfg).is_simulation is False


def test_real_backend_has_no_ground_truth_base_state(bus_cfg):
    """Contract: consumers must handle None and use the state estimator."""
    assert create_backend("robstride02_can", bus_cfg).base_state() is None


def test_fault_bits_map_onto_ros_flags():
    from robodog_hardware.backends.robstride_can import _motor_faults_to_flags
    from robodog_hardware.protocol.robstride02 import FaultBit
    assert _motor_faults_to_flags(1 << FaultBit.OVERTEMPERATURE) == int(Fault.OVERTEMPERATURE)
    assert _motor_faults_to_flags(1 << FaultBit.HALL_ENCODING) == int(Fault.ENCODER)
    assert _motor_faults_to_flags(0) == 0
