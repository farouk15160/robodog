"""
Safety monitor tests.

These are the specification of the safety envelope. Each one names a failure
the layer exists to prevent, so the assertion doubles as the rationale.
"""
import os

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from robodog_control.safety import SafetyLimits, SafetyMonitor
from robodog_hardware.types import NJ, ControlMode, Fault, JointCommand, JointState

NAMES = [f"{l}_{k}_joint" for l in ("FL", "FR", "RL", "RR") for k in ("haa", "hfe", "kfe")]
RATE = 400.0


@pytest.fixture(scope="module")
def cfg():
    share = get_package_share_directory("robodog_description")
    with open(os.path.join(share, "config", "robot_parameters.yaml")) as f:
        P = yaml.safe_load(f)
    with open(os.path.join(share, "config", "robstride02.yaml")) as f:
        RS = yaml.safe_load(f)
    return P, RS


@pytest.fixture
def mon(cfg):
    P, RS = cfg
    return SafetyMonitor(SafetyLimits.from_config(P, RS, NAMES), NAMES, RATE)


def state(**kw) -> JointState:
    s = JointState()
    for k, v in kw.items():
        getattr(s, k)[:] = v
    return s


def mid(mon) -> np.ndarray:
    """A valid target for every joint: the middle of its soft range. The three
    joint kinds have different ranges (KFE is [-2.60, 0]), so a single scalar
    target is not legal for all twelve."""
    return (mon.lim.position_lower + mon.lim.position_upper) / 2.0


def stand_q(P) -> np.ndarray:
    p = P["named_poses"]["stand"]
    return np.array([p["haa"], p["hfe"], p["kfe"]] * 4)


def impedance(pos=0.0, kp=100.0, kd=2.0, eff=0.0) -> JointCommand:
    c = JointCommand()
    c.mode[:] = int(ControlMode.IMPEDANCE)
    c.position[:] = pos
    c.kp[:] = kp
    c.kd[:] = kd
    c.effort[:] = eff
    return c


# --------------------------------------------------------------------------- #
# limits derived from configuration
# --------------------------------------------------------------------------- #
def test_soft_limits_are_inset_from_the_urdf_limits(cfg, mon):
    P, RS = cfg
    margin = RS["operational_limits"]["position_margin_rad"]
    for i, n in enumerate(NAMES):
        kind = n.split("_")[1]
        assert mon.lim.position_lower[i] == pytest.approx(P["joint_limits"][kind]["lower"] + margin)
        assert mon.lim.position_upper[i] == pytest.approx(P["joint_limits"][kind]["upper"] - margin)


# --------------------------------------------------------------------------- #
# position
# --------------------------------------------------------------------------- #
def test_a_command_beyond_the_soft_limit_is_clamped_not_dropped(mon):
    """Dropping a cycle mid-stride is more dangerous than saturating one."""
    out, rep = mon.apply(state(), impedance(pos=99.0, kp=0.0, kd=0.0))
    assert np.all(out.position <= mon.lim.position_upper + 1e-9)
    assert rep.faults.any()
    assert int(rep.faults[0]) & int(Fault.POSITION_LIMIT)


def test_position_setpoints_are_rate_limited(mon):
    """A step setpoint into a stiff impedance loop is a torque step. The limiter
    turns it into a ramp so nothing slams into an end stop."""
    m = state()
    target = mid(mon)
    first, _ = mon.apply(m, impedance(pos=target, kp=0.0, kd=0.0))
    assert np.all(np.abs(first.position) <= mon.lim.max_position_step_rad + 1e-12)
    steps = int(np.ceil(np.abs(target).max() / mon.lim.max_position_step_rad)) + 2
    for _ in range(steps):
        out, _ = mon.apply(m, impedance(pos=target, kp=0.0, kd=0.0))
    assert out.position == pytest.approx(target, abs=1e-6)


def test_idle_tracks_the_measurement_so_the_next_move_starts_from_reality(mon, cfg):
    """Regression: an IDLE command carries no setpoint. If the limiter kept the
    placeholder zero, the first enabled cycle would ramp from a position the
    robot was never at and demand full torque -- a real jolt on hardware."""
    q = stand_q(cfg[0])
    m = state(position=q)
    idle = JointCommand()
    idle.mode[:] = int(ControlMode.IDLE)
    idle.position[:] = q
    for _ in range(10):
        mon.apply(m, idle)
    out, rep = mon.apply(m, impedance(pos=q, kp=120.0, kd=2.0))
    assert out.position == pytest.approx(q, abs=1e-9)
    assert not np.any(rep.faults & np.uint16(Fault.TORQUE_LIMIT))


def test_a_joint_measured_past_its_hard_limit_raises_a_fault(mon):
    _, rep = mon.apply(state(position=5.0), impedance(kp=0.0, kd=0.0))
    assert int(rep.faults[0]) & int(Fault.POSITION_LIMIT)


# --------------------------------------------------------------------------- #
# velocity and torque
# --------------------------------------------------------------------------- #
def test_velocity_command_is_clamped_to_the_operational_limit(mon):
    c = impedance(kp=0.0, kd=0.0)
    c.velocity[:] = 100.0
    out, _ = mon.apply(state(), c)
    assert np.all(out.velocity <= mon.lim.velocity_max + 1e-9)


def test_impedance_torque_is_limited_by_scaling_the_whole_triple(mon):
    """The motor computes kp*e + kd*de + tau_ff in firmware, so clamping only
    `effort` would not limit the torque. Scaling kp, kd and effort together
    limits the magnitude while preserving the commanded impedance direction."""
    m = state(position=0.0)
    c = impedance(pos=1.0, kp=500.0, kd=0.0, eff=0.0)   # 500 N.m demanded
    out, rep = mon.apply(m, c)
    tau = out.kp[0] * (out.position[0] - m.position[0]) + out.effort[0]
    assert abs(tau) <= mon.lim.peak_torque + 1e-6
    assert out.kd[0] / out.kp[0] == pytest.approx(c.kd[0] / c.kp[0]) if c.kd[0] else True
    assert int(rep.faults[0]) & int(Fault.TORQUE_LIMIT)


def test_i2t_pulls_the_budget_back_to_continuous_after_sustained_overload(mon):
    """Peak torque is allowed briefly, then the limit must fall to continuous,
    or a stalled leg cooks its winding while reporting a healthy temperature."""
    m = state(position=0.0)
    c = impedance(pos=1.0, kp=500.0, kd=0.0)
    first, _ = mon.apply(m, c)
    tau_first = abs(first.kp[0] * (first.position[0] - m.position[0]))
    for _ in range(int(RATE * 4)):                      # 4 s of hard overload
        out, _ = mon.apply(m, c)
    tau_late = abs(out.kp[0] * (out.position[0] - m.position[0]))
    assert tau_late < tau_first
    assert tau_late <= mon.lim.continuous_torque * 1.05


def test_i2t_recovers_when_the_load_goes_away(mon):
    m = state(position=0.0)
    hard = impedance(pos=1.0, kp=500.0, kd=0.0)
    for _ in range(int(RATE * 4)):
        mon.apply(m, hard)
    assert mon.i2t.max() > 0
    idle = JointCommand()
    idle.mode[:] = int(ControlMode.IDLE)
    for _ in range(int(RATE * 30)):
        mon.apply(m, idle)
    assert mon.i2t.max() == pytest.approx(0.0, abs=1e-6)


# --------------------------------------------------------------------------- #
# thermal
# --------------------------------------------------------------------------- #
def test_torque_derates_linearly_between_the_derate_and_fault_temperatures(mon):
    """Cutting output abruptly at a threshold would drop the robot."""
    lim = mon.lim
    mid = (lim.temp_derate_c + lim.temp_fault_c) / 2.0
    m = state(position=0.0, temperature=mid)
    out, _ = mon.apply(m, impedance(pos=1.0, kp=500.0, kd=0.0))
    tau = abs(out.kp[0] * (out.position[0] - m.position[0]))
    assert tau == pytest.approx(lim.peak_torque * 0.5, rel=0.1)


def test_warning_temperature_raises_a_flag_without_stopping_the_robot(mon):
    m = state(temperature=mon.lim.temp_warn_c + 1.0)
    out, rep = mon.apply(m, impedance(pos=0.0, kp=100.0, kd=2.0))
    assert int(rep.faults[0]) & int(Fault.OVERTEMPERATURE)
    assert not rep.estop


def test_fault_temperature_latches_an_estop(mon):
    m = state(temperature=mon.lim.temp_fault_c + 5.0)
    out, rep = mon.apply(m, impedance(pos=0.0, kp=100.0))
    assert rep.estop and rep.latched
    assert np.all(out.mode == int(ControlMode.IDLE))
    assert np.all(out.kp == 0.0) and np.all(out.effort == 0.0)


# --------------------------------------------------------------------------- #
# watchdog and e-stop
# --------------------------------------------------------------------------- #
def test_a_stale_command_falls_back_to_holding_position_not_going_limp(mon, cfg):
    """Going limp mid-stance drops the robot; holding is the safe fallback."""
    q = stand_q(cfg[0])
    m = state(position=q)
    out, rep = mon.apply(m, impedance(pos=mid(mon), kp=200.0), command_age_s=1.0)
    assert np.any(rep.faults & np.uint16(Fault.WATCHDOG))
    assert np.all(out.mode == int(ControlMode.IMPEDANCE))
    assert np.all(out.kp > 0.0), "holding must keep stiffness, not go limp"
    assert out.position == pytest.approx(q, abs=1e-6)


def test_holding_pulls_a_joint_measured_outside_its_soft_range_back_inside(mon):
    """If a joint has been pushed past the soft limit, the hold target is the
    limit, not the measurement -- so the watchdog recovers the envelope rather
    than freezing outside it. The rate limiter keeps the return gentle."""
    over = mon.lim.position_upper + 0.2
    m = state(position=over)
    first, _ = mon.apply(m, impedance(), command_age_s=1.0)
    # walks back at the rate limit rather than jumping
    assert np.all(first.position < over)
    assert np.all(over - first.position <= mon.lim.max_position_step_rad + 1e-12)
    for _ in range(10):
        out, _ = mon.apply(m, impedance(), command_age_s=1.0)
    assert np.all(out.position <= mon.lim.position_upper + 1e-9)


def test_estop_zeroes_every_output(mon):
    mon.engage_estop("test")
    out, rep = mon.apply(state(), impedance(pos=1.0, kp=500.0, eff=10.0))
    assert rep.estop
    assert np.all(out.kp == 0.0) and np.all(out.kd == 0.0) and np.all(out.effort == 0.0)
    assert np.all(out.mode == int(ControlMode.IDLE))


def test_estop_cannot_be_cleared_while_the_cause_is_still_present(mon):
    hot = state(temperature=mon.lim.temp_fault_c + 5.0)
    mon.apply(hot, impedance())
    ok, msg = mon.clear_estop(hot)
    assert not ok and "cannot clear" in msg
    assert mon.latched


def test_estop_clears_once_the_cause_is_gone(mon):
    hot = state(temperature=mon.lim.temp_fault_c + 5.0)
    mon.apply(hot, impedance())
    ok, _ = mon.clear_estop(state(temperature=25.0))
    assert ok and not mon.latched


def test_a_communication_fault_latches(mon):
    m = state()
    m.faults[3] = int(Fault.COMMUNICATION)
    _, rep = mon.apply(m, impedance())
    assert rep.latched, "a silently stale joint must stop the robot"


def test_clamp_events_are_counted_for_diagnostics(mon):
    before = mon.clamp_events
    mon.apply(state(), impedance(pos=99.0, kp=0.0, kd=0.0))
    assert mon.clamp_events > before
