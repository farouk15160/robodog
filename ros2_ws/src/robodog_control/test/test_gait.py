"""
Gait generator tests.

These are pure: they exercise the trajectory generator against no physics at
all, which is the right level for the property that matters most here. A foot
trajectory can be checked for continuity without knowing anything about
contact, and the bug that kept this robot from walking for most of the
project's life was a continuity bug.
"""
import os

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from robodog_control.gait import (DEFAULT_DUTY, GAIT_OFFSETS, BodyFeedback,
                                  GaitGenerator, GaitParams)
from robodog_control.kinematics import LEGS, LegGeometry

DT = 1 / 400.0


@pytest.fixture(scope="module")
def P():
    with open(os.path.join(get_package_share_directory("robodog_description"),
                           "config", "robot_parameters.yaml")) as f:
        return yaml.safe_load(f)


@pytest.fixture
def gen(P):
    pose = P["named_poses"]["stand"]
    q0 = np.array([pose["haa"], pose["hfe"], pose["kfe"]])
    return GaitGenerator(LegGeometry.from_params(P), q0,
                         P["mass_budget"]["total_kg"])


def _targets(gen, gait, vx, v_meas, seconds=4.0, **kw):
    """Commanded foot targets over time, with the body reporting v_meas."""
    gen.set_params(GaitParams(gait=gait, step_frequency_hz=2.2, duty_factor=0.5,
                              step_height_m=0.06, stance_height_m=0.32,
                              vx=vx, **kw))
    fb = BodyFeedback(height=0.32, v_xy=(v_meas, 0.0))
    out = []
    for _ in range(int(seconds / DT)):
        out.append(gen.update(DT, fb).foot_target.copy())
    return np.array(out)                       # (n, 4, 3)


# --------------------------------------------------------------------------- #
# continuity -- the property the robot could not walk without
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("v_meas", [0.0, 0.15, 0.30, 0.45])
def test_the_foot_command_never_jumps(gen, v_meas):
    """The commanded foot position must be continuous across BOTH the
    stance->swing and swing->stance transitions, at any measured velocity.

    This is the regression test for the bug that defined this project. Stance
    retracts the foot at the measured body velocity, but the swing arc used to
    be rebuilt each cycle from the nominal foot position assuming the full
    COMMANDED stride had been swept. Whenever the robot was not already
    travelling at the commanded speed the two disagreed, and the difference was
    commanded in a single 2.5 ms tick at every lift-off: 41 mm from a
    standstill at 0.30 m/s. That step drove 0.33 rad of tracking error, which
    saturated the hips at 17 N.m and destroyed the force distribution the
    balance layer had just computed -- so the robot could not accelerate, so
    the measured velocity stayed at zero, so the step never shrank.

    A foot moves at most ~0.7 m/s during swing, which is 1.8 mm per tick.
    """
    tgt = _targets(gen, "trot", 0.30, v_meas)
    step = np.linalg.norm(np.diff(tgt, axis=0), axis=2)      # (n-1, 4)
    worst = float(step.max())
    assert worst < 0.005, (
        f"foot command jumped {worst * 1000:.1f} mm in one 2.5 ms tick at "
        f"v_meas={v_meas} m/s; the trajectory is discontinuous")


@pytest.mark.parametrize("gait", sorted(set(GAIT_OFFSETS) - {"stand"}))
def test_every_gait_is_continuous(gen, gait):
    gen.reset()
    gen.set_params(GaitParams(gait=gait, step_frequency_hz=2.0,
                              duty_factor=DEFAULT_DUTY[gait], step_height_m=0.06,
                              stance_height_m=0.32, vx=0.30))
    fb = BodyFeedback(height=0.32, v_xy=(0.0, 0.0))
    prev = None
    worst = 0.0
    for _ in range(int(3.0 / DT)):
        cur = gen.update(DT, fb).foot_target.copy()
        if prev is not None:
            worst = max(worst, float(np.linalg.norm(cur - prev, axis=1).max()))
        prev = cur
    assert worst < 0.006, f"{gait} jumped {worst * 1000:.1f} mm in one tick"


def test_continuity_holds_while_the_velocity_estimate_moves(gen):
    """The body accelerating mid-stride must not tear the trajectory: the
    landing point tracks Raibert through the swing, so it moves under the foot
    while the foot is in the air."""
    gen.set_params(GaitParams(gait="trot", step_frequency_hz=2.2, duty_factor=0.5,
                              step_height_m=0.06, stance_height_m=0.32, vx=0.40))
    prev, worst = None, 0.0
    for i in range(int(4.0 / DT)):
        v = 0.40 * min(1.0, i * DT / 2.0)          # ramp 0 -> 0.40 m/s
        cur = gen.update(DT, BodyFeedback(height=0.32, v_xy=(v, 0.0))).foot_target.copy()
        if prev is not None:
            worst = max(worst, float(np.linalg.norm(cur - prev, axis=1).max()))
        prev = cur
    assert worst < 0.006, f"jumped {worst * 1000:.1f} mm while accelerating"


# --------------------------------------------------------------------------- #
# the trajectory means what it says
# --------------------------------------------------------------------------- #
def test_stance_retracts_the_foot_at_the_commanded_velocity(gen):
    """With the default sweep gain the stance foot tracks the COMMANDED body
    velocity, which is what turns a velocity error into a tangential force
    through the leg impedance."""
    tgt = _targets(gen, "trot", 0.30, 0.30)
    fl = tgt[:, 0, :]
    contact_like = np.diff(fl[:, 0]) < 0          # retracting
    rate = np.median(np.diff(fl[contact_like.nonzero()[0], 0]) / DT)
    assert rate == pytest.approx(-0.30, abs=0.06), (
        f"stance foot retracted at {rate:+.3f} m/s, expected -0.30")


def test_a_swinging_foot_lifts_and_comes_back_down(gen):
    tgt = _targets(gen, "trot", 0.30, 0.30)
    z = tgt[:, 0, 2]
    assert z.max() - z.min() > 0.04, "the foot barely leaves the ground"
    assert z.max() - z.min() < 0.09, "the foot lifts further than commanded"


def test_standing_still_commands_a_still_foot(gen):
    """Zero commanded velocity must produce no stride at all -- otherwise the
    robot marches on the spot and wears the feet for nothing."""
    tgt = _targets(gen, "trot", 0.0, 0.0)
    spread = float(np.ptp(tgt[:, 0, 0]))
    assert spread < 0.02, f"foot swept {spread * 1000:.0f} mm at zero velocity"


def test_faster_commands_produce_longer_strides(gen, P):
    """Guards against the commanded velocity quietly ceasing to reach the
    trajectory, which is how the swing arc lost its stride term once."""
    spans = []
    for vx in (0.10, 0.30, 0.50):
        gen.reset()
        tgt = _targets(gen, "trot", vx, vx)
        spans.append(float(np.ptp(tgt[len(tgt) // 2:, 0, 0])))
    assert spans[0] < spans[1] < spans[2], f"stride did not grow with speed: {spans}"


def test_the_gait_reaches_every_leg(gen):
    tgt = _targets(gen, "trot", 0.30, 0.30)
    for i, leg in enumerate(LEGS):
        assert float(np.ptp(tgt[:, i, 2])) > 0.03, f"{leg} never lifted"
