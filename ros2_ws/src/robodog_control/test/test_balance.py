"""
Balance layer tests.

These lock in the behaviour that was measured to work. The locomotion status
they encode is deliberately modest: standing is solid and attitude regulation
during stepping is good; velocity tracking is not yet achieved and these tests
do not pretend otherwise (see docs section "Measured locomotion performance").
"""
import numpy as np
import pytest

from robodog_control.balance import BalanceGains, BodyStabiliser, roll_pitch_from_quat

MASS = 10.0
G = MASS * 9.81
# foot positions in the body frame at the nominal stance
FEET = np.array([[0.221, 0.1445, -0.30], [0.221, -0.1445, -0.30],
                 [-0.221, 0.1445, -0.30], [-0.221, -0.1445, -0.30]])
ALL = np.ones(4, bool)
TROT = np.array([True, False, False, True])       # FL + RR diagonal


@pytest.fixture
def s():
    return BodyStabiliser(MASS)


# --------------------------------------------------------------------------- #
# wrench
# --------------------------------------------------------------------------- #
def test_level_and_on_height_asks_only_for_the_weight(s):
    f, m = s.wrench(0.32, 0.32, 0.0, 0.0, 0.0, np.zeros(3), np.zeros(2), np.zeros(2))
    assert f[2] == pytest.approx(G)
    assert f[0] == pytest.approx(0.0) and f[1] == pytest.approx(0.0)
    assert m == pytest.approx(np.zeros(3))


def test_weight_is_fed_forward_not_left_to_the_pd(s):
    """If the PD had to carry the body weight it would need m*g/kp = 109 mm of
    height error before the legs pushed hard enough to stand up."""
    f, _ = s.wrench(0.32, 0.32, 0.0, 0.0, 0.0, np.zeros(3), np.zeros(2), np.zeros(2))
    assert f[2] > 0.9 * G


def test_being_low_asks_for_more_vertical_force(s):
    low, _ = s.wrench(0.30, 0.32, 0.0, 0.0, 0.0, np.zeros(3), np.zeros(2), np.zeros(2))
    high, _ = s.wrench(0.34, 0.32, 0.0, 0.0, 0.0, np.zeros(3), np.zeros(2), np.zeros(2))
    assert low[2] > G > high[2]


def test_roll_and_pitch_errors_produce_opposing_moments(s):
    _, m = s.wrench(0.32, 0.32, 0.0, 0.1, 0.0, np.zeros(3), np.zeros(2), np.zeros(2))
    assert m[0] < 0, "a positive roll must produce a negative restoring moment"
    _, m = s.wrench(0.32, 0.32, 0.0, 0.0, 0.1, np.zeros(3), np.zeros(2), np.zeros(2))
    assert m[1] < 0


def test_velocity_error_produces_a_force_in_the_direction_of_travel(s):
    """Verified against physics: +30 N of commanded ground reaction with all
    four feet planted moves the robot forward 0.52 m; -30 N moves it back."""
    f, _ = s.wrench(0.32, 0.32, 0.0, 0.0, 0.0, np.zeros(3),
                    np.zeros(2), np.array([0.3, 0.0]))
    assert f[0] > 0, "commanding forward velocity must push forward"


# --------------------------------------------------------------------------- #
# force distribution
# --------------------------------------------------------------------------- #
def test_distribution_reproduces_the_requested_wrench(s):
    force = np.array([12.0, -4.0, G])
    moment = np.array([1.5, -2.0, 0.0])
    f = s.distribute(FEET, ALL, force, moment)
    assert f.sum(axis=0) == pytest.approx(force, abs=1e-6)
    got_m = sum(np.cross(FEET[i], f[i]) for i in range(4))
    assert got_m == pytest.approx(moment, abs=1e-6)


def test_a_trot_pair_can_still_support_and_trim(s):
    """Two feet is exactly determined for a 6-DOF wrench only because the
    diagonal is symmetric; the solve must not blow up."""
    f = s.distribute(FEET, TROT, np.array([0.0, 0.0, G]), np.zeros(3))
    assert np.all(np.isfinite(f))
    assert f[TROT, 2].sum() == pytest.approx(G, rel=1e-6)
    assert np.allclose(f[~TROT], 0.0), "airborne feet must be given no force"


def test_no_stance_foot_means_no_force(s):
    f = s.distribute(FEET, np.zeros(4, bool), np.array([0.0, 0.0, G]), np.zeros(3))
    assert np.allclose(f, 0.0), "during flight there is nothing to push on"


def test_feet_never_pull(s):
    """A huge pitch moment would mathematically want a downward force on one
    foot. The contact cannot deliver it, so it must be clamped to zero."""
    f = s.distribute(FEET, ALL, np.array([0.0, 0.0, G]), np.array([0.0, 200.0, 0.0]))
    assert np.all(f[:, 2] >= 0.0)


def test_tangential_force_stays_inside_the_friction_cone(s):
    f = s.distribute(FEET, ALL, np.array([500.0, 0.0, G]), np.zeros(3))
    for i in range(4):
        assert np.linalg.norm(f[i, :2]) <= s.g.friction_coefficient * f[i, 2] + 1e-9


def test_vertical_force_is_capped(s):
    f = s.distribute(FEET, ALL, np.array([0.0, 0.0, 10000.0]), np.zeros(3))
    assert np.all(f[:, 2] <= s.g.max_foot_force_n + 1e-9)


def test_even_load_when_nothing_distinguishes_the_feet(s):
    f = s.distribute(FEET, ALL, np.array([0.0, 0.0, G]), np.zeros(3))
    assert f[:, 2] == pytest.approx(np.full(4, G / 4), rel=1e-6)


# --------------------------------------------------------------------------- #
# foot placement
# --------------------------------------------------------------------------- #
def test_placement_leads_in_the_direction_of_travel(s):
    off = s.foot_placement(np.array([0.4, 0.0]), np.array([0.4, 0.0]), 0.25)
    assert off[0] > 0, "at the target velocity the foot leads by v*T/2"


def test_placement_corrects_a_velocity_shortfall(s):
    """Too slow: the foot should land further back so the leg can push."""
    slow = s.foot_placement(np.array([0.0, 0.0]), np.array([0.5, 0.0]), 0.25)
    hold = s.foot_placement(np.array([0.5, 0.0]), np.array([0.5, 0.0]), 0.25)
    assert slow[0] < hold[0]


def test_placement_is_bounded(s):
    off = s.foot_placement(np.array([10.0, 10.0]), np.zeros(2), 0.25)
    assert np.linalg.norm(off) <= s.g.max_placement_m + 1e-9


# --------------------------------------------------------------------------- #
# quaternion helper
# --------------------------------------------------------------------------- #
def test_identity_quaternion_is_level():
    assert roll_pitch_from_quat(np.array([0.0, 0, 0, 1])) == pytest.approx((0.0, 0.0))


@pytest.mark.parametrize("axis,idx", [(0, 0), (1, 1)])
def test_quaternion_recovers_the_applied_angle(axis, idx):
    a = 0.3
    q = np.zeros(4)
    q[axis] = np.sin(a / 2)
    q[3] = np.cos(a / 2)
    assert roll_pitch_from_quat(q)[idx] == pytest.approx(a, abs=1e-9)


def test_gains_are_configurable():
    s = BodyStabiliser(MASS, BalanceGains(kp_height=1500.0))
    f, _ = s.wrench(0.30, 0.32, 0.0, 0.0, 0.0, np.zeros(3), np.zeros(2), np.zeros(2))
    assert f[2] == pytest.approx(G + 1500.0 * 0.02)
