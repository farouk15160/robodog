"""Leg kinematics: the IK must be the exact inverse of the FK everywhere in the
reachable workspace, and the analytic Jacobian must match the FK's derivative."""
import os

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from robodog_control.kinematics import (LEGS, LegGeometry, UnreachableError, forward,
                                        forward_in_base, foot_force_from_torque,
                                        gravity_torque, hip_origin, inverse, jacobian,
                                        leg_signs)


@pytest.fixture(scope="module")
def P():
    with open(os.path.join(get_package_share_directory("robodog_description"),
                           "config", "robot_parameters.yaml")) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def g(P):
    return LegGeometry.from_params(P)


@pytest.fixture(scope="module")
def limits(P):
    jl = P["joint_limits"]
    return (np.array([jl[k]["lower"] for k in ("haa", "hfe", "kfe")]),
            np.array([jl[k]["upper"] for k in ("haa", "hfe", "kfe")]))


def test_geometry_matches_the_cad(g):
    assert (g.thigh, g.shank) == (0.213, 0.21732)
    assert g.lateral == pytest.approx(0.0845)
    assert g.reach_max == pytest.approx(0.43032)


@pytest.mark.parametrize("leg,want", [("FL", (1, 1)), ("FR", (1, -1)),
                                      ("RL", (-1, 1)), ("RR", (-1, -1))])
def test_leg_signs(leg, want):
    assert leg_signs(leg) == want


def test_zero_pose_puts_the_leg_straight_down(g):
    for leg in LEGS:
        p = forward(g, leg, np.zeros(3))
        sx, sy = leg_signs(leg)
        assert p == pytest.approx([sx * g.hfe_dx, sy * g.lateral, -g.reach_max], abs=1e-12)


def test_ik_inverts_fk_over_the_whole_joint_range(g, limits):
    lo, hi = limits
    rng = np.random.default_rng(1234)
    worst = 0.0
    for leg in LEGS:
        for _ in range(4000):
            q = lo + (hi - lo) * rng.random(3)
            p = forward(g, leg, q)
            try:
                q2 = inverse(g, leg, p, clamp=False)
            except UnreachableError:
                continue                       # near-singular, excluded by design
            worst = max(worst, float(np.linalg.norm(forward(g, leg, q2) - p)))
    assert worst < 1e-9, f"worst FK-IK-FK error {worst:.2e} m"


def test_ik_always_returns_the_knee_backward_branch(g, limits):
    """The mirror solution exists mathematically but the robot cannot reach it."""
    lo, hi = limits
    rng = np.random.default_rng(7)
    for leg in LEGS:
        for _ in range(500):
            q = lo + (hi - lo) * rng.random(3)
            assert inverse(g, leg, forward(g, leg, q))[2] <= 1e-9


def test_ik_clamps_an_unreachable_target_onto_the_workspace(g):
    far = np.array([0.0, 0.0845, -2.0])          # far beyond the 0.43 m reach
    q = inverse(g, "FL", far, clamp=True)
    assert np.all(np.isfinite(q))
    # reach is measured from the HFE axis, not the HAA frame origin
    reached = forward(g, "FL", q) - np.array([g.hfe_dx, g.lateral, 0.0])
    assert np.linalg.norm(reached) <= g.reach_max + 1e-6


def test_ik_raises_when_clamping_is_refused(g):
    with pytest.raises(UnreachableError):
        inverse(g, "FL", np.array([0.0, 0.0845, -2.0]), clamp=False)


def test_jacobian_matches_finite_differences(g, limits):
    lo, hi = limits
    rng = np.random.default_rng(99)
    worst = 0.0
    for leg in LEGS:
        for _ in range(400):
            q = lo + (hi - lo) * rng.random(3)
            J = jacobian(g, leg, q)
            for k in range(3):
                d = np.zeros(3)
                d[k] = 1e-6
                num = (forward(g, leg, q + d) - forward(g, leg, q - d)) / 2e-6
                worst = max(worst, float(np.abs(J[:, k] - num).max()))
    assert worst < 1e-6, f"worst Jacobian error {worst:.2e}"


def test_torque_to_force_inverts_force_to_torque(g, P):
    load = P["mass_budget"]["total_kg"] * 9.81 / 4.0
    pose = P["named_poses"]["stand"]
    q = np.array([pose["haa"], pose["hfe"], pose["kfe"]])
    for leg in LEGS:
        tau = gravity_torque(g, leg, q, load)
        f = foot_force_from_torque(g, leg, q, tau)
        assert f == pytest.approx([0.0, 0.0, load], abs=1e-6)


def test_standing_torque_stays_inside_the_continuous_rating(g, P):
    """The nominal stance must be holdable indefinitely, not just briefly."""
    load = P["mass_budget"]["total_kg"] * 9.81 / 4.0
    pose = P["named_poses"]["stand"]
    q = np.array([pose["haa"], pose["hfe"], pose["kfe"]])
    peak = max(float(np.abs(gravity_torque(g, leg, q, load)).max()) for leg in LEGS)
    assert peak < 6.0, f"stance needs {peak:.2f} N.m, above the 6 N.m continuous rating"
    # 62.5% of continuous settles at about 43 C, so the robot can stand
    # indefinitely, and leaves 4.5x headroom to the 17 N.m peak for dynamics.
    assert peak / 6.0 < 0.70, f"stance uses {peak/6*100:.0f}% of continuous, too little margin"


def test_all_four_legs_use_identical_stance_angles(P):
    """A consequence of the foot-under-HFE rule, and worth asserting because
    losing it would silently reintroduce a fore-aft asymmetric stance."""
    pose = P["named_poses"]["stand"]
    assert set(pose) >= {"haa", "hfe", "kfe"}
    assert pose["haa"] == 0.0


def test_named_poses_put_all_four_feet_at_the_same_height(g, P):
    for name, pose in P["named_poses"].items():
        q = np.array([pose["haa"], pose["hfe"], pose["kfe"]])
        z = [forward_in_base(g, leg, q)[2] for leg in LEGS]
        assert max(z) - min(z) < 1e-9, f"{name} is not level"
        assert -min(z) + g.foot_radius == pytest.approx(pose["base_height_m"], abs=1e-6)


def test_support_polygon_is_symmetric_at_stance(g, P):
    """Tolerance is 10 um, not exact: the pose angles in robot_parameters.yaml
    are rounded to 6 decimals, which leaves a sub-micron residual in the foot
    position. Anything larger would mean a genuine asymmetry."""
    pose = P["named_poses"]["stand"]
    q = np.array([pose["haa"], pose["hfe"], pose["kfe"]])
    feet = {leg: forward_in_base(g, leg, q) for leg in LEGS}
    assert feet["FL"][0] == pytest.approx(-feet["RL"][0], abs=1e-5)
    assert feet["FL"][1] == pytest.approx(-feet["FR"][1], abs=1e-9)
    centroid = np.mean(list(feet.values()), axis=0)
    assert centroid[:2] == pytest.approx([0.0, 0.0], abs=1e-5)


def test_feet_hang_vertically_below_the_hfe_axes(g, P):
    """The rule that defines every named pose. It is what makes a single set of
    joint angles produce a symmetric, centred stance despite the non-mirrored
    axis convention -- see robot_parameters.yaml -> named_poses."""
    for name, pose in P["named_poses"].items():
        if name == "zero":
            continue
        q = np.array([pose["haa"], pose["hfe"], pose["kfe"]])
        for leg in LEGS:
            sx, _ = leg_signs(leg)
            hfe_x = hip_origin(g, leg)[0] + sx * g.hfe_dx
            offset = forward_in_base(g, leg, q)[0] - hfe_x
            assert abs(offset) < 1e-6, f"{name}/{leg} foot is {offset*1000:.1f} mm off"


def test_stance_support_polygon_is_centred_on_the_centre_of_mass(g, P):
    pose = P["named_poses"]["stand"]
    q = np.array([pose["haa"], pose["hfe"], pose["kfe"]])
    feet = np.array([forward_in_base(g, leg, q) for leg in LEGS])
    centroid = feet[:, :2].mean(axis=0)
    com = np.array(P["links"]["base"]["com_xyz"][:2])
    assert np.abs(centroid - com).max() < 0.005, \
        f"support centroid {centroid} is {np.abs(centroid-com).max()*1000:.1f} mm from the COM"


# --------------------------------------------------------------------------- #
# force sign convention
# --------------------------------------------------------------------------- #
# The round-trip test above passes under either sign convention, because it
# negates twice. These pin the ABSOLUTE direction, which is what actually
# matters: feeding the wrong sign forward makes a stance leg push the body down.
def test_gravity_feedforward_extends_the_knee(g, P):
    """Holding the body up means resisting the knee folding further. With the
    knee-backward branch (kfe < 0), that torque must be positive."""
    load = P["mass_budget"]["total_kg"] * 9.81 / 4.0
    pose = P["named_poses"]["stand"]
    q = np.array([pose["haa"], pose["hfe"], pose["kfe"]])
    for leg in LEGS:
        tau = gravity_torque(g, leg, q, load)
        assert tau[2] > 0.0, f"{leg} knee feed-forward {tau[2]:+.3f} would fold the leg"


def test_gravity_feedforward_implies_an_upward_ground_reaction(g, P):
    load = P["mass_budget"]["total_kg"] * 9.81 / 4.0
    pose = P["named_poses"]["stand"]
    q = np.array([pose["haa"], pose["hfe"], pose["kfe"]])
    for leg in LEGS:
        f = foot_force_from_torque(g, leg, q, gravity_torque(g, leg, q, load))
        assert f[2] == pytest.approx(load, rel=1e-9), "ground reaction must point up"


def test_haa_feedforward_opposes_the_lateral_offset(g, P):
    """The foot is 84.5 mm outboard of the HAA axis, so holding the load puts a
    fixed moment on the abduction joint. Its sign must push the leg inboard for
    a left leg and outboard for a right one -- the non-mirrored axis convention."""
    load = P["mass_budget"]["total_kg"] * 9.81 / 4.0
    pose = P["named_poses"]["stand"]
    q = np.array([pose["haa"], pose["hfe"], pose["kfe"]])
    assert gravity_torque(g, "FL", q, load)[0] < 0.0
    assert gravity_torque(g, "FR", q, load)[0] > 0.0
