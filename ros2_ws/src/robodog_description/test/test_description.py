"""
Model integrity tests.

These guard the CAD -> URDF pipeline: if someone regenerates
robot_parameters.yaml from a changed CAD assembly, or edits a limit by hand,
these catch the mistakes that are expensive to find in hardware.
"""
import math
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

PKG = get_package_share_directory("robodog_description")
LEGS = ["FL", "FR", "RL", "RR"]
KINDS = ["haa", "hfe", "kfe"]


@pytest.fixture(scope="module")
def params():
    with open(os.path.join(PKG, "config", "robot_parameters.yaml")) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def rs02():
    with open(os.path.join(PKG, "config", "robstride02.yaml")) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def urdf():
    out = subprocess.run(
        ["xacro", os.path.join(PKG, "urdf", "robodog.urdf.xacro"), "hardware:=sim"],
        capture_output=True, text=True, check=True)
    return ET.fromstring(out.stdout)


# --------------------------------------------------------------------------- #
# structure
# --------------------------------------------------------------------------- #
def test_has_twelve_actuated_joints(urdf):
    rev = [j.get("name") for j in urdf.findall("joint") if j.get("type") == "revolute"]
    assert len(rev) == 12, rev
    assert set(rev) == {f"{l}_{k}_joint" for l in LEGS for k in KINDS}


def test_kinematic_tree_is_connected(urdf):
    links = {l.get("name") for l in urdf.findall("link")}
    children = {j.find("child").get("link") for j in urdf.findall("joint")}
    roots = links - children
    assert roots == {"base_link"}, f"expected a single root base_link, got {roots}"


#: Links allowed to carry no inertia, and why. Anything else missing an
#: inertial element is a modelling mistake, not a decision.
MASSLESS_OK = {
    "camera_mast": "visual bracket; its mass is inside the electronics payload",
}


def test_every_link_has_inertia_or_is_a_frame(urdf):
    for l in urdf.findall("link"):
        name = l.get("name")
        if l.find("inertial") is None:
            # pure coordinate frames, plus the explicitly listed exceptions
            ok = (name.endswith(("_foot", "imu_link", "_frame", "_optical_frame"))
                  or name in MASSLESS_OK)
            assert ok, f"{name} has no inertial and is not a known frame"
        else:
            m = float(l.find("inertial/mass").get("value"))
            assert m > 0, f"{name} has non-positive mass {m}"


def test_total_mass_matches_design_target(urdf, params):
    total = sum(float(l.find("inertial/mass").get("value"))
                for l in urdf.findall("link") if l.find("inertial") is not None)
    assert total == pytest.approx(params["meta"]["target_mass_kg"], abs=0.01), total


def test_inertia_tensors_are_physically_valid(urdf):
    """Triangle inequality on the principal moments; a violation means the
    aggregation or a hand edit produced a tensor no rigid body can have."""
    import numpy as np
    for l in urdf.findall("link"):
        i = l.find("inertial/inertia")
        if i is None:
            continue
        g = lambda k: float(i.get(k))
        I = np.array([[g("ixx"), g("ixy"), g("ixz")],
                      [g("ixy"), g("iyy"), g("iyz")],
                      [g("ixz"), g("iyz"), g("izz")]])
        w = sorted(np.linalg.eigvalsh(I))
        name = l.get("name")
        assert w[0] > 0, f"{name}: non-positive-definite inertia {w}"
        assert w[0] + w[1] >= w[2] * (1 - 1e-9), f"{name}: triangle inequality violated {w}"


# --------------------------------------------------------------------------- #
# conventions
# --------------------------------------------------------------------------- #
def test_joint_axes_follow_the_documented_convention(urdf):
    """HAA = +x, HFE/KFE = +y, identical for all four legs (not mirrored)."""
    want = {"haa": "1 0 0", "hfe": "0 1 0", "kfe": "0 1 0"}
    for leg in LEGS:
        for kind in KINDS:
            j = urdf.find(f".//joint[@name='{leg}_{kind}_joint']")
            axis = " ".join(str(int(float(v))) for v in j.find("axis").get("xyz").split())
            assert axis == want[kind], f"{leg}_{kind}: axis {axis} != {want[kind]}"


def test_leg_mounting_points_are_symmetric(urdf, params):
    g = params["geometry"]
    for leg in LEGS:
        sx, sy = params["legs"][leg]["sx"], params["legs"][leg]["sy"]
        xyz = [float(v) for v in
               urdf.find(f".//joint[@name='{leg}_haa_joint']").find("origin").get("xyz").split()]
        assert xyz == pytest.approx([sx * g["haa_x_m"], sy * g["haa_y_m"], 0.0], abs=1e-9)


def test_zero_pose_puts_feet_where_the_model_says(params):
    """Forward kinematics of the documented zero pose, done independently of
    the generator, must land on the documented foot position."""
    g = params["geometry"]
    z = -(g["thigh_length_m"] + g["shank_length_m"])
    y = g["haa_y_m"] + g["hfe_dr_m"] + g["thigh_lateral_m"]
    x = g["haa_x_m"] + g["hfe_dx_m"]
    assert (x, y, -z) == pytest.approx((0.221, 0.1445, 0.43032), abs=1e-6)
    assert params["named_poses"]["zero"]["base_height_m"] == pytest.approx(
        -z + g["foot_radius_m"], abs=1e-6)


# --------------------------------------------------------------------------- #
# limits
# --------------------------------------------------------------------------- #
def test_urdf_limits_are_the_hardware_maxima(urdf, rs02):
    tau = rs02["performance"]["peak_torque_nm"]
    vel = rs02["performance"]["no_load_speed_rad_s"]
    for leg in LEGS:
        for kind in KINDS:
            lim = urdf.find(f".//joint[@name='{leg}_{kind}_joint']").find("limit")
            assert float(lim.get("effort")) == pytest.approx(tau)
            assert float(lim.get("velocity")) == pytest.approx(vel)


def test_operational_limits_are_strictly_inside_hardware_limits(rs02):
    o, p = rs02["operational_limits"], rs02["performance"]
    assert o["continuous_torque_nm"] < o["peak_torque_nm"] <= p["peak_torque_nm"]
    assert o["velocity_rad_s"] < p["no_load_speed_rad_s"]
    assert o["temperature_warn_c"] < o["temperature_derate_c"] < o["temperature_fault_c"]


def test_soft_limits_are_inside_hard_limits(urdf):
    for j in urdf.findall("joint"):
        if j.get("type") != "revolute":
            continue
        lim, safe = j.find("limit"), j.find("safety_controller")
        assert safe is not None, j.get("name")
        assert float(safe.get("soft_lower_limit")) > float(lim.get("lower"))
        assert float(safe.get("soft_upper_limit")) < float(lim.get("upper"))


def test_named_poses_are_reachable(params):
    jl = params["joint_limits"]
    for name, pose in params["named_poses"].items():
        for kind in KINDS:
            assert jl[kind]["lower"] <= pose[kind] <= jl[kind]["upper"], f"{name}.{kind}"


def test_knee_cannot_hyperextend(params):
    assert params["joint_limits"]["kfe"]["upper"] == 0.0


# --------------------------------------------------------------------------- #
# assets
# --------------------------------------------------------------------------- #
def test_all_referenced_meshes_exist(urdf):
    missing = []
    for m in urdf.iter("mesh"):
        fn = m.get("filename")
        assert fn.startswith("package://robodog_description/meshes/"), fn
        p = os.path.join(PKG, "meshes", os.path.basename(fn))
        if not os.path.exists(p):
            missing.append(p)
    assert not missing, missing


def test_camera_frame_chain_is_complete(urdf):
    links = {l.get("name") for l in urdf.findall("link")}
    for f in ["camera_link", "camera_color_frame", "camera_color_optical_frame",
              "camera_depth_frame", "camera_depth_optical_frame"]:
        assert f in links, f


def test_optical_frames_use_the_ros_optical_convention(urdf):
    """z forward, x right, y down relative to the parent body frame."""
    for f in ["camera_color_optical_joint", "camera_depth_optical_joint"]:
        rpy = [float(v) for v in urdf.find(f".//joint[@name='{f}']").find("origin").get("rpy").split()]
        assert rpy == pytest.approx([-math.pi / 2, 0.0, -math.pi / 2], abs=1e-9)
