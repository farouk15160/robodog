"""
MuJoCo model tests.

The most valuable ones here compare the MJCF against the URDF. Two descriptions
of one robot drift, and a drift between the model the physics uses and the model
the IK, TF and RViz use presents as a control bug that is very hard to
attribute. These make the drift a test failure instead.
"""
import os
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

mujoco = pytest.importorskip("mujoco", reason="pip install mujoco")

DESC = get_package_share_directory("robodog_description")
LEGS = ("FL", "FR", "RL", "RR")
KINDS = ("haa", "hfe", "kfe")
JOINTS = [f"{l}_{k}_joint" for l in LEGS for k in KINDS]


@pytest.fixture(scope="module")
def P():
    with open(os.path.join(DESC, "config", "robot_parameters.yaml")) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def RS():
    with open(os.path.join(DESC, "config", "robstride02.yaml")) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def models_dir(tmp_path_factory, P, RS):
    """Generate into a temp dir so the test never depends on a stale artefact."""
    from robodog_sim.generate_models import main
    out = str(tmp_path_factory.mktemp("models"))
    main(["--out", out])
    return out


@pytest.fixture(scope="module")
def model(models_dir):
    return mujoco.MjModel.from_xml_path(os.path.join(models_dir, "robodog_scene.xml"))


@pytest.fixture(scope="module")
def urdf():
    out = subprocess.run(["xacro", os.path.join(DESC, "urdf", "robodog.urdf.xacro")],
                         capture_output=True, text=True, check=True)
    return ET.fromstring(out.stdout)


def _id(m, objtype, name):
    i = mujoco.mj_name2id(m, objtype, name)
    assert i >= 0, f"{objtype.name} '{name}' missing from the MJCF"
    return i


# --------------------------------------------------------------------------- #
# structure
# --------------------------------------------------------------------------- #
def test_all_worlds_load(models_dir):
    for fn in ["robodog.xml", "robodog_scene.xml", "robodog_house.xml"]:
        m = mujoco.MjModel.from_xml_path(os.path.join(models_dir, fn))
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        assert m.nu == 12


def test_twelve_hinge_joints_plus_one_free_joint(model):
    assert model.nq == 19 and model.nv == 18
    for j in JOINTS:
        i = _id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        assert model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE
    assert model.jnt_type[_id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")] == \
        mujoco.mjtJoint.mjJNT_FREE


def test_one_actuator_per_joint_named_after_it(model):
    """The backend resolves actuators by joint name; a mismatch would drive the
    wrong joint, which is the worst possible silent failure."""
    assert model.nu == 12
    for j in JOINTS:
        a = _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, j)
        assert model.actuator_trnid[a, 0] == _id(model, mujoco.mjtObj.mjOBJ_JOINT, j)


def test_sensors_the_backend_depends_on_exist(model):
    for s in ["imu_accel", "imu_gyro"] + [f"{l}_touch" for l in LEGS]:
        _id(model, mujoco.mjtObj.mjOBJ_SENSOR, s)


def test_named_poses_are_keyframes(model, P):
    for name in P["named_poses"]:
        _id(model, mujoco.mjtObj.mjOBJ_KEY, name)


# --------------------------------------------------------------------------- #
# MJCF vs URDF
# --------------------------------------------------------------------------- #
def test_total_mass_matches_the_urdf(model, P):
    assert model.body_subtreemass[1] == pytest.approx(P["mass_budget"]["total_kg"], abs=1e-4)


def test_every_link_mass_matches_the_urdf(model, urdf):
    for link in urdf.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        name = link.get("name")
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if i < 0:
            continue                       # pure frames are not MuJoCo bodies
        want = float(inertial.find("mass").get("value"))
        assert model.body_mass[i] == pytest.approx(want, rel=1e-6), name


def test_joint_origins_match_the_urdf(model, urdf):
    """Body positions in the MJCF are the URDF joint origins. If these drift,
    the IK and the physics disagree about where the legs are."""
    for j in JOINTS:
        uj = urdf.find(f".//joint[@name='{j}']")
        want = [float(v) for v in uj.find("origin").get("xyz").split()]
        child = uj.find("child").get("link")
        i = _id(model, mujoco.mjtObj.mjOBJ_BODY, child)
        assert model.body_pos[i] == pytest.approx(want, abs=1e-9), j


def test_joint_axes_match_the_urdf(model, urdf):
    for j in JOINTS:
        want = [float(v) for v in urdf.find(f".//joint[@name='{j}']").find("axis").get("xyz").split()]
        i = _id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        assert model.jnt_axis[i] == pytest.approx(want, abs=1e-9), j


def test_visual_geom_orientations_match_the_urdf(model, urdf, P):
    """Every visual mesh must sit at the same pose in the MJCF as in the model
    parameters the URDF is built from.

    Regression for a convention bug the physics could not see: URDF rpy is an
    EXTRINSIC x-y-z rotation, MuJoCo's `euler` attribute is INTRINSIC. The two
    agree for a single-axis rotation -- which is why the mostly single-axis
    collision primitives looked right -- and diverge by up to 1.55 rad for the
    CAD-derived visual meshes. Every test passed, the robot stood, the contact
    forces were correct, and the rendered legs floated beside the body they
    belonged to. The generator now emits quaternions; this keeps it that way.

    Geoms are matched by mesh name and position rather than by order, because
    MuJoCo does not promise to preserve the order they were written in.
    """
    from scipy.spatial.transform import Rotation
    checked = 0
    for link_name, link in P["links"].items():
        body = link_name if link_name != "base" else "base_link"
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        assert bid >= 0, body
        geoms = []
        for g in range(model.ngeom):
            if model.geom_bodyid[g] != bid or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mid = int(model.geom_dataid[g])
            mesh = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mid)
            # The compiler re-centres each mesh on its centre of mass and
            # re-orients it to principal axes, then folds that transform into
            # the geom:
            #     geom_quat = quat (x) mesh_quat
            #     geom_pos  = pos + R(quat) @ mesh_pos
            # Undo it to recover what the generator actually wrote.
            gq = Rotation.from_quat(np.roll(model.geom_quat[g], -1))
            mq = Rotation.from_quat(np.roll(model.mesh_quat[mid], -1))
            q = gq * mq.inv()
            pos = np.array(model.geom_pos[g]) - q.as_matrix() @ np.array(model.mesh_pos[mid])
            geoms.append((mesh, pos, q.as_matrix()))
        assert len(geoms) == len(link["visuals"]), (
            f"{body}: {len(geoms)} mesh geoms vs {len(link['visuals'])} expected")

        for v in link["visuals"]:
            stem = os.path.splitext(v["mesh"])[0]
            want_pos = np.array(v["xyz"])
            hit = [g for g in geoms
                   if g[0] == stem and np.abs(g[1] - want_pos).max() < 1e-5]
            assert hit, (f"{body}: no geom for {v['mesh']} at {v['xyz']}; "
                         f"have {[(g[0], np.round(g[1], 4).tolist()) for g in geoms]}")
            want = Rotation.from_euler("xyz", v["rpy"]).as_matrix()
            got = hit[0][2]
            # 1e-4, not machine epsilon: robot_parameters.yaml rounds rpy to six
            # decimals. The bug this guards against was 1.55 -- four orders of
            # magnitude away, so the tolerance costs nothing.
            assert np.abs(want - got).max() < 1e-4, (
                f"{body}/{v['mesh']}: orientation differs by "
                f"{np.abs(want - got).max():.4f}")
            checked += 1
    assert checked >= 40, f"only checked {checked} visual meshes"


def test_collision_primitive_orientations_match_the_model(model, P):
    """Same check for the analytic collision shapes."""
    from scipy.spatial.transform import Rotation
    checked = 0
    for link_name, link in P["links"].items():
        body = link_name if link_name != "base" else "base_link"
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        for i, c in enumerate(link["collisions"]):
            if c["type"] == "capsule":
                continue                       # placed by fromto, not pos/quat
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{body}_col{i}")
            if gid < 0:
                gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                        f"{link_name}_col{i}")
            assert gid >= 0, f"{body}_col{i} missing"
            assert model.geom_bodyid[gid] == bid
            want = Rotation.from_euler("xyz", c["rpy"]).as_matrix()
            w, x, y, z = model.geom_quat[gid]
            got = Rotation.from_quat([x, y, z, w]).as_matrix()
            assert np.abs(want - got).max() < 1e-4, f"{body}_col{i}"
            checked += 1
    assert checked >= 10


def test_joint_ranges_match_the_urdf(model, urdf):
    for j in JOINTS:
        lim = urdf.find(f".//joint[@name='{j}']").find("limit")
        i = _id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        assert model.jnt_range[i] == pytest.approx(
            [float(lim.get("lower")), float(lim.get("upper"))], abs=1e-9), j


# --------------------------------------------------------------------------- #
# actuator fidelity
# --------------------------------------------------------------------------- #
def test_armature_carries_the_reflected_rotor_inertia(model, RS):
    """Without this the legs accelerate about four times too easily: the
    reflected rotor inertia (J_rotor * 7.75^2) is larger than the calf's own
    inertia about the knee."""
    want = RS["joint_dynamics"]["armature_kgm2"]
    for j in JOINTS:
        i = _id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        assert model.dof_armature[model.jnt_dofadr[i]] == pytest.approx(want, rel=1e-6), j


def test_reflected_inertia_actually_dominates_the_calf(model, P, RS):
    calf = P["links"]["FL_calf"]
    L1 = P["geometry"]["thigh_length_m"]
    about_knee = calf["inertia"]["iyy"] + calf["mass_kg"] * calf["com_xyz"][2] ** 2
    assert RS["joint_dynamics"]["armature_kgm2"] > about_knee, (
        "if this ever reverses, the armature comment in mjcf.py is wrong")


def test_actuator_torque_is_limited_to_the_datasheet_peak(model, RS):
    peak = RS["performance"]["peak_torque_nm"]
    for j in JOINTS:
        a = _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, j)
        assert model.actuator_ctrlrange[a] == pytest.approx([-peak, peak], rel=1e-6)


def test_joint_damping_and_friction_come_from_the_datasheet(model, RS):
    jd = RS["joint_dynamics"]
    for j in JOINTS:
        i = _id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        dof = model.jnt_dofadr[i]
        assert model.dof_damping[dof] == pytest.approx(jd["damping_nms_per_rad"], rel=1e-6)
        assert model.dof_frictionloss[dof] == pytest.approx(jd["coulomb_friction_nm"], rel=1e-6)


def test_timestep_divides_the_control_period(model):
    """400 Hz control with a 0.5 ms step is exactly 5 substeps; a non-integer
    ratio would make the effective command latency jitter."""
    n = (1.0 / 400.0) / model.opt.timestep
    assert n == pytest.approx(round(n), abs=1e-9) and round(n) >= 2


# --------------------------------------------------------------------------- #
# physics
# --------------------------------------------------------------------------- #
def _backend(models_dir, RS):
    from robodog_hardware.registry import create_backend
    return create_backend("mujoco", dict(
        model_path=os.path.join(models_dir, "robodog_scene.xml"), keyframe="stand",
        peak_torque_nm=RS["performance"]["peak_torque_nm"],
        no_load_speed_rad_s=RS["performance"]["no_load_speed_rad_s"]))


def _stand(models_dir, RS, P, feedforward: bool, seconds=4.0):
    from robodog_control.kinematics import LEGS as KL, LegGeometry, gravity_torque
    from robodog_hardware.types import ControlMode, JointCommand
    pose = P["named_poses"]["stand"]
    q = np.array([pose["haa"], pose["hfe"], pose["kfe"]] * 4)
    g = LegGeometry.from_params(P)
    b = _backend(models_dir, RS)
    b.configure()
    b.enable()
    cmd = JointCommand()
    cmd.mode[:] = int(ControlMode.IMPEDANCE)
    cmd.position[:] = q
    cmd.kp[:] = 120.0
    cmd.kd[:] = 2.5
    if feedforward:
        load = P["mass_budget"]["total_kg"] * 9.81 / 4.0
        for i, leg in enumerate(KL):
            cmd.effort[3 * i:3 * i + 3] = gravity_torque(g, leg, q[3 * i:3 * i + 3], load)
    for _ in range(int(seconds * 400)):
        b.write(cmd)
        b.step(1 / 400.0)
    st, bs = b.read(), b.base_state()
    b.shutdown()
    return st, bs, q


def test_robot_stands_without_falling(models_dir, RS, P):
    st, bs, q = _stand(models_dir, RS, P, feedforward=True)
    assert bs.position[2] > 0.25, f"collapsed to {bs.position[2]*1000:.0f} mm"
    assert abs(bs.orientation[0]) < 0.05 and abs(bs.orientation[1]) < 0.05, "not level"
    assert bs.foot_contact.all(), "a foot lifted while standing"


def test_ground_reaction_equals_the_robot_weight(models_dir, RS, P):
    _, bs, _ = _stand(models_dir, RS, P, feedforward=True)
    assert bs.foot_force.sum() == pytest.approx(P["mass_budget"]["total_kg"] * 9.81, rel=0.02)


def test_standing_stays_inside_the_continuous_torque_rating(models_dir, RS, P):
    st, _, _ = _stand(models_dir, RS, P, feedforward=True)
    peak = float(np.abs(st.effort).max())
    assert peak < RS["operational_limits"]["continuous_torque_nm"], f"{peak:.2f} N.m"


def test_gravity_feedforward_reduces_sag(models_dir, RS, P):
    """Regression for a sign error in gravity_torque: with the sign inverted
    the feed-forward pushed the body DOWN and roughly doubled the sag, while
    every magnitude-only test still passed."""
    target = P["named_poses"]["stand"]["base_height_m"]
    _, no_ff, _ = _stand(models_dir, RS, P, feedforward=False)
    _, with_ff, _ = _stand(models_dir, RS, P, feedforward=True)
    sag_off = target - no_ff.position[2]
    sag_on = target - with_ff.position[2]
    assert sag_off > 0.003, "test is not loading the legs enough to be meaningful"
    assert abs(sag_on) < sag_off / 3.0, (
        f"feed-forward left {sag_on*1000:.2f} mm of sag versus {sag_off*1000:.2f} mm without")


def test_feet_track_the_commanded_joint_angles(models_dir, RS, P):
    st, _, q = _stand(models_dir, RS, P, feedforward=True)
    assert float(np.abs(st.position - q).max()) < 0.02


# --------------------------------------------------------------------------- #
# locomotion: what is actually verified to work
# --------------------------------------------------------------------------- #
def _run_gait(models_dir, RS, P, gait_name, vx, seconds=4.0, balance=True):
    """Drive the gait generator against MuJoCo and report what the body did."""
    import yaml as _yaml
    from ament_index_python.packages import get_package_share_directory
    from robodog_control.balance import roll_pitch_from_quat
    from robodog_control.gait import BodyFeedback, GaitGenerator, GaitParams
    from robodog_control.kinematics import LegGeometry
    from robodog_hardware.registry import create_backend
    from robodog_hardware.types import ControlMode, JointCommand

    with open(os.path.join(get_package_share_directory("robodog_control"),
                           "config", "gaits.yaml")) as f:
        gaits = _yaml.safe_load(f)["gaits"]
    pose = P["named_poses"]["stand"]
    q0 = np.array([pose["haa"], pose["hfe"], pose["kfe"]])
    g = LegGeometry.from_params(P)
    mass = P["mass_budget"]["total_kg"]

    b = create_backend("mujoco", dict(
        model_path=os.path.join(models_dir, "robodog_scene.xml"), keyframe="stand",
        peak_torque_nm=RS["performance"]["peak_torque_nm"],
        no_load_speed_rad_s=RS["performance"]["no_load_speed_rad_s"]))
    b.configure()
    b.enable()
    gen = GaitGenerator(g, q0, mass)
    d = gaits[gait_name]
    gen.set_params(GaitParams(gait=gait_name, step_frequency_hz=d["step_frequency_hz"],
                              step_height_m=d["step_height_m"], duty_factor=d["duty_factor"],
                              stance_height_m=d["stance_height_m"], vx=vx))
    dt = 1 / 400.0
    tilt = 0.0
    min_h = 9.9
    drift = 0.0
    x0 = float(b.base_state().position[0])
    for _ in range(int(seconds / dt)):
        bs = b.base_state()
        fb = None
        if balance:
            roll, pitch = roll_pitch_from_quat(bs.orientation)
            fb = BodyFeedback(height=float(bs.position[2]), vz=float(bs.linear_velocity[2]),
                              roll=roll, pitch=pitch, omega=tuple(bs.angular_velocity),
                              v_xy=(float(bs.linear_velocity[0]), float(bs.linear_velocity[1])))
        out = gen.update(dt, fb)
        c = JointCommand()
        c.mode[:] = int(ControlMode.IMPEDANCE)
        c.position[:] = out.q
        c.velocity[:] = out.qd
        c.effort[:] = out.tau_ff
        c.kp[:] = np.repeat(np.where(out.contact, 90.0, 60.0), 3)
        c.kd[:] = np.repeat(np.where(out.contact, 2.0, 1.5), 3)
        b.write(c)
        b.step(dt)
        bs = b.base_state()
        q = bs.orientation
        tilt = max(tilt, float(np.degrees(2 * np.arcsin(
            np.clip(np.hypot(q[0], q[1]), 0, 1)))))
        min_h = min(min_h, float(bs.position[2]))
        drift = max(drift, abs(float(bs.position[1])))
    bs = b.base_state()
    out_h = float(bs.position[2])
    travel = float(bs.position[0]) - x0
    b.shutdown()
    return dict(tilt=tilt, height=out_h, min_height=min_h, drift=drift,
                travel=travel, vx=travel / seconds)


def test_stand_gait_holds_height_and_attitude(models_dir, RS, P):
    r = _run_gait(models_dir, RS, P, "stand", 0.0)
    assert r["tilt"] < 2.0, f"tilted {r['tilt']:.1f} deg while merely standing"
    assert abs(r["height"] - P["named_poses"]["stand"]["base_height_m"]) < 0.01


def test_balance_layer_keeps_the_body_up_while_trotting(models_dir, RS, P):
    """The measured value of the balance layer. Without it the same trot
    reaches 25 deg of tilt and saturates the actuators; with it the body stays
    within a few degrees of level at the commanded height.
    """
    r = _run_gait(models_dir, RS, P, "trot", 0.30, balance=True)
    assert r["tilt"] < 12.0, f"tilted {r['tilt']:.1f} deg"
    assert r["min_height"] > 0.25, f"body dropped to {r['min_height']:.3f} m"


def test_trotting_without_balance_is_measurably_worse(models_dir, RS, P):
    """Guards the balance layer against being quietly disabled or broken.

    What this asserts CHANGED once the robot could walk, and the change is
    worth recording. This test used to compare body tilt, because without
    balance the old gait reached 25 degrees and saturated the actuators. With
    the swing arc continuous and the hips free, the open-loop gait is well
    behaved on its own and the two are now within a few tenths of a degree of
    each other -- 5.1 with balance against 4.9 without. Tilt is no longer where
    the layer earns its place.

    Velocity tracking and course holding are. Measured over 6 s at 0.5 m/s, the
    lateral drift is 0.054 m with balance and 0.299 m without, because the
    Raibert term is the only thing steering the feet back under the body. A
    robot that wanders 0.3 m sideways in 6 s does not get through a doorway.
    """
    fast = 0.50
    with_b = _run_gait(models_dir, RS, P, "trot", fast, seconds=6.0, balance=True)
    without = _run_gait(models_dir, RS, P, "trot", fast, seconds=6.0, balance=False)
    assert with_b["drift"] < without["drift"] * 0.6, (
        f"balance did not hold course: drifted {with_b['drift']:.3f} m vs "
        f"{without['drift']:.3f} m without it")
    assert abs(with_b["vx"] - fast) < abs(without["vx"] - fast), (
        f"balance tracked velocity no better: {with_b['vx']:+.3f} vs "
        f"{without['vx']:+.3f} against a {fast:+.2f} command")


# --------------------------------------------------------------------------- #
# locomotion
# --------------------------------------------------------------------------- #
def test_the_trot_actually_travels(models_dir, RS, P):
    """The robot must go somewhere, at roughly the speed it was asked for.

    For most of this project it did not: it stepped in place and drifted
    backwards at 0.04 m/s. Two bugs, both now covered by their own tests
    below -- a discontinuous swing arc and a jammed front hip.
    """
    r = _run_gait(models_dir, RS, P, "trot", 0.30, seconds=6.0)
    assert r["vx"] > 0.20, f"trot managed {r['vx']:+.3f} m/s against a 0.30 command"
    assert abs(r["vx"] - 0.30) < 0.10, f"poor tracking: {r['vx']:+.3f} m/s"


def test_the_walk_actually_travels(models_dir, RS, P):
    r = _run_gait(models_dir, RS, P, "walk", 0.15, seconds=6.0)
    assert abs(r["vx"] - 0.15) < 0.07, f"walk managed {r['vx']:+.3f} m/s vs 0.15"


def test_commanded_speed_changes_actual_speed(models_dir, RS, P):
    """Velocity tracking, not just motion: asking for more must deliver more.
    A gait that free-runs at whatever speed its geometry settles at would pass
    the two tests above and fail this one."""
    slow = _run_gait(models_dir, RS, P, "trot", 0.20, seconds=6.0)
    fast = _run_gait(models_dir, RS, P, "trot", 0.50, seconds=6.0)
    assert fast["vx"] > slow["vx"] + 0.15, (
        f"0.20 -> {slow['vx']:+.3f} m/s but 0.50 -> {fast['vx']:+.3f} m/s")


def test_every_hip_can_swing_through_its_commanded_range(models_dir, RS, P):
    """Regression: the thigh capsule and the HAA actuator cylinder are both
    conservative bounding volumes of parts that clear each other in the CAD,
    and MuJoCo does not auto-filter the pair because the thigh is the base's
    grandchild. Left in, the FRONT hips jammed 13 mrad above the standing pose
    and would not move under 16.6 N.m, while the rear hips swung 1.59 rad on
    3 N.m. The robot could not walk and no amount of gain tuning would have
    fixed it.

    Drives each hip alone and requires them all to move, and to move ALIKE --
    the front/rear asymmetry is the signature of the bug.
    """
    from robodog_hardware.registry import create_backend
    from robodog_hardware.types import ControlMode, JointCommand

    swept = {}
    for i, leg in enumerate(("FL", "FR", "RL", "RR")):
        b = create_backend("mujoco", dict(
            model_path=os.path.join(models_dir, "robodog.xml"),
            peak_torque_nm=RS["performance"]["peak_torque_nm"],
            no_load_speed_rad_s=RS["performance"]["no_load_speed_rad_s"]))
        b.configure()
        b.enable()
        j = 3 * i + 1                       # <leg>_hfe
        q0 = float(b.read().position[j])
        c = JointCommand()
        c.mode[:] = int(ControlMode.TORQUE)
        c.kp[:] = 0.0
        c.kd[:] = 0.0
        c.effort[:] = 0.0
        c.effort[j] = 3.0
        for _ in range(200):
            b.write(c)
            b.step(1 / 400.0)
        swept[leg] = float(b.read().position[j]) - q0
        b.shutdown()
    for leg, d in swept.items():
        assert d > 1.0, (
            f"{leg}_hfe moved only {d:.3f} rad under 3 N.m -- it is jammed "
            f"against something. All four: {swept}")
    spread = max(swept.values()) - min(swept.values())
    assert spread < 0.20, f"front/rear hips behave differently: {swept}"


# --------------------------------------------------------------------------- #
# thermal load
# --------------------------------------------------------------------------- #
def _rms_torque(models_dir, RS, P, gait_name, vx, seconds=6.0):
    """Peak and worst-joint RMS torque over a gait, skipping the transient."""
    import yaml as _yaml
    from ament_index_python.packages import get_package_share_directory
    from robodog_control.balance import roll_pitch_from_quat
    from robodog_control.gait import BodyFeedback, GaitGenerator, GaitParams
    from robodog_control.kinematics import LegGeometry
    from robodog_hardware.registry import create_backend
    from robodog_hardware.types import ControlMode, JointCommand

    with open(os.path.join(get_package_share_directory("robodog_control"),
                           "config", "gaits.yaml")) as f:
        gaits = _yaml.safe_load(f)["gaits"]
    pose = P["named_poses"]["stand"]
    q0 = np.array([pose["haa"], pose["hfe"], pose["kfe"]])
    g = LegGeometry.from_params(P)
    b = create_backend("mujoco", dict(
        model_path=os.path.join(models_dir, "robodog_scene.xml"), keyframe="stand",
        peak_torque_nm=RS["performance"]["peak_torque_nm"],
        no_load_speed_rad_s=RS["performance"]["no_load_speed_rad_s"]))
    b.configure()
    b.enable()
    gen = GaitGenerator(g, q0, P["mass_budget"]["total_kg"])
    d = gaits[gait_name]
    gen.set_params(GaitParams(gait=gait_name, step_frequency_hz=d["step_frequency_hz"],
                              step_height_m=d["step_height_m"], duty_factor=d["duty_factor"],
                              stance_height_m=d["stance_height_m"], vx=vx))
    dt, log = 1 / 400.0, []
    for i in range(int(seconds / dt)):
        bs = b.base_state()
        roll, pitch = roll_pitch_from_quat(bs.orientation)
        fb = BodyFeedback(height=float(bs.position[2]), vz=float(bs.linear_velocity[2]),
                          roll=roll, pitch=pitch, omega=tuple(bs.angular_velocity),
                          v_xy=(float(bs.linear_velocity[0]), float(bs.linear_velocity[1])))
        out = gen.update(dt, fb)
        c = JointCommand()
        c.mode[:] = int(ControlMode.IMPEDANCE)
        c.position[:] = out.q
        c.velocity[:] = out.qd
        c.effort[:] = out.tau_ff
        c.kp[:] = np.repeat(np.where(out.contact, 90.0, 60.0), 3)
        c.kd[:] = np.repeat(np.where(out.contact, 2.0, 1.5), 3)
        b.write(c)
        b.step(dt)
        if i * dt > 1.0:
            log.append(np.abs(b.read().effort))
    b.shutdown()
    t = np.array(log)
    return float(t.max()), float(np.sqrt((t ** 2).mean(axis=0)).max())


def _steady_temp(rms, RS):
    el, th = RS["electrical"], RS["thermal"]
    return (th["ambient_temp_c"] + 3 * (rms / el["torque_constant_nm_per_arms"]) ** 2
            * el["phase_resistance_ohm"] * th["thermal_resistance_k_per_w"])


def test_standing_is_thermally_sustainable(models_dir, RS, P):
    """Standing indefinitely must not cook the windings. RMS, not peak, is what
    decides that: copper loss goes as current squared."""
    _, rms = _rms_torque(models_dir, RS, P, "stand", 0.0)
    cont = RS["operational_limits"]["continuous_torque_nm"]
    temp = _steady_temp(rms, RS)
    assert rms < cont, f"{rms:.2f} N.m RMS exceeds the {cont} N.m continuous rating"
    assert temp < RS["operational_limits"]["temperature_warn_c"], (
        f"standing would settle at {temp:.0f} C")


def test_trot_is_thermally_sustainable(models_dir, RS, P):
    """This was an expected failure for most of the project's life, at 173% of
    continuous. It did not turn green by being tuned: the gait was commanding a
    41 mm backward step at every lift-off, and the front hips were jammed
    against a spurious self-collision. Both are fixed, and the same measurement
    now reads 91%."""
    peak, rms = _rms_torque(models_dir, RS, P, "trot", 0.30)
    cont = RS["operational_limits"]["continuous_torque_nm"]
    temp = _steady_temp(rms, RS)
    assert rms < cont, (
        f"trot draws {rms:.2f} N.m RMS ({rms/cont:.0%} of continuous), peak "
        f"{peak:.1f}, implying {temp:.0f} C steady state")
    assert temp < RS["operational_limits"]["temperature_warn_c"], (
        f"trot would settle at {temp:.0f} C")


def test_walk_is_thermally_sustainable(models_dir, RS, P):
    """The gait for stairs and narrow passages, so it is the one most likely to
    be held for a long time."""
    _, rms = _rms_torque(models_dir, RS, P, "walk", 0.15)
    cont = RS["operational_limits"]["continuous_torque_nm"]
    assert rms < cont, f"walk draws {rms:.2f} N.m RMS ({rms/cont:.0%})"


def test_the_thermal_model_agrees_with_the_datasheet(RS):
    """Sanity check on the numbers the two tests above depend on: continuous
    rated torque should settle around the calibration point."""
    cont = RS["operational_limits"]["continuous_torque_nm"]
    assert _steady_temp(cont, RS) == pytest.approx(80.0, abs=3.0)
