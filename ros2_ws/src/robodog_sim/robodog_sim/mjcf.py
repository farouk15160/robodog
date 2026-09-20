"""
MJCF generation: robot_parameters.yaml -> MuJoCo model.

The MuJoCo model is GENERATED, never hand-edited, for the same reason the URDF
is: two hand-maintained descriptions of one robot drift, and a drift between
the URDF (what RViz, TF and the IK use) and the MJCF (what the physics uses)
shows up as a control bug that is extremely hard to attribute.

Differences from the URDF, all deliberate:
  * capsules instead of cylinders for the leg segments. URDF has no capsule;
    MuJoCo does, and capsule contact is far better conditioned than the
    flat-ended cylinder a URDF has to approximate it with.
  * `armature` on every joint, carrying the reflected rotor inertia
    (4.805e-3 kg.m2 = J_rotor * 7.75^2). Without it the legs accelerate about
    four times too easily, because that figure is larger than the calf's own
    inertia about the knee.
  * `frictionloss` and `damping` from robstride02.yaml -> joint_dynamics.
  * contact sites at the feet for touch sensors, and an IMU site at base_link.
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

import numpy as np

LEGS = ("FL", "FR", "RL", "RR")
KINDS = ("haa", "hfe", "kfe")
AXIS = {"haa": "1 0 0", "hfe": "0 1 0", "kfe": "0 1 0"}


def _f(*v) -> str:
    return " ".join(f"{float(x):.6g}" for x in np.ravel(v))


def _sub(parent, tag, **attrs) -> ET.Element:
    return ET.SubElement(parent, tag, {k.rstrip("_"): str(v) for k, v in attrs.items()})


def _quat(rpy) -> str:
    """URDF rpy -> MuJoCo quaternion (w x y z).

    NOT interchangeable with MuJoCo's `euler` attribute. URDF rpy is an
    EXTRINSIC x-y-z rotation (R = Rz*Ry*Rx); MuJoCo's default eulerseq is
    INTRINSIC x-y-z. The two agree for a rotation about a single axis -- which
    is why the collision primitives, almost all single-axis, looked correct --
    and diverge by up to 1.55 rad for the CAD-derived visual meshes, which left
    the rendered legs floating beside the body they belonged to.

    A quaternion means the same thing in both conventions, so the generator
    emits quaternions everywhere and the question cannot arise again.
    """
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_euler("xyz", list(rpy)).as_quat()
    return _f(w, x, y, z)


def _quat_from_matrix(R) -> str:
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_matrix(np.asarray(R)).as_quat()
    return _f(w, x, y, z)


def pretty(root: ET.Element) -> str:
    raw = ET.tostring(root, encoding="unicode")
    out = minidom.parseString(raw).toprettyxml(indent="  ")
    return "\n".join(l for l in out.split("\n") if l.strip())


# --------------------------------------------------------------------------- #
# Absolute clipping planes for the simulated depth camera, in metres. They
# bracket the NUWA HP60C's 0.15-6 m usable range with margin.
CAMERA_ZNEAR_M = 0.05
# 60 m, not 30: the proving ground is 24 m across, and a free camera framing
# the whole of it sits far enough back that 30 m clipped the scene to nothing.
# Depth precision at the sensor's 6 m range is unaffected -- with znear at
# 0.05 m the buffer resolution there is still under 0.1 mm.
CAMERA_ZFAR_M = 60.0


def add_defaults(root: ET.Element, rs: dict, timestep: float,
                 meshdir: str = "../meshes", extent: float = 1.0) -> None:
    jd = rs["joint_dynamics"]
    # meshdir is resolved at generation time to robodog_description's mesh
    # directory. The generated models are build artefacts, not source: rerun
    # `ros2 run robodog_sim generate_models` after checkout or after any change
    # to robot_parameters.yaml.
    _sub(root, "compiler", angle="radian", meshdir=meshdir, autolimits="true",
         balanceinertia="true", discardvisual="false")
    # implicitfast integrates the joint damping implicitly, which is what makes
    # a 2 kHz step stable with armature and stiff contacts.
    _sub(root, "option", timestep=timestep, integrator="implicitfast",
         cone="elliptic", impratio="10", gravity="0 0 -9.81")
    _sub(root, "size", njmax="2000", nconmax="500")

    # Off-screen rendering for the simulated depth camera.
    #
    # offwidth/offheight must be at least the largest off-screen image anyone
    # asks for: the camera node's colour stream (config/nuwa_hp60c.yaml,
    # 1280x720) and the preview renders in tools/render_previews.py. MuJoCo's
    # default framebuffer is 640x480 and it raises rather than downscaling.
    #
    # znear and zfar are FRACTIONS OF THE MODEL EXTENT, not metres. Left at
    # nominal values in a 12 m house they put the near clipping plane at about
    # 1.05 m, so the simulated camera would have a 1 m minimum range instead of
    # the device's 0.15 m -- and close-range obstacle avoidance, the thing an
    # indoor quadruped most needs, would be untestable. `extent` is pinned here
    # so the fractions below resolve to the absolute distances named above.
    _sub(root, "statistic", extent=extent)
    vis = _sub(root, "visual")
    _sub(vis, "global", offwidth="1920", offheight="1200", azimuth="140", elevation="-20")
    _sub(vis, "map", znear=CAMERA_ZNEAR_M / extent, zfar=CAMERA_ZFAR_M / extent)
    _sub(vis, "quality", shadowsize="2048", offsamples="4")

    d = _sub(root, "default")
    robot = _sub(d, "default", class_="robot")
    _sub(robot, "joint", damping=jd["damping_nms_per_rad"],
         frictionloss=jd["coulomb_friction_nm"], armature=jd["armature_kgm2"])
    _sub(robot, "motor", ctrllimited="true",
         ctrlrange=_f(-rs["performance"]["peak_torque_nm"], rs["performance"]["peak_torque_nm"]))
    vis = _sub(robot, "default", class_="visual")
    _sub(vis, "geom", contype="0", conaffinity="0", group="2", density="0")
    col = _sub(robot, "default", class_="collision")
    # solref/solimp: a slightly soft, well-damped contact. Perfectly rigid
    # contact at 2 kHz makes the solver ring on a 10 kg robot with 0.02 m feet.
    _sub(col, "geom", group="3", condim="4", friction="0.9 0.02 0.001",
         solref="0.004 1", solimp="0.95 0.99 0.001", density="0", rgba="0.8 0.2 0.2 0.35")
    foot = _sub(robot, "default", class_="foot")
    _sub(foot, "geom", group="3", condim="6", friction="1.0 0.05 0.002",
         solref="0.006 1", solimp="0.95 0.99 0.001", density="0",
         priority="2", rgba="0.1 0.1 0.12 1")


def add_assets(root: ET.Element, params: dict) -> None:
    a = _sub(root, "asset")
    meshes = sorted({v["mesh"] for L in params["links"].values() for v in L["visuals"]})
    for m in meshes:
        _sub(a, "mesh", name=os.path.splitext(m)[0], file=m)
    _sub(a, "texture", name="sky", type="skybox", builtin="gradient",
         rgb1="0.35 0.45 0.60", rgb2="0.08 0.10 0.14", width="256", height="256")
    _sub(a, "texture", name="grid", type="2d", builtin="checker", width="512", height="512",
         rgb1="0.28 0.30 0.33", rgb2="0.22 0.24 0.27")
    _sub(a, "material", name="grid", texture="grid", texrepeat="12 12",
         texuniform="true", reflectance="0.05")
    for name, rgba in [("shell", "0.26 0.28 0.32 1"), ("actuator", "0.88 0.42 0.09 1"),
                       ("structure", "0.72 0.74 0.78 1"), ("foot", "0.10 0.10 0.12 1"),
                       ("bearing", "0.62 0.81 0.93 1"), ("sensor", "0.13 0.55 0.75 1")]:
        _sub(a, "material", name=name, rgba=rgba)


MATERIAL_OF = {
    "robstride02.stl": "actuator", "foot_ball.stl": "foot", "61804_2rs.stl": "bearing",
    "body.stl": "shell", "body_panel.stl": "shell", "handle.stl": "shell",
}


def _visuals(body: ET.Element, link: dict) -> None:
    for v in link["visuals"]:
        name = os.path.splitext(v["mesh"])[0]
        _sub(body, "geom", type="mesh", mesh=name, class_="visual",
             pos=_f(v["xyz"]), quat=_quat(v["rpy"]),
             material=MATERIAL_OF.get(v["mesh"], "structure"))


def _collisions(body: ET.Element, link: dict, prefix: str) -> None:
    for i, c in enumerate(link["collisions"]):
        t = c["type"]
        if t == "box":
            _sub(body, "geom", name=f"{prefix}_col{i}", type="box", class_="collision",
                 size=_f(np.array(c["size"]) / 2.0), pos=_f(c["xyz"]), quat=_quat(c["rpy"]))
        elif t == "cylinder":
            _sub(body, "geom", name=f"{prefix}_col{i}", type="cylinder", class_="collision",
                 size=_f(c["radius"], c["length"] / 2.0), pos=_f(c["xyz"]), quat=_quat(c["rpy"]))
        elif t == "sphere":
            cls = "foot" if "foot" in c.get("note", "") else "collision"
            _sub(body, "geom", name=f"{prefix}_col{i}", type="sphere", class_=cls,
                 size=_f(c["radius"]), pos=_f(c["xyz"]))
        elif t == "capsule":
            # native capsule: the reason the MJCF is not a URDF conversion
            _sub(body, "geom", name=f"{prefix}_col{i}", type="capsule", class_="collision",
                 size=_f(c["radius"]), fromto=_f(list(c["from_"]) + list(c["to_"])))


def _inertial(body: ET.Element, link: dict) -> None:
    I = link["inertia"]
    _sub(body, "inertial", pos=_f(link["com_xyz"]), mass=link["mass_kg"],
         fullinertia=_f(I["ixx"], I["iyy"], I["izz"], I["ixy"], I["ixz"], I["iyz"]))


def build_robot(root: ET.Element, params: dict, *, spawn=(0.0, 0.0), yaw=0.0) -> ET.Element:
    g = params["geometry"]
    jl = params["joint_limits"]
    stand = params["named_poses"]["stand"]
    world = root.find("worldbody")

    base = _sub(world, "body", name="base_link", childclass="robot",
                pos=_f(spawn[0], spawn[1], stand["base_height_m"] + 0.02),
                quat=_quat((0.0, 0.0, yaw)))
    _sub(base, "freejoint", name="base_free")
    _inertial(base, params["links"]["base"])
    _visuals(base, params["links"]["base"])
    _collisions(base, params["links"]["base"], "base")
    _sub(base, "site", name="imu_site", pos="0 0 0", size="0.01", rgba="0.1 0.6 0.8 0.6")
    # Camera mount read from robot_parameters.yaml, the same source the URDF
    # uses, so the two models cannot drift on where the camera is.
    cm = params["camera_mount"]
    # Camera bracket, drawn on base_link so it is rigid with the shell. Mirrors
    # the `camera_mast` link in the URDF.
    mast_h = cm["xyz"][2] - cm["mast_base_z_m"]
    _sub(base, "geom", type="box", class_="visual", material="shell",
         size=_f(cm["mast_section_m"] / 2, cm["mast_section_m"] / 2, mast_h / 2),
         pos=_f(cm["xyz"][0], 0.0, cm["mast_base_z_m"] + mast_h / 2))
    cam = _sub(base, "body", name="camera_link", pos=_f(cm["xyz"]),
               quat=_quat((0.0, cm["pitch_rad"], 0.0)))
    # Must carry the same 68 g as the URDF: every visual geom in this model has
    # density=0, so without an explicit inertial the camera would be massless
    # here and present in the URDF, and the two models would disagree on the
    # total mass and the centre of mass.
    # The body origin is the OPTICAL CENTRE; the housing sits behind it, so the
    # sensor is not rendering the inside of its own case. Mirrors the URDF.
    _sub(cam, "inertial", pos="-0.045 0 0", mass=cm["mass_kg"],
         diaginertia="1.0e-5 5.0e-5 5.0e-5")
    _sub(cam, "geom", type="box", size="0.045 0.0125 0.0125", pos="-0.045 0 0",
         class_="visual", material="sensor")
    _sub(cam, "site", name="camera_site", pos="0 0 0", size="0.008", rgba="0.1 0.6 0.8 0.8")
    # MuJoCo camera looks down its own -z with +y up; the ROS optical frame is
    # +z forward, +y down. euler="1.5708 -1.5708 0" maps one onto the other.
    # MuJoCo cameras look down their own -z with +y up; the parent is a ROS body
    # frame (x forward, y left, z up). Written as the rotation matrix whose
    # columns are the camera axes in the parent frame, because that is checkable
    # by inspection: camera +x is parent -y (right), +y is parent +z (up), and
    # -z is parent +x (forward). test_mjcf.py asserts the resulting world-space
    # look direction.
    _sub(cam, "camera", name="depth_camera", mode="fixed", pos="0 0 0",
         quat=_quat_from_matrix([[0.0, 0.0, -1.0],
                                 [-1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0]]), fovy="49")

    for leg in LEGS:
        sx, sy = params["legs"][leg]["sx"], params["legs"][leg]["sy"]
        hip = _sub(base, "body", name=f"{leg}_hip",
                   pos=_f(sx * g["haa_x_m"], sy * g["haa_y_m"], 0))
        _sub(hip, "joint", name=f"{leg}_haa_joint", type="hinge", axis=AXIS["haa"],
             range=_f(jl["haa"]["lower"], jl["haa"]["upper"]))
        _inertial(hip, params["links"][f"{leg}_hip"])
        _visuals(hip, params["links"][f"{leg}_hip"])
        _collisions(hip, params["links"][f"{leg}_hip"], f"{leg}_hip")

        thigh = _sub(hip, "body", name=f"{leg}_thigh",
                     pos=_f(sx * g["hfe_dx_m"], sy * g["hfe_dr_m"], 0))
        _sub(thigh, "joint", name=f"{leg}_hfe_joint", type="hinge", axis=AXIS["hfe"],
             range=_f(jl["hfe"]["lower"], jl["hfe"]["upper"]))
        _inertial(thigh, params["links"][f"{leg}_thigh"])
        _visuals(thigh, params["links"][f"{leg}_thigh"])
        _collisions(thigh, params["links"][f"{leg}_thigh"], f"{leg}_thigh")

        calf = _sub(thigh, "body", name=f"{leg}_calf",
                    pos=_f(0, sy * g["thigh_lateral_m"], -g["thigh_length_m"]))
        _sub(calf, "joint", name=f"{leg}_kfe_joint", type="hinge", axis=AXIS["kfe"],
             range=_f(jl["kfe"]["lower"], jl["kfe"]["upper"]))
        _inertial(calf, params["links"][f"{leg}_calf"])
        _visuals(calf, params["links"][f"{leg}_calf"])
        _collisions(calf, params["links"][f"{leg}_calf"], f"{leg}_calf")
        _sub(calf, "site", name=f"{leg}_foot_site", pos=_f(0, 0, -g["shank_length_m"]),
             size=_f(g["foot_radius_m"] * 1.02), type="sphere", rgba="0 0 0 0")

    # ---------------- actuators ----------------
    act = _sub(root, "actuator")
    for leg in LEGS:
        for k in KINDS:
            j = f"{leg}_{k}_joint"
            # Plain torque sources. The impedance law is evaluated in the
            # backend at every physics step, mirroring the RS02 firmware,
            # rather than being delegated to a MuJoCo position actuator.
            _sub(act, "motor", name=j, joint=j, gear="1", class_="robot")

    # ---------------- sensors ----------------
    sen = _sub(root, "sensor")
    _sub(sen, "accelerometer", name="imu_accel", site="imu_site")
    _sub(sen, "gyro", name="imu_gyro", site="imu_site")
    _sub(sen, "framequat", name="imu_quat", objtype="site", objname="imu_site")
    for leg in LEGS:
        _sub(sen, "touch", name=f"{leg}_touch", site=f"{leg}_foot_site")
    for leg in LEGS:
        for k in KINDS:
            _sub(sen, "jointpos", name=f"{leg}_{k}_pos", joint=f"{leg}_{k}_joint")
            _sub(sen, "jointvel", name=f"{leg}_{k}_vel", joint=f"{leg}_{k}_joint")

    return root


def add_keyframes(root: ET.Element, params: dict, world=None, *,
                  spawn=(0.0, 0.0), yaw=0.0) -> None:
    """Named poses as MuJoCo keyframes.

    MuJoCo requires a keyframe to specify the ENTIRE qpos vector, and the
    movable props in a world each contribute a 7-DOF free joint. Those come
    after the robot only because `build_robot` runs first -- if the world were
    added first the robot's free joint would not start at qpos[0], the keyframe
    would silently misalign, and the robot would spawn at the origin inside the
    floor. Hence: robot first, then the world, then this.
    """
    kf = _sub(root, "keyframe")
    props = [p for p in (world.prims if world else []) if p.movable]
    tail: list[float] = []
    for p in props:
        tail += [p.pos[0], p.pos[1], p.pos[2], 1.0, 0.0, 0.0, 0.0]
    for name, pose in params["named_poses"].items():
        q = [pose["haa"], pose["hfe"], pose["kfe"]] * 4
        base_z = pose["base_height_m"] + 0.002       # 2 mm above contact
        qpos = ([spawn[0], spawn[1], base_z,
                 math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)] + q + tail)
        _sub(kf, "key", name=name, qpos=_f(qpos))
