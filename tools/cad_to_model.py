#!/usr/bin/env python3
"""
CAD -> canonical robot model pipeline.

Reads the flat Onshape export (85 part-links, 12 revolute DOF, saved at an
arbitrary mate pose) and produces `robot_parameters.yaml`, the single source of
truth consumed by the URDF/Xacro and the MuJoCo model generator.

What it does
------------
1. Forward-kinematics every CAD part into the `root` frame at the CAD pose.
2. Cut the tree at the 12 revolute joints -> 13 rigid bodies.
3. Recover the pose-invariant leg geometry (link lengths, axis offsets).
4. Solve the CAD joint angles against a *canonical* zero definition.
5. Re-express every visual mesh and every inertia in canonical body frames.
6. Substitute the CAD's RMD-X8 placeholder actuators with ROBSTRIDE02 mass
   properties and top up to the 10 kg design target with an electronics payload.

Canonical convention (REP-103)
------------------------------
  base_link : x forward, y left, z up, origin at the body geometric centre.
  Leg signs : sx = +1 front / -1 rear,  sy = +1 left / -1 right.
  Axes are NOT mirrored: every HAA axis is +x, every HFE/KFE axis is +y,
  expressed in base_link orientation. Consequence: positive HAA abducts the
  left legs and adducts the right legs.
  Zero pose : all joints 0 -> legs fully extended, straight down.

Run:  python3 tools/cad_to_model.py
"""
from __future__ import annotations
import collections, json, math, os, pickle, sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAD_URDF = os.path.join(ROOT, "cad/robot/onshape_export/urdf/urdf.urdf")
OUT_YAML = os.path.join(ROOT, "ros2_ws/src/robodog_description/config/robot_parameters.yaml")
RS02_YAML = os.path.join(ROOT, "ros2_ws/src/robodog_description/config/robstride02.yaml")

TARGET_MASS_KG = 10.0            # design target for the complete robot
RS02_MASS_KG = 0.380             # ROBSTRIDE02, replaces the CAD RMD-X8 placeholder
RS02_RADIUS_M = 0.03925
RS02_DEPTH_M = 0.0415

# CAD leg id -> canonical leg id.  h* = hind -> R* = rear.
LEGS = {
    "FL": dict(cad=("dof_fl0", "dof_fl1", "dof_fl2"), foot="frame_fl_foot", sx=+1, sy=+1),
    "FR": dict(cad=("dof_fr0", "dof_fr1_inv", "dof_fr2"), foot="frame_fr_foot", sx=+1, sy=-1),
    "RL": dict(cad=("dof_hl0", "dof_hl1", "dof_hl2"), foot="frame_hl_foot", sx=-1, sy=+1),
    "RR": dict(cad=("dof_hr0", "dof_hr1_inv", "dof_hr2_inv"), foot="frame_hr_foot", sx=-1, sy=-1),
}
EX, EY, EZ = np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])


# --------------------------------------------------------------------------- #
# small transform helpers
# --------------------------------------------------------------------------- #
def rpy2R(r, p, y):
    return Rotation.from_euler("xyz", [r, p, y]).as_matrix()


def R2rpy(R):
    return Rotation.from_matrix(R).as_euler("xyz")


def T(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    M = np.eye(4)
    M[:3, :3] = rpy2R(*rpy)
    M[:3, 3] = xyz
    return M


def Rx(a):
    return T(rpy=(a, 0, 0))


def Ry(a):
    return T(rpy=(0, a, 0))


def inv(M):
    O = np.eye(4)
    O[:3, :3] = M[:3, :3].T
    O[:3, 3] = -M[:3, :3].T @ M[:3, 3]
    return O


def signed_angle(v_from, v_to, axis):
    """Signed angle rotating v_from onto v_to about `axis` (right-hand rule)."""
    a = axis / np.linalg.norm(axis)
    f = v_from - np.dot(v_from, a) * a
    t = v_to - np.dot(v_to, a) * a
    return math.atan2(float(np.dot(np.cross(f, t), a)), float(np.dot(f, t)))


# --------------------------------------------------------------------------- #
# 1. parse the CAD export
# --------------------------------------------------------------------------- #
def parse_cad():
    root = ET.parse(CAD_URDF).getroot()

    def origin(e):
        if e is None:
            return (0, 0, 0), (0, 0, 0)
        g = lambda k, d: [float(v) for v in e.get(k, d).split()]
        return g("xyz", "0 0 0"), g("rpy", "0 0 0")

    links = {}
    for l in root.findall("link"):
        n = l.get("name")
        d = dict(name=n, mass=0.0, com=np.zeros(3), I=np.zeros((3, 3)), visuals=[])
        i = l.find("inertial")
        if i is not None:
            d["mass"] = float(i.find("mass").get("value"))
            d["com"] = np.array(origin(i.find("origin"))[0])
            it = i.find("inertia")
            g = lambda k: float(it.get(k))
            d["I"] = np.array([[g("ixx"), g("ixy"), g("ixz")],
                               [g("ixy"), g("iyy"), g("iyz")],
                               [g("ixz"), g("iyz"), g("izz")]])
        for v in l.findall("visual"):
            m = v.find("geometry/mesh")
            if m is None:
                continue
            xyz, rpy = origin(v.find("origin"))
            c = v.find("material/color")
            d["visuals"].append(dict(T=T(xyz, rpy), mesh=os.path.basename(m.get("filename")),
                                     rgba=c.get("rgba") if c is not None else "0.7 0.7 0.7 1"))
        links[n] = d

    joints = []
    for j in root.findall("joint"):
        xyz, rpy = origin(j.find("origin"))
        a = j.find("axis")
        joints.append(dict(name=j.get("name"), type=j.get("type"),
                           parent=j.find("parent").get("link"),
                           child=j.find("child").get("link"), T=T(xyz, rpy),
                           axis=np.array([float(v) for v in a.get("xyz").split()]) if a is not None else None))
    return links, joints


def forward_kinematics(links, joints):
    children = collections.defaultdict(list)
    for j in joints:
        children[j["parent"]].append(j)
    W, stack = {"root": np.eye(4)}, ["root"]
    while stack:
        p = stack.pop()
        for j in children[p]:
            W[j["child"]] = W[p] @ j["T"]
            stack.append(j["child"])
    assert len(W) == len(links), f"disconnected CAD tree: {len(W)}/{len(links)}"
    return W, children


def group_bodies(joints, children):
    """Cut the tree at revolute joints -> one rigid body per DOF, plus BASE."""
    body_of = {}

    def flood(start, tag):
        stack = [start]
        while stack:
            n = stack.pop()
            body_of[n] = tag
            for j in children[n]:
                if j["type"] != "revolute":
                    stack.append(j["child"])

    flood("root", "BASE")
    for j in joints:
        if j["type"] == "revolute":
            flood(j["child"], j["name"])
    return body_of


# --------------------------------------------------------------------------- #
# 2. recover pose-invariant leg geometry
# --------------------------------------------------------------------------- #
def joint_frames(W, joints):
    """World position + unit axis of each revolute joint at the CAD pose."""
    out = {}
    for j in joints:
        if j["type"] != "revolute":
            continue
        Wj = W[j["child"]]
        ax = Wj[:3, :3] @ j["axis"]
        out[j["name"]] = dict(p=Wj[:3, 3], axis=ax / np.linalg.norm(ax))
    return out


def extract_geometry(jf, W, links):
    """Link lengths and axis offsets, all invariant to the CAD mate pose."""
    g = collections.defaultdict(list)
    for leg, cfg in LEGS.items():
        j0, j1, j2 = cfg["cad"]
        a, b, c = jf[j0], jf[j1], jf[j2]
        foot = W[[n for n in links if n.startswith("foot_ball")][0]]  # placeholder
        # HAA -> HFE : split into "along the HAA axis" and "radial in the y-z plane"
        d01 = b["p"] - a["p"]
        n0 = a["axis"] * cfg["sx"]          # CAD HAA axis flips sign front/rear
        g["haa_x"].append(abs(a["p"][0]))
        g["haa_y"].append(abs(a["p"][1]))
        g["haa_z"].append(a["p"][2])
        g["hfe_dx"].append(abs(float(np.dot(d01, EX))))
        g["hfe_dr"].append(float(np.linalg.norm(d01 - np.dot(d01, EX) * EX)))
        # HFE -> KFE : perpendicular distance = thigh length, along-axis = lateral offset
        d12 = c["p"] - b["p"]
        par = float(np.dot(d12, b["axis"]))
        g["thigh_len"].append(float(np.linalg.norm(d12 - par * b["axis"])))
        g["thigh_lat"].append(abs(par))
    return {k: (float(np.mean(v)), float(np.ptp(v))) for k, v in g.items()}


def shank_length(jf, W, feet):
    out = []
    for leg, cfg in LEGS.items():
        j2 = cfg["cad"][2]
        d = feet[leg] - jf[j2]["p"]
        par = float(np.dot(d, jf[j2]["axis"]))
        out.append(float(np.linalg.norm(d - par * jf[j2]["axis"])))
    return float(np.mean(out)), float(np.ptp(out))


# --------------------------------------------------------------------------- #
# 3. canonical chain + solve the CAD joint angles
# --------------------------------------------------------------------------- #
class Canonical:
    """Canonical kinematic chain. All axes +x (HAA) / +y (HFE, KFE)."""

    def __init__(self, geo, shank):
        self.bx, self.by = geo["haa_x"][0], geo["haa_y"][0]
        self.hfe_dx, self.hfe_dr = geo["hfe_dx"][0], geo["hfe_dr"][0]
        self.L1, self.lat = geo["thigh_len"][0], geo["thigh_lat"][0]
        self.L2 = shank

    def base_to_hip(self, leg, q=0.0):
        c = LEGS[leg]
        return T((c["sx"] * self.bx, c["sy"] * self.by, 0.0)) @ Rx(q)

    def hip_to_thigh(self, leg, q=0.0):
        c = LEGS[leg]
        return T((c["sx"] * self.hfe_dx, c["sy"] * self.hfe_dr, 0.0)) @ Ry(q)

    def thigh_to_calf(self, leg, q=0.0):
        c = LEGS[leg]
        return T((0.0, c["sy"] * self.lat, -self.L1)) @ Ry(q)

    def calf_to_foot(self, leg):
        return T((0.0, 0.0, -self.L2))

    def chain(self, leg, q):
        """World transforms of [hip, thigh, calf, foot] for joint vector q."""
        H = self.base_to_hip(leg, q[0])
        Th = H @ self.hip_to_thigh(leg, q[1])
        C = Th @ self.thigh_to_calf(leg, q[2])
        F = C @ self.calf_to_foot(leg)
        return H, Th, C, F


def solve_cad_angles(canon, leg, jf, foot_p):
    """Joint angles that place the canonical chain at the CAD pose."""
    c = LEGS[leg]
    j0, j1, j2 = c["cad"]
    # q0: rotate the canonical HAA->HFE radial vector onto the CAD one about +x
    r_cad = jf[j1]["p"] - jf[j0]["p"]
    r_can = np.array([0.0, c["sy"] * canon.hfe_dr, 0.0])
    q0 = signed_angle(r_can, r_cad, EX)
    # q1: thigh direction in the hip frame, canonical zero points -z
    H = canon.base_to_hip(leg, q0)
    Rh = H[:3, :3]
    d12 = Rh.T @ (jf[j2]["p"] - jf[j1]["p"])
    q1 = signed_angle(-EZ, d12, EY)
    # q2: shank direction in the thigh frame, canonical zero points -z
    Th = H @ canon.hip_to_thigh(leg, q1)
    d2f = Th[:3, :3].T @ (foot_p - jf[j2]["p"])
    q2 = signed_angle(-EZ, d2f, EY)
    return np.array([q0, q1, q2])


# --------------------------------------------------------------------------- #
# 4. re-express visuals + inertia in canonical body frames
# --------------------------------------------------------------------------- #
def body_world_frames(canon, q_cad):
    """Canonical body frames evaluated at the CAD pose, in root coordinates."""
    F = {"base": np.eye(4)}
    for leg in LEGS:
        H, Th, C, _ = canon.chain(leg, q_cad[leg])
        F[f"{leg}_hip"], F[f"{leg}_thigh"], F[f"{leg}_calf"] = H, Th, C
    return F


def aggregate(body_links, links, W, Fb, mass_override):
    """Total mass, COM and inertia tensor of one body, in its canonical frame."""
    Finv = inv(Fb)
    m_tot, mc = 0.0, np.zeros(3)
    parts = []
    for n in body_links:
        m = mass_override.get(n, links[n]["mass"])
        if m <= 0:
            continue
        Tb = Finv @ W[n]                              # part frame in body frame
        com_b = (Tb @ np.append(links[n]["com"], 1))[:3]
        # scale the CAD inertia tensor with the mass substitution
        s = m / links[n]["mass"] if links[n]["mass"] > 0 else 1.0
        parts.append((m, com_b, Tb[:3, :3] @ (links[n]["I"] * s) @ Tb[:3, :3].T))
        m_tot += m
        mc += m * com_b
    com = mc / m_tot if m_tot > 0 else np.zeros(3)
    I = np.zeros((3, 3))
    for m, c, Ip in parts:
        r = c - com
        I += Ip + m * (float(np.dot(r, r)) * np.eye(3) - np.outer(r, r))
    return m_tot, com, I


def visuals_in_body(body_links, links, W, Fb, mesh_rename):
    Finv = inv(Fb)
    out = []
    for n in sorted(body_links):
        for v in links[n]["visuals"]:
            M = Finv @ W[n] @ v["T"]
            mesh = normalise_mesh(v["mesh"])
            if mesh is None:
                continue
            out.append(dict(mesh=mesh, xyz=[round(float(x), 6) for x in M[:3, 3]],
                            rpy=[round(float(x), 6) for x in R2rpy(M[:3, :3])],
                            rgba=v["rgba"], source_part=n))
    return out


# --------------------------------------------------------------------------- #
# 5. mass budget: swap the CAD's RMD-X8 placeholders for ROBSTRIDE02
# --------------------------------------------------------------------------- #
# The CAD assembly was drawn around RMD-X8 V3 actuators (760.8 g each). The
# design uses ROBSTRIDE02 (380 g each), so every `rmdx8v3*` part is re-massed.
# Inertia tensors are scaled by the mass ratio, which is exact for a uniform
# density change and a good approximation here because the RS02 envelope
# (78.5 x 41.5 mm) is slightly smaller than the RMD-X8 (98 x 45.5 mm) -- i.e.
# the result is marginally conservative (over-estimates distal inertia).
PAYLOAD = [
    # name,                       mass kg, position in base_link (x, y, z)
    ("battery_12s_lipo",          0.700,  (-0.010,  0.000, -0.018)),
    ("onboard_computer",          0.200,  (-0.060,  0.000,  0.030)),
    ("power_distribution_wiring", 0.130,  ( 0.040,  0.000,  0.025)),
    ("imu",                       0.010,  ( 0.000,  0.000,  0.000)),
]
CAMERA_MASS_KG = 0.068          # Yahboom NUWA HP60C, separate link

MESH_RENAME = {
    "RMDX8V3.stl": "robstride02.stl",   # actuator swap, see above
    "XYZ_frame.stl": None,              # CAD construction frames: drop
}


def normalise_mesh(name: str) -> str | None:
    """Mirror the naming rule in tools/prepare_meshes.py, which strips the
    leading underscore Onshape adds to digit-initial part names and lowercases
    everything, so the two tools cannot drift apart."""
    if name in MESH_RENAME:
        return MESH_RENAME[name]
    return name.lstrip("_").lower()


def build_mass_override(links):
    ov = {}
    for n in links:
        if n.startswith("rmdx8v3"):
            ov[n] = RS02_MASS_KG
    return ov


# --------------------------------------------------------------------------- #
# 6. joint limits and named poses
# --------------------------------------------------------------------------- #
# The Onshape mate limits in the CAD export are modelling artefacts (asymmetric,
# e.g. dof_fl0 = [-0.704, 0.866]) and are NOT used. These are design limits
# chosen from the mechanical envelope; refine once self-collision checking runs.
JOINT_LIMITS = {
    "haa": dict(lower=-0.80, upper=0.80),    # +/- 45.8 deg abduction envelope
    "hfe": dict(lower=-1.40, upper=2.40),    # -80.2 .. +137.5 deg
    "kfe": dict(lower=-2.60, upper=0.00),    # -149.0 .. 0 deg, no hyperextension
}
# Named poses are specified as a BASE HEIGHT, not as joint angles.
#
# Each foot hangs vertically below its own HFE axis. That single rule gives all
# four of the properties a nominal stance needs at once:
#   * identical joint angles on all four legs, despite the non-mirrored axis
#     convention (a foot under the HAA axis instead would need different front
#     and rear angles, because the HFE sits 60 mm outboard of the HAA);
#   * a support polygon of 442 x 289 mm whose centroid is at x = 0, within
#     3.5 mm of the body centre of mass;
#   * the lowest peak joint torque of any centred foot placement -- 62.5% of
#     the 6 N.m continuous rating at the nominal height, which settles at about
#     43 C, so the robot can stand indefinitely.
#
# See tools/optimise_stance.py for the scan this came from.
NAMED_POSE_HEIGHTS = {
    "stand":  0.320,   # 69.7% leg extension: clear of the singularity, good margin
    "crouch": 0.240,
    "rest":   0.160,   # folded; disable the joints after arriving
}


def pose_from_height(L1: float, L2: float, height: float, foot_radius: float) -> dict:
    """Joint angles placing the foot vertically below the HFE axis."""
    reach = height - foot_radius
    cos_q2 = (reach ** 2 - L1 ** 2 - L2 ** 2) / (2.0 * L1 * L2)
    q2 = -math.acos(max(-1.0, min(1.0, cos_q2)))          # knee folds backward
    q1 = -math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return dict(haa=0.0, hfe=round(q1, 6), kfe=round(q2, 6))


def leg_fk_2d(L1, L2, q1, q2):
    """Foot position in the hip frame's x-z plane for HFE=q1, KFE=q2."""
    return (-L1 * math.sin(q1) - L2 * math.sin(q1 + q2),
            -L1 * math.cos(q1) - L2 * math.cos(q1 + q2))


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def main():
    import yaml

    links, joints = parse_cad()
    W, children = forward_kinematics(links, joints)
    body_of = group_bodies(joints, children)
    jf = joint_frames(W, joints)

    feet = {}
    for leg, cfg in LEGS.items():
        fj = next(j for j in joints if j["name"] == cfg["foot"])
        feet[leg] = W[fj["child"]][:3, 3]

    geo = extract_geometry(jf, W, links)
    L2, L2_spread = shank_length(jf, W, feet)

    print("=" * 74)
    print("CAD GEOMETRY  (mean over 4 legs, spread = max-min across legs)")
    print("=" * 74)
    for k, (m, s) in geo.items():
        print(f"  {k:12s} = {m*1000:9.3f} mm   spread {s*1e6:7.2f} um")
    print(f"  {'shank_len':12s} = {L2*1000:9.3f} mm   spread {L2_spread*1e6:7.2f} um")

    canon = Canonical(geo, L2)
    q_cad = {leg: solve_cad_angles(canon, leg, jf, feet[leg]) for leg in LEGS}

    print("\n" + "=" * 74)
    print("CAD MATE POSE, solved against the canonical zero  (deg)")
    print("=" * 74)
    max_err = 0.0
    for leg in LEGS:
        q = q_cad[leg]
        H, Th, C, F = canon.chain(leg, q)
        errs = [np.linalg.norm(H[:3, 3] - jf[LEGS[leg]["cad"][0]]["p"]),
                np.linalg.norm(Th[:3, 3] - jf[LEGS[leg]["cad"][1]]["p"]),
                np.linalg.norm(C[:3, 3] - jf[LEGS[leg]["cad"][2]]["p"]),
                np.linalg.norm(F[:3, 3] - feet[leg])]
        max_err = max(max_err, max(errs))
        print(f"  {leg}: HAA={math.degrees(q[0]):+7.2f}  HFE={math.degrees(q[1]):+7.2f}"
              f"  KFE={math.degrees(q[2]):+7.2f}   FK residual max {max(errs)*1e6:6.2f} um")
    print(f"\n  >> canonical chain reproduces the CAD to {max_err*1e6:.2f} um")
    if max_err > 1e-5:
        print("  !! WARNING: residual above 10 um, canonical model does not match CAD")

    # ---- bodies -----------------------------------------------------------
    Fb = body_world_frames(canon, q_cad)
    mass_ov = build_mass_override(links)
    cad_body = {"base": "BASE"}
    for leg, cfg in LEGS.items():
        for part, jn in zip(("hip", "thigh", "calf"), cfg["cad"]):
            cad_body[f"{leg}_{part}"] = jn
    parts_of = collections.defaultdict(list)
    for n, b in body_of.items():
        parts_of[b].append(n)

    bodies, total = {}, 0.0
    for name, cadtag in cad_body.items():
        pl = parts_of[cadtag]
        m, com, I = aggregate(pl, links, W, Fb[name], mass_ov)
        vis = visuals_in_body(pl, links, W, Fb[name], MESH_RENAME)
        bodies[name] = dict(mass=m, com=com, I=I, visuals=vis, cad_parts=sorted(pl))
        total += m

    payload_m = sum(p[1] for p in PAYLOAD)
    print("\n" + "=" * 74)
    print("MASS BUDGET")
    print("=" * 74)
    print(f"  CAD structure + 12x ROBSTRIDE02          {total:8.4f} kg")
    print(f"  electronics payload (base_link)          {payload_m:8.4f} kg")
    print(f"  camera link (Yahboom NUWA HP60C)         {CAMERA_MASS_KG:8.4f} kg")
    print(f"  {'-'*54}")
    print(f"  TOTAL                                    {total + payload_m + CAMERA_MASS_KG:8.4f} kg"
          f"   (target {TARGET_MASS_KG:.1f} kg)")

    emit(canon, bodies, q_cad, geo, L2, total, payload_m)




# --------------------------------------------------------------------------- #
# 7. emit robot_parameters.yaml
# --------------------------------------------------------------------------- #
FOOT_RADIUS_M = 0.020            # foot_ball.stl bounding box is 40 x 40 x 40 mm
BODY_BOX = (0.235, 0.120, 0.115)  # body.stl core, excluding the HAA actuators

# Depth-camera mount. Single source of truth for BOTH the URDF and the MuJoCo
# model, which previously carried the same numbers independently.
#
# Height is set by self-occlusion, not by styling. The front hip assemblies --
# the HAA actuator, the bracket and the HFE actuator -- occupy roughly
# x = 0.11..0.26 m and reach z = +0.05 m. A camera on the body front face looks
# straight into them: rendering the simulated depth image from z = 0.020 put
# 95% of the frame on the robot's own legs at 74 mm, and z = 0.090 still left
# 48%. Clearing the hip corner at (0.26, 0.05) with the bottom edge of a 49 deg
# vertical frustum pitched down 0.26 rad needs
#     z > 0.05 + (0.26 - 0.1315) * tan(0.26 + 49deg/2) = 0.155 m
# so the camera sits on a short mast, as it does on every production quadruped.
# The 68 g at this height moves the robot's centre of mass up by 0.6 mm.
CAMERA_MOUNT_XYZ = (BODY_BOX[0] / 2 + 0.014, 0.0, 0.175)
CAMERA_MOUNT_PITCH_RAD = 0.26    # down-tilt; the ground enters view ~0.6 m ahead
CAMERA_MAST_SECTION_M = 0.028    # square bracket section carrying the camera


def r6(x):
    return round(float(x), 6)


def collisions_for(name, canon, leg=None):
    """Collision primitives per body. `capsule` is native in MuJoCo; the URDF
    emitter degrades it to a cylinder because URDF has no capsule geometry."""
    if name == "base":
        prims = [dict(type="box", size=[r6(v) for v in BODY_BOX], xyz=[0, 0, 0], rpy=[0, 0, 0],
                      note="body shell")]
        for lg, c in LEGS.items():
            prims.append(dict(type="cylinder", radius=RS02_RADIUS_M, length=RS02_DEPTH_M,
                              xyz=[r6(c["sx"] * (canon.bx - RS02_DEPTH_M / 2)), r6(c["sy"] * canon.by), 0.0],
                              rpy=[0, r6(math.pi / 2), 0], note=f"{lg} HAA actuator"))
        return prims
    sy = LEGS[leg]["sy"]
    sx = LEGS[leg]["sx"]
    kind = name.split("_")[1]
    if kind == "hip":
        return [
            dict(type="box", size=[r6(canon.hfe_dx + 0.05), 0.055, 0.075],
                 xyz=[r6(sx * canon.hfe_dx / 2), r6(sy * canon.hfe_dr / 2), 0.0], rpy=[0, 0, 0],
                 note="HAA-HFE bracket"),
            dict(type="cylinder", radius=RS02_RADIUS_M, length=RS02_DEPTH_M,
                 xyz=[r6(sx * canon.hfe_dx), r6(sy * (canon.hfe_dr + RS02_DEPTH_M / 2)), 0.0],
                 rpy=[r6(math.pi / 2), 0, 0], note="HFE actuator"),
        ]
    if kind == "thigh":
        return [
            dict(type="cylinder", radius=RS02_RADIUS_M, length=RS02_DEPTH_M,
                 xyz=[0.0, r6(sy * (canon.lat - RS02_DEPTH_M / 2)), -0.045],
                 rpy=[r6(math.pi / 2), 0, 0], note="KFE actuator, proximally mounted"),
            dict(type="capsule", radius=0.030, from_=[0.0, r6(sy * 0.034), -0.020],
                 to_=[0.0, r6(sy * 0.034), r6(-canon.L1 + 0.015)], note="thigh structure"),
        ]
    if kind == "calf":
        return [
            dict(type="capsule", radius=0.017, from_=[0.0, 0.0, -0.015],
                 to_=[0.0, 0.0, r6(-canon.L2 + 0.030)], note="shank structure"),
            dict(type="sphere", radius=FOOT_RADIUS_M, xyz=[0.0, 0.0, r6(-canon.L2)],
                 rpy=[0, 0, 0], note="foot contact sphere"),
        ]
    raise ValueError(name)


def emit(canon, bodies, q_cad, geo, L2, struct_mass, payload_m):
    import yaml, hashlib, subprocess

    sha = hashlib.sha256(open(CAD_URDF, "rb").read()).hexdigest()[:16]
    L1 = canon.L1

    # named poses: derive joint angles from the specified base height
    named = {"zero": dict(haa=0.0, hfe=0.0, kfe=0.0)}
    for pname, h in NAMED_POSE_HEIGHTS.items():
        named[pname] = pose_from_height(L1, L2, h, FOOT_RADIUS_M)
    poses = {}
    for pname, q in named.items():
        fx, fz = leg_fk_2d(L1, L2, q["hfe"], q["kfe"])
        poses[pname] = dict(haa=q["haa"], hfe=q["hfe"], kfe=q["kfe"],
                            foot_x_in_hip=r6(fx), foot_z_in_hip=r6(fz),
                            base_height_m=r6(-fz + FOOT_RADIUS_M))

    doc = {
        "meta": {
            "description": "Canonical 12-DOF quadruped model derived from the Onshape CAD export.",
            "generated_by": "tools/cad_to_model.py -- DO NOT EDIT BY HAND, regenerate instead",
            "cad_source": os.path.relpath(CAD_URDF, ROOT),
            "cad_sha256_16": sha,
            "actuator": "ROBSTRIDE02 (see robstride02.yaml)",
            "target_mass_kg": TARGET_MASS_KG,
        },
        "conventions": {
            "frame": "REP-103: x forward, y left, z up; base_link at the body geometric centre",
            "axes": "NOT mirrored -- every HAA axis is +x, every HFE/KFE axis is +y in base_link orientation",
            "haa_sign": "positive HAA abducts the LEFT legs and adducts the RIGHT legs",
            "hfe_sign": "positive HFE swings the thigh backward (-x)",
            "kfe_sign": "negative KFE folds the knee backward; 0 = fully extended, no hyperextension",
            "zero_pose": "all joints 0 -> legs straight down, feet at (+/-0.221, +/-0.1445, -0.43032)",
            "leg_order": ["FL", "FR", "RL", "RR"],
            "joint_order_per_leg": ["haa", "hfe", "kfe"],
        },
        "geometry": {
            "haa_x_m": r6(canon.bx), "haa_y_m": r6(canon.by),
            "hfe_dx_m": r6(canon.hfe_dx), "hfe_dr_m": r6(canon.hfe_dr),
            "thigh_length_m": r6(L1), "thigh_lateral_m": r6(canon.lat),
            "shank_length_m": r6(L2), "foot_radius_m": FOOT_RADIUS_M,
            "hip_to_foot_lateral_m": r6(canon.hfe_dr + canon.lat),
            "max_leg_extension_m": r6(L1 + L2),
            "body_box_m": [r6(v) for v in BODY_BOX],
            "nominal_footprint_m": {"length": r6(2 * (canon.bx + canon.hfe_dx)),
                                    "width": r6(2 * (canon.by + canon.hfe_dr + canon.lat))},
        },
        "legs": {lg: {"sx": c["sx"], "sy": c["sy"], "cad_joints": list(c["cad"]),
                      "cad_mate_pose_rad": [r6(v) for v in q_cad[lg]]}
                 for lg, c in LEGS.items()},
        "joint_limits": {k: dict(lower=v["lower"], upper=v["upper"]) for k, v in JOINT_LIMITS.items()},
        "named_poses": poses,
        "payload": {"items": [{"name": n, "mass_kg": m, "xyz": list(p)} for n, m, p in PAYLOAD],
                    "total_kg": r6(payload_m)},
        "camera_mount": {
            "xyz": [r6(v) for v in CAMERA_MOUNT_XYZ],
            "pitch_rad": CAMERA_MOUNT_PITCH_RAD,
            "mass_kg": CAMERA_MASS_KG,
            # The bracket from the body shell up to the camera. Its mass is
            # inside the electronics payload, so it is visual only.
            "mast_section_m": CAMERA_MAST_SECTION_M,
            "mast_base_z_m": r6(BODY_BOX[2] / 2 - 0.01),
            "note": ("front face on a bracket, above the HAA actuators; see "
                     "tools/cad_to_model.py for why the height is not lower"),
        },
        "mass_budget": {
            "cad_structure_and_actuators_kg": r6(struct_mass),
            "electronics_payload_kg": r6(payload_m),
            "camera_kg": CAMERA_MASS_KG,
            "total_kg": r6(struct_mass + payload_m + CAMERA_MASS_KG),
        },
        "links": {},
    }

    for name, b in bodies.items():
        leg = name.split("_")[0] if "_" in name else None
        m, com, I = b["mass"], b["com"], b["I"]
        if name == "base":                       # fold the electronics payload in
            mc = m * com + sum(pm * np.array(pp) for _, pm, pp in PAYLOAD)
            m2 = m + sum(pm for _, pm, _ in PAYLOAD)
            com2 = mc / m2
            r = com - com2
            I2 = I + b["mass"] * (float(np.dot(r, r)) * np.eye(3) - np.outer(r, r))
            for _, pm, pp in PAYLOAD:            # payload items as point masses
                r = np.array(pp) - com2
                I2 = I2 + pm * (float(np.dot(r, r)) * np.eye(3) - np.outer(r, r))
            m, com, I = m2, com2, I2
        doc["links"][name] = {
            "mass_kg": r6(m),
            "com_xyz": [r6(v) for v in com],
            "inertia": {k: r6(I[i, j]) for k, (i, j) in
                        dict(ixx=(0, 0), ixy=(0, 1), ixz=(0, 2),
                             iyy=(1, 1), iyz=(1, 2), izz=(2, 2)).items()},
            "visuals": b["visuals"],
            "collisions": collisions_for(name, canon, leg),
            "cad_parts": b["cad_parts"],
        }

    os.makedirs(os.path.dirname(OUT_YAML), exist_ok=True)
    with open(OUT_YAML, "w") as f:
        f.write("# AUTO-GENERATED by tools/cad_to_model.py -- do not edit by hand.\n")
        f.write(f"# CAD source: {doc['meta']['cad_source']} (sha256[:16]={sha})\n\n")
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None, width=110)
    print(f"\n  -> wrote {os.path.relpath(OUT_YAML, ROOT)}")

    print("\n" + "=" * 74)
    print("LINK INERTIAL SUMMARY (canonical frames)")
    print("=" * 74)
    for n, l in doc["links"].items():
        i = l["inertia"]
        print(f"  {n:10s} m={l['mass_kg']*1000:8.1f} g  com={l['com_xyz']}  "
              f"Ixx/Iyy/Izz=({i['ixx']:.2e},{i['iyy']:.2e},{i['izz']:.2e})")
    print("\n" + "=" * 74)
    print("NAMED POSES")
    print("=" * 74)
    for n, p in poses.items():
        print(f"  {n:7s} haa={p['haa']:+.2f} hfe={p['hfe']:+.2f} kfe={p['kfe']:+.2f}"
              f"  -> base height {p['base_height_m']*1000:6.1f} mm, foot x offset {p['foot_x_in_hip']*1000:+6.1f} mm")


if __name__ == "__main__":
    main()
