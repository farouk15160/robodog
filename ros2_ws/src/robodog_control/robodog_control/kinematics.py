"""
Leg kinematics: forward, inverse and Jacobian.

Pure numpy, no ROS. Every quantity uses the canonical convention documented in
robot_parameters.yaml:

    HAA axis +x, HFE axis +y, KFE axis +y, all in base_link orientation and NOT
    mirrored between left and right. Zero = leg straight down.

Chain for one leg, from the HAA joint origin to the foot:

    p_foot = Rx(q0) * ( d1 + Ry(q1) * ( d2 + Ry(q2) * d3 ) )
        d1 = (sx*hfe_dx, sy*hfe_dr, 0)   HAA origin  -> HFE origin
        d2 = (0, sy*thigh_lat, -L1)      HFE origin  -> KFE origin
        d3 = (0, 0, -L2)                 KFE origin  -> foot centre

Because Ry leaves y unchanged, the lateral offset collapses to a constant
    Y = hfe_dr + thigh_lat = 84.5 mm
which is what makes the inverse kinematics closed-form: q0 is fixed by the
(y, z) projection alone, and what remains is a planar two-link problem.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

LEGS = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class LegGeometry:
    """Derived once from robot_parameters.yaml -> geometry."""
    haa_x: float          # base_link -> HAA joint, longitudinal
    haa_y: float          # base_link -> HAA joint, lateral
    hfe_dx: float         # HAA -> HFE, along the HAA axis
    hfe_dr: float         # HAA -> HFE, radial in the y-z plane
    thigh: float          # L1, HFE -> KFE perpendicular distance
    thigh_lat: float      # HFE -> KFE, along the HFE axis
    shank: float          # L2, KFE -> foot centre
    foot_radius: float

    @property
    def lateral(self) -> float:
        """Constant lateral offset of the leg plane from the HAA axis."""
        return self.hfe_dr + self.thigh_lat

    @property
    def reach_max(self) -> float:
        return self.thigh + self.shank

    @property
    def reach_min(self) -> float:
        return abs(self.thigh - self.shank)

    @classmethod
    def from_params(cls, p: dict) -> "LegGeometry":
        g = p["geometry"]
        return cls(g["haa_x_m"], g["haa_y_m"], g["hfe_dx_m"], g["hfe_dr_m"],
                   g["thigh_length_m"], g["thigh_lateral_m"], g["shank_length_m"],
                   g["foot_radius_m"])


def leg_signs(leg: str) -> tuple[float, float]:
    """(sx, sy): +1 front / -1 rear, +1 left / -1 right."""
    return (1.0 if leg[0] == "F" else -1.0, 1.0 if leg[1] == "L" else -1.0)


def hip_origin(g: LegGeometry, leg: str) -> np.ndarray:
    sx, sy = leg_signs(leg)
    return np.array([sx * g.haa_x, sy * g.haa_y, 0.0])


# --------------------------------------------------------------------------- #
# forward kinematics
# --------------------------------------------------------------------------- #
def forward(g: LegGeometry, leg: str, q: np.ndarray) -> np.ndarray:
    """Foot centre in the HAA joint frame (axes parallel to base_link)."""
    sx, sy = leg_signs(leg)
    q0, q1, q2 = float(q[0]), float(q[1]), float(q[2])
    # planar part, in the frame after the HAA rotation
    x = sx * g.hfe_dx - g.thigh * np.sin(q1) - g.shank * np.sin(q1 + q2)
    z = -g.thigh * np.cos(q1) - g.shank * np.cos(q1 + q2)
    y = sy * g.lateral
    c, s = np.cos(q0), np.sin(q0)
    return np.array([x, y * c - z * s, y * s + z * c])


def forward_in_base(g: LegGeometry, leg: str, q: np.ndarray) -> np.ndarray:
    return hip_origin(g, leg) + forward(g, leg, q)


# --------------------------------------------------------------------------- #
# inverse kinematics
# --------------------------------------------------------------------------- #
class UnreachableError(ValueError):
    """Target outside the leg's workspace. Carries the clamped solution so a
    caller can choose to saturate rather than abort a gait cycle."""

    def __init__(self, message: str, q: np.ndarray, reach: float) -> None:
        super().__init__(message)
        self.q = q
        self.reach = reach


def inverse(g: LegGeometry, leg: str, p: np.ndarray, *, clamp: bool = True) -> np.ndarray:
    """Joint angles placing the foot centre at `p` in the HAA joint frame.

    Branch selection: the knee always folds backward (q2 <= 0), matching the
    limit kfe in [-2.60, 0] in robot_parameters.yaml. There is a second,
    mirror-image solution with q2 >= 0 that this function never returns,
    because the robot cannot reach it.

    With clamp=True an out-of-reach target is pulled onto the workspace
    boundary and returned; with clamp=False it raises UnreachableError.
    """
    sx, sy = leg_signs(leg)
    px, py, pz = (float(v) for v in p)

    # ---- q0 from the (y, z) projection --------------------------------------
    y_leg = sy * g.lateral
    r2 = py * py + pz * pz
    if r2 < y_leg * y_leg:
        # foot closer to the HAA axis than the leg plane offset: impossible
        if not clamp:
            raise UnreachableError("target inside the leg-plane offset", np.zeros(3), np.sqrt(r2))
        r2 = y_leg * y_leg
    z_leg = -np.sqrt(max(r2 - y_leg * y_leg, 0.0))           # leg hangs below
    q0 = np.arctan2(pz, py) - np.arctan2(z_leg, y_leg)
    q0 = (q0 + np.pi) % (2 * np.pi) - np.pi                  # wrap to [-pi, pi]

    # ---- planar two-link problem in (x, z) ----------------------------------
    u = px - sx * g.hfe_dx
    w = z_leg
    reach = float(np.hypot(u, w))
    lo, hi = g.reach_min + 1e-6, g.reach_max - 1e-6
    if not (lo <= reach <= hi):
        if not clamp:
            raise UnreachableError(
                f"reach {reach:.4f} m outside [{lo:.4f}, {hi:.4f}]", np.zeros(3), reach)
        scale = np.clip(reach, lo, hi) / max(reach, 1e-12)
        u, w = u * scale, w * scale
        reach = float(np.hypot(u, w))

    cos_q2 = (reach ** 2 - g.thigh ** 2 - g.shank ** 2) / (2.0 * g.thigh * g.shank)
    q2 = -np.arccos(np.clip(cos_q2, -1.0, 1.0))              # knee folds backward
    alpha = np.arctan2(-u, -w)                               # foot bearing from -z
    beta = np.arctan2(g.shank * np.sin(q2), g.thigh + g.shank * np.cos(q2))
    q1 = alpha - beta
    return np.array([q0, q1, q2])


# --------------------------------------------------------------------------- #
# Jacobian
# --------------------------------------------------------------------------- #
def jacobian(g: LegGeometry, leg: str, q: np.ndarray) -> np.ndarray:
    """d(foot position in the HAA frame) / dq, 3x3, analytic.

    Used for two things: mapping a desired foot velocity to joint velocities in
    the gait generator, and mapping measured joint torques to an estimated
    contact force, tau = J^T * f.
    """
    sx, sy = leg_signs(leg)
    q0, q1, q2 = float(q[0]), float(q[1]), float(q[2])
    s1, c1 = np.sin(q1), np.cos(q1)
    s12, c12 = np.sin(q1 + q2), np.cos(q1 + q2)
    L1, L2 = g.thigh, g.shank

    y = sy * g.lateral
    z = -L1 * c1 - L2 * c12
    dz_dq1 = L1 * s1 + L2 * s12
    dz_dq2 = L2 * s12
    dx_dq1 = -L1 * c1 - L2 * c12
    dx_dq2 = -L2 * c12
    c0, s0 = np.cos(q0), np.sin(q0)

    J = np.zeros((3, 3))
    J[0, 1], J[0, 2] = dx_dq1, dx_dq2
    J[1, 0] = -y * s0 - z * c0
    J[1, 1], J[1, 2] = -dz_dq1 * s0, -dz_dq2 * s0
    J[2, 0] = y * c0 - z * s0
    J[2, 1], J[2, 2] = dz_dq1 * c0, dz_dq2 * c0
    return J


# --------------------------------------------------------------------------- #
# static force / torque
# --------------------------------------------------------------------------- #
# Sign convention, stated once and used everywhere:
#
#   `f` is always the force the ENVIRONMENT applies TO the foot, i.e. the ground
#   reaction. For a leg in stance that is +z (upward).
#
#   An external force f produces joint torques J^T f, so to hold station the
#   actuators must supply the opposite:
#
#       tau_actuator = -J^T f              f = -(J^T)^-1 tau_actuator
#
#   Getting this backwards is not a subtle error: feeding the wrong sign
#   forward makes a stance leg push the body DOWN, roughly doubling the sag
#   instead of removing it.
def foot_force_from_torque(g: LegGeometry, leg: str, q: np.ndarray,
                           tau: np.ndarray) -> np.ndarray:
    """Ground reaction at the foot implied by measured actuator torques.

    Near a singular configuration (leg fully extended, which the workspace clamp
    keeps us just off) J^T is ill-conditioned, so this is a least-squares solve
    rather than an inverse.
    """
    JT = jacobian(g, leg, q).T
    f, *_ = np.linalg.lstsq(JT, -np.asarray(tau, dtype=float), rcond=1e-6)
    return f


def torque_for_force(g: LegGeometry, leg: str, q: np.ndarray,
                     f: np.ndarray) -> np.ndarray:
    """Actuator torques that produce the ground reaction `f` at the foot.

    The general form of `gravity_torque`, which is this with f = (0, 0, load).
    """
    return -jacobian(g, leg, q).T @ np.asarray(f, dtype=float)


def gravity_torque(g: LegGeometry, leg: str, q: np.ndarray, load_n: float) -> np.ndarray:
    """Actuator torques that hold a downward load of `load_n` at the foot.

    The gait layer feeds this forward so the impedance loop only corrects the
    residual. Without it, a stance leg carries the whole body weight as position
    error: at kp = 120 N.m/rad that is 34 mrad of joint error and 5.3 mm of body
    sag, which is measurable in simulation and matches theory exactly.
    """
    return -jacobian(g, leg, q).T @ np.array([0.0, 0.0, load_n])
