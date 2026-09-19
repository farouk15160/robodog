"""
Body stabilisation: attitude and height regulation on top of the gait.

Why this exists
---------------
A foot-trajectory generator alone does not walk. Measured in MuJoCo with the
open-loop gait and no balance term, a 0.3 m/s trot reached 25 degrees of body
tilt, saturated the actuators at the 17 N.m peak and travelled 28 mm in six
seconds. The feet were following their commanded paths; the body was not being
held up.

Two things are missing from pure trajectory following, and both are here:

1. FORCE DISTRIBUTION. The naive feed-forward gives every stance leg
   m*g/n_stance. That holds the weight but applies no moment, so nothing
   corrects roll or pitch. Instead, a PD controller produces a desired body
   wrench, and the vertical foot forces are solved so that they reproduce it:

       sum(f_i)          = F_z          (support the body)
       sum(y_i * f_i)    = M_x          (roll)
       sum(-x_i * f_i)   = M_y          (pitch)

   That is three equations in n_stance unknowns: over-determined for a trot
   (n=2), exactly determined for a walk (n=3), under-determined for a full
   stance (n=4). A least-squares solve handles all three, and the result is
   clamped non-negative because a foot can push but not pull.

2. FOOT PLACEMENT. Attitude control alone cannot regulate velocity -- a legged
   robot steers by where it puts its feet. The Raibert heuristic places the
   landing point half a stance-length ahead of the hip in the direction of
   travel, plus a term proportional to the velocity error:

       p_land = v_measured * T_stance / 2  +  k_v * (v_measured - v_desired)

This is deliberately not an MPC or a whole-body QP. Those need a model whose
parameters have been identified on hardware; this needs four gains and is
honest about being a stabiliser rather than a planner.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .kinematics import LEGS


@dataclass
class BalanceGains:
    # height: critically damped around a ~1.5 Hz body bounce
    kp_height: float = 900.0      # N/m
    kd_height: float = 180.0      # N.s/m
    # attitude: roll and pitch, about the body axes
    kp_roll: float = 70.0         # N.m/rad
    kd_roll: float = 8.0          # N.m.s/rad
    kp_pitch: float = 90.0
    kd_pitch: float = 10.0
    # Raibert velocity feedback on foot placement
    k_velocity: float = 0.10      # s
    max_placement_m: float = 0.12
    # horizontal velocity tracking: this is what actually propels the robot.
    # Attitude control holds it up; only a tangential ground force moves it.
    kp_velocity: float = 90.0     # N per m/s of velocity error
    # yaw is regulated by a moment, since foot placement alone is slow at it
    kp_yaw: float = 25.0          # N.m per rad/s of yaw-rate error
    # a foot pushes, never pulls; and never past the actuator's ability
    max_foot_force_n: float = 220.0
    # Coulomb limit used to project the solution into the friction cone.
    # Deliberately below the simulated ground friction (0.9) so the controller
    # does not plan right at the edge of slipping.
    friction_coefficient: float = 0.6


class BodyStabiliser:
    def __init__(self, mass_kg: float, gains: BalanceGains | None = None) -> None:
        self.mass = float(mass_kg)
        self.g = gains or BalanceGains()

    # ------------------------------------------------------------------ #
    def wrench(self, height: float, height_des: float, vz: float,
               roll: float, pitch: float, omega: np.ndarray,
               v_xy: np.ndarray, v_xy_des: np.ndarray,
               yaw_rate_des: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
        """Desired body wrench (force, moment) about the centre of mass.

        The weight is fed forward so the PD only corrects the residual; making
        the PD carry the whole body weight would need a height error of
        m*g/kp = 109 mm before the legs pushed hard enough to stand up.
        """
        g = self.g
        fxy = g.kp_velocity * (np.asarray(v_xy_des, float) - np.asarray(v_xy, float))
        fz = self.mass * 9.81 + g.kp_height * (height_des - height) - g.kd_height * vz
        force = np.array([fxy[0], fxy[1], fz])
        moment = np.array([
            -g.kp_roll * roll - g.kd_roll * float(omega[0]),
            -g.kp_pitch * pitch - g.kd_pitch * float(omega[1]),
            g.kp_yaw * (yaw_rate_des - float(omega[2])),
        ])
        return force, moment

    def distribute(self, feet_xyz: np.ndarray, contact: np.ndarray,
                   force: np.ndarray, moment: np.ndarray) -> np.ndarray:
        """Solve for the 3-D ground reaction at each stance foot.

        Find f_i such that the stance feet together produce the desired body
        wrench about the centre of mass:

            sum(f_i)            = F        (3 equations)
            sum(r_i x f_i)      = M        (3 equations)

        Six equations in 3*n_stance unknowns: exactly determined for a trot
        (n = 2), under-determined for a walk or a full stance. Least squares
        gives the minimum-norm solution, which also spreads the load most
        evenly -- the right bias when nothing else distinguishes the options.

        The result is then projected back into what a contact can actually
        deliver: a foot pushes but never pulls, and the tangential force is
        limited by the friction cone. Clamping perturbs the wrench slightly;
        the attitude and velocity PD see the residual next cycle. Commanding a
        force the ground cannot produce would instead make the foot slip, which
        the controller has no way to notice.
        """
        out = np.zeros((4, 3))
        idx = np.flatnonzero(contact)
        n = idx.size
        if n == 0:
            return out                      # flight phase: nothing to push on
        A = np.zeros((6, 3 * n))
        for k, i in enumerate(idx):
            r = feet_xyz[i]
            A[0:3, 3 * k:3 * k + 3] = np.eye(3)
            A[3:6, 3 * k:3 * k + 3] = np.array([[0.0, -r[2], r[1]],
                                                [r[2], 0.0, -r[0]],
                                                [-r[1], r[0], 0.0]])
        b = np.concatenate([force, moment])
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        f = sol.reshape(n, 3)
        fz = np.clip(f[:, 2], 0.0, self.g.max_foot_force_n)
        tangential = f[:, :2]
        limit = self.g.friction_coefficient * fz
        mag = np.linalg.norm(tangential, axis=1)
        scale = np.where(mag > limit, limit / np.maximum(mag, 1e-9), 1.0)
        out[idx, 0:2] = tangential * scale[:, None]
        out[idx, 2] = fz
        return out

    # ------------------------------------------------------------------ #
    def foot_placement(self, v_meas: np.ndarray, v_des: np.ndarray,
                       t_stance: float) -> np.ndarray:
        """Raibert offset applied to the swing landing point, in the body frame."""
        g = self.g
        off = v_meas * (t_stance * 0.5) + g.k_velocity * (v_meas - v_des)
        n = float(np.linalg.norm(off))
        if n > g.max_placement_m:
            off = off * (g.max_placement_m / n)
        return off


def roll_pitch_from_quat(q: np.ndarray) -> tuple[float, float]:
    """(roll, pitch) from an xyzw quaternion."""
    x, y, z, w = (float(v) for v in q)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return float(roll), float(pitch)
