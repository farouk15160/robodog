"""
Floating-base state estimation.

A simulation backend returns ground truth from `base_state()`. The real robot
cannot: there is no sensor for absolute base pose. This module produces the
same BaseState from what the real robot does have -- the IMU and the leg
kinematics -- so everything downstream (the GUI, the gait layer, RViz) reads one
interface regardless of which side of the boundary it is running on.

Method
------
Attitude: complementary filter. The gyro integrates cleanly over short
intervals but drifts; gravity measured by the accelerometer is absolutely
referenced but corrupted by body acceleration. Blending with a long time
constant (`tau_attitude_s`) keeps the gyro for fast motion and lets gravity
correct the slow drift. Yaw is NOT observable from an IMU without a
magnetometer, so it is integrated from the gyro and flagged as drifting.

Height and velocity: leg odometry. Every foot in contact is a temporary fixed
point. For those feet,
    v_base = -R * (J(q) * qd)      and    h = -mean(z of contact feet in base)
averaged over the contact set. This is exact while feet do not slip, and
degrades gracefully as they do, which is why contact detection feeds it.

Deliberately NOT a full EKF. Until the robot exists and the noise is
characterised, an EKF's covariances would be invented numbers. This is honest,
debuggable, and enough to stand, walk and visualise.
"""
from __future__ import annotations

import numpy as np

from robodog_hardware.types import BaseState

from .kinematics import LEGS, LegGeometry, foot_force_from_torque, forward_in_base, jacobian


def quat_from_gravity_and_yaw(g_body: np.ndarray, yaw: float) -> np.ndarray:
    """Quaternion (xyzw) from a measured gravity direction plus a yaw angle."""
    gz = g_body / max(np.linalg.norm(g_body), 1e-9)
    roll = np.arctan2(-gz[1], -gz[2])
    pitch = np.arctan2(gz[0], np.hypot(gz[1], gz[2]))
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy,
                     cr * cp * cy + sr * sp * sy])


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class StateEstimator:
    def __init__(self, geometry: LegGeometry, *, tau_attitude_s: float = 1.0,
                 contact_force_threshold_n: float = 8.0) -> None:
        self.g = geometry
        self.tau = tau_attitude_s
        self.contact_threshold = contact_force_threshold_n
        self.roll = self.pitch = self.yaw = 0.0
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.yaw_drifting = True          # no magnetometer
        self._initialised = False

    def reset(self) -> None:
        self.roll = self.pitch = self.yaw = 0.0
        self.position[:] = 0.0
        self.velocity[:] = 0.0
        self._initialised = False

    def update(self, q: np.ndarray, qd: np.ndarray, accel: np.ndarray,
               gyro: np.ndarray, dt: float,
               contact: np.ndarray | None = None) -> BaseState:
        # ---- attitude -------------------------------------------------------
        a = np.asarray(accel, float)
        norm = float(np.linalg.norm(a))
        roll_acc = np.arctan2(-a[1], -a[2]) if norm > 1e-6 else self.roll
        pitch_acc = np.arctan2(a[0], np.hypot(a[1], a[2])) if norm > 1e-6 else self.pitch
        if not self._initialised:
            self.roll, self.pitch = roll_acc, pitch_acc
            self._initialised = True
        else:
            # Trust gravity only when the measured magnitude looks like gravity;
            # during a hard push-off it does not.
            trust = float(np.exp(-abs(norm - 9.81) / 2.0))
            alpha = self.tau / (self.tau + dt)
            blend = alpha + (1.0 - alpha) * (1.0 - trust)
            self.roll = blend * (self.roll + gyro[0] * dt) + (1 - blend) * roll_acc
            self.pitch = blend * (self.pitch + gyro[1] * dt) + (1 - blend) * pitch_acc
        self.yaw += float(gyro[2]) * dt
        quat = quat_from_gravity_and_yaw(np.array([np.sin(self.pitch),
                                                   -np.sin(self.roll) * np.cos(self.pitch),
                                                   -np.cos(self.roll) * np.cos(self.pitch)]),
                                         self.yaw)
        R = quat_to_matrix(quat)

        # ---- leg odometry ---------------------------------------------------
        feet_b, vels_b = [], []
        for i, leg in enumerate(LEGS):
            qi = q[3 * i:3 * i + 3]
            if contact is not None and not contact[i]:
                continue
            feet_b.append(forward_in_base(self.g, leg, qi))
            vels_b.append(jacobian(self.g, leg, qi) @ qd[3 * i:3 * i + 3])

        if feet_b:
            feet_b = np.array(feet_b)
            # Height above the contact plane, measured along world -z through
            # the current attitude, plus the foot radius.
            z_world = (R @ feet_b.T).T[:, 2]
            height = float(-np.mean(z_world) + self.g.foot_radius)
            self.velocity = -R @ np.mean(np.array(vels_b), axis=0)
        else:
            height = float(self.position[2])
            self.velocity += (R @ a + np.array([0, 0, -9.81])) * dt   # ballistic

        self.position[0:2] += self.velocity[0:2] * dt
        self.position[2] = height

        st = BaseState(ground_truth=False)
        st.position = self.position.copy()
        st.orientation = quat
        st.linear_velocity = self.velocity.copy()
        st.angular_velocity = np.asarray(gyro, float).copy()
        st.linear_acceleration = a.copy()
        if contact is not None:
            st.foot_contact = np.asarray(contact, bool).copy()
        return st

    def contact_from_force(self, foot_force: np.ndarray) -> np.ndarray:
        return np.asarray(foot_force) > self.contact_threshold

    def contact_from_torque(self, q: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """Contact estimate for hardware with no foot sensor: the vertical
        component of J^-T tau. Noisier than a load cell but needs no extra
        wiring, and the gait's expected contact schedule gates it."""
        out = np.zeros(4, bool)
        for i, leg in enumerate(LEGS):
            f = foot_force_from_torque(self.g, leg, q[3 * i:3 * i + 3], tau[3 * i:3 * i + 3])
            out[i] = f[2] > self.contact_threshold
        return out
