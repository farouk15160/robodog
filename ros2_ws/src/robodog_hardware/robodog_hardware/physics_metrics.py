"""Bounded-memory applied-force measurements at the physics integration rate.

Call with generalized actuator torque qfrc_actuator at each selected hinge DOF,
not ctrl or actuator_force: MuJoCo transmission gears can scale both. Velocity
is the joint velocity when that force was evaluated. Motor means actuator
OUTPUT before an external belt; internal rotor gearing is already included.
Thresholds are comparisons, not declarations of any motor's continuous rating.
When a world position is supplied, the matching peak row also records the
post-step base position and cumulative planar path length for spike location.
"""
from __future__ import annotations

import numpy as np

from .transmission import validate_transmission


def body_planar_velocity(quaternion_xyzw, world_velocity):
    """World linear velocity projected into the yaw-aligned command frame."""
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=float)
    velocity = np.asarray(world_velocity, dtype=float)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    cosine, sine = np.cos(yaw), np.sin(yaw)
    return np.array([cosine * velocity[0] + sine * velocity[1],
                     -sine * velocity[0] + cosine * velocity[1]])


class PhysicsMetrics:
    def __init__(self, names, ratio=1., efficiency=1., thresholds=(8., 11.), warmup_s=0.):
        self.names = tuple(names)
        self.n = len(self.names)
        if not self.n or not np.isfinite(warmup_s) or warmup_s < 0:
            raise ValueError("nonempty names and finite nonnegative warmup required")
        self.ratio, efficiency = validate_transmission(ratio, efficiency, self.n)
        self.gain = self.ratio * efficiency
        self.warmup_s = float(warmup_s)
        self.thresholds = tuple(float(t) for t in thresholds)
        if any(not np.isfinite(t) or t <= 0 for t in self.thresholds):
            raise ValueError("thresholds must be finite and positive")
        self.elapsed_s = 0.
        self.rms_duration_s = 0.
        self.samples = 0
        self._peak = np.zeros(self.n)
        self._signed_peak = np.zeros(self.n)
        self._peak_time = np.zeros(self.n)
        self._speed_at_peak = np.zeros(self.n)
        self._peak_world_position = np.zeros((self.n, 3))
        self._peak_position_available = np.zeros(self.n, dtype=bool)
        self._peak_travel = np.zeros(self.n)
        self._previous_world_position = None
        self._travel = 0.0
        self._max_speed = np.zeros(self.n)
        self._squared = np.zeros(self.n)
        self._full_squared = np.zeros(self.n)
        self._threshold = {t: {key: np.zeros(self.n) for key in (
            "total", "run", "longest", "speed_peak", "speed_squared")} for t in self.thresholds}
        self._clipping = {kind: {
            "events": np.zeros(self.n, dtype=int), "steps": np.zeros(self.n, dtype=int),
            "duration": np.zeros(self.n), "previous": np.zeros(self.n, dtype=bool),
        } for kind in ("peak", "speed", "physics")}

    def update(self, joint_torque, joint_velocity, dt, *, peak_clipped=None,
               speed_clipped=None, physics_clipped=None, world_position=None):
        """Accumulate one applied-force sample and its optional base location."""
        torque = np.asarray(joint_torque, dtype=float)
        velocity = np.asarray(joint_velocity, dtype=float)
        if torque.shape != (self.n,) or velocity.shape != (self.n,) or not (
                np.isfinite(torque).all() and np.isfinite(velocity).all()):
            raise ValueError("finite joint torque/velocity vectors matching names required")
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("physics dt must be finite and positive")
        position = None if world_position is None else np.asarray(world_position, dtype=float)
        if position is not None and (position.shape != (3,) or not np.isfinite(position).all()):
            raise ValueError("world position must be a finite xyz vector")
        if position is not None:
            if self._previous_world_position is not None:
                self._travel += float(np.linalg.norm(position[:2] - self._previous_world_position[:2]))
            self._previous_world_position = position.copy()
        motor, speed = torque / self.gain, velocity * self.ratio
        magnitude = np.abs(motor)
        new_peak = magnitude > self._peak
        self._peak = np.maximum(self._peak, magnitude)
        self._signed_peak = np.where(new_peak, motor, self._signed_peak)
        self._peak_time = np.where(new_peak, self.elapsed_s, self._peak_time)
        self._speed_at_peak = np.where(new_peak, speed, self._speed_at_peak)
        if position is not None:
            self._peak_world_position[new_peak] = position
            self._peak_position_available[new_peak] = True
            self._peak_travel[new_peak] = self._travel
        else:
            self._peak_position_available[new_peak] = False
        self._max_speed = np.maximum(self._max_speed, np.abs(speed))
        weight = max(0., min(dt, self.elapsed_s + dt - self.warmup_s))
        self._squared = self._squared + motor ** 2 * weight
        self._full_squared = self._full_squared + motor ** 2 * dt
        self.rms_duration_s += weight
        for threshold, values in self._threshold.items():
            above = magnitude > threshold
            values["total"] = values["total"] + above * dt
            values["run"] = np.where(above, values["run"] + dt, 0.)
            values["longest"] = np.maximum(values["longest"], values["run"])
            values["speed_peak"] = np.maximum(values["speed_peak"], np.where(above, np.abs(speed), 0.))
            values["speed_squared"] = values["speed_squared"] + above * speed ** 2 * dt
        self._update_clipping(dt, peak=peak_clipped, speed=speed_clipped, physics=physics_clipped)
        self.elapsed_s += dt
        self.samples += 1

    def _update_clipping(self, dt, **masks):
        for kind, mask in masks.items():
            active = np.zeros(self.n, bool) if mask is None else np.asarray(mask, dtype=bool)
            if active.shape != (self.n,):
                raise ValueError("clipping masks must match joint count")
            values = self._clipping[kind]
            values["events"] = values["events"] + (active & ~values["previous"])
            values["steps"] = values["steps"] + active
            values["duration"] = values["duration"] + active * dt
            values["previous"] = active.copy()

    def summary(self):
        if not self.samples:
            raise ValueError("physics samples required before summary")
        rms = np.sqrt(self._squared / max(self.rms_duration_s, 1e-30))
        full_rms = np.sqrt(self._full_squared / self.elapsed_s)
        rows = []
        for i, name in enumerate(self.names):
            thresholds = {f"{t:g}": {
                "total_time_s": float(v["total"][i]),
                "longest_time_s": float(v["longest"][i]),
                "peak_abs_motor_speed_rad_s": float(v["speed_peak"][i]),
                "rms_motor_speed_rad_s": float(np.sqrt(v["speed_squared"][i] / max(v["total"][i], 1e-30))),
            } for t, v in self._threshold.items()}
            rows.append(dict(
                name=name, joint_peak_nm=float(self._peak[i] * self.gain[i]),
                motor_peak_nm=float(self._peak[i]), motor_peak_signed_nm=float(self._signed_peak[i]),
                motor_peak_time_s=float(self._peak_time[i]),
                motor_peak_world_position_m=(self._peak_world_position[i].tolist()
                                             if self._peak_position_available[i] else None),
                motor_peak_travel_m=(float(self._peak_travel[i])
                                     if self._peak_position_available[i] else None),
                motor_speed_at_peak_rad_s=float(self._speed_at_peak[i]),
                motor_peak_speed_rad_s=float(self._max_speed[i]),
                joint_rms_nm=float(rms[i] * self.gain[i]), motor_rms_nm=float(rms[i]),
                full_run_joint_rms_nm=float(full_rms[i] * self.gain[i]),
                full_run_motor_rms_nm=float(full_rms[i]),
                duration_s=self.elapsed_s, rms_duration_s=self.rms_duration_s,
                threshold_metrics=thresholds,
                peak_abs_motor_speed_when_above_11=thresholds.get("11", {}).get("peak_abs_motor_speed_rad_s", 0.),
                clipping={kind: {"events": int(v["events"][i]), "steps": int(v["steps"][i]),
                                 "duration_s": float(v["duration"][i])}
                          for kind, v in self._clipping.items()},
            ))
        return rows
