#!/usr/bin/env python3
"""
Hardware capability envelope, measured independently of the gait controller.

"Would it walk, climb and jump" is two questions. Whether the CURRENT
controller does is already answered -- it does not travel, see the locomotion
status in the documentation. Whether the MACHINE can is separate, and this
measures it with deliberately crude open-loop controllers so the answer
reflects the hardware rather than any particular control scheme.

What it reports:
  1. vertical jump   flight height, torque cost, and whether the landing works
  2. payload         how much extra mass the stance torque budget allows
  3. step-up         maximum step, which is set by remaining leg travel
  4. slope           static stance on an incline, torque and friction needed
  5. actuator        rated and peak mechanical power, power-to-weight

Run:  MUJOCO_GL=egl python3 tools/capability_report.py
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "ros2_ws/src/robodog_control"))


def main() -> int:
    from ament_index_python.packages import get_package_share_directory
    from robodog_control.kinematics import LEGS, LegGeometry, gravity_torque
    from robodog_hardware.registry import create_backend
    from robodog_hardware.types import ControlMode, JointCommand

    share = get_package_share_directory("robodog_description")
    P = yaml.safe_load(open(os.path.join(share, "config", "robot_parameters.yaml")))
    RS = yaml.safe_load(open(os.path.join(share, "config", "robstride02.yaml")))
    g = LegGeometry.from_params(P)
    mass = P["mass_budget"]["total_kg"]
    peak = RS["performance"]["peak_torque_nm"]
    cont = RS["operational_limits"]["continuous_torque_nm"]
    noload = RS["performance"]["no_load_speed_rad_s"]
    model = os.path.join(get_package_share_directory("robodog_sim"), "models",
                         "robodog_scene.xml")
    dt = 1 / 400.0

    def pose(h):
        r = h - g.foot_radius
        c = (r ** 2 - g.thigh ** 2 - g.shank ** 2) / (2 * g.thigh * g.shank)
        q2 = -math.acos(max(-1.0, min(1.0, c)))
        q1 = -math.atan2(g.shank * math.sin(q2), g.thigh + g.shank * math.cos(q2))
        return np.array([0.0, q1, q2])

    def tau_at(h, load):
        q = pose(h)
        return max(float(np.abs(gravity_torque(g, leg, q, load)).max()) for leg in LEGS)

    bar = "=" * 78

    # ---------------- 1. jump ----------------
    print(bar)
    print("1. VERTICAL JUMP   crouch, then full-torque extension")
    print(bar)
    b = create_backend("mujoco", dict(model_path=model, keyframe="crouch",
                                      peak_torque_nm=peak, no_load_speed_rad_s=noload))
    b.configure()
    b.enable()
    c = JointCommand()
    c.mode[:] = int(ControlMode.IMPEDANCE)
    c.position[:] = np.concatenate([pose(0.20)] * 4)
    c.kp[:] = 140.0
    c.kd[:] = 4.0
    for _ in range(int(0.8 / dt)):
        b.write(c)
        b.step(dt)
    c.position[:] = np.concatenate([pose(0.43)] * 4)
    c.kp[:] = 500.0
    c.kd[:] = 2.0
    tau_push, vto, air = 0.0, 0.0, 0
    for _ in range(int(0.45 / dt)):
        b.write(c)
        b.step(dt)
        bs = b.base_state()
        tau_push = max(tau_push, float(np.abs(b.read().effort).max()))
        if not bs.foot_contact.any():
            if air == 0:
                vto = float(bs.linear_velocity[2])
            air += 1
    c.position[:] = np.concatenate([pose(0.30)] * 4)
    c.kp[:] = 90.0
    c.kd[:] = 3.0
    tau_land, apex = 0.0, 0.0
    for _ in range(int(1.2 / dt)):
        b.write(c)
        b.step(dt)
        bs = b.base_state()
        apex = max(apex, float(bs.position[2]))
        tau_land = max(tau_land, float(np.abs(b.read().effort).max()))
    bs = b.base_state()
    q = bs.orientation
    tilt = float(np.degrees(2 * np.arcsin(np.clip(np.hypot(q[0], q[1]), 0.0, 1.0))))
    b.shutdown()
    flight = vto * vto / (2 * 9.81)
    print(f"  take-off            {vto:5.2f} m/s")
    print(f"  FLIGHT height       {flight*1000:5.0f} mm   (all four feet clear, {air*dt*1000:.0f} ms airborne)")
    print(f"  apex of the body    {apex*1000:5.0f} mm   (flight plus leg extension)")
    print(f"  energy at take-off  {0.5*mass*vto*vto:5.1f} J")
    print(f"  peak torque         push {tau_push:.1f}, land {tau_land:.1f} N.m  "
          f"(rating {peak})")
    print(f"  landing             tilt {tilt:.0f} deg -> "
          f"{'survived' if tilt < 25 else 'FELL OVER (open loop, no attitude control)'}")

    # ---------------- 2. payload ----------------
    print()
    print(bar)
    print("2. PAYLOAD   static stance torque against added mass")
    print(bar)
    print(f"  {'payload':>9} {'total':>8} {'per foot':>10} {'peak tau':>10} {'of cont':>9}")
    for extra in (0.0, 2.0, 5.0, 8.0, 12.0, 20.0):
        t = tau_at(0.32, (mass + extra) * 9.81 / 4)
        flag = "" if t < cont else "   over continuous"
        print(f"  {extra:7.1f}kg {mass+extra:7.1f}kg {(mass+extra)*9.81/4:9.1f}N "
              f"{t:9.2f} {t/cont:8.0%}{flag}")

    # ---------------- 3. step-up ----------------
    print()
    print(bar)
    print("3. STEP-UP   the limit is remaining leg travel, so it depends on stance")
    print(bar)
    print(f"  {'stance':>8} {'travel':>8} {'max step':>10} {'3-leg tau':>11} {'of cont':>9}")
    for h in (0.24, 0.28, 0.32, 0.36, 0.40):
        travel = g.reach_max - (h - g.foot_radius)
        t = tau_at(h, mass * 9.81 / 3)
        print(f"  {h*1000:6.0f}mm {travel*1000:6.0f}mm {(travel-0.02)*1000:8.0f}mm "
              f"{t:10.2f} {t/cont:8.0%}")

    # ---------------- 4. slope ----------------
    print()
    print(bar)
    print("4. STATIC SLOPE   stance held on an incline")
    print(bar)
    print(f"  {'slope':>7} {'friction needed':>16} {'peak tau':>10} {'of cont':>9}")
    for deg in (10, 20, 30, 35, 40):
        a = math.radians(deg)
        t = tau_at(0.32, mass * 9.81 * math.cos(a) / 4)
        note = "" if math.tan(a) < 0.7 else "   near the friction limit"
        print(f"  {deg:5d}deg {math.tan(a):15.2f} {t:9.2f} {t/cont:8.0%}{note}")

    # ---------------- 5. actuator ----------------
    print()
    print(bar)
    print("5. ACTUATOR ENVELOPE")
    print(bar)
    rated_w = RS["performance"]["rated_torque_nm"] * RS["performance"]["rated_speed_rad_s"]
    peak_w = peak * noload / 4.0
    print(f"  rated mechanical power   {rated_w:6.1f} W/joint  x12 = {rated_w*12:7.0f} W")
    print(f"  peak mechanical power    {peak_w:6.1f} W/joint  x12 = {peak_w*12:7.0f} W")
    print(f"  peak power-to-weight     {peak_w*12/mass:6.0f} W/kg")
    print()
    print("  For scale: steady walking at 0.5 m/s costs roughly")
    print(f"  CoT x m x g x v = 1.0 x {mass:.0f} x 9.81 x 0.5 = {mass*9.81*0.5:.0f} W of NET")
    print("  mechanical work, about 7% of the rated figure. Almost all the torque a")
    print("  quadruped spends is holding itself up, which does no work at all.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
