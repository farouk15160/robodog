"""
Design study: what actually reduces the torque the actuators have to carry?

Sweeps the two levers that change the stance lever arm -- link length and
stance height -- and reports joint torque, mass, leg travel left over, and the
steady-state winding temperature each implies.

This is a fixed-mass geometry sensitivity study, not a prediction of a rebuilt
robot's mass or terrain capability. It evaluates equal-load static support and
converts all joint torques through the active transmission configuration.
With a reduced knee drive, a direct-drive hip can become the limiting motor;
shorter legs therefore need not reduce the maximum motor utilization.

Run:  python3 tools/design_study.py
"""
import numpy as np, yaml, math
from ament_index_python.packages import get_package_share_directory
from robodog_control.kinematics import LegGeometry, forward, gravity_torque, LEGS
from robodog_hardware.transmission import transmission_arrays
from robodog_hardware.types import JOINT_NAMES

share = get_package_share_directory("robodog_description")
P = yaml.safe_load(open(f"{share}/config/robot_parameters.yaml"))
RS = yaml.safe_load(open(f"{share}/config/{P['actuator_config']}"))
RATIO, EFFICIENCY = transmission_arrays(RS, list(JOINT_NAMES))
GAIN = (RATIO * EFFICIENCY).reshape(4, 3)
g0 = LegGeometry.from_params(P)
CONT = RS["operational_limits"]["continuous_torque_nm"]
KT = RS["electrical"]["torque_constant_nm_per_arms"]; RPH = RS["electrical"]["phase_resistance_ohm"]
RTH = RS["thermal"]["thermal_resistance_k_per_w"]; AMB = RS["thermal"]["ambient_temp_c"]

# Hold mass fixed: no identified mass-vs-length model exists for the new CAD.
# This isolates geometry instead of reusing the old RS02 limb-mass decomposition.
BASE_MASS = P["mass_budget"]["total_kg"]

def geom(k):
    return LegGeometry(g0.haa_x, g0.haa_y, g0.hfe_dx, g0.hfe_dr,
                       g0.thigh * k, g0.thigh_lat, g0.shank * k, g0.foot_radius)

def pose_for_height(g, h):
    """Foot vertically below the HFE axis, as the named poses define it."""
    reach = h - g.foot_radius
    c = (reach**2 - g.thigh**2 - g.shank**2) / (2 * g.thigh * g.shank)
    if not -1 <= c <= 1:
        return None
    q2 = -math.acos(c)
    q1 = -math.atan2(g.shank * math.sin(q2), g.thigh + g.shank * math.cos(q2))
    return np.array([0.0, q1, q2])

def evaluate(k, h):
    g = geom(k)
    q = pose_for_height(g, h)
    if q is None:
        return None
    mass = BASE_MASS
    load = mass * 9.81 / 4
    joint = np.stack([np.abs(gravity_torque(g, leg, q, load)) for leg in LEGS])
    tau = float((joint / GAIN).max())
    ext = (h - g.foot_radius) / g.reach_max
    T = AMB + 3 * (tau / KT) ** 2 * RPH * RTH
    return dict(mass=mass, tau=tau, util=tau / CONT, ext=ext, temp=T,
                reach=g.reach_max, lift=g.reach_max - (h - g.foot_radius))

print(f"{RS['model']}: static point-load study, fixed {BASE_MASS:.3f} kg.")
print("tau = worst MOTOR-output torque; knee belt mapping included.")
print("Temperature is uncalibrated; reach reserve does not prove stair capability.")
print("Limb gravity and dynamics omitted; use torque_report.py for actual gait loads.")
print("=" * 96)
print("A. STANCE HEIGHT AT THE CURRENT LEG LENGTH  (free: a config change, no hardware)")
print("=" * 96)
print(f"{'height':>8} {'tau':>7} {'% cont':>8} {'T_inf':>7} {'extension':>10} {'lift left':>10}  note")
for h in (0.24, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.40):
    r = evaluate(1.0, h)
    if not r: continue
    note = "SINGULAR-ish" if r["ext"] > 0.90 else ("tight" if r["ext"] > 0.85 else "")
    star = "  <- current" if abs(h - 0.32) < 1e-9 else ""
    print(f"{h*1000:7.0f}mm {r['tau']:7.2f} {r['util']:7.1%} {r['temp']:6.0f}C {r['ext']:9.1%} "
          f"{r['lift']*1000:9.0f}mm  {note}{star}")

print()
print("=" * 96)
print("B. SHORTER LEGS, each at an illustrative stance height (85% extension)")
print("=" * 96)
print(f"{'scale':>6} {'L1':>7} {'L2':>7} {'mass':>7} {'height':>8} {'tau':>7} {'% cont':>8} "
      f"{'T_inf':>7} {'reserve':>7}")
for k in (1.00, 0.95, 0.90, 0.85, 0.80, 0.75, 0.70):
    g = geom(k)
    h = 0.85 * g.reach_max + g.foot_radius
    r = evaluate(k, h)
    print(f"{k:6.2f} {g.thigh*1000:6.0f}mm {g.shank*1000:6.0f}mm {r['mass']:6.3f}kg "
          f"{h*1000:7.0f}mm {r['tau']:7.2f} {r['util']:7.1%} {r['temp']:6.0f}C "
          f"{r['lift']*1000:6.0f}mmm")

print()
print("=" * 96)
print("C. SHORTER LEGS AT A FIXED 320 mm STANCE  (same ride height, shorter links)")
print("=" * 96)
print(f"{'scale':>6} {'L1':>7} {'L2':>7} {'mass':>7} {'tau':>7} {'% cont':>8} {'T_inf':>7} {'extension':>10}")
for k in (1.00, 0.95, 0.90, 0.85, 0.80, 0.75):
    g = geom(k); r = evaluate(k, 0.32)
    if not r:
        print(f"{k:6.2f} {g.thigh*1000:6.0f}mm {g.shank*1000:6.0f}mm   cannot reach 320 mm")
        continue
    print(f"{k:6.2f} {g.thigh*1000:6.0f}mm {g.shank*1000:6.0f}mm {r['mass']:6.3f}kg "
          f"{r['tau']:7.2f} {r['util']:7.1%} {r['temp']:6.0f}C {r['ext']:9.1%}")
