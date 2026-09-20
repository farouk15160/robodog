"""
Design study: what actually reduces the torque the actuators have to carry?

Sweeps the two levers that change the stance lever arm -- link length and
stance height -- and reports joint torque, mass, leg travel left over, and the
steady-state winding temperature each implies.

The headline result is that the two levers are the same lever. Holding a load
costs torque proportional to the horizontal distance from the knee to the foot,
and both shorter links and a taller stance shrink it by straightening the leg.
One of them is free and reversible; the other is not.

Both are bounded by the same cost: a straighter leg has less travel left to
lift a foot with, and a worse-conditioned Jacobian with which to produce
horizontal force. Below roughly 60 mm of remaining lift the robot can no longer
climb its own 80 mm stairs.

Run:  python3 tools/design_study.py
"""
import numpy as np, yaml, math
from ament_index_python.packages import get_package_share_directory
from robodog_control.kinematics import LegGeometry, forward, gravity_torque, LEGS

share = get_package_share_directory("robodog_description")
P = yaml.safe_load(open(f"{share}/config/robot_parameters.yaml"))
RS = yaml.safe_load(open(f"{share}/config/robstride02.yaml"))
g0 = LegGeometry.from_params(P)
CONT = RS["operational_limits"]["continuous_torque_nm"]
KT = RS["electrical"]["torque_constant_nm_per_arms"]; RPH = RS["electrical"]["phase_resistance_ohm"]
RTH = RS["thermal"]["thermal_resistance_k_per_w"]; AMB = RS["thermal"]["ambient_temp_c"]

# Which mass scales with leg length and which does not.
# thigh 703.4 g = 380 motor + 231.6 long parts + 91.8 fittings
# calf  171.7 g =              88.9 long part  + 82.8 bearings + foot
SCALING_PER_LEG = (231.6 + 88.9) / 1000.0     # kg that shrink with the legs
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
    mass = BASE_MASS - 4 * SCALING_PER_LEG * (1 - k)
    load = mass * 9.81 / 4
    tau = max(float(np.abs(gravity_torque(g, leg, q, load)).max()) for leg in LEGS)
    ext = (h - g.foot_radius) / g.reach_max
    T = AMB + 3 * (tau / KT) ** 2 * RPH * RTH
    return dict(mass=mass, tau=tau, util=tau / CONT, ext=ext, temp=T,
                reach=g.reach_max, lift=g.reach_max - (h - g.foot_radius))

print("=" * 96)
print("A. STANCE HEIGHT AT THE CURRENT LEG LENGTH  (free: a config change, no hardware)")
print("=" * 96)
print(f"{'height':>8} {'tau':>7} {'% cont':>8} {'T_inf':>7} {'extension':>10} {'lift left':>10}  note")
for h in (0.24, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.40):
    r = evaluate(1.0, h)
    if not r: continue
    note = "SINGULAR-ish" if r["ext"] > 0.90 else ("tight" if r["ext"] > 0.85 else "")
    star = "  <- current" if abs(h - 0.32) < 1e-9 else ""
    print(f"{h*1000:7.0f}m {r['tau']:7.2f} {r['util']:7.1%} {r['temp']:6.0f}C {r['ext']:9.1%} "
          f"{r['lift']*1000:9.0f}mm  {note}{star}")

print()
print("=" * 96)
print("B. SHORTER LEGS, each at its own best stance height (85% extension)")
print("=" * 96)
print(f"{'scale':>6} {'L1':>7} {'L2':>7} {'mass':>7} {'height':>8} {'tau':>7} {'% cont':>8} "
      f"{'T_inf':>7} {'step':>7}")
for k in (1.00, 0.95, 0.90, 0.85, 0.80, 0.75, 0.70):
    g = geom(k)
    h = 0.85 * g.reach_max + g.foot_radius
    r = evaluate(k, h)
    print(f"{k:6.2f} {g.thigh*1000:6.0f}m {g.shank*1000:6.0f}m {r['mass']:6.3f}kg "
          f"{h*1000:7.0f}m {r['tau']:7.2f} {r['util']:7.1%} {r['temp']:6.0f}C "
          f"{r['lift']*1000:6.0f}mm")

print()
print("=" * 96)
print("C. SHORTER LEGS AT A FIXED 320 mm STANCE  (same ride height, shorter links)")
print("=" * 96)
print(f"{'scale':>6} {'L1':>7} {'L2':>7} {'mass':>7} {'tau':>7} {'% cont':>8} {'T_inf':>7} {'extension':>10}")
for k in (1.00, 0.95, 0.90, 0.85, 0.80, 0.75):
    g = geom(k); r = evaluate(k, 0.32)
    if not r:
        print(f"{k:6.2f} {g.thigh*1000:6.0f}m {g.shank*1000:6.0f}m   cannot reach 320 mm")
        continue
    print(f"{k:6.2f} {g.thigh*1000:6.0f}m {g.shank*1000:6.0f}m {r['mass']:6.3f}kg "
          f"{r['tau']:7.2f} {r['util']:7.1%} {r['temp']:6.0f}C {r['ext']:9.1%}")
