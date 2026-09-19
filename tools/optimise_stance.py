"""Choose the nominal stance: minimise peak actuator utilisation while
standing, subject to a usable body height and margin from the singularity."""
import numpy as np, yaml
from robodog_control.kinematics import LegGeometry, forward, gravity_torque
P=yaml.safe_load(open("src/robodog_description/config/robot_parameters.yaml"))
g=LegGeometry.from_params(P)
M=P["mass_budget"]["total_kg"]; F=M*9.81/4.0
CONT=6.0
jl=P["joint_limits"]

print(f"mass {M} kg -> {F:.2f} N per foot (static, even load), continuous limit {CONT} N.m\n")
print(f"{'height':>7} {'hfe':>6} {'kfe':>6} {'tau_hfe':>8} {'tau_kfe':>8} {'peak%':>7} {'extension':>10}")
print("-"*62)
best={}
for h_target in [0.24,0.26,0.28,0.30,0.32,0.34,0.36]:
    rows=[]
    for q1 in np.arange(0.20, 2.20, 0.002):
        for q2 in np.arange(-2.55, -0.10, 0.002):
            z = -g.thigh*np.cos(q1) - g.shank*np.cos(q1+q2)
            h = -z + g.foot_radius
            if abs(h-h_target) > 0.0015: continue
            x = -g.thigh*np.sin(q1) - g.shank*np.sin(q1+q2)
            if abs(x) > 0.05: continue                      # foot under the hip
            t = gravity_torque(g,"FL",np.array([0.0,q1,q2]),F)
            rows.append((max(abs(t[1]),abs(t[2])), q1,q2,t[1],t[2],np.hypot(x,z)))
    if not rows: continue
    rows.sort(); pk,q1,q2,th,tk,ext = rows[0]
    best[round(h_target,2)]=(q1,q2,pk)
    print(f"{h_target:7.2f} {q1:6.3f} {q2:6.3f} {th:8.3f} {tk:8.3f} {pk/CONT*100:6.1f}% {ext/g.reach_max*100:9.1f}%")

print("\nShipped 'stand' pose (0.80, -1.60) for comparison:")
t=gravity_torque(g,"FL",np.array([0.0,0.80,-1.60]),F)
z=forward(g,"FL",np.array([0.0,0.80,-1.60]))[2]
print(f"  height {(-z+g.foot_radius)*1000:.1f} mm  tau=({t[1]:+.3f},{t[2]:+.3f})  peak {max(abs(t[1]),abs(t[2]))/CONT*100:.1f}%")

h=0.30; q1,q2,pk=best[h]
print(f"\nRecommended stance at {h:.2f} m: hfe={q1:.3f} kfe={q2:.3f} -> peak {pk/CONT*100:.1f}% of continuous")
print(f"  dynamic headroom to peak torque (17 N.m): {17.0/pk:.2f}x")
print(f"  all-four-feet payload margin before hitting continuous: {CONT/pk:.2f}x static load")
