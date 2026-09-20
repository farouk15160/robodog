"""
Thermal load report: what the actuators actually see during each gait.

Peak torque is the wrong question to ask of a motor. What decides whether a
winding survives is the RMS torque, because copper loss goes as current
squared. A 17 N.m spike lasting 20 ms is harmless; 10 N.m sustained is not.

This runs each gait in MuJoCo, logs every joint, and reports peak, 99th
percentile and RMS torque, the fraction of time above the continuous rating,
and the steady-state winding temperature that RMS implies through the thermal
model in robstride02.yaml.

Run:  MUJOCO_GL=egl python3 tools/torque_report.py
"""
import os, numpy as np, yaml
from ament_index_python.packages import get_package_share_directory
from robodog_hardware.registry import create_backend
from robodog_hardware.types import ControlMode, JointCommand, JOINT_NAMES
from robodog_hardware.thermal import ThermalModel
from robodog_control.kinematics import LegGeometry
from robodog_control.gait import BodyFeedback, GaitGenerator, GaitParams
from robodog_control.balance import roll_pitch_from_quat

share = get_package_share_directory("robodog_description")
P = yaml.safe_load(open(f"{share}/config/robot_parameters.yaml"))
RS = yaml.safe_load(open(f"{share}/config/robstride02.yaml"))
GAITS = yaml.safe_load(open(os.path.join(get_package_share_directory("robodog_control"),
                                         "config", "gaits.yaml")))["gaits"]
g = LegGeometry.from_params(P); MASS = P["mass_budget"]["total_kg"]
stand = P["named_poses"]["stand"]; q0 = np.array([stand["haa"], stand["hfe"], stand["kfe"]])
CONT = RS["operational_limits"]["continuous_torque_nm"]
PEAK = RS["performance"]["peak_torque_nm"]
KT = RS["electrical"]["torque_constant_nm_per_arms"]
RPH = RS["electrical"]["phase_resistance_ohm"]
TH = RS["thermal"]
DT = 1/400.0

def run(gait, vx, seconds=8.0):
    b = create_backend("mujoco", dict(
        model_path=os.path.join(get_package_share_directory("robodog_sim"),
                                "models", "robodog_scene.xml"),
        keyframe="stand", peak_torque_nm=PEAK,
        no_load_speed_rad_s=RS["performance"]["no_load_speed_rad_s"]))
    b.configure(); b.enable()
    gen = GaitGenerator(g, q0, MASS)
    d = GAITS[gait]
    gen.set_params(GaitParams(gait=gait, step_frequency_hz=d["step_frequency_hz"],
        step_height_m=d["step_height_m"], duty_factor=d["duty_factor"],
        stance_height_m=d["stance_height_m"], vx=vx))
    log = []
    for i in range(int(seconds/DT)):
        bs = b.base_state()
        roll, pitch = roll_pitch_from_quat(bs.orientation)
        fb = BodyFeedback(height=float(bs.position[2]), vz=float(bs.linear_velocity[2]),
            roll=roll, pitch=pitch, omega=tuple(bs.angular_velocity),
            v_xy=(float(bs.linear_velocity[0]), float(bs.linear_velocity[1])))
        out = gen.update(DT, fb)
        c = JointCommand(); c.mode[:] = int(ControlMode.IMPEDANCE)
        c.position[:] = out.q; c.velocity[:] = out.qd; c.effort[:] = out.tau_ff
        c.kp[:] = np.repeat(np.where(out.contact, 90.0, 60.0), 3)
        c.kd[:] = np.repeat(np.where(out.contact, 2.0, 1.5), 3)
        b.write(c); b.step(DT)
        if i * DT > 1.0:                       # skip the start-up transient
            log.append(b.read().effort.copy())
    b.shutdown()
    return np.array(log)

def steady_temp(rms):
    return TH["ambient_temp_c"] + 3*(rms/KT)**2*RPH*TH["thermal_resistance_k_per_w"]

print(f"ROBSTRIDE02: {CONT} N.m continuous, {PEAK} N.m peak,  Kt={KT}, R_phase={RPH}")
print(f"{'case':16s} {'peak':>7s} {'p99':>7s} {'RMS':>7s} {'RMS/cont':>9s} "
      f"{'%time>cont':>11s} {'T_steady':>9s}  verdict")
print("-"*92)
for gait, vx in [("stand", 0.0), ("walk", 0.15), ("trot", 0.30), ("trot", 0.50), ("bound", 0.30)]:
    t = np.abs(run(gait, vx))
    peak, p99 = t.max(), np.percentile(t, 99)
    worst = int(np.argmax(t.std(axis=0) + t.mean(axis=0)))   # busiest joint
    rms_j = np.sqrt((t**2).mean(axis=0))
    rms = rms_j.max()
    over = (t > CONT).mean()*100
    Ts = steady_temp(rms)
    verdict = ("fine" if Ts < 70 else "warm" if Ts < 85 else "OVER LIMIT")
    print(f"{gait+' '+str(vx):16s} {peak:7.2f} {p99:7.2f} {rms:7.2f} {rms/CONT:8.0%} "
          f"{over:10.1f}% {Ts:8.1f}C  {verdict}  (hottest {JOINT_NAMES[int(np.argmax(rms_j))]})")
