#!/usr/bin/env python3
"""Official Go2 physics with the project's shared gait controller, not factory policy."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for _package in ("robodog_control", "robodog_hardware", "robodog_sim"):
    sys.path.insert(0, str(ROOT / "ros2_ws/src" / _package))
from robodog_control.kinematics import LegGeometry

OFFICIAL_REVISION = "1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d"


def model_xml(repo: Path) -> str:
    """Preserve official robot; normalize only documented environment settings."""
    source = repo / "unitree_robots/go2/go2.xml"
    root = ET.parse(source).getroot()
    root.find("compiler").set("meshdir", str(source.parent / "assets"))
    option = root.find("option")
    option.attrib.update(timestep="0.0005", integrator="implicitfast", cone="elliptic",
                         impratio="10", gravity="0 0 -9.81")
    foot = root.find(".//default[@class='foot']/geom")
    foot.attrib.update(friction="1.0 0.05 0.002", condim="6", priority="2",
                       solref="0.006 1", solimp="0.95 0.99 0.001", margin="0")
    ET.SubElement(root.find("worldbody"), "geom", name="benchmark_ground", type="plane",
                  size="30 30 .1", friction="0.9 0.02 0.001", condim="3", margin="0",
                  solref="0.02 1", solimp="0.9 0.95 0.001")
    return ET.tostring(root, encoding="unicode")


def load_model(repo: Path):
    return mujoco.MjModel.from_xml_string(model_xml(repo))


class Go2Adapter:
    """Explicit mapping: canonical FL/FR/RL/RR vs official actuator FR/FL/RR/RL."""
    legs = ("FL", "FR", "RL", "RR")
    kinds = ("hip", "thigh", "calf")
    names = tuple(f"{leg}_{kind}_joint" for leg in ("FL", "FR", "RL", "RR")
                  for kind in ("haa", "hfe", "kfe"))

    def __init__(self, model):
        self.model = model
        ids = [model.joint(f"{leg}_{kind}_joint").id for leg in self.legs for kind in self.kinds]
        self.qpos = model.jnt_qposadr[ids].copy()
        self.dofs = model.jnt_dofadr[ids].copy()
        self.actuators = np.array([model.actuator(f"{leg}_{kind}").id
                                   for leg in self.legs for kind in self.kinds])
        self.feet = np.array([model.geom(leg).id for leg in self.legs])
        self.base = model.body("base_link").id
        self.limits = model.actuator_ctrlrange[self.actuators].copy()
        self.joint_limits = model.jnt_range[ids].copy()
        # The official collision sphere is at (-.002,0,-.213), not the foot body origin.
        self.knee_bias = math.atan2(0.002, 0.213)
        self.geometry = LegGeometry(.1934, .0465, 0.0, .0955, .213, 0.0,
                                    math.hypot(.002, .213), .022)
        self.mass = float(model.body_subtreemass[self.base])

    def to_official_position(self, canonical):
        return np.asarray(canonical, dtype=float) - np.tile([0.0, 0.0, self.knee_bias], 4)

    def to_canonical_position(self, official):
        return np.asarray(official, dtype=float) + np.tile([0.0, 0.0, self.knee_bias], 4)

    def apply_torque(self, data, torque):
        torque = np.asarray(torque, dtype=float)
        if torque.shape != (12,) or not np.isfinite(torque).all():
            raise ValueError("twelve finite joint torque values required")
        clipped = np.clip(torque, self.limits[:, 0], self.limits[:, 1])
        data.ctrl[self.actuators] = clipped
        return np.abs(clipped - torque) > 1e-10


def validate_step(data, previous_time, dt):
    """Reject MuJoCo's automatic reset and invalid state instead of false success."""
    if abs(float(data.time) - previous_time - dt) > 1e-10:
        raise RuntimeError("MuJoCo time reset during Go2 benchmark")
    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
        raise RuntimeError("nonfinite Go2 simulation state")


def run_case(repo, *, vx, seconds=20., warmup=1., gait_overrides=None):
    """Run an unforced floating base with the same gait generator and PD gains."""
    from collections import deque
    import yaml
    from robodog_control.balance import roll_pitch_from_quat
    from robodog_control.gait import BodyFeedback, GaitGenerator, GaitParams
    from robodog_control.kinematics import inverse
    from robodog_hardware.physics_metrics import PhysicsMetrics, body_planar_velocity

    if not np.isfinite([vx, seconds, warmup]).all() or not 0 <= warmup < seconds:
        raise ValueError("finite duration > warmup >= 0 and velocity required")
    controls = yaml.safe_load((ROOT / "ros2_ws/src/robodog_control/config/control.yaml").read_text())["/**"]["ros__parameters"]
    gaits = yaml.safe_load((ROOT / "ros2_ws/src/robodog_control/config/gaits.yaml").read_text())["gaits"]
    fields = ("step_frequency_hz", "duty_factor", "step_height_m", "stance_height_m")
    settings = {key: gaits["trot"][key] for key in fields}
    settings = {**settings, **(gait_overrides or {})}
    adapter = Go2Adapter(load_model(Path(repo)))
    model, geometry = adapter.model, adapter.geometry
    data = mujoco.MjData(model)
    stand_height = gaits["stand"]["stance_height_m"]
    nominal = inverse(geometry, "FL", np.array([0., geometry.lateral,
                                               -(stand_height - geometry.foot_radius)]), clamp=False)
    data.qpos[:7] = [0., 0., stand_height + .002, 1., 0., 0., 0.]
    data.qpos[adapter.qpos] = adapter.to_official_position(np.tile(nominal, 4))
    mujoco.mj_forward(model, data)
    generator = GaitGenerator(geometry, nominal, adapter.mass)
    generator.set_params(GaitParams(gait="trot" if vx else "stand", vx=vx, **settings))
    metrics = PhysicsMetrics(adapter.names, warmup_s=warmup)
    dt = float(model.opt.timestep)
    control_dt = 1 / float(controls["control_rate_hz"])
    substeps = int(round(control_dt / dt))
    if abs(substeps * dt - control_dt) > 1e-12:
        raise ValueError("physics time step must divide controller period")
    count = int(round(seconds / control_dt))
    if abs(count * control_dt - seconds) > 1e-10:
        raise ValueError("duration must contain an integer number of control periods")
    zero = np.zeros(12)
    idle = (data.qpos[adapter.qpos].copy(), zero.copy(), zero.copy(), zero.copy(), zero.copy())
    delay = deque([idle, idle])  # Two 0.5 ms steps = own robot's assumed 1 ms command delay.
    gains = controls["gains"]
    positions, tilts, clipped_targets = [], [], 0
    initial = data.qpos[:3].copy()
    for _ in range(count):
        xyzw = data.qpos[[4, 5, 6, 3]]
        roll, pitch = roll_pitch_from_quat(xyzw)
        feedback = BodyFeedback(height=float(data.qpos[2]), vz=float(data.qvel[2]),
            roll=roll, pitch=pitch, omega=tuple(data.qvel[3:6]), v_xy=tuple(body_planar_velocity(xyzw, data.qvel[:3])))
        output = generator.update(control_dt, feedback)
        requested = adapter.to_official_position(output.q)
        target = np.clip(requested, adapter.joint_limits[:, 0], adapter.joint_limits[:, 1])
        clipped_targets += int(np.any(np.abs(target - requested) > 1e-10))
        kp = np.repeat(np.where(output.contact, gains["stance_kp"], gains["swing_kp"]), 3)
        kd = np.repeat(np.where(output.contact, gains["stance_kd"], gains["swing_kd"]), 3)
        command = (target, output.qd.copy(), output.tau_ff.copy(), kp, kd)
        for _ in range(substeps):
            delay.append(command)
            qstar, vstar, feedforward, kp_delayed, kd_delayed = delay.popleft()
            velocity = data.qvel[adapter.dofs].copy()
            torque = (kp_delayed * (qstar - data.qpos[adapter.qpos])
                      + kd_delayed * (vstar - velocity) + feedforward)
            clipped = adapter.apply_torque(data, torque)
            previous_time = float(data.time)
            mujoco.mj_step(model, data)
            validate_step(data, previous_time, dt)
            applied = data.qfrc_actuator[adapter.dofs].copy()
            physics_clipped = np.abs(applied - data.ctrl[adapter.actuators]) > 1e-8
            metrics.update(applied, velocity, dt, peak_clipped=clipped,
                           physics_clipped=physics_clipped)
        positions.append(data.qpos[:3].copy())
        tilts.append(float(np.degrees(2 * np.arcsin(np.clip(np.linalg.norm(data.qpos[4:6]), 0, 1)))))
    if not np.isfinite(data.qpos).all():
        raise RuntimeError("nonfinite Go2 simulation state")
    rows = metrics.summary()
    locations = np.asarray(positions)
    actual = float((locations[-1, 0] - initial[0]) / seconds)
    groups = {}
    for index, kind in enumerate(("hip_roll", "hip_pitch", "knee")):
        members = rows[index::3]
        groups[kind] = {
            "peak_joint_nm": max(j["joint_peak_nm"] for j in members),
            "worst_joint_rms_nm": max(j["joint_rms_nm"] for j in members),
            "pooled_joint_rms_nm": float(np.sqrt(np.mean([j["joint_rms_nm"] ** 2 for j in members]))),
        }
    return dict(commanded_vx_m_s=vx, duration_s=seconds, warmup_s=warmup,
        actual_vx_m_s=actual, travel_m=float(locations[-1, 0] - initial[0]),
        lateral_drift_m=float(np.abs(locations[:, 1] - initial[1]).max()),
        max_tilt_deg=max(tilts), min_height_m=float(locations[:, 2].min()),
        stable=bool(max(tilts) < 15 and locations[:, 2].min() > .25),
        tracks_velocity=bool(abs(actual-vx) < .07), mass_kg=adapter.mass,
        physics_samples=metrics.samples, initial_height_m=stand_height + .002,
        command_delay_s=2 * dt, physics_dt_s=dt, control_dt_s=control_dt, gait=settings, gains=gains,
        target_limit_cycles=clipped_targets, joints=rows, groups=groups)


def write_report(output: Path, report):
    output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    csv_rows = [{"commanded_vx_m_s": case["commanded_vx_m_s"],
                 "actual_vx_m_s": case["actual_vx_m_s"], "joint": row["name"],
                 "joint_rms_nm": row["joint_rms_nm"], "joint_peak_nm": row["joint_peak_nm"],
                 "speed_at_peak_rad_s": row["motor_speed_at_peak_rad_s"],
                 "above_8_s": row["threshold_metrics"]["8"]["total_time_s"],
                 "above_11_s": row["threshold_metrics"]["11"]["total_time_s"],
                 "torque_clip_steps": row["clipping"]["peak"]["steps"]}
                for case in report["cases"] for row in case["joints"]]
    with output.with_suffix(".csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(csv_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(csv_rows)
    lines = ["# Official Go2 model: shared-controller benchmark", "",
        "This uses the official Unitree Go2 MJCF with robodog's gait generator, "
        "not Unitree's factory walking controller. The floating base is never forced. "
        "Model details and limitations are in [the source audit](go2_benchmark_sources.md). "
        "The separate [official viewer/SDK smoke test](go2_runner_smoke.md) records its "
        "successful runtime and unresolved shutdown segmentation fault.", "",
        f"Official revision: `{report['official_revision']}`. Model mass: "
        f"**{report['cases'][0]['mass_kg']:.6f} kg**. MuJoCo {report['mujoco_version']}.", "",
        "Common environment: obstacle-free plane, gravity 9.81 m/s², "
        "0.5 ms physics / 2.5 ms control, implicitfast, elliptic cone, impratio 10. "
        "Foot friction is 1.0 / 0.05 / 0.002, condim 6; floor friction is "
        "0.9 / 0.02 / 0.001, condim 3. Foot priority 2, solref/solimp and zero "
        "foot/floor margins match robodog. "
        "These contact/solver overrides replace the official defaults solely for comparison.", "",
        "The official link inertias, collisions, damping, armature and actuator torque "
        "limits remain unchanged. Official hip outputs clip at 23.7 N·m, knees at "
        "45.43 N·m. This model supplies no calibrated continuous torque, thermal or "
        "torque-speed envelope; none is invented here. Its ideal torque envelope differs "
        "from robodog's RS06 model and safety limiter. The 8/11 N·m thresholds in JSON "
        "are comparison thresholds, not Go2 continuous ratings.", "",
        "The velocity step starts immediately from a standing IK pose with 2 mm "
        "ground clearance. Exact floating-base feedback is available to both simulations.", "",
        "Applied generalized joint torque (`qfrc_actuator`) is sampled every physics "
        "step, paired with pre-step joint velocity. Peaks include startup; RMS omits "
        "the warmup specified below. No external knee transmission is added to the Go2 model.", "",
        "| Command m/s | Actual m/s | Max tilt ° | Min height m | Stable / tracks |",
        "|---:|---:|---:|---:|---|"]
    for case in report["cases"]:
        lines.append(f"| {case['commanded_vx_m_s']:.2f} | {case['actual_vx_m_s']:.3f} | "
            f"{case['max_tilt_deg']:.1f} | {case['min_height_m']:.3f} | "
            f"{case['stable']} / {case['tracks_velocity']} |")
    for case in report["cases"]:
        clip_steps = sum(row["clipping"]["peak"]["steps"] for row in case["joints"])
        lines += ["", f"## {case['commanded_vx_m_s']:.2f} m/s command", "",
                  f"Duration {case['duration_s']:g} s; RMS warmup {case['warmup_s']:g} s; "
                  f"initial height {case['initial_height_m']:.3f} m; command delay {case['command_delay_s']*1000:g} ms. "
                  f"Trot: {case['gait']['step_frequency_hz']:g} Hz, "
                  f"duty {case['gait']['duty_factor']:g}, "
                  f"lift {case['gait']['step_height_m']:g} m, "
                  f"height {case['gait']['stance_height_m']:g} m. "
                  f"Stance PD {case['gains']['stance_kp']:g}/{case['gains']['stance_kd']:g}; "
                  f"swing PD {case['gains']['swing_kp']:g}/{case['gains']['swing_kd']:g}.", "",
                  f"Torque clipping: {clip_steps} joint-substeps; target-limit cycles: {case['target_limit_cycles']}.", "",
                  "| Joint group | Worst joint RMS N·m | Peak joint N·m |",
                  "|---|---:|---:|"]
        for kind, row in case["groups"].items():
            lines.append(f"| {kind.replace('_',' ')} | {row['worst_joint_rms_nm']:.2f} | {row['peak_joint_nm']:.2f} |")
        lines += ["", "| Joint | RMS N·m | Peak N·m | Speed at peak rad/s | Time above 8 / 11 N·m |",
                  "|---|---:|---:|---:|---:|"]
        for row in case["joints"]:
            thresholds = row["threshold_metrics"]
            lines.append(f"| {row['name']} | {row['joint_rms_nm']:.2f} | {row['joint_peak_nm']:.2f} | "
                f"{row['motor_speed_at_peak_rad_s']:.2f} | {thresholds['8']['total_time_s']:.3f} / "
                f"{thresholds['11']['total_time_s']:.3f} s |")
    lines += ["", "Reproduce after cloning the pinned official repository:", "", "```bash",
              "python3 tools/go2_benchmark.py --repo /tmp/robodog-unitree-mujoco "
              "--seconds 20 --speeds 0.4 0.5 --output docs/go2_benchmark", "```", ""]
    output.with_suffix(".md").write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path("/tmp/robodog-unitree-mujoco"))
    parser.add_argument("--seconds", type=float, default=20.)
    parser.add_argument("--speeds", type=float, nargs="+", default=[.4, .5])
    parser.add_argument("--output", type=Path, default=ROOT / "docs/go2_benchmark")
    args = parser.parse_args()
    revision = subprocess.check_output(["git", "-C", str(args.repo), "rev-parse", "HEAD"], text=True).strip()
    if revision != OFFICIAL_REVISION:
        raise ValueError(f"expected official revision {OFFICIAL_REVISION}, found {revision}")
    original = args.repo / "unitree_robots/go2/go2.xml"
    report = dict(official_revision=revision, model_source=str(original),
        original_model_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
        mujoco_version=mujoco.__version__, controller="robodog shared GaitGenerator, not factory policy",
        cases=[run_case(args.repo, vx=speed, seconds=args.seconds) for speed in args.speeds])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".xml").write_text(model_xml(args.repo))
    write_report(args.output, report)
    for case in report["cases"]:
        print({key:case[key] for key in ("commanded_vx_m_s", "actual_vx_m_s", "max_tilt_deg", "stable", "tracks_velocity")})


if __name__ == "__main__":
    main()
