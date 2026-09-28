"""
Commissioning tool: determine `direction` and `offset_rad` for one joint.

Run once per joint, with the robot on a stand and the leg free to move.

    ros2 run robodog_hardware calibrate_joint --joint FL_haa_joint

Procedure
---------
1. DIRECTION. Apply a small feed-forward torque (default 0.5 N.m, well under
   the 6 N.m continuous rating) and observe which way the motor-reported
   position moves. The operator confirms against the URDF convention printed on
   screen; the sign that makes the two agree is written out.
2. OFFSET. The operator drives the joint to a known mechanical reference --
   the hard stop at the joint's lower limit -- and confirms. The offset is the
   motor reading at that point minus the known canonical angle.

Nothing is written automatically: the tool prints a YAML fragment for
config/robstride_bus.yaml so the change is reviewed before it reaches the robot.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import yaml

from ..registry import create_backend
from ..backend import BackendError
from ..transmission import backend_config
from ..types import JOINT_INDEX, JOINT_NAMES, ControlMode, JointCommand, Fault

CONVENTION = {
    "haa": "+ rotates about +x (base frame): LEFT legs move outboard, RIGHT legs inboard",
    "hfe": "+ rotates about +y: the thigh swings BACKWARD (-x)",
    "kfe": "+ rotates about +y: the knee EXTENDS toward straight (0 = fully extended)",
}


def _ask(prompt: str) -> str:
    try:
        return input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print("\naborted")
        sys.exit(1)


def _joint_position(backend, index):
    """Never infer a calibration from an absent/stale CAN measurement."""
    deadline = time.monotonic() + .2
    while True:
        state = backend.read()
        faults = int(state.faults[index]) & ~int(Fault.NOT_ENABLED)
        if not faults and math.isfinite(state.position[index]):
            return float(state.position[index])
        if time.monotonic() >= deadline:
            raise BackendError("calibration requires fresh, fault-free joint feedback")
        time.sleep(.005)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", required=True, choices=list(JOINT_NAMES))
    ap.add_argument("--config", default="config/robstride_bus.yaml")
    ap.add_argument("--actuator-config", help="actuator YAML; default is active robot configuration")
    ap.add_argument("--torque", type=float, default=0.5, help="joint-side probe torque [N.m], maximum 1")
    ap.add_argument("--duration", type=float, default=1.0, help="probe duration [s]")
    ap.add_argument("--lower-limit", type=float, required=True,
                    help="canonical angle [rad] of the hard stop used as reference")
    a = ap.parse_args(argv)
    if not (math.isfinite(a.torque) and 0 < a.torque <= 1 and
            math.isfinite(a.duration) and 0 < a.duration <= 2 and math.isfinite(a.lower_limit)):
        ap.error("probe requires finite 0 < torque <= 1 Nm, 0 < duration <= 2 s and a finite angle")

    idx = JOINT_INDEX[a.joint]
    kind = a.joint.split("_")[1]
    with open(a.config) as f:
        cfg = yaml.safe_load(f)
    if a.actuator_config:
        spec_path = Path(a.actuator_config)
    else:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("robodog_description"))
        params = yaml.safe_load((share / "config/robot_parameters.yaml").read_text())
        spec_path = share / "config" / params.get("actuator_config", "robstride06.yaml")
    spec = yaml.safe_load(spec_path.read_text())
    cfg = {**cfg, **backend_config(spec, list(JOINT_NAMES))}

    print(f"\n=== calibrating {a.joint} ===")
    print(f"convention: {CONVENTION[kind]}")
    print("Ensure the robot is on a stand and this leg can move freely.")
    if _ask("continue? [y/N] ") != "y":
        return 1

    backend_name = {"ROBSTRIDE02": "robstride02_can", "ROBSTRIDE06": "robstride06_can"}[spec["model"]]
    backend = create_backend(backend_name, cfg)
    backend.configure()
    try:
        mask = [i == idx for i in range(len(JOINT_NAMES))]
        if backend_name == "robstride06_can":
            backend.enable(mask, commissioning_joint=a.joint)
        else:
            backend.enable(mask)

        # ---- step 1: direction ----
        q0 = _joint_position(backend, idx)
        cmd = JointCommand()
        cmd.mode[idx] = int(ControlMode.TORQUE)
        cmd.effort[idx] = a.torque
        t_end = time.monotonic() + a.duration
        while time.monotonic() < t_end:
            backend.write(cmd)
            time.sleep(1.0 / cfg.get("control_rate_hz", 500.0))
        backend.write(JointCommand())
        q1 = _joint_position(backend, idx)
        backend.disable()
        moved = q1 - q0
        print(f"\napplied +{a.torque:.2f} N.m -> motor position moved {moved:+.4f} rad")
        print(f"Per the convention above, did the joint move in the POSITIVE direction?")
        answer = _ask("[y/n] ")
        if answer not in ("y", "n"):
            print("Direction was not explicitly confirmed; no calibration produced.")
            return 1
        direction = int(backend.direction[idx]) * (1 if answer == "y" else -1)

        # ---- step 2: offset ----
        print(f"\nDrive the joint by hand onto the hard stop at "
              f"{a.lower_limit:+.4f} rad (canonical).")
        if _ask("at the stop? [y/N] ") != "y":
            return 1
        backend.write(JointCommand())  # zero-gain request obtains fresh feedback while disabled
        q_stop = _joint_position(backend, idx)
        # Undo the old calibration to recover the actuator OUTPUT encoder;
        # offsets live before the external belt, not in canonical knee units.
        motor_stop = q_stop * backend.direction[idx] * backend.ratio[idx] + backend.offset[idx]
        offset = motor_stop - direction * backend.ratio[idx] * a.lower_limit

        print(f"\nProposed mapping: direction={direction}, motor offset={offset:+.6f} rad")
        if _ask("Confirm physical direction and reference angle are verified? [y/N] ") != "y":
            return 1
        print("\n=== review and add to config/robstride_bus.yaml ===")
        entry = {**cfg["joints"][a.joint], "direction": direction,
                 "offset_rad": round(float(offset), 6), "calibrated": True}
        print(yaml.safe_dump({a.joint: entry}, default_flow_style=True).strip())
        return 0
    finally:
        backend.disable()
        backend.shutdown()


if __name__ == "__main__":
    sys.exit(main())
