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
import sys
import time

import yaml

from ..registry import create_backend
from ..types import JOINT_INDEX, JOINT_NAMES, ControlMode, JointCommand

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


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", required=True, choices=list(JOINT_NAMES))
    ap.add_argument("--config", default="config/robstride_bus.yaml")
    ap.add_argument("--torque", type=float, default=0.5, help="probe torque [N.m]")
    ap.add_argument("--duration", type=float, default=1.0, help="probe duration [s]")
    ap.add_argument("--lower-limit", type=float, required=True,
                    help="canonical angle [rad] of the hard stop used as reference")
    a = ap.parse_args(argv)

    idx = JOINT_INDEX[a.joint]
    kind = a.joint.split("_")[1]
    with open(a.config) as f:
        cfg = yaml.safe_load(f)

    print(f"\n=== calibrating {a.joint} ===")
    print(f"convention: {CONVENTION[kind]}")
    print("Ensure the robot is on a stand and this leg can move freely.")
    if _ask("continue? [y/N] ") != "y":
        return 1

    backend = create_backend("robstride02_can", cfg)
    backend.configure()
    try:
        mask = [i == idx for i in range(len(JOINT_NAMES))]
        backend.enable(mask)

        # ---- step 1: direction ----
        q0 = backend.read().position[idx]
        cmd = JointCommand()
        cmd.mode[idx] = int(ControlMode.TORQUE)
        cmd.effort[idx] = a.torque
        t_end = time.monotonic() + a.duration
        while time.monotonic() < t_end:
            backend.write(cmd)
            time.sleep(1.0 / cfg.get("control_rate_hz", 500.0))
        backend.write(JointCommand())
        q1 = backend.read().position[idx]
        moved = q1 - q0
        print(f"\napplied +{a.torque:.2f} N.m -> motor position moved {moved:+.4f} rad")
        print(f"Per the convention above, did the joint move in the POSITIVE direction?")
        direction = 1 if _ask("[y/n] ") == "y" else -1

        # ---- step 2: offset ----
        print(f"\nDrive the joint by hand onto the hard stop at "
              f"{a.lower_limit:+.4f} rad (canonical).")
        if _ask("at the stop? [y/N] ") != "y":
            return 1
        q_stop = backend.read().position[idx]
        offset = q_stop - direction * a.lower_limit

        print("\n=== add to config/robstride_bus.yaml ===")
        entry = cfg["joints"][a.joint].copy()
        entry.update(direction=direction, offset_rad=round(float(offset), 6))
        print(yaml.safe_dump({a.joint: entry}, default_flow_style=True).strip())
        return 0
    finally:
        backend.disable()
        backend.shutdown()


if __name__ == "__main__":
    sys.exit(main())
