"""CLI: run a configurable joint exercise.

    ros2 run robodog_control joint_test --mode sine --joints FL_kfe_joint --amplitude 0.3
    ros2 run robodog_control joint_test --mode sequential --dwell 3
    ros2 run robodog_control joint_test --mode sweep --f-start 0.2 --f-end 4.0

Publishes directly on robodog/joint_command, so it exercises the same path an
external controller would use, including the safety layer and the watchdog.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from robodog_msgs.msg import JointCommand, JointCommandArray

LEGS = ("FL", "FR", "RL", "RR")
KINDS = ("haa", "hfe", "kfe")
NAMES = [f"{l}_{k}_joint" for l in LEGS for k in KINDS]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", default="sine", choices=["sine", "sweep", "step", "sequential"])
    ap.add_argument("--joints", nargs="*", default=[], choices=NAMES,
                    help="default: all twelve")
    ap.add_argument("--amplitude", type=float, default=0.2, help="rad")
    ap.add_argument("--frequency", type=float, default=0.5, help="Hz")
    ap.add_argument("--f-start", type=float, default=0.2)
    ap.add_argument("--f-end", type=float, default=3.0)
    ap.add_argument("--duration", type=float, default=20.0, help="s")
    ap.add_argument("--dwell", type=float, default=2.0, help="s, step/sequential")
    ap.add_argument("--rate", type=float, default=100.0, help="Hz command rate")
    ap.add_argument("--kp", type=float, default=80.0)
    ap.add_argument("--kd", type=float, default=2.0)
    a = ap.parse_args(argv if argv is not None else rclpy.utilities.remove_ros_args()[1:])

    share = get_package_share_directory("robodog_description")
    with open(f"{share}/config/robot_parameters.yaml") as f:
        P = yaml.safe_load(f)
    stand = P["named_poses"]["stand"]
    base = [stand[k] for k in KINDS] * 4
    jl = P["joint_limits"]
    lower = [jl[n.split("_")[1]]["lower"] for n in NAMES]
    upper = [jl[n.split("_")[1]]["upper"] for n in NAMES]

    sel = [NAMES.index(n) for n in a.joints] if a.joints else list(range(12))
    # never let a test drive a joint into its end stop
    amp = [min(a.amplitude, max(min(upper[i] - base[i], base[i] - lower[i]), 0.0))
           if i in sel else 0.0 for i in range(12)]
    clipped = [NAMES[i] for i in sel if amp[i] < a.amplitude - 1e-9]
    if clipped:
        print(f"amplitude reduced to stay inside the joint limits on: {', '.join(clipped)}")

    rclpy.init()
    node = Node("robodog_joint_test_cli")
    pub = node.create_publisher(JointCommandArray, "robodog/joint_command", 10)
    print(f"mode={a.mode} joints={[NAMES[i] for i in sel]} amplitude<={a.amplitude} rad "
          f"duration={a.duration}s  (Ctrl-C to stop)")
    t0 = time.monotonic()
    period = 1.0 / a.rate
    try:
        while rclpy.ok():
            t = time.monotonic() - t0
            if t >= a.duration:
                break
            q = list(base)
            if a.mode == "sine":
                for i in sel:
                    q[i] += amp[i] * math.sin(2 * math.pi * a.frequency * t)
            elif a.mode == "sweep":
                k = (a.f_end - a.f_start) / max(a.duration, 1e-6)
                ph = 2 * math.pi * (a.f_start * t + 0.5 * k * t * t)
                for i in sel:
                    q[i] += amp[i] * math.sin(ph)
            elif a.mode == "step":
                s = 1.0 if int(t // a.dwell) % 2 else -1.0
                for i in sel:
                    q[i] += amp[i] * s
            else:                                    # sequential
                j = sel[int(t // a.dwell) % len(sel)]
                q[j] += amp[j] * math.sin(2 * math.pi * a.frequency * t)

            msg = JointCommandArray()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.names = list(NAMES)
            msg.commands = [
                JointCommand(mode=JointCommand.MODE_IMPEDANCE, position=float(q[i]),
                             velocity=0.0, effort=0.0, kp=a.kp, kd=a.kd)
                for i in range(12)]
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nreturning to the stand pose")
        msg = JointCommandArray()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.names = list(NAMES)
        msg.commands = [JointCommand(mode=JointCommand.MODE_IMPEDANCE, position=float(base[i]),
                                     kp=a.kp, kd=a.kd) for i in range(12)]
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
