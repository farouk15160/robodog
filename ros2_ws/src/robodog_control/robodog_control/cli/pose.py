"""CLI: move the robot to a named pose.

    ros2 run robodog_control pose stand
    ros2 run robodog_control pose rest --duration 4.0
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node

from robodog_msgs.srv import SetNamedPose


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pose", choices=["zero", "stand", "crouch", "rest"])
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds; 0 = pick automatically from the speed limit")
    ap.add_argument("--timeout", type=float, default=10.0)
    a = ap.parse_args(argv if argv is not None else rclpy.utilities.remove_ros_args()[1:])

    rclpy.init()
    node = Node("robodog_pose_cli")
    cli = node.create_client(SetNamedPose, "robodog/set_named_pose")
    try:
        if not cli.wait_for_service(timeout_sec=a.timeout):
            print("robodog/set_named_pose is not available -- is the control node running?",
                  file=sys.stderr)
            return 1
        req = SetNamedPose.Request(pose=a.pose, duration_s=a.duration)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=a.timeout)
        r = fut.result()
        if r is None:
            print("no response", file=sys.stderr)
            return 1
        print(f"{'ok' if r.success else 'FAILED'}: {r.message} "
              f"({r.planned_duration_s:.2f} s)")
        return 0 if r.success else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
