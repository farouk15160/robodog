#!/usr/bin/env python3
"""Trigger one greeting and emit its base-stability measurements as JSON.

Run this from a sourced ROS workspace while a MuJoCo bringup is active. The
script is read-only apart from calling the simulation-only greeting service.
"""
from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from robodog_msgs.msg import RobotState
from std_srvs.srv import Trigger


def _roll_pitch(q) -> tuple[float, float]:
    sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
    cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = (math.copysign(math.pi / 2.0, sin_pitch)
             if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch))
    return roll, pitch


class GreetingMeasurement(Node):
    def __init__(self) -> None:
        super().__init__("robodog_greeting_validation")
        self.latest: RobotState | None = None
        self.samples: list[tuple[float, float, float, int]] = []
        self.create_subscription(
            RobotState, "/robodog/robot_state", self._on_state, 20)
        self.client = self.create_client(Trigger, "/robodog/greeting")

    def _on_state(self, message: RobotState) -> None:
        self.latest = message
        if message.controller.active_controller != "greeting":
            return
        roll, pitch = _roll_pitch(message.base_pose.orientation)
        self.samples.append((
            float(message.base_height_m), abs(roll), abs(pitch),
            sum(bool(foot.contact) for foot in message.feet),
        ))


def measure(timeout_s: float) -> dict:
    node = GreetingMeasurement()
    try:
        deadline = time.monotonic() + timeout_s
        while node.latest is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest is None:
            raise RuntimeError("no /robodog/robot_state received")
        if not node.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/robodog/greeting service unavailable")

        future = node.client.call_async(Trigger.Request())
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        response = future.result() if future.done() else None
        if response is None or not response.success:
            reason = response.message if response is not None else "service timeout"
            raise RuntimeError(f"greeting rejected: {reason}")

        saw_greeting = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            active = (node.latest is not None and
                      node.latest.controller.active_controller == "greeting")
            saw_greeting = saw_greeting or active
            if saw_greeting and not active:
                break
        if not node.samples:
            raise RuntimeError("greeting produced no telemetry samples")

        heights, rolls, pitches, contacts = zip(*node.samples)
        final = node.latest
        return {
            "schema": "robodog.greeting_validation.v1",
            "service_message": response.message,
            "samples": len(node.samples),
            "min_base_height_m": min(heights),
            "max_abs_roll_rad": max(rolls),
            "max_abs_pitch_rad": max(pitches),
            "min_contact_count": min(contacts),
            "final_controller": final.controller.active_controller if final else "",
            "final_state": final.state_name if final else "",
        }
    finally:
        node.destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    rclpy.init()
    try:
        print(json.dumps(measure(args.timeout), indent=2, sort_keys=True))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
