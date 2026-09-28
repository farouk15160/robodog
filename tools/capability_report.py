#!/usr/bin/env python3
"""Static load screening for the active robot; not a locomotion certificate.

Equal foot loading and a point body mass omit limb gravity, acceleration,
contact transitions and center-of-mass offsets. Use torque_report.py for dynamic
per-joint measurements with the gait and safety layer.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from robodog_control.kinematics import LEGS, LegGeometry, gravity_torque
from robodog_hardware.transmission import transmission_arrays
from robodog_hardware.types import JOINT_NAMES


def main() -> int:
    cfg = Path(get_package_share_directory("robodog_description")) / "config"
    parameters = yaml.safe_load((cfg / "robot_parameters.yaml").read_text())
    actuator = yaml.safe_load((cfg / parameters["actuator_config"]).read_text())
    geometry = LegGeometry.from_params(parameters)
    mass = parameters["mass_budget"]["total_kg"]
    continuous = actuator["operational_limits"]["continuous_torque_nm"]
    ratio, efficiency = transmission_arrays(actuator, list(JOINT_NAMES))
    gain = (ratio * efficiency).reshape(4, 3)

    def loads(height, load_mass, supports):
        reach = height - geometry.foot_radius
        cosine = ((reach ** 2 - geometry.thigh ** 2 - geometry.shank ** 2)
                  / (2 * geometry.thigh * geometry.shank))
        if not -1.0 <= cosine <= 1.0:
            return None
        knee = -math.acos(cosine)
        hip = -math.atan2(geometry.shank * math.sin(knee),
                          geometry.thigh + geometry.shank * math.cos(knee))
        q = np.array([0.0, hip, knee])
        joint = np.stack([np.abs(gravity_torque(geometry, leg, q,
                         load_mass * 9.81 / supports)) for leg in LEGS])
        return joint, joint / gain

    print(f"{actuator['model']}, {mass:.3f} kg; STATIC POINT-LOAD SCREEN ONLY")
    print("Continuous rating assumes manufacturer cooling. No belt strength assessment.")
    print("Limb gravity, asymmetric loading, dynamics and motor heating are omitted.\n")
    height = parameters["named_poses"]["stand"]["base_height_m"]
    print(f"Payload screen at {height * 1000:.0f} mm, four equal supports")
    print("payload kg | max joint Nm | max motor Nm | motor continuous utilization")
    for extra in (0.0, 2.0, 5.0, 8.0, 12.0, 20.0):
        result = loads(height, mass + extra, 4)
        if result is None:
            raise ValueError("configured standing height is unreachable")
        joint, motor = result
        print(f"{extra:10.1f} | {joint.max():12.2f} | {motor.max():12.2f} | "
              f"{motor.max() / continuous:8.1%}")
    print("\nThree-support stance screen; reach reserve is NOT demonstrated step height")
    print("height mm | reach reserve mm | max joint Nm | max motor Nm | utilization")
    for height in (0.24, 0.28, 0.30, 0.32, 0.34, 0.36):
        result = loads(height, mass, 3)
        if result is None:
            continue
        joint, motor = result
        reserve = geometry.reach_max - height + geometry.foot_radius
        print(f"{height * 1000:9.0f} | {reserve * 1000:16.0f} | {joint.max():12.2f} | "
              f"{motor.max():12.2f} | {motor.max() / continuous:8.1%}")
    print("\nJump, slope and terrain capability require dedicated dynamic experiments.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
