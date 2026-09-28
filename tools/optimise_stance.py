#!/usr/bin/env python3
"""Screen standing poses by motor utilization; static equal-load estimate only."""
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from robodog_control.kinematics import LegGeometry, gravity_torque
from robodog_hardware.transmission import transmission_arrays
from robodog_hardware.types import JOINT_NAMES


def main():
    cfg = Path(get_package_share_directory("robodog_description")) / "config"
    parameters = yaml.safe_load((cfg / "robot_parameters.yaml").read_text())
    actuator = yaml.safe_load((cfg / parameters["actuator_config"]).read_text())
    geometry = LegGeometry.from_params(parameters)
    mass = parameters["mass_budget"]["total_kg"]
    load = mass * 9.81 / 4
    continuous = actuator["operational_limits"]["continuous_torque_nm"]
    ratio, efficiency = transmission_arrays(actuator, list(JOINT_NAMES))
    gain = (ratio * efficiency)[:3]
    limits = parameters["joint_limits"]
    hips = np.arange(limits["hfe"]["lower"], limits["hfe"]["upper"], 0.005)
    knees = np.arange(limits["kfe"]["lower"], min(-0.1, limits["kfe"]["upper"]), 0.005)
    hip, knee = np.meshgrid(hips, knees)
    z = -geometry.thigh * np.cos(hip) - geometry.shank * np.cos(hip + knee)
    x = -geometry.thigh * np.sin(hip) - geometry.shank * np.sin(hip + knee)
    heights = -z + geometry.foot_radius
    print(f"{actuator['model']}, {mass:.3f} kg, {load:.2f} N/foot, "
          f"{continuous:.1f} Nm MOTOR continuous")
    print("Equal-load point-mass screen; excludes limb gravity, dynamics and thermal validation.")
    print("height mm | HFE rad | KFE rad | worst motor Nm | continuous | extension")
    for height in (0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36):
        selected = np.nonzero((np.abs(heights - height) < 0.0015) & (np.abs(x) < 0.05))
        candidates = []
        for row, col in zip(*selected):
            q = np.array([0.0, hip[row, col], knee[row, col]])
            motor = np.abs(gravity_torque(geometry, "FL", q, load)) / gain
            candidates.append((float(motor.max()), q, float(np.hypot(x[row, col], z[row, col]))))
        if not candidates:
            continue
        worst, pose, reach = min(candidates, key=lambda item: item[0])
        print(f"{height * 1000:9.0f} | {pose[1]:7.3f} | {pose[2]:7.3f} | "
              f"{worst:14.3f} | {worst / continuous:9.1%} | {reach / geometry.reach_max:8.1%}")


if __name__ == "__main__":
    main()
