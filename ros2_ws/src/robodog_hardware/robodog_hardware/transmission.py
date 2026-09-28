"""Motor-output / joint mapping shared by hardware, simulation and telemetry.

Ratio is motor OUTPUT revolutions per joint revolution (not the internal 9:1
gearbox). Constant efficiency is a motoring approximation, also used for
conservative absolute motor-load estimates during braking; it is not a belt
compliance, hysteresis or regenerative-power model.
"""
from __future__ import annotations

import numpy as np


def transmission_arrays(config: dict, joint_names: list[str]) -> tuple[np.ndarray, np.ndarray]:
    transmissions = config.get("transmissions", {})
    entries = [transmissions.get(n.split("_")[1], {}) for n in joint_names]
    ratio = np.asarray([e.get("ratio", 1.0) for e in entries], dtype=float)
    efficiency = np.asarray([e.get("efficiency", 1.0) for e in entries], dtype=float)
    return validate_transmission(ratio, efficiency, len(joint_names))


def validate_transmission(ratio, efficiency, n: int) -> tuple[np.ndarray, np.ndarray]:
    ratio = np.broadcast_to(np.asarray(ratio, dtype=float), (n,)).copy()
    efficiency = np.broadcast_to(np.asarray(efficiency, dtype=float), (n,)).copy()
    if not np.all(np.isfinite(ratio) & (ratio > 0)):
        raise ValueError("transmission ratio must be finite and positive")
    if not np.all(np.isfinite(efficiency) & (efficiency > 0) & (efficiency <= 1)):
        raise ValueError("transmission efficiency must be in (0, 1]")
    return ratio, efficiency


def joint_limits(config: dict, joint_names: list[str]) -> dict[str, np.ndarray]:
    ratio, efficiency = transmission_arrays(config, joint_names)
    gain = ratio * efficiency
    operational = config["operational_limits"]
    return {
        "continuous_torque_nm": operational["continuous_torque_nm"] * gain,
        "peak_torque_nm": operational["peak_torque_nm"] * gain,
        "velocity_rad_s": operational["velocity_rad_s"] / ratio,
        "no_load_speed_rad_s": config["performance"]["no_load_speed_rad_s"] * voltage_scale(config) / ratio,
        "armature_kgm2": config["joint_dynamics"]["armature_kgm2"] * ratio ** 2,
    }


def backend_config(config: dict, joint_names: list[str]) -> dict:
    """Flatten MOTOR-side data for backends; each backend reflects it once."""
    ratio, efficiency = transmission_arrays(config, joint_names)
    return {
        **config["performance"], **config["electrical"],
        **config["thermal"], **config["joint_dynamics"],
        "motor_model": config["model"],
        "transmission_ratio": ratio.tolist(),
        "transmission_efficiency": efficiency.tolist(),
        "no_load_speed_rad_s": config["performance"]["no_load_speed_rad_s"] * voltage_scale(config),
        "supply_voltage_v": config["electrical"].get("supply_voltage_v", config["electrical"].get("rated_voltage_v", 48.0)),
    }


def voltage_scale(config: dict) -> float:
    """Estimated voltage-dependent no-load speed, not a measured T-N curve."""
    electrical = config["electrical"]
    rated = float(electrical.get("rated_voltage_v", 48.0))
    supply = float(electrical.get("supply_voltage_v", rated))
    if not np.isfinite(supply) or not electrical.get("voltage_min_v", 0) <= supply <= electrical.get("voltage_max_v", np.inf):
        raise ValueError("supply_voltage_v is outside the actuator operating range")
    return supply / rated
