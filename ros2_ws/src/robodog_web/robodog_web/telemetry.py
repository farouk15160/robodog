"""JSON metadata describing the torque reference used by the joint panel."""
from robodog_hardware.transmission import joint_limits, transmission_arrays


def joint_actuator_info(spec: dict, names: list[str]) -> dict:
    """Expose joint-side ratings and external belt gain without ROS dependencies."""
    ratios, efficiencies = transmission_arrays(spec, names)
    limits = joint_limits(spec, names)
    return {
        name: {
            "ratio": float(ratios[i]),
            "efficiency": float(efficiencies[i]),
            "torque_gain": float(ratios[i] * efficiencies[i]),
            "continuous_torque_nm": float(limits["continuous_torque_nm"][i]),
            "continuous_limit_basis": spec["performance"].get(
                "continuous_torque_basis", "configured limit"),
            "peak_torque_nm": float(limits["peak_torque_nm"][i]),
        }
        for i, name in enumerate(names)
    }
