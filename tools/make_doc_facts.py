#!/usr/bin/env python3
r"""
Emit docs/generated_facts.tex: every number the document quotes, as LaTeX
macros read from the generated model and the actuator datasheet.

The document cites \RdThighLength, not "213 mm". Regenerating the model after a
CAD change updates the prose automatically, and a figure that drifts out of the
text is impossible rather than merely unlikely.

Run:  python3 tools/make_doc_facts.py
"""
from __future__ import annotations

import math
import os
import subprocess

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, "ros2_ws/src/robodog_description/config")


def num(v, d=3):
    """Format, trimming only FRACTIONAL trailing zeros.

    A naive rstrip("0") turns 380 into 38 and 60 into 6, which is exactly the
    kind of error that reads as plausible in a table. Only strip when the
    formatted string actually has a decimal point.
    """
    if not isinstance(v, float):
        return str(v)
    s = f"{v:.{d}f}"
    return s.rstrip("0").rstrip(".") if "." in s else s


def sci(v, d=3):
    """Scientific notation as LaTeX math, e.g. 4.805\\times10^{-3}."""
    m = f"{v:.{d}e}"
    mant, exp = m.split("e")
    return (f"\\ensuremath{{{mant.rstrip('0').rstrip('.')}"
            f"\\times 10^{{{int(exp)}}}}}")


def main() -> int:
    P = yaml.safe_load(open(os.path.join(CFG, "robot_parameters.yaml")))
    RS = yaml.safe_load(open(os.path.join(CFG, "robstride02.yaml")))
    g, jl, mb = P["geometry"], P["joint_limits"], P["mass_budget"]
    perf, el, mech = RS["performance"], RS["electrical"], RS["mechanical"]
    ol, sen, can = RS["operational_limits"], RS["sensing"], RS["can"]
    stand = P["named_poses"]["stand"]

    load = mb["total_kg"] * 9.81 / 4.0
    ext = (g["thigh_length_m"] + g["shank_length_m"])

    facts = {
        # geometry
        "RdThighLength": num(g["thigh_length_m"] * 1000, 2),
        "RdShankLength": num(g["shank_length_m"] * 1000, 2),
        "RdHaaX": num(g["haa_x_m"] * 1000, 1),
        "RdHaaY": num(g["haa_y_m"] * 1000, 1),
        "RdHfeDx": num(g["hfe_dx_m"] * 1000, 1),
        "RdHfeDr": num(g["hfe_dr_m"] * 1000, 1),
        "RdThighLat": num(g["thigh_lateral_m"] * 1000, 1),
        "RdLateral": num(g["hip_to_foot_lateral_m"] * 1000, 1),
        "RdReach": num(ext * 1000, 2),
        "RdFootRadius": num(g["foot_radius_m"] * 1000, 0),
        "RdFootprintL": num(g["nominal_footprint_m"]["length"] * 1000, 0),
        "RdFootprintW": num(g["nominal_footprint_m"]["width"] * 1000, 0),
        "RdBodyL": num(g["body_box_m"][0] * 1000, 0),
        "RdBodyW": num(g["body_box_m"][1] * 1000, 0),
        "RdBodyH": num(g["body_box_m"][2] * 1000, 0),
        # mass
        "RdMassTotal": num(mb["total_kg"], 3),
        "RdMassStruct": num(mb["cad_structure_and_actuators_kg"], 3),
        "RdMassPayload": num(mb["electronics_payload_kg"], 3),
        "RdMassCamera": num(mb["camera_kg"], 3),
        "RdMassActuator": num(mech["mass_kg"] * 1000, 0),
        "RdMassTwelve": num(mech["mass_kg"] * 12, 2),
        "RdMassBase": num(P["links"]["base"]["mass_kg"], 3),
        "RdMassHip": num(P["links"]["FL_hip"]["mass_kg"] * 1000, 1),
        "RdMassThigh": num(P["links"]["FL_thigh"]["mass_kg"] * 1000, 1),
        "RdMassCalf": num(P["links"]["FL_calf"]["mass_kg"] * 1000, 1),
        "RdComX": num(P["links"]["base"]["com_xyz"][0] * 1000, 2),
        # poses
        "RdStandHeight": num(stand["base_height_m"] * 1000, 1),
        "RdStandHfe": num(stand["hfe"], 4),
        "RdStandKfe": num(stand["kfe"], 4),
        "RdZeroHeight": num(P["named_poses"]["zero"]["base_height_m"] * 1000, 1),
        "RdCrouchHeight": num(P["named_poses"]["crouch"]["base_height_m"] * 1000, 1),
        "RdRestHeight": num(P["named_poses"]["rest"]["base_height_m"] * 1000, 1),
        "RdStandExtension": num((stand["base_height_m"] - g["foot_radius_m"]) / ext * 100, 1),
        "RdStaticLoad": num(load, 2),
        # limits
        "RdHaaLo": num(jl["haa"]["lower"], 2), "RdHaaHi": num(jl["haa"]["upper"], 2),
        "RdHfeLo": num(jl["hfe"]["lower"], 2), "RdHfeHi": num(jl["hfe"]["upper"], 2),
        "RdKfeLo": num(jl["kfe"]["lower"], 2), "RdKfeHi": num(jl["kfe"]["upper"], 2),
        "RdHaaDeg": num(math.degrees(jl["haa"]["upper"]), 1),
        # actuator
        "RdTorqueCont": num(ol["continuous_torque_nm"], 1),
        "RdTorquePeak": num(perf["peak_torque_nm"], 1),
        "RdTorqueRated": num(perf["rated_torque_nm"], 1),
        "RdSpeedRated": num(perf["rated_speed_rad_s"], 2),
        "RdSpeedNoLoad": num(perf["no_load_speed_rad_s"], 2),
        "RdSpeedOp": num(ol["velocity_rad_s"], 1),
        "RdGearRatio": num(mech["gear_ratio"], 2),
        "RdKt": num(el["torque_constant_nm_per_arms"], 2),
        "RdRphase": num(el["phase_resistance_ohm"], 2),
        "RdVoltage": num(el["rated_voltage_v"], 0),
        "RdVoltMin": num(el["voltage_min_v"], 0),
        "RdVoltMax": num(el["voltage_max_v"], 0),
        "RdEncoderBits": num(sen["encoder_bits"]),
        "RdEncoderRes": sci(sen["output_resolution_rad"], 3),
        "RdArmature": sci(RS["joint_dynamics"]["armature_kgm2"], 3),
        "RdRotorInertia": sci(mech["rotor_inertia_kgm2"], 1),
        "RdTempWarn": num(ol["temperature_warn_c"], 0),
        "RdTempDerate": num(ol["temperature_derate_c"], 0),
        "RdTempFault": num(ol["temperature_fault_c"], 0),
        "RdRthermal": num(RS["thermal"]["thermal_resistance_k_per_w"], 2),
        "RdCthermal": num(RS["thermal"]["thermal_capacitance_j_per_k"], 0),
        "RdTauThermal": num(RS["thermal"]["time_constant_s"] / 60.0, 1),
        # CAN
        "RdCanRate": num(can["bitrate_bps"] // 1000000),
        "RdCanBuses": num(can["recommended_bus_count"]),
        "RdControlRate": num(can["recommended_control_rate_hz"], 0),
        "RdBitsPerFrame": num(can["bits_per_frame_design"]),
        "RdBusLoad": num(6 * 2 * can["bits_per_frame_design"]
                         * can["recommended_control_rate_hz"] / can["bitrate_bps"] * 100, 0),
        # derived electrical
        "RdRatedArms": num(el["rated_phase_current_arms"], 2),
        "RdCopperLoss": num(3 * el["rated_phase_current_arms"] ** 2
                            * el["phase_resistance_ohm"], 1),
        # provenance
        "RdCadSha": P["meta"]["cad_sha256_16"],
    }

    out = os.path.join(ROOT, "docs", "generated_facts.tex")
    with open(out, "w") as f:
        f.write("% AUTO-GENERATED by tools/make_doc_facts.py -- do not edit.\n")
        f.write(f"% Derived from robot_parameters.yaml (CAD {facts['RdCadSha']})\n")
        f.write("% and robstride02.yaml. Regenerate after any model change.\n\n")
        for k, v in facts.items():
            f.write(f"\\newcommand{{\\{k}}}{{{v}}}\n")
    print(f"wrote docs/generated_facts.tex with {len(facts)} macros")
    for k in ("RdMassTotal", "RdThighLength", "RdStandHeight", "RdBusLoad", "RdCopperLoss"):
        print(f"  \\{k} = {facts[k]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
