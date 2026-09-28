#!/usr/bin/env python3
"""Evaluate RS06 sizing with deployed gains and SafetyMonitor enabled.

Source the ROS workspace, then run:
  MUJOCO_GL=egl python3 tools/torque_report.py --output docs/rs06_feasibility.json
Thermal output is an uncalibrated estimate, not a cooling qualification.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from robodog_sim.evaluation import run_case
from robodog_sim.generate_models import make_model
from robodog_sim.world_spec import World


def load(package, filename):
    path = Path(get_package_share_directory(package)) / "config" / filename
    return yaml.safe_load(path.read_text())


def markdown(report):
    lines = ["# RS06 configuration: simulation feasibility", "",
             f"Working mass: **{report['mass_kg']:.3f} kg**. Knee reduction: motor output turns twice per knee turn; "
             "95% belt efficiency assumed. Hips are direct drive.", "",
             "All runs use the configured 400 Hz controller gains, motor speed/torque limits and SafetyMonitor. "
             "They start at the standing keyframe on an obstacle-free plane. Applied torque is read from MuJoCo qfrc_actuator "
             "at every 0.5 ms physics step, after command limits and transmission gears. Peaks and overload durations include startup; "
             "RMS excludes the stated warmup. Commands are steps from standing, not speed ramps. "
             "The adjacent XML records the evaluated model; the GUI's proving ground has a barrier at 9 m and is unsuitable for long speed runs.", "",
             "| Gait | Command m/s / yaw rad/s | Actual m/s | Max tilt ° | Min height m | Worst motor RMS / peak N·m | Stall-reference % | Safety-clamped cycles | Stable / tracks |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---|"]
    for case in report['cases']:
        if not case['joints']:
            lines.append(f"| {case['gait']} | {case['commanded_vx_m_s']:.2f} | unavailable | — | — | — | — | — | Failed before valid physics samples |")
            continue
        worst = max(case['joints'], key=lambda j: j['motor_rms_nm'])
        peak = max(j['motor_peak_nm'] for j in case['joints'])
        lines.append(f"| {case['gait']} | {case['commanded_vx_m_s']:.2f} / {case.get('commanded_wz_rad_s', 0):.2f} | {case['actual_vx_m_s']:.3f} | "
                     f"{case['max_tilt_deg']:.1f} | {case['min_height_m']:.3f} | {worst['motor_rms_nm']:.2f} / {peak:.2f} | "
                     f"{100 * worst['continuous_utilisation']:.0f} | {case['safety_clamped_cycle_fraction']:.1%} | {case['stable']} / {case['tracks_velocity']} |")
    lines += ["", "## Interpretation", "",
              "This is a flat-ground simulation result, not hardware approval. Continuous utilisation uses the September specification's "
              "8 N·m continuous stall rating. The 11 N·m rating at 100 rpm requires a 200 × 200 mm aluminum heatsink; "
              "the central electronics plate does not establish equivalent cooling for 12 motors. "
              "Output inertia and line resistance use the September specification; phase resistance uses an equivalent-wye conversion. "
              "Thermal impedance, friction and belt efficiency remain assumptions. The I²t limiter is a conservative software policy, "
              "not a reproduction of the manufacturer's overload algorithm. "
              "Temperatures and steady-state extrapolations below cannot verify overheating margins: "
              "the lumped model omits per-phase stall hotspots, resistance changes with temperature and high-current torque nonlinearity. "
              "Belt compliance, pulley strength, motor mounting fit, bus power/regeneration, battery sag and real sensor feedback require validation. "
              "Hardware travel is blocked until live IMU/base-state estimation is integrated. "
              "See [the datasheet audit](rs06_datasheet_audit.md) for ratings, current conventions and conflicting temperature thresholds.", "",
              f"Assumed bus voltage: {report['supply_voltage_v']:.1f} V (two 6S packs in series). "
              "The speed envelope is a voltage-scaled approximation. A 6S parallel arrangement requires a separate evaluation.", ""]
    for case in report['cases']:
        lines += [f"## {case['gait']} {case['commanded_vx_m_s']:.2f} m/s, yaw {case.get('commanded_wz_rad_s', 0):.2f} rad/s", "",
                  f"Duration {case['duration_s']:.1f}s, RMS warmup {case['warmup_s']:.1f}s; "
                  f"travel {case['travel_m']:.3f}m, lateral drift {case['lateral_drift_m']:.3f}m; "
                  f"safety latch {case['safety_latched']}, clamp events {case['safety_clamp_events']}, "
                  f"clamped cycles {case['safety_clamped_cycle_fraction']:.1%}, fault mask {case['fault_flags']}.", "",
                  "| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |",
                  "|---|---:|---:|---:|---:|---:|"]
        if not case['completed'] or not case['stable']:
            lines += ["**Failed operating point:** " + '; '.join(case['stability_failure_reasons'])
                      + ". Loads include the unstable/fallen portion; they do not describe successful locomotion.", ""]
        for j in case['joints']:
            lines.append(f"| {j['name']} | {j['joint_peak_nm']:.2f} / {j['joint_rms_nm']:.2f} | "
                         f"{j['motor_peak_nm']:.2f} / {j['motor_rms_nm']:.2f} | {j['motor_peak_speed_rad_s']:.2f} | "
                         f"{100*j['continuous_utilisation']:.0f} | {j['estimated_final_temperature_c']:.1f} / "
                         f"{j['estimated_steady_temperature_c']:.1f} |")
        lines.append("")
        lines += overload_table(case)
    return '\n'.join(lines)


def overload_table(case):
    lines = ["Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, "
             "not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous "
             "motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.", "",
             "| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |",
             "|---|---:|---:|---:|---:|---:|---:|"]
    for joint in case['joints']:
        thresholds = joint['threshold_metrics']
        above = thresholds['11']
        clipping = joint['clipping']
        durations = [joint['safety_torque_clipped_duration_s']] + [
            clipping[k]['duration_s'] for k in ('peak', 'speed', 'physics')]
        lines.append(f"| {joint['name']} | {joint['motor_peak_nm']:.3f} @ {joint['motor_peak_time_s']:.4f} | "
                     f"{joint['motor_speed_at_peak_rad_s']:.3f} | {above['total_time_s']:.4f} / {above['longest_time_s']:.4f} | "
                     f"{above['peak_abs_motor_speed_rad_s']:.3f} | {thresholds['8']['total_time_s']:.4f} | "
                     + ' / '.join(f'{v:.4f}' for v in durations) + ' |')
    return lines + ['']


def flat_row(case, joint):
    row = dict(gait=case['gait'], commanded_vx_m_s=case['commanded_vx_m_s'],
               commanded_wz_rad_s=case.get('commanded_wz_rad_s', 0),
               **{k: v for k, v in joint.items() if not isinstance(v, dict)})
    for threshold, values in joint['threshold_metrics'].items():
        row.update({f'above_{threshold}nm_{k}': v for k, v in values.items()})
    for kind, values in joint['clipping'].items():
        row.update({f'{kind}_clipping_{k}': v for k, v in values.items()})
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds', type=float, default=8.0)
    ap.add_argument('--warmup', type=float, default=1.0)
    ap.add_argument('--gait', choices=['stand', 'walk', 'trot', 'pace', 'bound'])
    ap.add_argument('--velocity', type=float, default=0.15)
    ap.add_argument('--speeds', type=float, nargs='+', help='evaluate each speed with --gait')
    ap.add_argument('--vy', type=float, default=0.0)
    ap.add_argument('--wz', type=float, default=0.0, help='constant yaw rate, rad/s')
    ap.add_argument('--study', action='store_true', help='20-second-style walk/trot speed sweep through2m/s plus turns; duration still --seconds')
    ap.add_argument('--output', type=Path, default=Path('docs/rs06_feasibility.json'))
    args = ap.parse_args()
    if args.speeds and not args.gait:
        ap.error('--speeds requires --gait')
    if not all(math.isfinite(v) for v in [args.velocity, args.vy, args.wz, *(args.speeds or [])]):
        ap.error('velocity commands must be finite')
    params = load('robodog_description', 'robot_parameters.yaml')
    actuator = load('robodog_description', params['actuator_config'])
    gaits = load('robodog_control', 'gaits.yaml')['gaits']
    controls = load('robodog_control', 'control.yaml')['/**']['ros__parameters']
    # A long fast run in the GUI's course reaches the9m terminal barrier and
    # measures pushing against it, not gait speed. Use the same robot/dynamics
    # on a dedicated plane, preserving the exact generated XML with the report.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    model = args.output.with_suffix('.xml')
    meshdir = str(Path(get_package_share_directory('robodog_description')) / 'meshes')
    model.write_text(make_model(params, actuator, World('sizing', size=(60, 60), centred=True),
                                timestep=0.0005, meshdir=meshdir))
    cases = [(args.gait, 0.0 if args.gait == 'stand' else args.velocity)] if args.gait else [
        ('stand', 0.0), ('walk', 0.15), ('trot', 0.30), ('trot', 0.50)]
    if args.speeds:
        cases = [(args.gait, speed) for speed in args.speeds]
    cases = [(gait, vx, args.vy, args.wz) for gait, vx in cases]
    if args.study:
        cases = [('stand', 0., 0., 0.), ('walk', .15, 0., 0.)] + [
            (gait, speed, 0., 0.) for gait in ('walk', 'trot') for speed in (.4, .5, .75, 1., 1.5, 2.)
        ] + [('trot', .4, 0., .3), ('trot', .5, 0., .3)]
    report = dict(mass_kg=params['mass_budget']['total_kg'], actuator=actuator['model'],
                  supply_voltage_v=actuator['electrical'].get('supply_voltage_v', 48.0),
                  cad_sha256_16=params['meta']['cad_sha256_16'],
                  world='obstacle-free sizing plane',
                  model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                  configuration=dict(actuator=actuator, gaits=gaits, controls=controls), cases=[])
    for gait, vx, vy, wz in cases:
        print(f'Running {gait} {vx:.2f} m/s, yaw {wz:.2f} for {args.seconds}s with safety enabled...', flush=True)
        result = run_case(params, actuator, gaits, controls, model, gait=gait, vx=vx, vy=vy, wz=wz,
                          seconds=args.seconds, warmup=args.warmup)
        report['cases'].append(result)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        if not result['joints']:
            print(f"  failed without valid physics samples: {result['failure_reason']}", flush=True)
            continue
        worst = max(result['joints'], key=lambda j: j['motor_rms_nm'])
        print(f"  actual {result['actual_vx_m_s']:.3f} m/s, tilt {result['max_tilt_deg']:.1f}°, "
              f"motor RMS {worst['motor_rms_nm']:.2f} N·m ({worst['continuous_utilisation']:.0%}), "
              f"latch {result['safety_latched']}", flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    args.output.with_suffix('.md').write_text(markdown(report))
    rows = [flat_row(c, j)
            for c in report['cases'] for j in c['joints']]
    with args.output.with_suffix('.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(f'Report: {args.output.with_suffix(".md")}')


if __name__ == '__main__':
    main()
