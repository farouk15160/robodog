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
import shlex
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from robodog_sim.evaluation import run_case
from robodog_sim.generate_models import make_model
from robodog_sim.world_spec import World
from robodog_hardware.types import Fault


def load(package, filename):
    path = Path(get_package_share_directory(package)) / "config" / filename
    return yaml.safe_load(path.read_text())


def fault_names(mask):
    """Decode the shared safety bitmask for human-readable reports."""
    return [flag.name for flag in Fault if flag is not Fault.NONE and mask & int(flag)]


def markdown(report):
    label = report.get('study_label')
    title = "RS06 configuration: simulation feasibility"
    if label:
        title += f" — {label.replace('_', ' ')}"
    lines = [f"# {title}", "",
             f"Working mass: **{report['mass_kg']:.3f} kg**. Knee reduction: motor output turns twice per knee turn; "
             "95% belt efficiency assumed. Hips are direct drive.", "",
             "All runs use the configured 400 Hz controller gains, motor speed/torque limits and SafetyMonitor. "
             "They start at the standing keyframe on an obstacle-free plane. Applied torque is read from MuJoCo qfrc_actuator "
             "at every 0.5 ms physics step, after command limits and transmission gears. Peaks and overload durations include startup; "
             "RMS excludes the stated warmup. Commands are steps from standing, not speed ramps. "
             "The adjacent XML records the evaluated model; the GUI's proving ground has a barrier at 9 m and is unsuitable for long speed runs.", "",
             "| Gait | Command m/s / yaw rad/s | Actual m/s | Max tilt ° | Min height m | Worst motor RMS / peak N·m | Stall-reference % | Safety-clamped cycles | Stable / tracks / margin pass |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---|"]
    for case in report['cases']:
        if not case['joints']:
            lines.append(f"| {case['gait']} | {case['commanded_vx_m_s']:.2f} | unavailable | — | — | — | — | — | Failed before valid physics samples |")
            continue
        worst = max(case['joints'], key=lambda j: j['motor_rms_nm'])
        peak = max(j['motor_peak_nm'] for j in case['joints'])
        lines.append(f"| {case['gait']} | {case['commanded_vx_m_s']:.2f} / {case.get('commanded_wz_rad_s', 0):.2f} | {case['actual_vx_m_s']:.3f} | "
                     f"{case['max_tilt_deg']:.1f} | {case['min_height_m']:.3f} | {worst['motor_rms_nm']:.2f} / {peak:.2f} | "
                     f"{100 * worst['continuous_utilisation']:.0f} | {case['safety_clamped_cycle_fraction']:.1%} | "
                     f"{case['stable']} / {case['tracks_velocity']} / {case.get('operating_point_pass', False)} |")
    failed = [case for case in report['cases'] if not case['stable'] or not case['tracks_velocity']]
    limited = [case for case in report['cases'] if case.get('safety_clamped_cycle_fraction', 0) > 0]
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
    if failed:
        labels = ', '.join(f"{case['gait']} {case['commanded_vx_m_s']:.2f} m/s" for case in failed)
        lines += [f"**Rejected operating point(s):** {labels}. A completed simulation is not a successful gait when it falls or misses the command.", ""]
    if limited:
        labels = ', '.join(f"{case['gait']} {case['safety_clamped_cycle_fraction']:.1%}" for case in limited)
        lines += [f"**Active limiting:** safety changed commands during {labels} of control cycles. Treat RMS together with peak, overload and clipping data.", ""]
    margin_failed = [case for case in report['cases'] if not case.get('operating_point_pass', False)]
    if margin_failed:
        labels = '; '.join(
            f"{case['gait']} ({', '.join(case.get('margin_failure_reasons', ['margin not assessed']))})"
            for case in margin_failed)
        lines += [f"**Motor-margin verdict: FAIL.** {labels}. Stable motion and speed tracking alone do not establish comfortable actuator margin.", ""]
    for case in report['cases']:
        if case.get('lateral_drift_m', 0) > 1:
            lines += [f"**World-path drift:** {case['gait']} accumulated {case['lateral_drift_m']:.3f} m maximum lateral displacement. "
                      "Its body-frame speed tracking result does not establish straight-line navigation accuracy.", ""]
    if report.get('reproduction_command'):
        lines += ["## Reproduce", "", f"`{report['reproduction_command']}`", ""]
    baseline = report.get('baseline_reference')
    if baseline:
        lines += ["## Baseline comparison", "",
                  f"Baseline: [{baseline['path']}]({Path(baseline['path']).name}) "
                  f"(SHA-256 `{baseline['sha256']}`).", ""]
        for case in report['cases']:
            old = next((item for item in baseline.get('cases', [])
                        if item['gait'] == case['gait']
                        and abs(item['commanded_vx_m_s'] - case['commanded_vx_m_s']) < 1e-12
                        and abs(item.get('commanded_wz_rad_s', 0) - case.get('commanded_wz_rad_s', 0)) < 1e-12), None)
            if old is None or not case['joints']:
                continue
            new = case_summary(case)
            lines += [f"### {case['gait']} {case['commanded_vx_m_s']:.2f} m/s", "",
                      "| Metric | Baseline | Current | Change |",
                      "|---|---:|---:|---:|"]
            for label, key in (("Actual body speed m/s", "actual_vx_m_s"),
                               ("Lateral drift m", "lateral_drift_m"),
                               ("Safety-clamped cycle fraction", "safety_clamped_cycle_fraction"),
                               ("Worst motor RMS Nm", "worst_motor_rms_nm"),
                               ("Peak motor Nm", "peak_motor_nm")):
                before, after = old[key], new[key]
                lines.append(f"| {label} | {before:.3f} | {after:.3f} | {after-before:+.3f} |")
            lines.append("")
    for case in report['cases']:
        faults = fault_names(case['fault_flags'])
        fault_summary = ', '.join(faults) if faults else 'none'
        lines += [f"## {case['gait']} {case['commanded_vx_m_s']:.2f} m/s, yaw {case.get('commanded_wz_rad_s', 0):.2f} rad/s", "",
                  f"Duration {case['duration_s']:.1f}s, RMS warmup {case['warmup_s']:.1f}s; "
                  f"travel {case['travel_m']:.3f}m, lateral drift {case['lateral_drift_m']:.3f}m; "
                  f"safety latch {case['safety_latched']}, clamp events {case['safety_clamp_events']}, "
                  f"clamped cycles {case['safety_clamped_cycle_fraction']:.1%}, fault mask {case['fault_flags']} "
                  f"({fault_summary}).", "",
                  "| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |",
                  "|---|---:|---:|---:|---:|---:|"]
        for j in case['joints']:
            lines.append(f"| {j['name']} | {j['joint_peak_nm']:.2f} / {j['joint_rms_nm']:.2f} | "
                         f"{j['motor_peak_nm']:.2f} / {j['motor_rms_nm']:.2f} | {j['motor_peak_speed_rad_s']:.2f} | "
                         f"{100*j['continuous_utilisation']:.0f} | {j['estimated_final_temperature_c']:.1f} / "
                         f"{j['estimated_steady_temperature_c']:.1f} |")
        lines.append("")
        if not case['completed'] or not case['stable']:
            lines += ["**Failed operating point:** " + '; '.join(case['stability_failure_reasons'])
                      + ". Loads include the unstable/fallen portion; they do not describe successful locomotion.", ""]
        lines += overload_table(case)
    return '\n'.join(lines)


def overload_table(case):
    lines = ["Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, "
             "not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous "
             "motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.", "",
             "| Joint | Peak Nm @ time / travel | World xyz at peak m | Speed at peak rad/s | >8 / >11 / >36 Nm total s | >11 Nm longest s | Max speed while >11 rad/s | Safety / peak / speed-envelope / physics clipping s |",
             "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for joint in case['joints']:
        thresholds = joint['threshold_metrics']
        above = thresholds['11']
        clipping = joint['clipping']
        durations = [joint['safety_torque_clipped_duration_s']] + [
            clipping[k]['duration_s'] for k in ('peak', 'speed', 'physics')]
        position = joint.get('motor_peak_world_position_m')
        location = ('unavailable' if position is None else
                    f"[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
        travel = joint.get('motor_peak_travel_m')
        travelled = 'unavailable' if travel is None else f'{travel:.3f} m'
        threshold_times = [thresholds.get(str(value), {}).get('total_time_s', 0.)
                           for value in (8, 11, 36)]
        lines.append(f"| {joint['name']} | {joint['motor_peak_nm']:.3f} @ {joint['motor_peak_time_s']:.4f} s / {travelled} | "
                     f"{location} | {joint['motor_speed_at_peak_rad_s']:.3f} | "
                     + ' / '.join(f'{value:.4f}' for value in threshold_times) + f" | {above['longest_time_s']:.4f} | "
                     f"{above['peak_abs_motor_speed_rad_s']:.3f} | "
                     + ' / '.join(f'{v:.4f}' for v in durations) + ' |')
    return lines + ['']


def flat_row(case, joint):
    row = dict(gait=case['gait'], commanded_vx_m_s=case['commanded_vx_m_s'],
               commanded_wz_rad_s=case.get('commanded_wz_rad_s', 0),
               operating_point_pass=case.get('operating_point_pass', False),
               margin_failure_reasons='; '.join(case.get('margin_failure_reasons', [])),
               **{k: v for k, v in joint.items() if not isinstance(v, (dict, list, tuple))})
    for threshold, values in joint['threshold_metrics'].items():
        row.update({f'above_{threshold}nm_{k}': v for k, v in values.items()})
    for kind, values in joint['clipping'].items():
        row.update({f'{kind}_clipping_{k}': v for k, v in values.items()})
    position = joint.get('motor_peak_world_position_m')
    for index, axis in enumerate('xyz'):
        row[f'motor_peak_world_{axis}_m'] = None if position is None else position[index]
    return row


def requested_cases(args):
    """Return the explicit gait/velocity cases requested by the CLI."""
    if args.study:
        return [('stand', 0., 0., 0.), ('walk', .15, 0., 0.)] + [
            (gait, speed, 0., 0.) for gait in ('walk', 'trot') for speed in (.4, .5, .75, 1., 1.5, 2.)
        ] + [('trot', .4, 0., .3), ('trot', .5, 0., .3)]
    if args.speeds:
        return [(args.gait, speed, args.vy, args.wz) for speed in args.speeds]
    if args.gaits:
        return [(gait, 0.0 if gait == 'stand' else args.velocity, args.vy, args.wz)
                for gait in args.gaits]
    if args.gait:
        return [(args.gait, 0.0 if args.gait == 'stand' else args.velocity, args.vy, args.wz)]
    return [(gait, vx, args.vy, args.wz) for gait, vx in (
        ('stand', 0.0), ('walk', 0.15), ('trot', 0.30), ('trot', 0.50))]


def case_summary(case):
    """Compact, stable metrics used to compare a regenerated study."""
    return {
        "gait": case["gait"],
        "commanded_vx_m_s": case["commanded_vx_m_s"],
        "commanded_wz_rad_s": case.get("commanded_wz_rad_s", 0.0),
        "actual_vx_m_s": case["actual_vx_m_s"],
        "lateral_drift_m": case["lateral_drift_m"],
        "safety_clamped_cycle_fraction": case["safety_clamped_cycle_fraction"],
        "worst_motor_rms_nm": max((row["motor_rms_nm"] for row in case["joints"]), default=0.0),
        "peak_motor_nm": max((row["motor_peak_nm"] for row in case["joints"]), default=0.0),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds', type=float, default=8.0)
    ap.add_argument('--warmup', type=float, default=1.0)
    ap.add_argument('--gait', choices=['stand', 'walk', 'trot', 'pace', 'bound'])
    ap.add_argument('--gaits', choices=['stand', 'walk', 'trot', 'pace', 'bound'], nargs='+',
                    help='evaluate several gaits at --velocity in one report')
    ap.add_argument('--velocity', type=float, default=0.15)
    ap.add_argument('--speeds', type=float, nargs='+', help='evaluate each speed with --gait')
    ap.add_argument('--vy', type=float, default=0.0)
    ap.add_argument('--wz', type=float, default=0.0, help='constant yaw rate, rad/s')
    ap.add_argument('--study', action='store_true', help='20-second-style walk/trot speed sweep through2m/s plus turns; duration still --seconds')
    ap.add_argument('--study-label', default='',
                    help='short identifier embedded in JSON and shown in Markdown')
    ap.add_argument('--baseline-report', type=Path,
                    help='prior JSON report to hash and compare against')
    ap.add_argument('--output', type=Path, default=Path('docs/rs06_feasibility.json'))
    args = ap.parse_args()
    if args.speeds and not args.gait:
        ap.error('--speeds requires --gait')
    if args.gaits and (args.gait or args.speeds or args.study):
        ap.error('--gaits cannot be combined with --gait, --speeds or --study')
    if args.study and (args.gait or args.speeds):
        ap.error('--study cannot be combined with --gait or --speeds')
    if not all(math.isfinite(v) for v in [args.velocity, args.vy, args.wz, *(args.speeds or [])]):
        ap.error('velocity commands must be finite')
    params = load('robodog_description', 'robot_parameters.yaml')
    actuator = load('robodog_description', params['actuator_config'])
    gaits = load('robodog_control', 'gaits.yaml')['gaits']
    controls = load('robodog_control', 'control.yaml')['/**']['ros__parameters']
    baseline_reference = None
    if args.baseline_report:
        baseline_bytes = args.baseline_report.read_bytes()
        baseline_report = json.loads(baseline_bytes)
        baseline_reference = {
            "path": str(args.baseline_report),
            "sha256": hashlib.sha256(baseline_bytes).hexdigest(),
            "study_label": baseline_report.get("study_label", ""),
            "cases": [case_summary(case) for case in baseline_report.get("cases", [])],
        }
    # A long fast run in the GUI's course reaches the9m terminal barrier and
    # measures pushing against it, not gait speed. Use the same robot/dynamics
    # on a dedicated plane, preserving the exact generated XML with the report.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    model = args.output.with_suffix('.xml')
    meshdir = str(Path(get_package_share_directory('robodog_description')) / 'meshes')
    model.write_text(make_model(params, actuator, World('sizing', size=(60, 60), centred=True),
                                timestep=0.0005, meshdir=meshdir))
    cases = requested_cases(args)
    report = dict(study_label=args.study_label, mass_kg=params['mass_budget']['total_kg'], actuator=actuator['model'],
                  supply_voltage_v=actuator['electrical'].get('supply_voltage_v', 48.0),
                  cad_sha256_16=params['meta']['cad_sha256_16'],
                  world='obstacle-free sizing plane',
                  model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                  reproduction_command=shlex.join([sys.executable, *sys.argv]),
                  measurement_protocol=dict(
                      controller_rate_hz=float(controls['control_rate_hz']),
                      physics_timestep_s=0.0005,
                      physics_sample_rate_hz=2000.0,
                      rms_warmup_s=args.warmup,
                      peak_and_exposure_window='full run including startup',
                      rms_window='after warmup through end of run',
                      knee_external_reduction_ratio=actuator['transmissions']['kfe']['ratio'],
                      knee_belt_efficiency_assumed=actuator['transmissions']['kfe']['efficiency'],
                  ),
                  baseline_reference=baseline_reference,
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
