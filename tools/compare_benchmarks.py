#!/usr/bin/env python3
"""Compare recorded applied-torque studies; never treat a peak cap as continuous."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def straight_case(report, gait, speed):
    matches = [c for c in report['cases'] if (c['gait'] if isinstance(c.get('gait'), str) else 'trot') == gait
               and abs(c['commanded_vx_m_s'] - speed) < 1e-9
               and c.get('commanded_wz_rad_s', 0) == 0]
    if len(matches) != 1:
        raise ValueError(f'Expected one {gait} {speed} case, got {len(matches)}')
    return matches[0]


def group(case, kind, reference):
    rows = [j for j in case['joints'] if j['name'].split('_')[1] == kind]
    return max(j[f'{reference}_rms_nm'] for j in rows), max(j[f'{reference}_peak_nm'] for j in rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robodog', type=Path, default=Path('docs/rs06_speed_study.json'))
    parser.add_argument('--go2', type=Path, default=Path('docs/go2_benchmark.json'))
    parser.add_argument('--output', type=Path, default=Path('docs/locomotion_validation.md'))
    args = parser.parse_args()
    own, go2 = json.loads(args.robodog.read_text()), json.loads(args.go2.read_text())
    figdir = args.output.parent / 'images'
    figdir.mkdir(parents=True, exist_ok=True)
    lines = ['# Locomotion and applied-torque validation', '',
             'The normal launch dependency and command-ordering paths were repaired. The experiment uses the current '
             '19.719 kg model, 12 RS06 actuators and 2:1 external knee reduction with assumed 95% efficiency. '
             'GUI velocity range reaches 2 m/s in simulation; accepting a command does not establish that a gait can execute it.', '',
             'The normal ROS viewer launch also passed a direct `/cmd_vel` smoke test: a 0.2 m/s command moved the robot '
             '0.616 m in 3 simulated seconds, with finite applied-torque telemetry on all 12 joints and an acknowledged return to standing. '
             'The browser walking test advanced 0.806 m, verified all 12 torque/current/temperature rows and the simulation slider range. '
             '[Live test evidence](live_launch_validation.json) records both checks.', '',
             '**Remaining lifecycle issue:** an interrupt-driven viewer shutdown produced teardown errors, including a controller '
             'SIGSEGV. The official Unitree viewer also crashed during shutdown. Their causes remain unresolved; the successful '
             'walking checks do not establish clean viewer shutdown.', '',
             '## Speed sweep', '',
             'Every case starts from the standing keyframe with a velocity step, on an obstacle-free plane. '
             'Each run lasts 20 simulated seconds; RMS excludes the first second, while peaks/overload durations include it. '
             'Physics runs at 2 kHz, control at 400 Hz, command delay 1 ms. Robot safety remains active. '
             'A successful speed must stay upright, track its command, and be assessed together with clipping—not RMS alone.', '',
             '| Gait | Command / actual m/s | Worst motor RMS / peak Nm | Safety-clamped cycles | Stable / tracks |',
             '|---|---:|---:|---:|---|']
    for case in own['cases']:
        if case.get('commanded_wz_rad_s', 0):
            continue
        rms = max(j['motor_rms_nm'] for j in case['joints'])
        peak = max(j['motor_peak_nm'] for j in case['joints'])
        lines.append(f"| {case['gait']} | {case['commanded_vx_m_s']:.2f} / {case['actual_vx_m_s']:.3f} | "
                     f"{rms:.2f} / {peak:.2f} | {case['safety_clamped_cycle_fraction']:.1%} | "
                     f"{case['stable']} / {case['tracks_velocity']} |")
    lines += ['', 'Failed/fallen runs include loads after instability and are not valid operating points. '
              'Frequent safety limiting also prevents calling a tracking run comfortable. '
              'The separate speed-envelope reduction is an approximate motor model, not a measured RS06 torque-speed curve.', '',
              '![Speed tracking](images/speed_tracking.png)', '', '## Turning load tests', '',
              '| Forward command / actual m/s | Yaw command / actual rad/s | Worst motor RMS / peak Nm | Stable / tracks both |',
              '|---|---|---|---|']
    for case in own['cases']:
        if not case.get('commanded_wz_rad_s', 0):
            continue
        lines.append(f"| {case['commanded_vx_m_s']:.2f} / {case['actual_vx_m_s']:.3f} | "
                     f"{case['commanded_wz_rad_s']:.2f} / {case['actual_wz_rad_s']:.3f} | "
                     f"{max(j['motor_rms_nm'] for j in case['joints']):.2f} / "
                     f"{max(j['motor_peak_nm'] for j in case['joints']):.2f} | {case['stable']} / {case['tracks_velocity']} |")
    lines += ['', 'Actual forward/lateral speed is measured in the body yaw frame. These runs expose yaw tracking error; '
              'they must not be represented as successful execution of the requested turn rate.', '',
              '## Official Go2 model comparison', '',
              'Both models use MuJoCo 3.13.0, the same plane/contact parameters, gravity, solver, timestep, '
              '20-second duration, 1-second RMS warmup, 1 ms delay, 2 mm initial clearance and shared trot generator/gains. '
              'Masses differ: Robodog 19.719 kg versus Go2 model 15.206408 kg. Go2 retains its official inertias and actuator caps; '
              'Robodog retains its RS06 speed envelope and safety limiter. This is a shared-controller model comparison, '
              '**not Unitree factory-controller performance**. No external force drives either floating base.', '',
              'The primary comparison is **joint-output torque**. The extra Robodog motor column is before the knee belt; '
              'its knee value is therefore smaller than joint torque by 1.9. Each cell is worst-leg RMS / worst-leg peak, '
              'which can belong to different legs.', '',
              '| Speed | Joint group | Robodog joint RMS / peak Nm | Robodog motor RMS / peak Nm | Go2 joint RMS / peak Nm |',
              '|---:|---|---:|---:|---:|']
    labels = {'haa': 'Hip roll', 'hfe': 'Hip pitch', 'kfe': 'Knee'}
    for speed in (.4, .5):
        a, b = straight_case(own, 'trot', speed), straight_case(go2, 'trot', speed)
        for kind, label in labels.items():
            values = [group(a, kind, 'joint'), group(a, kind, 'motor'), group(b, kind, 'joint')]
            lines.append(f'| {speed:.1f} | {label} | ' + ' | '.join(f'{r:.2f} / {p:.2f}' for r, p in values) + ' |')
    lines += ['', 'Go2 model limits are 23.7 Nm for hip roll/pitch and 45.43 Nm for knees. They are peak model caps, '
              'not continuous thermal ratings. [Pinned source audit](go2_benchmark_sources.md) and '
              '[full Go2 results](go2_benchmark.md) give provenance and all 12 joints.', '',
              '![Joint torque comparison](images/go2_torque_comparison.png)', '',
              '## How physical parameters and torque are calculated', '',
              '1. **Geometry/mass:** joint transforms, link masses, COMs and inertia tensors come from the supplied CAD URDF. '
              'Twelve motor placeholders are replaced by 0.621 kg RS06 motors; battery/electronics/belt additions give 19.719 kg. '
              'Known placeholders are replaced rather than counted twice.',
              '2. **Inertia:** rotate each CAD inertia into its link frame and add it about the combined COM using '
              '`I = Σ(R I_part Rᵀ + m[(r·r)I₃ − rrᵀ])`. Motor housing inertia uses an estimated uniform 88×49 mm cylinder; '
              'payload additions use declared point-mass locations. RS06 output-equivalent armature is 0.012 kg·m²; '
              'the 2:1 knee reflects this to 0.048 kg·m². Those are distinct from housing/link rigid-body inertia.',
              '3. **Requested joint torque:** `τ_req = Kp(q_target−q) + Kd(q̇_target−q̇) + JᵀF`. '
              'The balance controller distributes support/acceleration forces across stance feet; nominal static support is '
              '`mg/4 ≈ 48.4 N` per foot. It then applies joint limits, the overload/temperature gate and the motor envelope.',
              '4. **Reported torque:** read MuJoCo `qfrc_actuator` at each hinge DOF every 0.5 ms. This is applied actuator '
              'torque after clamping/gearing—not the PD request, and not the total impact/reaction torque on bearings or structures. '
              'For the knee, `τ_motor = τ_joint/(2×0.95)` and `ω_motor = 2ω_joint`; hips use ratio 1.',
              '5. **Peak/RMS/overload:** `peak=max(|τ_motor|)`; `RMS=√(Στ_motor²Δt/ΣΔt)`. '
              'For every joint, record cumulative and longest consecutive time above 8 and 11 Nm, speed at peak, '
              'speed during overload, and separate software/peak/speed-envelope/MuJoCo clipping. '
              'The threshold durations include startup and are sampled at the physics rate.', '',
              'Temperatures remain uncalibrated model estimates. These results do not establish cooling, belt strength, '
              'motor mounting, repeated-impact life or battery/regeneration capability.', '',
              '## Reproduce', '', '```bash',
              'source /opt/ros/humble/setup.bash', 'source ros2_ws/install/setup.bash',
              'python3 tools/torque_report.py --study --seconds 20 --output docs/rs06_speed_study.json',
              'python3 tools/go2_benchmark.py --repo /tmp/robodog-unitree-mujoco --seconds 20 --output docs/go2_benchmark.json',
              'python3 tools/compare_benchmarks.py', '```', '',
              'The Go2 repository checkout must match the pinned revision in its report. '
              '[All RS06 joint measurements](rs06_speed_study.md), [machine-readable JSON](rs06_speed_study.json) and '
              '[flat CSV](rs06_speed_study.csv) retain the detailed measurements. '
              'The [official viewer/SDK smoke result](go2_runner_smoke.md) is separate from the matched-model benchmark.']
    args.output.write_text('\n'.join(lines) + '\n')

    fig, ax = plt.subplots(figsize=(7.4, 4.2), layout='constrained')
    ax.plot([0, 2], [0, 2], '--', color='0.6', label='Command = actual')
    for gait, color in [('walk', '#2676b8'), ('trot', '#d47623')]:
        cases = [c for c in own['cases'] if c['gait'] == gait and not c.get('commanded_wz_rad_s', 0)]
        ax.plot([c['commanded_vx_m_s'] for c in cases], [c['actual_vx_m_s'] for c in cases],
                color=color, alpha=.55, label=gait)
        for c in cases:
            ax.scatter(c['commanded_vx_m_s'], c['actual_vx_m_s'], color=color,
                       marker='o' if c['stable'] else 'x', s=55)
    ax.set(xlabel='Commanded forward speed (m/s)', ylabel='Actual body-forward speed (m/s)',
           title='20-second speed sweep; × marks instability')
    ax.grid(alpha=.2); ax.legend()
    fig.savefig(figdir / 'speed_tracking.png', dpi=180)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2), sharey=True, layout='constrained')
    x = np.arange(3)
    for ax, speed in zip(axes, (.4, .5)):
        a, b = straight_case(own, 'trot', speed), straight_case(go2, 'trot', speed)
        for offset, case, color, label in [(-.18, a, '#2676b8', 'Robodog'), (.18, b, '#d47623', 'Go2 model')]:
            values = [group(case, kind, 'joint') for kind in labels]
            ax.bar(x+offset, [v[0] for v in values], width=.32, color=color, label=label+' RMS')
            ax.scatter(x+offset, [v[1] for v in values], color=color, marker='_', s=190, label=label+' peak')
        ax.set_xticks(x, labels.values()); ax.set_title(f'Trot command {speed:.1f} m/s')
        ax.grid(axis='y', alpha=.2)
    axes[0].set_ylabel('Joint-output torque (Nm), worst leg')
    axes[1].legend(fontsize=8)
    fig.savefig(figdir / 'go2_torque_comparison.png', dpi=180)
    plt.close(fig)
    print(args.output)


if __name__ == '__main__':
    main()
