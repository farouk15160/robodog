# RS06 configuration: simulation feasibility

> Historical RS06 run: superseded by [the current speed sweep](rs06_speed_study.md) and [Go2 comparison](locomotion_validation.md). The newer results sample applied torque at every physics step and correct the command delay to 1 ms.

Working mass: **19.719 kg**. Knee reduction: motor output turns twice per knee turn; 95% belt efficiency assumed. Hips are direct drive.

All runs use the configured 400 Hz controller gains, motor speed/torque limits and SafetyMonitor. They start at the standing keyframe on an obstacle-free plane. Peaks include startup; RMS excludes the stated warmup. The adjacent XML records the evaluated model; the GUI's proving ground has a barrier at 9 m and is unsuitable for long speed runs.

| Gait | Command m/s | Actual m/s | Max tilt ° | Min height m | Worst motor RMS N·m | Continuous % | Stable / tracks |
|---|---:|---:|---:|---:|---:|---:|---|
| stand | 0.00 | -0.000 | 0.1 | 0.320 | 3.40 | 43 | True / True |
| walk | 0.15 | 0.172 | 4.5 | 0.320 | 4.87 | 61 | True / True |
| trot | 0.30 | 0.314 | 3.1 | 0.321 | 4.63 | 58 | True / True |
| trot | 0.50 | 0.516 | 4.4 | 0.321 | 4.74 | 59 | True / True |

## Interpretation

This is a flat-ground simulation result, not hardware approval. Continuous utilisation uses the September specification's 8 N·m continuous stall rating. The 11 N·m rating at 100 rpm requires a 200 × 200 mm aluminum heatsink; the central electronics plate does not establish equivalent cooling for 12 motors. Output inertia and line resistance use the September specification; phase resistance uses an equivalent-wye conversion. Thermal impedance, friction and belt efficiency remain assumptions. The I²t limiter is a conservative software policy, not a reproduction of the manufacturer's overload algorithm. Temperatures and steady-state extrapolations below cannot verify overheating margins: the lumped model omits per-phase stall hotspots, resistance changes with temperature and high-current torque nonlinearity. Belt compliance, pulley strength, motor mounting fit, bus power/regeneration, battery sag and real sensor feedback require validation. Hardware travel is blocked until live IMU/base-state estimation is integrated. See [the datasheet audit](rs06_datasheet_audit.md) for ratings, current conventions and conflicting temperature thresholds.

Assumed bus voltage: 44.4 V (two 6S packs in series). The speed envelope is a voltage-scaled approximation. A 6S parallel arrangement requires a separate evaluation.

## stand 0.00 m/s

Duration 20.0s, RMS warmup 1.0s; travel -0.003m, lateral drift 0.000m; safety latch False, clamp events 0, clamped cycles 0.0%, fault mask 0.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 3.80 / 3.34 | 3.80 / 3.34 | 0.89 | 42 | 20.2 / 24.2 |
| FL_hfe_joint | 0.59 / 0.51 | 0.59 / 0.51 | 0.23 | 6 | 20.0 / 20.1 |
| FL_kfe_joint | 6.96 / 5.73 | 3.66 / 3.02 | 1.79 | 38 | 20.2 / 23.4 |
| FR_haa_joint | 3.74 / 3.31 | 3.74 / 3.31 | 0.89 | 41 | 20.2 / 24.1 |
| FR_hfe_joint | 0.60 / 0.53 | 0.60 / 0.53 | 0.22 | 7 | 20.0 / 20.1 |
| FR_kfe_joint | 6.93 / 5.68 | 3.65 / 2.99 | 1.79 | 37 | 20.2 / 23.3 |
| RL_haa_joint | 3.79 / 3.40 | 3.79 / 3.40 | 0.89 | 43 | 20.2 / 24.3 |
| RL_hfe_joint | 0.52 / 0.37 | 0.52 / 0.37 | 0.21 | 5 | 20.0 / 20.1 |
| RL_kfe_joint | 6.88 / 6.07 | 3.62 / 3.20 | 1.79 | 40 | 20.2 / 23.8 |
| RR_haa_joint | 3.73 / 3.36 | 3.73 / 3.36 | 0.89 | 42 | 20.2 / 24.2 |
| RR_hfe_joint | 0.53 / 0.39 | 0.53 / 0.39 | 0.20 | 5 | 20.0 / 20.1 |
| RR_kfe_joint | 6.84 / 6.02 | 3.60 / 3.17 | 1.79 | 40 | 20.2 / 23.8 |

## walk 0.15 m/s

Duration 20.0s, RMS warmup 1.0s; travel 3.439m, lateral drift 0.025m; safety latch False, clamp events 0, clamped cycles 0.0%, fault mask 0.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 10.19 / 4.21 | 10.19 / 4.21 | 2.23 | 53 | 20.3 / 26.6 |
| FL_hfe_joint | 7.74 / 2.22 | 7.74 / 2.22 | 6.62 | 28 | 20.1 / 21.8 |
| FL_kfe_joint | 14.78 / 6.72 | 7.78 / 3.54 | 12.08 | 44 | 20.2 / 24.7 |
| FR_haa_joint | 9.79 / 4.23 | 9.79 / 4.23 | 2.18 | 53 | 20.3 / 26.7 |
| FR_hfe_joint | 7.33 / 2.27 | 7.33 / 2.27 | 5.67 | 28 | 20.1 / 21.9 |
| FR_kfe_joint | 15.44 / 6.82 | 8.13 / 3.59 | 12.20 | 45 | 20.2 / 24.8 |
| RL_haa_joint | 10.33 / 4.87 | 10.33 / 4.87 | 1.67 | 61 | 20.4 / 28.9 |
| RL_hfe_joint | 8.42 / 4.56 | 8.42 / 4.56 | 6.42 | 57 | 20.4 / 27.8 |
| RL_kfe_joint | 18.05 / 9.18 | 9.50 / 4.83 | 13.94 | 60 | 20.4 / 28.7 |
| RR_haa_joint | 10.96 / 4.72 | 10.96 / 4.72 | 1.74 | 59 | 20.4 / 28.3 |
| RR_hfe_joint | 8.56 / 4.51 | 8.56 / 4.51 | 6.11 | 56 | 20.4 / 27.6 |
| RR_kfe_joint | 16.22 / 9.03 | 8.54 / 4.75 | 13.79 | 59 | 20.4 / 28.4 |

## trot 0.30 m/s

Duration 20.0s, RMS warmup 1.0s; travel 6.274m, lateral drift 0.044m; safety latch False, clamp events 0, clamped cycles 0.0%, fault mask 0.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 8.32 / 4.17 | 8.32 / 4.17 | 1.78 | 52 | 20.3 / 26.5 |
| FL_hfe_joint | 6.91 / 2.25 | 6.91 / 2.25 | 4.16 | 28 | 20.1 / 21.9 |
| FL_kfe_joint | 22.18 / 7.80 | 11.67 / 4.10 | 10.45 | 51 | 20.3 / 26.3 |
| FR_haa_joint | 7.38 / 4.04 | 7.38 / 4.04 | 1.33 | 50 | 20.3 / 26.1 |
| FR_hfe_joint | 6.88 / 2.28 | 6.88 / 2.28 | 3.79 | 28 | 20.1 / 21.9 |
| FR_kfe_joint | 12.13 / 7.80 | 6.38 / 4.11 | 10.45 | 51 | 20.3 / 26.3 |
| RL_haa_joint | 6.71 / 3.54 | 6.71 / 3.54 | 1.48 | 44 | 20.2 / 24.7 |
| RL_hfe_joint | 7.47 / 2.53 | 7.47 / 2.53 | 3.88 | 32 | 20.1 / 22.4 |
| RL_kfe_joint | 15.84 / 8.79 | 8.34 / 4.63 | 11.50 | 58 | 20.4 / 28.0 |
| RR_haa_joint | 8.03 / 3.52 | 8.03 / 3.52 | 1.27 | 44 | 20.2 / 24.6 |
| RR_hfe_joint | 7.61 / 2.48 | 7.61 / 2.48 | 4.39 | 31 | 20.1 / 22.3 |
| RR_kfe_joint | 23.13 / 8.70 | 12.18 / 4.58 | 11.67 | 57 | 20.4 / 27.8 |

## trot 0.50 m/s

Duration 20.0s, RMS warmup 1.0s; travel 10.318m, lateral drift 0.061m; safety latch False, clamp events 0, clamped cycles 0.0%, fault mask 0.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 8.79 / 3.95 | 8.79 / 3.95 | 1.57 | 49 | 20.3 / 25.8 |
| FL_hfe_joint | 9.88 / 3.13 | 9.88 / 3.13 | 6.14 | 39 | 20.2 / 23.7 |
| FL_kfe_joint | 22.94 / 8.00 | 12.07 / 4.21 | 12.42 | 53 | 20.3 / 26.6 |
| FR_haa_joint | 7.35 / 3.83 | 7.35 / 3.83 | 1.51 | 48 | 20.3 / 25.5 |
| FR_hfe_joint | 10.02 / 3.16 | 10.02 / 3.16 | 5.69 | 40 | 20.2 / 23.7 |
| FR_kfe_joint | 12.75 / 8.02 | 6.71 / 4.22 | 12.04 | 53 | 20.3 / 26.6 |
| RL_haa_joint | 7.16 / 3.17 | 7.16 / 3.17 | 1.81 | 40 | 20.2 / 23.8 |
| RL_hfe_joint | 11.85 / 3.55 | 11.85 / 3.55 | 5.48 | 44 | 20.2 / 24.7 |
| RL_kfe_joint | 17.05 / 9.00 | 8.97 / 4.74 | 12.47 | 59 | 20.4 / 28.4 |
| RR_haa_joint | 8.45 / 3.14 | 8.45 / 3.14 | 1.43 | 39 | 20.2 / 23.7 |
| RR_hfe_joint | 11.89 / 3.49 | 11.89 / 3.49 | 6.29 | 44 | 20.2 / 24.6 |
| RR_kfe_joint | 25.52 / 8.92 | 13.43 / 4.69 | 13.18 | 59 | 20.4 / 28.2 |
