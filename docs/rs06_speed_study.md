# RS06 configuration: simulation feasibility

Working mass: **19.719 kg**. Knee reduction: motor output turns twice per knee turn; 95% belt efficiency assumed. Hips are direct drive.

All runs use the configured 400 Hz controller gains, motor speed/torque limits and SafetyMonitor. They start at the standing keyframe on an obstacle-free plane. Applied torque is read from MuJoCo qfrc_actuator at every 0.5 ms physics step, after command limits and transmission gears. Peaks and overload durations include startup; RMS excludes the stated warmup. Commands are steps from standing, not speed ramps. The adjacent XML records the evaluated model; the GUI's proving ground has a barrier at 9 m and is unsuitable for long speed runs.

| Gait | Command m/s / yaw rad/s | Actual m/s | Max tilt ° | Min height m | Worst motor RMS / peak N·m | Stall-reference % | Safety-clamped cycles | Stable / tracks |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| stand | 0.00 / 0.00 | -0.000 | 0.1 | 0.321 | 3.28 / 4.05 | 41 | 0.0% | True / True |
| walk | 0.15 / 0.00 | 0.171 | 4.6 | 0.320 | 4.94 / 10.77 | 62 | 0.0% | True / True |
| walk | 0.40 / 0.00 | 0.420 | 5.5 | 0.320 | 5.36 / 23.59 | 67 | 0.0% | True / True |
| walk | 0.50 / 0.00 | 0.521 | 5.5 | 0.321 | 5.91 / 29.30 | 74 | 60.6% | True / True |
| walk | 0.75 / 0.00 | 0.005 | 28.8 | 0.151 | 7.67 / 30.75 | 96 | 98.4% | False / False |
| walk | 1.00 / 0.00 | 0.015 | 25.3 | 0.144 | 7.68 / 31.63 | 96 | 98.8% | False / False |
| walk | 1.50 / 0.00 | 0.028 | 180.0 | 0.057 | 6.90 / 34.64 | 86 | 99.4% | False / False |
| walk | 2.00 / 0.00 | 0.026 | 180.0 | 0.059 | 7.01 / 35.70 | 88 | 99.6% | False / False |
| trot | 0.40 / 0.00 | 0.414 | 3.8 | 0.322 | 4.73 / 11.78 | 59 | 0.0% | True / True |
| trot | 0.50 / 0.00 | 0.515 | 4.5 | 0.322 | 4.79 / 13.34 | 60 | 0.0% | True / True |
| trot | 0.75 / 0.00 | 0.756 | 6.2 | 0.322 | 4.99 / 21.19 | 62 | 0.0% | True / True |
| trot | 1.00 / 0.00 | 0.979 | 8.8 | 0.322 | 5.38 / 28.15 | 67 | 26.8% | True / True |
| trot | 1.50 / 0.00 | 1.477 | 10.1 | 0.318 | 6.49 / 30.37 | 81 | 99.4% | True / True |
| trot | 2.00 / 0.00 | 0.052 | 39.2 | 0.152 | 7.21 / 35.12 | 90 | 99.9% | False / False |
| trot | 0.40 / 0.30 | 0.407 | 3.7 | 0.322 | 5.24 / 13.42 | 65 | 0.0% | True / False |
| trot | 0.50 / 0.30 | 0.507 | 4.4 | 0.322 | 5.01 / 13.69 | 63 | 0.0% | True / False |

## Interpretation

This is a flat-ground simulation result, not hardware approval. Continuous utilisation uses the September specification's 8 N·m continuous stall rating. The 11 N·m rating at 100 rpm requires a 200 × 200 mm aluminum heatsink; the central electronics plate does not establish equivalent cooling for 12 motors. Output inertia and line resistance use the September specification; phase resistance uses an equivalent-wye conversion. Thermal impedance, friction and belt efficiency remain assumptions. The I²t limiter is a conservative software policy, not a reproduction of the manufacturer's overload algorithm. Temperatures and steady-state extrapolations below cannot verify overheating margins: the lumped model omits per-phase stall hotspots, resistance changes with temperature and high-current torque nonlinearity. Belt compliance, pulley strength, motor mounting fit, bus power/regeneration, battery sag and real sensor feedback require validation. Hardware travel is blocked until live IMU/base-state estimation is integrated. See [the datasheet audit](rs06_datasheet_audit.md) for ratings, current conventions and conflicting temperature thresholds.

Assumed bus voltage: 44.4 V (two 6S packs in series). The speed envelope is a voltage-scaled approximation. A 6S parallel arrangement requires a separate evaluation.

## stand 0.00 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel -0.003m, lateral drift 0.000m; safety latch False, clamp events 0, clamped cycles 0.0%, fault mask 0.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 4.05 / 3.19 | 4.05 / 3.19 | 0.97 | 40 | 20.2 / 23.8 |
| FL_hfe_joint | 0.53 / 0.53 | 0.53 / 0.53 | 0.16 | 7 | 20.0 / 20.1 |
| FL_kfe_joint | 6.06 / 5.73 | 3.19 / 3.02 | 1.98 | 38 | 20.2 / 23.4 |
| FR_haa_joint | 4.05 / 3.16 | 4.05 / 3.16 | 0.98 | 39 | 20.2 / 23.7 |
| FR_hfe_joint | 0.52 / 0.51 | 0.52 / 0.51 | 0.19 | 6 | 20.0 / 20.1 |
| FR_kfe_joint | 6.08 / 5.67 | 3.20 / 2.98 | 2.05 | 37 | 20.2 / 23.3 |
| RL_haa_joint | 4.05 / 3.28 | 4.05 / 3.28 | 0.97 | 41 | 20.2 / 24.0 |
| RL_hfe_joint | 0.40 / 0.38 | 0.40 / 0.38 | 0.16 | 5 | 20.0 / 20.1 |
| RL_kfe_joint | 6.09 / 6.08 | 3.20 / 3.20 | 1.98 | 40 | 20.2 / 23.8 |
| RR_haa_joint | 4.05 / 3.24 | 4.05 / 3.24 | 0.97 | 40 | 20.2 / 23.9 |
| RR_hfe_joint | 0.42 / 0.39 | 0.42 / 0.39 | 0.15 | 5 | 20.0 / 20.1 |
| RR_kfe_joint | 6.02 / 6.02 | 3.17 / 3.17 | 1.98 | 40 | 20.2 / 23.7 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 4.049 @ 0.0010 | 0.000 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 19.9695 / 0.0000 |
| FL_hfe_joint | 0.533 @ 2.8705 | -0.000 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 17.0655 / 0.0000 |
| FL_kfe_joint | 3.188 @ 0.0230 | -0.639 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 15.2590 / 0.0000 |
| FR_haa_joint | 4.049 @ 0.0010 | 0.000 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 18.5070 / 0.0000 |
| FR_hfe_joint | 0.519 @ 3.1155 | -0.000 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 16.8760 / 0.0000 |
| FR_kfe_joint | 3.202 @ 0.0230 | -0.743 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 19.9395 / 0.0000 |
| RL_haa_joint | 4.049 @ 0.0010 | 0.000 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 17.1735 / 0.0000 |
| RL_hfe_joint | 0.403 @ 0.2030 | -0.011 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 0.6670 / 0.0000 |
| RL_kfe_joint | 3.205 @ 19.9995 | -0.000 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 0.0155 / 0.0000 |
| RR_haa_joint | 4.049 @ 0.0010 | 0.000 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 19.6455 / 0.0000 |
| RR_hfe_joint | 0.423 @ 0.2105 | -0.011 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 1.4860 / 0.0000 |
| RR_kfe_joint | 3.171 @ 19.9995 | -0.000 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 0.0155 / 0.0000 |

## walk 0.15 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 3.416m, lateral drift 0.063m; safety latch False, clamp events 0, clamped cycles 0.0%, fault mask 0.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 10.17 / 4.28 | 10.17 / 4.28 | 2.43 | 54 | 20.3 / 26.9 |
| FL_hfe_joint | 8.60 / 2.28 | 8.60 / 2.28 | 6.67 | 28 | 20.1 / 21.9 |
| FL_kfe_joint | 13.72 / 6.78 | 7.22 / 3.57 | 12.10 | 45 | 20.2 / 24.7 |
| FR_haa_joint | 10.02 / 4.31 | 10.02 / 4.31 | 2.49 | 54 | 20.3 / 26.9 |
| FR_hfe_joint | 7.94 / 2.35 | 7.94 / 2.35 | 5.65 | 29 | 20.1 / 22.1 |
| FR_kfe_joint | 13.90 / 6.80 | 7.32 / 3.58 | 12.29 | 45 | 20.2 / 24.8 |
| RL_haa_joint | 9.95 / 4.94 | 9.95 / 4.94 | 1.72 | 62 | 20.4 / 29.1 |
| RL_hfe_joint | 8.97 / 4.76 | 8.97 / 4.76 | 6.42 | 60 | 20.4 / 28.5 |
| RL_kfe_joint | 16.17 / 9.29 | 8.51 / 4.89 | 14.13 | 61 | 20.4 / 28.9 |
| RR_haa_joint | 10.77 / 4.80 | 10.77 / 4.80 | 1.78 | 60 | 20.4 / 28.6 |
| RR_hfe_joint | 9.41 / 4.51 | 9.41 / 4.51 | 6.15 | 56 | 20.4 / 27.6 |
| RR_kfe_joint | 15.42 / 8.95 | 8.12 / 4.71 | 13.94 | 59 | 20.4 / 28.3 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 10.166 @ 0.7630 | 0.286 | 0.0000 / 0.0000 | 0.000 | 3.2035 | 0.0000 / 0.0000 / 8.0390 / 0.0000 |
| FL_hfe_joint | 8.605 @ 0.6660 | -2.355 | 0.0000 / 0.0000 | 0.000 | 0.0020 | 0.0000 / 0.0000 / 11.2345 / 0.0000 |
| FL_kfe_joint | 7.222 @ 0.0205 | -0.161 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 9.6115 / 0.0000 |
| FR_haa_joint | 10.022 @ 1.8310 | 0.045 | 0.0000 / 0.0000 | 0.000 | 3.2590 | 0.0000 / 0.0000 / 8.1280 / 0.0000 |
| FR_hfe_joint | 7.935 @ 0.9985 | -2.808 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 11.0155 / 0.0000 |
| FR_kfe_joint | 7.315 @ 0.5680 | -0.833 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 9.4110 / 0.0000 |
| RL_haa_joint | 9.947 @ 0.3555 | 0.382 | 0.0000 / 0.0000 | 0.000 | 1.9225 | 0.0000 / 0.0000 / 13.8775 / 0.0000 |
| RL_hfe_joint | 8.966 @ 0.8335 | -2.761 | 0.0000 / 0.0000 | 0.000 | 0.3560 | 0.0000 / 0.0000 / 15.4115 / 0.0000 |
| RL_kfe_joint | 8.512 @ 1.5105 | 2.709 | 0.0000 / 0.0000 | 0.000 | 0.6855 | 0.0000 / 0.0000 / 11.8100 / 0.0000 |
| RR_haa_joint | 10.773 @ 0.6955 | -0.255 | 0.0000 / 0.0000 | 0.000 | 1.7705 | 0.0000 / 0.0000 / 13.9760 / 0.0000 |
| RR_hfe_joint | 9.415 @ 1.1660 | -2.587 | 0.0000 / 0.0000 | 0.000 | 0.0985 | 0.0000 / 0.0000 / 15.5480 / 0.0000 |
| RR_kfe_joint | 8.117 @ 2.0460 | -0.486 | 0.0000 / 0.0000 | 0.000 | 0.3805 | 0.0000 / 0.0000 / 11.5380 / 0.0000 |

## walk 0.40 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 8.404m, lateral drift 0.287m; safety latch False, clamp events 0, clamped cycles 0.0%, fault mask 2.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 10.40 / 4.16 | 10.40 / 4.16 | 2.08 | 52 | 20.3 / 26.5 |
| FL_hfe_joint | 19.15 / 5.12 | 19.15 / 5.12 | 14.18 | 64 | 20.5 / 29.8 |
| FL_kfe_joint | 20.47 / 8.31 | 10.77 / 4.37 | 26.05 | 55 | 20.4 / 27.1 |
| FR_haa_joint | 10.56 / 4.14 | 10.56 / 4.14 | 2.21 | 52 | 20.3 / 26.4 |
| FR_hfe_joint | 16.93 / 5.08 | 16.93 / 5.08 | 12.57 | 63 | 20.5 / 29.6 |
| FR_kfe_joint | 17.59 / 8.24 | 9.26 / 4.33 | 15.99 | 54 | 20.3 / 27.0 |
| RL_haa_joint | 12.36 / 4.60 | 12.36 / 4.60 | 2.80 | 58 | 20.4 / 27.9 |
| RL_hfe_joint | 23.59 / 5.36 | 23.59 / 5.36 | 13.32 | 67 | 20.5 / 30.7 |
| RL_kfe_joint | 19.39 / 9.54 | 10.20 / 5.02 | 19.64 | 63 | 20.4 / 29.4 |
| RR_haa_joint | 12.29 / 4.50 | 12.29 / 4.50 | 2.46 | 56 | 20.4 / 27.6 |
| RR_hfe_joint | 23.12 / 4.99 | 23.12 / 4.99 | 14.73 | 62 | 20.4 / 29.3 |
| RR_kfe_joint | 18.88 / 8.95 | 9.94 / 4.71 | 17.12 | 59 | 20.4 / 28.3 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 10.403 @ 1.4860 | -0.074 | 0.0000 / 0.0000 | 0.000 | 3.2310 | 0.0000 / 0.0000 / 7.8310 / 0.0000 |
| FL_hfe_joint | 19.151 @ 0.6660 | -4.422 | 0.2725 / 0.0190 | 8.612 | 3.0860 | 0.0000 / 0.0000 / 11.9755 / 0.0000 |
| FL_kfe_joint | 10.771 @ 0.6310 | -7.281 | 0.0000 / 0.0000 | 0.000 | 0.2010 | 0.0000 / 0.0000 / 10.9150 / 0.0000 |
| FR_haa_joint | 10.560 @ 1.1460 | 0.085 | 0.0000 / 0.0000 | 0.000 | 2.5335 | 0.0000 / 0.0000 / 8.4270 / 0.0000 |
| FR_hfe_joint | 16.932 @ 0.3335 | -4.003 | 0.2475 / 0.0090 | 4.003 | 2.7900 | 0.0000 / 0.0000 / 11.6495 / 0.0000 |
| FR_kfe_joint | 9.256 @ 1.6235 | 0.018 | 0.0000 / 0.0000 | 0.000 | 0.1660 | 0.0000 / 0.0000 / 10.7025 / 0.0000 |
| RL_haa_joint | 12.356 @ 1.0185 | -0.255 | 0.0225 / 0.0145 | 1.842 | 1.4320 | 0.0000 / 0.0000 / 12.8690 / 0.0000 |
| RL_hfe_joint | 23.589 @ 2.1660 | -3.001 | 0.3060 / 0.0105 | 4.904 | 2.2460 | 0.0000 / 0.0000 / 13.7195 / 0.0000 |
| RL_kfe_joint | 10.204 @ 0.7960 | -0.086 | 0.0000 / 0.0000 | 0.000 | 2.1295 | 0.0000 / 0.0000 / 12.7715 / 0.0000 |
| RR_haa_joint | 12.292 @ 1.3535 | 0.198 | 0.0060 / 0.0050 | 0.556 | 1.6150 | 0.0000 / 0.0000 / 13.4450 / 0.0000 |
| RR_hfe_joint | 23.116 @ 2.4985 | -2.993 | 0.3235 / 0.0105 | 8.858 | 1.8190 | 0.0000 / 0.0000 / 12.6655 / 0.0000 |
| RR_kfe_joint | 9.939 @ 1.3535 | 0.065 | 0.0000 / 0.0000 | 0.000 | 1.9795 | 0.0000 / 0.0000 / 12.5970 / 0.0000 |

## walk 0.50 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 10.119m, lateral drift 2.021m; safety latch False, clamp events 4862, clamped cycles 60.6%, fault mask 7.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 11.58 / 3.66 | 11.58 / 3.66 | 2.52 | 46 | 20.3 / 25.0 |
| FL_hfe_joint | 29.30 / 5.73 | 29.30 / 5.73 | 15.83 | 72 | 20.6 / 32.3 |
| FL_kfe_joint | 28.77 / 10.23 | 15.14 / 5.39 | 30.86 | 67 | 20.5 / 30.8 |
| FR_haa_joint | 11.62 / 3.71 | 11.62 / 3.71 | 2.48 | 46 | 20.2 / 25.1 |
| FR_hfe_joint | 23.66 / 5.71 | 23.66 / 5.71 | 15.70 | 71 | 20.6 / 32.2 |
| FR_kfe_joint | 20.98 / 9.78 | 11.04 / 5.15 | 20.97 | 64 | 20.5 / 29.9 |
| RL_haa_joint | 12.79 / 4.31 | 12.79 / 4.31 | 3.24 | 54 | 20.3 / 26.9 |
| RL_hfe_joint | 25.85 / 5.91 | 25.85 / 5.91 | 17.32 | 74 | 20.6 / 33.0 |
| RL_kfe_joint | 23.97 / 9.77 | 12.61 / 5.14 | 26.61 | 64 | 20.5 / 29.9 |
| RR_haa_joint | 13.00 / 4.03 | 13.00 / 4.03 | 2.51 | 50 | 20.3 / 26.1 |
| RR_hfe_joint | 27.21 / 5.56 | 27.21 / 5.56 | 17.14 | 69 | 20.5 / 31.5 |
| RR_kfe_joint | 21.90 / 8.18 | 11.53 / 4.30 | 21.47 | 54 | 20.3 / 26.9 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 11.575 @ 2.3575 | -0.475 | 0.0050 / 0.0050 | 0.475 | 1.5270 | 0.0000 / 0.0000 / 9.5635 / 0.0000 |
| FL_hfe_joint | 29.298 @ 0.5005 | -5.952 | 0.1805 / 0.0445 | 12.127 | 5.9665 | 6.8050 / 0.0000 / 15.3595 / 0.0000 |
| FL_kfe_joint | 15.144 @ 0.5210 | 8.134 | 0.0185 / 0.0090 | 8.134 | 2.3675 | 0.0000 / 0.0000 / 11.0010 / 0.0000 |
| FR_haa_joint | 11.624 @ 5.3600 | 0.566 | 0.0290 / 0.0060 | 0.566 | 0.7025 | 0.0000 / 0.0000 / 9.6025 / 0.0000 |
| FR_hfe_joint | 23.662 @ 2.9985 | -3.236 | 0.3455 / 0.0715 | 9.570 | 5.4035 | 4.7825 / 0.0000 / 13.9460 / 0.0000 |
| FR_kfe_joint | 11.044 @ 4.9560 | 0.060 | 0.0030 / 0.0005 | 1.482 | 0.8795 | 0.0000 / 0.0000 / 10.9875 / 0.0000 |
| RL_haa_joint | 12.787 @ 1.6860 | -0.200 | 0.1190 / 0.0135 | 1.901 | 0.8090 | 0.0000 / 0.0000 / 10.8025 / 0.0000 |
| RL_hfe_joint | 25.855 @ 1.4985 | -4.155 | 0.9280 / 0.0400 | 13.054 | 3.3490 | 0.6575 / 0.0000 / 11.3795 / 0.0000 |
| RL_kfe_joint | 12.614 @ 1.4610 | -1.568 | 0.0760 / 0.0135 | 11.125 | 2.4460 | 0.0000 / 0.0000 / 13.0610 / 0.0000 |
| RR_haa_joint | 13.004 @ 3.3535 | 0.259 | 0.1335 / 0.0165 | 0.888 | 1.5485 | 0.0000 / 0.0000 / 11.0290 / 0.0000 |
| RR_hfe_joint | 27.205 @ 2.4985 | -3.695 | 0.7340 / 0.0220 | 12.554 | 2.9510 | 0.2250 / 0.0000 / 8.5195 / 0.0000 |
| RR_kfe_joint | 11.529 @ 0.4585 | -1.429 | 0.0735 / 0.0020 | 5.815 | 1.2565 | 0.0000 / 0.0000 / 13.2015 / 0.0000 |

## walk 0.75 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 0.123m, lateral drift 0.226m; safety latch False, clamp events 15510, clamped cycles 98.4%, fault mask 7.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
**Failed operating point:** excessive tilt; body height below limit. Loads include the unstable/fallen portion; they do not describe successful locomotion.

| FL_haa_joint | 28.44 / 5.77 | 28.44 / 5.77 | 6.81 | 72 | 20.6 / 32.4 |
| FL_hfe_joint | 29.51 / 7.66 | 29.51 / 7.66 | 14.93 | 96 | 21.1 / 41.9 |
| FL_kfe_joint | 27.02 / 6.68 | 14.22 / 3.52 | 30.65 | 44 | 20.2 / 24.6 |
| FR_haa_joint | 29.80 / 5.66 | 29.80 / 5.66 | 6.70 | 71 | 20.6 / 32.0 |
| FR_hfe_joint | 23.81 / 7.67 | 23.81 / 7.67 | 15.30 | 96 | 21.1 / 41.9 |
| FR_kfe_joint | 27.55 / 6.69 | 14.50 / 3.52 | 30.43 | 44 | 20.2 / 24.6 |
| RL_haa_joint | 19.16 / 4.25 | 19.16 / 4.25 | 5.59 | 53 | 20.3 / 26.8 |
| RL_hfe_joint | 30.75 / 6.62 | 30.75 / 6.62 | 18.23 | 83 | 20.8 / 36.4 |
| RL_kfe_joint | 30.54 / 9.47 | 16.07 / 4.98 | 30.42 | 62 | 20.4 / 29.3 |
| RR_haa_joint | 18.21 / 4.27 | 18.21 / 4.27 | 4.11 | 53 | 20.3 / 26.8 |
| RR_hfe_joint | 30.74 / 6.65 | 30.74 / 6.65 | 19.49 | 83 | 20.8 / 36.5 |
| RR_kfe_joint | 35.34 / 9.22 | 18.60 / 4.85 | 32.55 | 61 | 20.4 / 28.8 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 28.438 @ 2.6660 | 0.365 | 0.4735 / 0.0110 | 5.744 | 3.6995 | 0.0000 / 0.0000 / 11.8165 / 0.0000 |
| FL_hfe_joint | 29.511 @ 0.3235 | -4.081 | 0.2625 / 0.1305 | 11.102 | 8.1930 | 18.2000 / 0.0000 / 11.3345 / 0.0000 |
| FL_kfe_joint | 14.222 @ 1.1860 | 9.868 | 0.0100 / 0.0100 | 9.868 | 0.6715 | 0.0000 / 0.0000 / 13.9805 / 0.0000 |
| FR_haa_joint | 29.800 @ 12.9985 | -0.710 | 0.5080 / 0.0580 | 5.732 | 3.1565 | 0.0000 / 0.0000 / 11.8755 / 0.0000 |
| FR_hfe_joint | 23.813 @ 0.5780 | 1.872 | 0.2705 / 0.2225 | 8.916 | 7.8805 | 18.1650 / 0.0000 / 11.3760 / 0.0000 |
| FR_kfe_joint | 14.503 @ 1.6660 | -12.600 | 0.0340 / 0.0170 | 25.430 | 0.7030 | 0.0000 / 0.0000 / 13.7000 / 0.0000 |
| RL_haa_joint | 19.160 @ 2.5185 | 0.119 | 0.7930 / 0.0360 | 1.699 | 1.5400 | 0.0000 / 0.0000 / 13.2330 / 0.0000 |
| RL_hfe_joint | 30.748 @ 0.5810 | -4.771 | 0.1910 / 0.0540 | 16.341 | 7.6405 | 10.6100 / 0.0000 / 11.0275 / 0.0000 |
| RL_kfe_joint | 16.074 @ 9.9985 | 0.037 | 0.2705 / 0.0175 | 24.843 | 0.9175 | 0.0000 / 0.0000 / 12.8165 / 0.0000 |
| RR_haa_joint | 18.208 @ 4.8560 | 0.072 | 0.7470 / 0.0445 | 2.056 | 1.6050 | 0.0000 / 0.0000 / 13.6000 / 0.0000 |
| RR_hfe_joint | 30.735 @ 0.3260 | -4.709 | 0.2030 / 0.0505 | 17.592 | 8.0085 | 11.3725 / 0.0000 / 10.9985 / 0.0000 |
| RR_kfe_joint | 18.603 @ 0.3535 | 30.937 | 0.2865 / 0.0245 | 30.937 | 1.1035 | 0.0000 / 0.0000 / 13.4610 / 0.0000 |

## walk 1.00 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 0.323m, lateral drift 0.114m; safety latch False, clamp events 15767, clamped cycles 98.8%, fault mask 7.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
**Failed operating point:** excessive tilt; body height below limit. Loads include the unstable/fallen portion; they do not describe successful locomotion.

| FL_haa_joint | 21.82 / 5.01 | 21.82 / 5.01 | 5.95 | 63 | 20.5 / 29.4 |
| FL_hfe_joint | 27.96 / 7.67 | 27.96 / 7.67 | 12.97 | 96 | 21.1 / 42.0 |
| FL_kfe_joint | 21.29 / 7.31 | 11.20 / 3.85 | 32.27 | 48 | 20.3 / 25.5 |
| FR_haa_joint | 23.78 / 5.03 | 23.78 / 5.03 | 6.81 | 63 | 20.4 / 29.5 |
| FR_hfe_joint | 31.63 / 7.68 | 31.63 / 7.68 | 16.71 | 96 | 21.1 / 42.0 |
| FR_kfe_joint | 22.24 / 7.34 | 11.70 / 3.86 | 31.26 | 48 | 20.3 / 25.6 |
| RL_haa_joint | 23.85 / 3.77 | 23.85 / 3.77 | 6.64 | 47 | 20.3 / 25.3 |
| RL_hfe_joint | 27.75 / 6.82 | 27.75 / 6.82 | 13.97 | 85 | 20.9 / 37.4 |
| RL_kfe_joint | 32.00 / 11.29 | 16.84 / 5.94 | 30.18 | 74 | 20.6 / 33.2 |
| RR_haa_joint | 19.47 / 3.67 | 19.47 / 3.67 | 4.64 | 46 | 20.2 / 25.0 |
| RR_hfe_joint | 28.56 / 6.78 | 28.56 / 6.78 | 19.02 | 85 | 20.9 / 37.2 |
| RR_kfe_joint | 31.91 / 11.18 | 16.80 / 5.89 | 34.11 | 74 | 20.6 / 32.9 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 21.818 @ 1.3335 | -0.731 | 0.3870 / 0.0745 | 4.557 | 1.5800 | 0.0000 / 0.0000 / 14.5535 / 0.0000 |
| FL_hfe_joint | 27.956 @ 0.2435 | -4.122 | 0.2940 / 0.0920 | 7.936 | 10.4980 | 18.5625 / 0.0000 / 8.4780 / 0.0000 |
| FL_kfe_joint | 11.204 @ 11.4985 | 13.146 | 0.0080 / 0.0020 | 15.548 | 1.2445 | 0.0000 / 0.0000 / 14.9655 / 0.0000 |
| FR_haa_joint | 23.779 @ 1.6660 | 0.325 | 0.3475 / 0.0195 | 4.712 | 1.4830 | 0.0000 / 0.0000 / 14.6620 / 0.0000 |
| FR_hfe_joint | 31.626 @ 0.3335 | -3.161 | 0.2885 / 0.1885 | 12.918 | 10.2680 | 18.4875 / 0.0000 / 8.5355 / 0.0000 |
| FR_kfe_joint | 11.704 @ 0.2960 | -5.256 | 0.0210 / 0.0095 | 15.977 | 1.2670 | 0.0000 / 0.0000 / 14.9815 / 0.0000 |
| RL_haa_joint | 23.848 @ 1.1860 | 1.017 | 0.3035 / 0.0555 | 4.951 | 1.8430 | 0.0000 / 0.0000 / 11.6410 / 0.0000 |
| RL_hfe_joint | 27.755 @ 0.4785 | -4.486 | 0.2265 / 0.1490 | 11.765 | 8.5420 | 11.9950 / 0.0000 / 9.7050 / 0.0000 |
| RL_kfe_joint | 16.840 @ 13.3335 | -0.230 | 0.3650 / 0.0090 | 13.114 | 2.3440 | 0.2100 / 0.0000 / 10.5385 / 0.0000 |
| RR_haa_joint | 19.470 @ 1.5185 | -0.378 | 0.2825 / 0.0560 | 2.519 | 1.6090 | 0.0000 / 0.0000 / 11.4885 / 0.0000 |
| RR_hfe_joint | 28.562 @ 0.3535 | 2.094 | 0.1975 / 0.0535 | 17.427 | 8.2585 | 11.6525 / 0.0000 / 9.8390 / 0.0000 |
| RR_kfe_joint | 16.797 @ 4.3335 | 0.005 | 0.4010 / 0.1705 | 18.883 | 2.5545 | 1.3975 / 0.0000 / 10.6240 / 0.0000 |

## walk 1.50 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 0.638m, lateral drift 0.271m; safety latch False, clamp events 15872, clamped cycles 99.4%, fault mask 7.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
**Failed operating point:** excessive tilt; body height below limit. Loads include the unstable/fallen portion; they do not describe successful locomotion.

| FL_haa_joint | 28.30 / 5.37 | 28.30 / 5.37 | 8.76 | 67 | 20.5 / 30.8 |
| FL_hfe_joint | 25.33 / 6.90 | 25.33 / 6.90 | 18.27 | 86 | 20.9 / 37.8 |
| FL_kfe_joint | 37.39 / 10.26 | 19.68 / 5.40 | 33.16 | 67 | 20.5 / 30.9 |
| FR_haa_joint | 23.94 / 5.21 | 23.94 / 5.21 | 9.45 | 65 | 20.5 / 30.1 |
| FR_hfe_joint | 31.62 / 6.84 | 31.62 / 6.84 | 18.23 | 85 | 20.9 / 37.5 |
| FR_kfe_joint | 36.27 / 10.06 | 19.09 / 5.29 | 29.99 | 66 | 20.5 / 30.5 |
| RL_haa_joint | 33.41 / 5.54 | 33.41 / 5.54 | 8.86 | 69 | 20.5 / 31.5 |
| RL_hfe_joint | 34.64 / 6.89 | 34.64 / 6.89 | 17.30 | 86 | 20.9 / 37.7 |
| RL_kfe_joint | 39.68 / 11.82 | 20.88 / 6.22 | 32.57 | 78 | 20.7 / 34.5 |
| RR_haa_joint | 31.91 / 5.61 | 31.91 / 5.61 | 11.63 | 70 | 20.6 / 31.8 |
| RR_hfe_joint | 26.24 / 6.88 | 26.24 / 6.88 | 17.91 | 86 | 20.9 / 37.7 |
| RR_kfe_joint | 36.17 / 11.78 | 19.03 / 6.20 | 34.04 | 78 | 20.7 / 34.4 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 28.301 @ 1.3335 | 5.973 | 1.0330 / 0.0305 | 5.973 | 3.7670 | 2.9425 / 0.0000 / 8.8400 / 0.0000 |
| FL_hfe_joint | 25.328 @ 0.2955 | 1.893 | 0.2395 / 0.0875 | 12.089 | 6.6810 | 16.5150 / 0.0000 / 11.2830 / 0.0000 |
| FL_kfe_joint | 19.678 @ 2.3335 | -5.967 | 0.3515 / 0.1420 | 22.829 | 2.9350 | 3.1075 / 0.0000 / 11.7805 / 0.0000 |
| FR_haa_joint | 23.943 @ 0.5470 | 0.937 | 0.9965 / 0.1275 | 6.380 | 3.9895 | 3.3825 / 0.0000 / 8.8205 / 0.0000 |
| FR_hfe_joint | 31.622 @ 0.1860 | -14.133 | 0.1835 / 0.0525 | 16.108 | 6.7040 | 16.6750 / 0.0000 / 11.3730 / 0.0000 |
| FR_kfe_joint | 19.090 @ 1.4335 | -4.773 | 0.3175 / 0.0840 | 29.579 | 3.0215 | 3.1600 / 0.0000 / 11.7985 / 0.0000 |
| RL_haa_joint | 33.408 @ 0.4985 | -0.483 | 0.3905 / 0.0245 | 7.422 | 4.3275 | 4.1100 / 0.0000 / 8.2860 / 0.0000 |
| RL_hfe_joint | 34.636 @ 0.1660 | -5.488 | 0.1745 / 0.0665 | 13.931 | 6.7520 | 17.3425 / 0.0000 / 11.5270 / 0.0000 |
| RL_kfe_joint | 20.885 @ 5.2635 | -4.329 | 0.3075 / 0.0190 | 28.202 | 3.5840 | 3.2200 / 0.0000 / 10.9010 / 0.0000 |
| RR_haa_joint | 31.913 @ 1.1660 | -0.714 | 0.3285 / 0.0250 | 10.951 | 4.5410 | 4.5100 / 0.0000 / 8.1800 / 0.0000 |
| RR_hfe_joint | 26.235 @ 0.1610 | -3.891 | 0.2030 / 0.0610 | 17.521 | 6.8535 | 17.1375 / 0.0000 / 11.7685 / 0.0000 |
| RR_kfe_joint | 19.034 @ 0.1160 | 8.806 | 0.3015 / 0.1365 | 28.151 | 3.7125 | 3.9900 / 0.0000 / 11.2540 / 0.0000 |

## walk 2.00 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 0.525m, lateral drift 0.361m; safety latch False, clamp events 15892, clamped cycles 99.6%, fault mask 7.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
**Failed operating point:** excessive tilt; body height below limit. Loads include the unstable/fallen portion; they do not describe successful locomotion.

| FL_haa_joint | 28.82 / 5.63 | 28.82 / 5.63 | 7.72 | 70 | 20.6 / 31.9 |
| FL_hfe_joint | 25.32 / 6.84 | 25.32 / 6.84 | 19.12 | 86 | 20.9 / 37.5 |
| FL_kfe_joint | 32.22 / 11.38 | 16.96 / 5.99 | 32.71 | 75 | 20.7 / 33.4 |
| FR_haa_joint | 35.52 / 5.52 | 35.52 / 5.52 | 9.00 | 69 | 20.6 / 31.4 |
| FR_hfe_joint | 27.15 / 6.87 | 27.15 / 6.87 | 20.15 | 86 | 20.9 / 37.6 |
| FR_kfe_joint | 49.11 / 11.28 | 25.85 / 5.94 | 32.08 | 74 | 20.6 / 33.2 |
| RL_haa_joint | 30.81 / 5.68 | 30.81 / 5.68 | 11.41 | 71 | 20.6 / 32.1 |
| RL_hfe_joint | 28.48 / 7.01 | 28.48 / 7.01 | 20.03 | 88 | 20.9 / 38.4 |
| RL_kfe_joint | 40.11 / 10.89 | 21.11 / 5.73 | 34.28 | 72 | 20.6 / 32.3 |
| RR_haa_joint | 35.70 / 5.74 | 35.70 / 5.74 | 13.10 | 72 | 20.6 / 32.3 |
| RR_hfe_joint | 27.62 / 6.98 | 27.62 / 6.98 | 19.73 | 87 | 20.9 / 38.2 |
| RR_kfe_joint | 44.06 / 10.81 | 23.19 / 5.69 | 33.70 | 71 | 20.6 / 32.1 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 28.819 @ 1.5560 | 2.860 | 0.5995 / 0.0345 | 5.792 | 4.6870 | 4.0125 / 0.0000 / 8.4850 / 0.0000 |
| FL_hfe_joint | 25.319 @ 0.0275 | 1.223 | 0.2170 / 0.0725 | 10.234 | 6.5420 | 16.4700 / 0.0000 / 11.3840 / 0.0000 |
| FL_kfe_joint | 16.956 @ 0.0785 | 9.187 | 0.2540 / 0.1040 | 21.068 | 4.2140 | 5.2225 / 0.0000 / 11.8650 / 0.0000 |
| FR_haa_joint | 35.522 @ 0.3535 | 0.617 | 0.3025 / 0.1875 | 7.350 | 5.4015 | 5.6125 / 0.0005 / 8.8085 / 0.0000 |
| FR_hfe_joint | 27.146 @ 0.1185 | -8.021 | 0.2215 / 0.0480 | 17.765 | 6.4690 | 16.3700 / 0.0000 / 11.4875 / 0.0000 |
| FR_kfe_joint | 25.845 @ 0.4560 | -1.505 | 0.2265 / 0.0510 | 25.476 | 4.1255 | 5.1275 / 0.0000 / 12.0515 / 0.0000 |
| RL_haa_joint | 30.813 @ 1.5560 | -1.940 | 0.4325 / 0.0170 | 9.665 | 4.5580 | 4.3325 / 0.0000 / 8.8870 / 0.0000 |
| RL_hfe_joint | 28.481 @ 0.1660 | -1.166 | 0.1730 / 0.0590 | 13.919 | 7.0380 | 16.5800 / 0.0000 / 10.9165 / 0.0000 |
| RL_kfe_joint | 21.110 @ 1.3335 | -1.582 | 0.3475 / 0.0270 | 25.192 | 3.5640 | 3.4175 / 0.0000 / 11.5910 / 0.0000 |
| RR_haa_joint | 35.699 @ 0.4985 | -3.322 | 0.3025 / 0.0405 | 12.651 | 4.9540 | 4.9275 / 0.0000 / 8.8530 / 0.0000 |
| RR_hfe_joint | 27.616 @ 0.0010 | 0.000 | 0.2060 / 0.0750 | 18.909 | 7.1075 | 16.4675 / 0.0000 / 11.0930 / 0.0000 |
| RR_kfe_joint | 23.192 @ 0.0735 | 4.970 | 0.2660 / 0.1065 | 26.080 | 3.5805 | 3.8800 / 0.0000 / 11.5955 / 0.0000 |

## trot 0.40 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 8.273m, lateral drift 0.205m; safety latch False, clamp events 0, clamped cycles 0.0%, fault mask 0.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 8.45 / 4.01 | 8.45 / 4.01 | 1.48 | 50 | 20.3 / 26.0 |
| FL_hfe_joint | 9.30 / 2.78 | 9.30 / 2.78 | 5.13 | 35 | 20.1 / 22.9 |
| FL_kfe_joint | 20.43 / 7.92 | 10.75 / 4.17 | 10.83 | 52 | 20.3 / 26.5 |
| FR_haa_joint | 7.34 / 3.89 | 7.34 / 3.89 | 1.36 | 49 | 20.3 / 25.7 |
| FR_hfe_joint | 9.37 / 2.81 | 9.37 / 2.81 | 4.66 | 35 | 20.1 / 22.9 |
| FR_kfe_joint | 12.29 / 7.91 | 6.47 / 4.17 | 11.04 | 52 | 20.3 / 26.5 |
| RL_haa_joint | 7.93 / 3.28 | 7.93 / 3.28 | 1.66 | 41 | 20.2 / 24.0 |
| RL_hfe_joint | 10.82 / 3.12 | 10.82 / 3.12 | 4.65 | 39 | 20.2 / 23.6 |
| RL_kfe_joint | 16.15 / 8.99 | 8.50 / 4.73 | 11.93 | 59 | 20.4 / 28.4 |
| RR_haa_joint | 9.57 / 3.28 | 9.57 / 3.28 | 1.64 | 41 | 20.2 / 24.0 |
| RR_hfe_joint | 10.83 / 3.03 | 10.83 / 3.03 | 5.33 | 38 | 20.2 / 23.4 |
| RR_kfe_joint | 22.38 / 8.80 | 11.78 / 4.63 | 12.17 | 58 | 20.4 / 28.0 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 8.446 @ 0.1175 | 0.108 | 0.0000 / 0.0000 | 0.000 | 0.0695 | 0.0000 / 0.0000 / 11.2110 / 0.0000 |
| FL_hfe_joint | 9.304 @ 0.4535 | -1.198 | 0.0000 / 0.0000 | 0.000 | 0.1510 | 0.0000 / 0.0000 / 11.2980 / 0.0000 |
| FL_kfe_joint | 10.751 @ 0.0155 | 0.259 | 0.0000 / 0.0000 | 0.000 | 0.0325 | 0.0000 / 0.0000 / 10.2655 / 0.0000 |
| FR_haa_joint | 7.337 @ 19.7735 | -0.258 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 11.4955 / 0.0000 |
| FR_hfe_joint | 9.366 @ 2.5010 | -0.920 | 0.0000 / 0.0000 | 0.000 | 0.1510 | 0.0000 / 0.0000 / 11.2595 / 0.0000 |
| FR_kfe_joint | 6.466 @ 9.4595 | -0.550 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 10.1705 / 0.0000 |
| RL_haa_joint | 7.926 @ 2.9535 | -0.008 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 10.4455 / 0.0000 |
| RL_hfe_joint | 10.821 @ 0.6810 | -1.225 | 0.0000 / 0.0000 | 0.000 | 0.4515 | 0.0000 / 0.0000 / 12.4425 / 0.0000 |
| RL_kfe_joint | 8.501 @ 0.2260 | 3.224 | 0.0000 / 0.0000 | 0.000 | 0.0025 | 0.0000 / 0.0000 / 11.1340 / 0.0000 |
| RR_haa_joint | 9.574 @ 0.0010 | 0.000 | 0.0000 / 0.0000 | 0.000 | 0.0080 | 0.0000 / 0.0000 / 10.6385 / 0.0000 |
| RR_hfe_joint | 10.833 @ 1.8185 | -1.092 | 0.0000 / 0.0000 | 0.000 | 0.4445 | 0.0000 / 0.0000 / 12.2005 / 0.0000 |
| RR_kfe_joint | 11.781 @ 0.0155 | 0.650 | 0.0110 / 0.0095 | 2.824 | 0.0955 | 0.0000 / 0.0000 / 10.9295 / 0.0000 |

## trot 0.50 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 10.294m, lateral drift 0.294m; safety latch False, clamp events 0, clamped cycles 0.0%, fault mask 0.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 8.67 / 3.92 | 8.67 / 3.92 | 1.63 | 49 | 20.3 / 25.7 |
| FL_hfe_joint | 11.22 / 3.24 | 11.22 / 3.24 | 6.15 | 40 | 20.2 / 23.9 |
| FL_kfe_joint | 20.83 / 8.04 | 10.96 / 4.23 | 12.40 | 53 | 20.3 / 26.7 |
| FR_haa_joint | 7.32 / 3.81 | 7.32 / 3.81 | 1.42 | 48 | 20.3 / 25.4 |
| FR_hfe_joint | 11.45 / 3.26 | 11.45 / 3.26 | 5.63 | 41 | 20.2 / 24.0 |
| FR_kfe_joint | 12.58 / 8.02 | 6.62 / 4.22 | 11.94 | 53 | 20.3 / 26.7 |
| RL_haa_joint | 8.56 / 3.12 | 8.56 / 3.12 | 1.80 | 39 | 20.2 / 23.6 |
| RL_hfe_joint | 13.34 / 3.66 | 13.34 / 3.66 | 5.46 | 46 | 20.2 / 25.0 |
| RL_kfe_joint | 15.39 / 9.10 | 8.10 / 4.79 | 12.44 | 60 | 20.4 / 28.6 |
| RR_haa_joint | 9.85 / 3.11 | 9.85 / 3.11 | 1.71 | 39 | 20.2 / 23.6 |
| RR_hfe_joint | 13.33 / 3.56 | 13.33 / 3.56 | 6.29 | 45 | 20.2 / 24.7 |
| RR_kfe_joint | 23.53 / 8.91 | 12.39 / 4.69 | 13.04 | 59 | 20.4 / 28.2 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 8.671 @ 0.1130 | 0.148 | 0.0000 / 0.0000 | 0.000 | 0.0780 | 0.0000 / 0.0000 / 10.9370 / 0.0000 |
| FL_hfe_joint | 11.218 @ 0.4535 | -1.250 | 0.0020 / 0.0005 | 1.250 | 0.3440 | 0.0000 / 0.0000 / 11.0345 / 0.0000 |
| FL_kfe_joint | 10.964 @ 0.0155 | 0.239 | 0.0000 / 0.0000 | 0.000 | 0.0355 | 0.0000 / 0.0000 / 10.2410 / 0.0000 |
| FR_haa_joint | 7.317 @ 2.5010 | -0.360 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 11.2400 / 0.0000 |
| FR_hfe_joint | 11.451 @ 1.1360 | -0.993 | 0.0130 / 0.0010 | 1.032 | 0.2840 | 0.0000 / 0.0000 / 10.9670 / 0.0000 |
| FR_kfe_joint | 6.621 @ 7.6435 | -0.496 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 10.1750 / 0.0000 |
| RL_haa_joint | 8.564 @ 5.2260 | 0.173 | 0.0000 / 0.0000 | 0.000 | 0.0085 | 0.0000 / 0.0000 / 10.5140 / 0.0000 |
| RL_hfe_joint | 13.337 @ 5.2260 | -1.142 | 0.0890 / 0.0045 | 1.153 | 1.6715 | 0.0000 / 0.0000 / 11.9435 / 0.0000 |
| RL_kfe_joint | 8.099 @ 0.2260 | 3.651 | 0.0000 / 0.0000 | 0.000 | 0.0010 | 0.0000 / 0.0000 / 10.6450 / 0.0000 |
| RR_haa_joint | 9.849 @ 0.0010 | 0.000 | 0.0000 / 0.0000 | 0.000 | 0.0260 | 0.0000 / 0.0000 / 10.7025 / 0.0000 |
| RR_hfe_joint | 13.332 @ 7.7260 | -1.154 | 0.1515 / 0.0045 | 1.499 | 1.5575 | 0.0000 / 0.0000 / 11.7435 / 0.0000 |
| RR_kfe_joint | 12.386 @ 0.0155 | 0.546 | 0.0185 / 0.0150 | 3.968 | 0.1575 | 0.0000 / 0.0000 / 10.4005 / 0.0000 |

## trot 0.75 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 15.109m, lateral drift 0.734m; safety latch False, clamp events 0, clamped cycles 0.0%, fault mask 0.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 9.16 / 3.58 | 9.16 / 3.58 | 1.88 | 45 | 20.2 / 24.8 |
| FL_hfe_joint | 16.85 / 4.18 | 16.85 / 4.18 | 8.66 | 52 | 20.3 / 26.5 |
| FL_kfe_joint | 21.95 / 8.28 | 11.55 / 4.36 | 18.60 | 54 | 20.3 / 27.1 |
| FR_haa_joint | 7.64 / 3.51 | 7.64 / 3.51 | 2.47 | 44 | 20.2 / 24.6 |
| FR_hfe_joint | 16.30 / 4.21 | 16.30 / 4.21 | 8.28 | 53 | 20.3 / 26.6 |
| FR_kfe_joint | 13.62 / 8.25 | 7.17 / 4.34 | 14.98 | 54 | 20.3 / 27.0 |
| RL_haa_joint | 9.04 / 2.68 | 9.04 / 2.68 | 2.17 | 34 | 20.1 / 22.7 |
| RL_hfe_joint | 21.19 / 4.97 | 21.19 / 4.97 | 7.74 | 62 | 20.4 / 29.2 |
| RL_kfe_joint | 14.74 / 9.48 | 7.76 / 4.99 | 15.61 | 62 | 20.4 / 29.3 |
| RR_haa_joint | 10.54 / 2.64 | 10.54 / 2.64 | 1.90 | 33 | 20.1 / 22.6 |
| RR_hfe_joint | 19.75 / 4.78 | 19.75 / 4.78 | 8.80 | 60 | 20.4 / 28.5 |
| RR_kfe_joint | 26.47 / 9.23 | 13.93 / 4.86 | 20.53 | 61 | 20.4 / 28.8 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 9.161 @ 0.1030 | 0.267 | 0.0000 / 0.0000 | 0.000 | 0.0865 | 0.0000 / 0.0000 / 9.2865 / 0.0000 |
| FL_hfe_joint | 16.851 @ 0.9085 | -0.933 | 0.3500 / 0.0695 | 2.179 | 1.5875 | 0.0000 / 0.0000 / 10.5665 / 0.0000 |
| FL_kfe_joint | 11.552 @ 0.0155 | 0.205 | 0.0085 / 0.0085 | 1.335 | 0.0545 | 0.0000 / 0.0000 / 9.9675 / 0.0000 |
| FR_haa_joint | 7.641 @ 0.4555 | 0.167 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 9.5920 / 0.0000 |
| FR_hfe_joint | 16.304 @ 1.1360 | -1.179 | 0.3385 / 0.0610 | 2.860 | 1.6255 | 0.0000 / 0.0000 / 10.3775 / 0.0000 |
| FR_kfe_joint | 7.166 @ 0.8410 | 0.085 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 9.8885 / 0.0000 |
| RL_haa_joint | 9.038 @ 7.0460 | 0.162 | 0.0000 / 0.0000 | 0.000 | 0.0710 | 0.0000 / 0.0000 / 9.5720 / 0.0000 |
| RL_hfe_joint | 21.186 @ 0.6810 | -0.885 | 1.4160 / 0.0390 | 3.816 | 2.5250 | 0.0000 / 0.0000 / 11.2745 / 0.0000 |
| RL_kfe_joint | 7.760 @ 1.2430 | -1.895 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 10.4970 / 0.0000 |
| RR_haa_joint | 10.535 @ 0.0010 | 0.000 | 0.0000 / 0.0000 | 0.000 | 0.0665 | 0.0000 / 0.0000 / 9.6800 / 0.0000 |
| RR_hfe_joint | 19.754 @ 2.7260 | -1.087 | 1.2270 / 0.0415 | 3.143 | 2.3925 | 0.0000 / 0.0000 / 10.9775 / 0.0000 |
| RR_kfe_joint | 13.931 @ 0.0155 | 0.319 | 0.0445 / 0.0375 | 5.612 | 0.2645 | 0.0000 / 0.0000 / 10.4450 / 0.0000 |

## trot 1.00 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 19.518m, lateral drift 1.479m; safety latch False, clamp events 2147, clamped cycles 26.8%, fault mask 6.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 9.65 / 3.25 | 9.65 / 3.25 | 2.71 | 41 | 20.2 / 23.9 |
| FL_hfe_joint | 20.82 / 5.37 | 20.82 / 5.37 | 13.05 | 67 | 20.5 / 30.8 |
| FL_kfe_joint | 24.98 / 8.68 | 13.15 / 4.57 | 28.19 | 57 | 20.4 / 27.8 |
| FR_haa_joint | 8.28 / 3.25 | 8.28 / 3.25 | 2.58 | 41 | 20.2 / 23.9 |
| FR_hfe_joint | 22.38 / 5.38 | 22.38 / 5.38 | 10.24 | 67 | 20.5 / 30.8 |
| FR_kfe_joint | 17.48 / 8.71 | 9.20 / 4.58 | 18.19 | 57 | 20.4 / 27.8 |
| RL_haa_joint | 13.89 / 2.34 | 13.89 / 2.34 | 4.41 | 29 | 20.1 / 22.1 |
| RL_hfe_joint | 28.15 / 5.27 | 28.15 / 5.27 | 9.66 | 66 | 20.5 / 30.4 |
| RL_kfe_joint | 18.92 / 10.18 | 9.96 / 5.36 | 22.03 | 67 | 20.5 / 30.7 |
| RR_haa_joint | 11.22 / 2.27 | 11.22 / 2.27 | 3.30 | 28 | 20.1 / 21.9 |
| RR_hfe_joint | 27.30 / 5.10 | 27.30 / 5.10 | 13.54 | 64 | 20.5 / 29.7 |
| RR_kfe_joint | 29.36 / 9.95 | 15.45 / 5.23 | 29.71 | 65 | 20.5 / 30.2 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 9.646 @ 0.0980 | 0.398 | 0.0000 / 0.0000 | 0.000 | 0.0945 | 0.0000 / 0.0000 / 9.1240 / 0.0000 |
| FL_hfe_joint | 20.821 @ 1.3635 | -0.895 | 1.2560 / 0.1055 | 4.678 | 3.1560 | 0.0000 / 0.0000 / 10.2190 / 0.0000 |
| FL_kfe_joint | 13.150 @ 0.2610 | 0.052 | 0.0285 / 0.0140 | 7.218 | 0.3475 | 0.0000 / 0.0000 / 10.0085 / 0.0000 |
| FR_haa_joint | 8.282 @ 0.9080 | 2.035 | 0.0000 / 0.0000 | 0.000 | 0.0110 | 0.0000 / 0.0000 / 9.3830 / 0.0000 |
| FR_hfe_joint | 22.375 @ 1.5910 | -0.827 | 1.1940 / 0.0645 | 4.841 | 3.0405 | 0.0000 / 0.0000 / 10.2145 / 0.0000 |
| FR_kfe_joint | 9.198 @ 0.4035 | -4.886 | 0.0000 / 0.0000 | 0.000 | 0.0390 | 0.0000 / 0.0000 / 9.9925 / 0.0000 |
| RL_haa_joint | 13.889 @ 0.4480 | -1.371 | 0.0225 / 0.0225 | 1.451 | 0.0960 | 0.0000 / 0.0000 / 10.1905 / 0.0000 |
| RL_hfe_joint | 28.149 @ 0.6810 | -0.293 | 0.9670 / 0.0560 | 6.023 | 3.4195 | 2.7350 / 0.0000 / 11.3445 / 0.0000 |
| RL_kfe_joint | 9.959 @ 0.4110 | 2.404 | 0.0000 / 0.0000 | 0.000 | 2.3775 | 0.0000 / 0.0000 / 10.2870 / 0.0000 |
| RR_haa_joint | 11.222 @ 0.0010 | 0.000 | 0.0005 / 0.0005 | 0.000 | 0.0410 | 0.0000 / 0.0000 / 10.2510 / 0.0000 |
| RR_hfe_joint | 27.295 @ 0.4535 | -0.066 | 1.1475 / 0.0740 | 6.324 | 3.4135 | 2.6325 / 0.0000 / 10.7245 / 0.0000 |
| RR_kfe_joint | 15.453 @ 0.0155 | 0.264 | 0.2395 / 0.2090 | 17.794 | 1.1075 | 0.0000 / 0.0000 / 10.2995 / 0.0000 |

## trot 1.50 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 29.534m, lateral drift 0.840m; safety latch False, clamp events 8106, clamped cycles 99.4%, fault mask 7.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 15.09 / 2.90 | 15.09 / 2.90 | 3.76 | 36 | 20.2 / 23.1 |
| FL_hfe_joint | 21.05 / 6.49 | 21.05 / 6.49 | 17.09 | 81 | 20.8 / 35.7 |
| FL_kfe_joint | 27.17 / 11.64 | 14.30 / 6.13 | 33.05 | 77 | 20.7 / 34.0 |
| FR_haa_joint | 13.47 / 2.98 | 13.47 / 2.98 | 2.61 | 37 | 20.2 / 23.3 |
| FR_hfe_joint | 28.59 / 6.48 | 28.59 / 6.48 | 16.50 | 81 | 20.8 / 35.7 |
| FR_kfe_joint | 34.39 / 11.67 | 18.10 / 6.14 | 33.32 | 77 | 20.7 / 34.1 |
| RL_haa_joint | 16.29 / 2.65 | 16.29 / 2.65 | 3.92 | 33 | 20.1 / 22.6 |
| RL_hfe_joint | 30.37 / 6.24 | 30.37 / 6.24 | 16.89 | 78 | 20.7 / 34.5 |
| RL_kfe_joint | 28.13 / 11.91 | 14.81 / 6.27 | 31.34 | 78 | 20.7 / 34.7 |
| RR_haa_joint | 12.60 / 2.55 | 12.60 / 2.55 | 2.70 | 32 | 20.1 / 22.4 |
| RR_hfe_joint | 25.15 / 6.21 | 25.15 / 6.21 | 16.99 | 78 | 20.7 / 34.4 |
| RR_kfe_joint | 37.83 / 11.73 | 19.91 / 6.17 | 32.82 | 77 | 20.7 / 34.2 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 15.090 @ 0.6810 | -0.141 | 0.0060 / 0.0060 | 1.970 | 0.1150 | 0.0000 / 0.0000 / 11.0090 / 0.0000 |
| FL_hfe_joint | 21.051 @ 0.0620 | 0.643 | 0.2180 / 0.1150 | 11.790 | 5.3840 | 8.6150 / 0.0000 / 10.3045 / 0.0000 |
| FL_kfe_joint | 14.301 @ 0.1160 | 12.231 | 0.1515 / 0.1190 | 23.098 | 3.9405 | 9.9825 / 0.0000 / 12.4025 / 0.0000 |
| FR_haa_joint | 13.466 @ 0.6810 | -1.724 | 0.0680 / 0.0590 | 2.392 | 0.1710 | 0.0000 / 0.0000 / 11.2365 / 0.0000 |
| FR_hfe_joint | 28.586 @ 0.3960 | -1.505 | 0.1580 / 0.1290 | 10.360 | 5.2055 | 8.5525 / 0.0000 / 10.5755 / 0.0000 |
| FR_kfe_joint | 18.100 @ 0.3760 | 0.198 | 0.1545 / 0.0560 | 30.607 | 3.9765 | 10.1375 / 0.0000 / 12.4585 / 0.0000 |
| RL_haa_joint | 16.293 @ 0.6930 | 0.942 | 0.0335 / 0.0310 | 2.330 | 0.1060 | 0.0000 / 0.0000 / 8.5290 / 0.0000 |
| RL_hfe_joint | 30.375 @ 0.3635 | -4.232 | 0.2645 / 0.0375 | 14.736 | 4.6125 | 8.3425 / 0.0000 / 11.3080 / 0.0000 |
| RL_kfe_joint | 14.806 @ 1.4010 | 0.908 | 0.0865 / 0.0220 | 14.019 | 4.6890 | 10.6625 / 0.0000 / 11.5560 / 0.0000 |
| RR_haa_joint | 12.597 @ 0.0010 | 0.000 | 0.0015 / 0.0015 | 0.468 | 0.0165 | 0.0000 / 0.0000 / 8.8095 / 0.0000 |
| RR_hfe_joint | 25.151 @ 0.0010 | 0.000 | 0.2455 / 0.1075 | 9.142 | 4.6610 | 8.3875 / 0.0000 / 11.1915 / 0.0000 |
| RR_kfe_joint | 19.911 @ 0.0985 | 6.188 | 0.1620 / 0.1440 | 30.272 | 4.5285 | 11.0450 / 0.0000 / 11.7165 / 0.0000 |

## trot 2.00 m/s, yaw 0.00 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 1.058m, lateral drift 0.153m; safety latch False, clamp events 15906, clamped cycles 99.9%, fault mask 7.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
**Failed operating point:** excessive tilt; body height below limit. Loads include the unstable/fallen portion; they do not describe successful locomotion.

| FL_haa_joint | 18.84 / 3.57 | 18.84 / 3.57 | 6.48 | 45 | 20.2 / 24.8 |
| FL_hfe_joint | 22.46 / 7.21 | 22.46 / 7.21 | 17.45 | 90 | 21.0 / 39.4 |
| FL_kfe_joint | 33.81 / 9.10 | 17.79 / 4.79 | 33.11 | 60 | 20.4 / 28.6 |
| FR_haa_joint | 20.79 / 3.46 | 20.79 / 3.46 | 6.13 | 43 | 20.2 / 24.5 |
| FR_hfe_joint | 29.86 / 7.19 | 29.86 / 7.19 | 20.39 | 90 | 21.0 / 39.3 |
| FR_kfe_joint | 43.75 / 9.36 | 23.03 / 4.92 | 26.80 | 62 | 20.4 / 29.1 |
| RL_haa_joint | 26.27 / 3.43 | 26.27 / 3.43 | 6.42 | 43 | 20.2 / 24.4 |
| RL_hfe_joint | 29.90 / 5.95 | 29.90 / 5.95 | 19.35 | 74 | 20.7 / 33.2 |
| RL_kfe_joint | 46.94 / 10.37 | 24.70 / 5.46 | 32.45 | 68 | 20.5 / 31.1 |
| RR_haa_joint | 13.97 / 3.50 | 13.97 / 3.50 | 5.23 | 44 | 20.2 / 24.6 |
| RR_hfe_joint | 35.12 / 6.03 | 35.12 / 6.03 | 16.96 | 75 | 20.7 / 33.6 |
| RR_kfe_joint | 41.93 / 10.17 | 22.07 / 5.35 | 31.98 | 67 | 20.5 / 30.7 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 18.837 @ 2.7260 | -0.577 | 0.2280 / 0.0135 | 3.333 | 1.1470 | 0.0000 / 0.0000 / 10.3995 / 0.0000 |
| FL_hfe_joint | 22.459 @ 0.0460 | 0.395 | 0.3740 / 0.0850 | 13.540 | 8.2635 | 14.2600 / 0.0000 / 11.0050 / 0.0000 |
| FL_kfe_joint | 17.795 @ 0.0735 | 8.895 | 0.1455 / 0.1020 | 21.629 | 2.1050 | 4.6400 / 0.0000 / 13.8730 / 0.0000 |
| FR_haa_joint | 20.790 @ 1.5910 | 0.806 | 0.2240 / 0.0195 | 4.616 | 0.9835 | 0.0000 / 0.0000 / 10.4610 / 0.0000 |
| FR_hfe_joint | 29.856 @ 0.3330 | -4.302 | 0.2895 / 0.0890 | 20.388 | 8.1255 | 14.1325 / 0.0020 / 10.9330 / 0.0000 |
| FR_kfe_joint | 23.028 @ 1.1360 | -3.401 | 0.2055 / 0.0210 | 14.408 | 2.0850 | 3.4825 / 0.0000 / 14.0060 / 0.0000 |
| RL_haa_joint | 26.273 @ 0.6810 | 0.007 | 0.4495 / 0.0335 | 4.964 | 1.2240 | 0.0000 / 0.0000 / 5.9850 / 0.0000 |
| RL_hfe_joint | 29.899 @ 0.2935 | -4.434 | 0.1950 / 0.0465 | 17.166 | 4.0180 | 6.2150 / 0.0000 / 11.8495 / 0.0000 |
| RL_kfe_joint | 24.704 @ 1.1360 | -3.721 | 0.1245 / 0.0270 | 20.921 | 2.3125 | 7.7300 / 0.0000 / 12.5390 / 0.0000 |
| RR_haa_joint | 13.975 @ 0.0010 | 0.000 | 0.3935 / 0.0130 | 3.194 | 1.1665 | 0.0000 / 0.0000 / 5.8875 / 0.0000 |
| RR_hfe_joint | 35.119 @ 0.0010 | 0.000 | 0.1615 / 0.0875 | 6.654 | 4.2080 | 6.8900 / 0.0000 / 11.7835 / 0.0000 |
| RR_kfe_joint | 22.068 @ 0.0610 | 4.552 | 0.1320 / 0.1165 | 14.518 | 2.6030 | 8.9975 / 0.0000 / 12.8870 / 0.0000 |

## trot 0.40 m/s, yaw 0.30 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 2.082m, lateral drift 5.845m; safety latch False, clamp events 0, clamped cycles 0.0%, fault mask 0.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 12.38 / 5.24 | 12.38 / 5.24 | 2.24 | 65 | 20.5 / 30.2 |
| FL_hfe_joint | 8.96 / 2.54 | 8.96 / 2.54 | 5.09 | 32 | 20.1 / 22.4 |
| FL_kfe_joint | 19.30 / 7.17 | 10.16 / 3.78 | 10.99 | 47 | 20.3 / 25.3 |
| FR_haa_joint | 6.67 / 2.81 | 6.67 / 2.81 | 1.47 | 35 | 20.1 / 23.0 |
| FR_hfe_joint | 10.01 / 3.45 | 10.01 / 3.45 | 4.62 | 43 | 20.2 / 24.4 |
| FR_kfe_joint | 13.04 / 8.48 | 6.86 / 4.46 | 10.91 | 56 | 20.4 / 27.4 |
| RL_haa_joint | 7.62 / 2.16 | 7.62 / 2.16 | 1.61 | 27 | 20.1 / 21.7 |
| RL_hfe_joint | 9.63 / 2.65 | 9.63 / 2.65 | 4.04 | 33 | 20.1 / 22.6 |
| RL_kfe_joint | 16.13 / 8.59 | 8.49 / 4.52 | 12.35 | 57 | 20.4 / 27.6 |
| RR_haa_joint | 13.42 / 4.64 | 13.42 / 4.64 | 2.31 | 58 | 20.4 / 28.0 |
| RR_hfe_joint | 11.15 / 3.71 | 11.15 / 3.71 | 5.57 | 46 | 20.3 / 25.1 |
| RR_kfe_joint | 23.00 / 9.38 | 12.11 / 4.93 | 12.18 | 62 | 20.4 / 29.1 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 12.376 @ 0.1285 | -0.024 | 0.1565 / 0.1560 | 0.344 | 3.3455 | 0.0000 / 0.0000 / 13.4070 / 0.0000 |
| FL_hfe_joint | 8.955 @ 0.4535 | -1.148 | 0.0000 / 0.0000 | 0.000 | 0.0750 | 0.0000 / 0.0000 / 8.6740 / 0.0000 |
| FL_kfe_joint | 10.155 @ 0.0130 | 0.777 | 0.0000 / 0.0000 | 0.000 | 0.0255 | 0.0000 / 0.0000 / 9.8650 / 0.0000 |
| FR_haa_joint | 6.671 @ 12.5105 | -0.316 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 6.2940 / 0.0000 |
| FR_hfe_joint | 10.009 @ 0.6810 | -1.059 | 0.0000 / 0.0000 | 0.000 | 0.1945 | 0.0000 / 0.0000 / 12.7345 / 0.0000 |
| FR_kfe_joint | 6.864 @ 12.6385 | -0.006 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 12.0235 / 0.0000 |
| RL_haa_joint | 7.623 @ 0.2260 | -0.906 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 7.0780 / 0.0000 |
| RL_hfe_joint | 9.635 @ 1.5910 | -1.199 | 0.0000 / 0.0000 | 0.000 | 0.1150 | 0.0000 / 0.0000 / 10.5715 / 0.0000 |
| RL_kfe_joint | 8.489 @ 0.2260 | 3.852 | 0.0000 / 0.0000 | 0.000 | 0.0025 | 0.0000 / 0.0000 / 12.8840 / 0.0000 |
| RR_haa_joint | 13.421 @ 0.0010 | 0.000 | 0.0030 / 0.0030 | 1.002 | 0.2525 | 0.0000 / 0.0000 / 19.2655 / 0.0000 |
| RR_hfe_joint | 11.145 @ 1.8185 | -1.015 | 0.0060 / 0.0005 | 1.155 | 1.7475 | 0.0000 / 0.0000 / 14.0555 / 0.0000 |
| RR_kfe_joint | 12.106 @ 0.0155 | 0.356 | 0.0150 / 0.0135 | 2.947 | 0.1550 | 0.0000 / 0.0000 / 10.2260 / 0.0000 |

## trot 0.50 m/s, yaw 0.30 rad/s

Duration 20.0s, RMS warmup 1.0s; travel 1.174m, lateral drift 7.014m; safety latch False, clamp events 0, clamped cycles 0.0%, fault mask 0.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 12.70 / 4.92 | 12.70 / 4.92 | 2.15 | 62 | 20.5 / 29.0 |
| FL_hfe_joint | 10.91 / 2.88 | 10.91 / 2.88 | 6.17 | 36 | 20.2 / 23.1 |
| FL_kfe_joint | 19.67 / 7.38 | 10.35 / 3.89 | 11.70 | 49 | 20.3 / 25.6 |
| FR_haa_joint | 6.85 / 2.88 | 6.85 / 2.88 | 1.58 | 36 | 20.1 / 23.1 |
| FR_hfe_joint | 11.76 / 3.82 | 11.76 / 3.82 | 5.49 | 48 | 20.3 / 25.5 |
| FR_kfe_joint | 13.27 / 8.55 | 6.98 / 4.50 | 11.66 | 56 | 20.4 / 27.6 |
| RL_haa_joint | 7.47 / 2.20 | 7.47 / 2.20 | 1.68 | 28 | 20.1 / 21.8 |
| RL_hfe_joint | 12.26 / 3.22 | 12.26 / 3.22 | 4.67 | 40 | 20.2 / 23.9 |
| RL_kfe_joint | 15.24 / 8.65 | 8.02 / 4.55 | 12.95 | 57 | 20.4 / 27.7 |
| RR_haa_joint | 13.69 / 4.18 | 13.69 / 4.18 | 2.26 | 52 | 20.3 / 26.5 |
| RR_hfe_joint | 13.52 / 4.12 | 13.52 / 4.12 | 6.57 | 52 | 20.3 / 26.4 |
| RR_kfe_joint | 24.02 / 9.52 | 12.64 / 5.01 | 13.35 | 63 | 20.5 / 29.4 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time s | Speed at peak rad/s | >11 Nm total / longest s | Max speed while >11 rad/s | >8 Nm total s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 12.700 @ 0.1260 | 0.009 | 0.1440 / 0.1440 | 0.365 | 2.2725 | 0.0000 / 0.0000 / 13.6915 / 0.0000 |
| FL_hfe_joint | 10.915 @ 0.4535 | -1.183 | 0.0000 / 0.0000 | 0.000 | 0.2335 | 0.0000 / 0.0000 / 8.8885 / 0.0000 |
| FL_kfe_joint | 10.352 @ 0.0130 | 0.759 | 0.0000 / 0.0000 | 0.000 | 0.0280 | 0.0000 / 0.0000 / 9.6755 / 0.0000 |
| FR_haa_joint | 6.848 @ 2.5105 | -0.388 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 8.4970 / 0.0000 |
| FR_hfe_joint | 11.758 @ 0.6810 | -1.060 | 0.0435 / 0.0015 | 1.060 | 0.8050 | 0.0000 / 0.0000 / 12.3080 / 0.0000 |
| FR_kfe_joint | 6.983 @ 12.6410 | 0.045 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 11.0180 / 0.0000 |
| RL_haa_joint | 7.471 @ 0.2260 | -0.966 | 0.0000 / 0.0000 | 0.000 | 0.0000 | 0.0000 / 0.0000 / 7.2450 / 0.0000 |
| RL_hfe_joint | 12.265 @ 1.5910 | -1.152 | 0.0685 / 0.0025 | 1.242 | 0.6940 | 0.0000 / 0.0000 / 10.3810 / 0.0000 |
| RL_kfe_joint | 8.023 @ 0.2260 | 4.445 | 0.0000 / 0.0000 | 0.000 | 0.0005 | 0.0000 / 0.0000 / 12.1500 / 0.0000 |
| RR_haa_joint | 13.695 @ 0.0010 | 0.000 | 0.0030 / 0.0030 | 1.036 | 0.2555 | 0.0000 / 0.0000 / 19.3990 / 0.0000 |
| RR_hfe_joint | 13.523 @ 12.7260 | -1.067 | 0.1925 / 0.0045 | 1.396 | 2.1500 | 0.0000 / 0.0000 / 13.2820 / 0.0000 |
| RR_kfe_joint | 12.642 @ 0.0130 | 0.888 | 0.0205 / 0.0170 | 3.985 | 0.1940 | 0.0000 / 0.0000 / 10.4735 / 0.0000 |
