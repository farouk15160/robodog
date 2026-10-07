# RS06 configuration: simulation feasibility — baseline before heading hold

Working mass: **19.719 kg**. Knee reduction: motor output turns twice per knee turn; 95% belt efficiency assumed. Hips are direct drive.

All runs use the configured 400 Hz controller gains, motor speed/torque limits and SafetyMonitor. They start at the standing keyframe on an obstacle-free plane. Applied torque is read from MuJoCo qfrc_actuator at every 0.5 ms physics step, after command limits and transmission gears. Peaks and overload durations include startup; RMS excludes the stated warmup. Commands are steps from standing, not speed ramps. The adjacent XML records the evaluated model; the GUI's proving ground has a barrier at 9 m and is unsuitable for long speed runs.

| Gait | Command m/s / yaw rad/s | Actual m/s | Max tilt ° | Min height m | Worst motor RMS / peak N·m | Stall-reference % | Safety-clamped cycles | Stable / tracks / margin pass |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| walk | 1.00 / 0.00 | -0.011 | 25.3 | 0.144 | 7.67 / 31.63 | 96 | 99.8% | False / False / False |
| trot | 1.00 / 0.00 | 0.984 | 8.8 | 0.322 | 5.36 / 28.15 | 67 | 30.6% | True / True / False |

## Interpretation

This is a flat-ground simulation result, not hardware approval. Continuous utilisation uses the September specification's 8 N·m continuous stall rating. The 11 N·m rating at 100 rpm requires a 200 × 200 mm aluminum heatsink; the central electronics plate does not establish equivalent cooling for 12 motors. Output inertia and line resistance use the September specification; phase resistance uses an equivalent-wye conversion. Thermal impedance, friction and belt efficiency remain assumptions. The I²t limiter is a conservative software policy, not a reproduction of the manufacturer's overload algorithm. Temperatures and steady-state extrapolations below cannot verify overheating margins: the lumped model omits per-phase stall hotspots, resistance changes with temperature and high-current torque nonlinearity. Belt compliance, pulley strength, motor mounting fit, bus power/regeneration, battery sag and real sensor feedback require validation. Hardware travel is blocked until live IMU/base-state estimation is integrated. See [the datasheet audit](rs06_datasheet_audit.md) for ratings, current conventions and conflicting temperature thresholds.

Assumed bus voltage: 44.4 V (two 6S packs in series). The speed envelope is a voltage-scaled approximation. A 6S parallel arrangement requires a separate evaluation.

**Rejected operating point(s):** walk 1.00 m/s. A completed simulation is not a successful gait when it falls or misses the command.

**Active limiting:** safety changed commands during walk 99.8%, trot 30.6% of control cycles. Treat RMS together with peak, overload and clipping data.

**Motor-margin verdict: FAIL.** walk (stability criteria failed, velocity tracking criteria failed, safety limiting exceeded 1.0% of control cycles); trot (safety limiting exceeded 1.0% of control cycles). Stable motion and speed tracking alone do not establish comfortable actuator margin.

**World-path drift:** trot accumulated 24.405 m maximum lateral displacement. Its body-frame speed tracking result does not establish straight-line navigation accuracy.

## Reproduce

`/usr/bin/python3 tools/torque_report.py --gaits walk trot --velocity 1.0 --seconds 100 --warmup 5 --output docs/rs06_1ms_100s_baseline.json`

## walk 1.00 m/s, yaw 0.00 rad/s

Duration 100.0s, RMS warmup 5.0s; travel -1.026m, lateral drift 0.321m; safety latch False, clamp events 79767, clamped cycles 99.8%, fault mask 7.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 21.82 / 5.06 | 21.82 / 5.06 | 5.95 | 63 | 22.1 / 29.6 |
| FL_hfe_joint | 27.96 / 7.67 | 27.96 / 7.67 | 12.97 | 96 | 24.8 / 42.0 |
| FL_kfe_joint | 22.20 / 7.35 | 11.68 / 3.87 | 32.27 | 48 | 21.2 / 25.6 |
| FR_haa_joint | 23.78 / 4.99 | 23.78 / 4.99 | 6.81 | 62 | 22.0 / 29.3 |
| FR_hfe_joint | 31.63 / 7.67 | 31.63 / 7.67 | 16.71 | 96 | 24.8 / 42.0 |
| FR_kfe_joint | 22.24 / 7.44 | 11.70 / 3.92 | 31.26 | 49 | 21.3 / 25.7 |
| RL_haa_joint | 23.85 / 3.63 | 23.85 / 3.63 | 6.64 | 45 | 21.1 / 24.9 |
| RL_hfe_joint | 27.75 / 6.80 | 27.75 / 6.80 | 13.97 | 85 | 23.8 / 37.3 |
| RL_kfe_joint | 32.00 / 11.12 | 16.84 / 5.85 | 30.18 | 73 | 22.8 / 32.8 |
| RR_haa_joint | 19.47 / 3.61 | 19.47 / 3.61 | 4.64 | 45 | 21.1 / 24.9 |
| RR_hfe_joint | 28.56 / 6.78 | 28.56 / 6.78 | 19.02 | 85 | 23.8 / 37.2 |
| RR_kfe_joint | 31.91 / 11.11 | 16.80 / 5.85 | 34.11 | 73 | 22.8 / 32.8 |

**Failed operating point:** excessive tilt; body height below limit. Loads include the unstable/fallen portion; they do not describe successful locomotion.

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time / travel | World xyz at peak m | Speed at peak rad/s | >8 / >11 / >36 Nm total s | >11 Nm longest s | Max speed while >11 rad/s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 21.818 @ 1.3335 s / 0.763 m | [0.702, 0.102, 0.161] | -0.731 | 7.6835 / 1.7705 / 0.0000 | 0.0745 | 4.557 | 0.0000 / 0.0000 / 73.6545 / 0.0000 |
| FL_hfe_joint | 27.956 @ 0.2435 s / 0.125 m | [0.121, 0.031, 0.365] | -4.122 | 51.8135 / 0.4290 / 0.0000 | 0.0920 | 7.936 | 94.3825 / 0.0000 / 42.3295 / 0.0000 |
| FL_kfe_joint | 11.684 @ 63.4985 s / 7.033 m | [-0.432, 0.156, 0.158] | 14.110 | 5.7945 / 0.0375 / 0.0000 | 0.0040 | 15.548 | 0.0000 / 0.0000 / 75.1040 / 0.0000 |
| FR_haa_joint | 23.779 @ 1.6660 s / 0.812 m | [0.692, 0.088, 0.153] | 0.325 | 7.4105 / 1.6415 / 0.0000 | 0.0195 | 4.712 | 0.0000 / 0.0000 / 75.0390 / 0.0000 |
| FR_hfe_joint | 31.626 @ 0.3335 s / 0.203 m | [0.196, 0.050, 0.387] | -3.161 | 51.2850 / 0.4230 / 0.0000 | 0.1885 | 12.918 | 94.3325 / 0.0000 / 42.4995 / 0.0000 |
| FR_kfe_joint | 11.704 @ 0.2960 s / 0.171 m | [0.166, 0.042, 0.386] | -5.256 | 6.5180 / 0.0435 / 0.0000 | 0.0095 | 15.977 | 0.0000 / 0.0000 / 75.1230 / 0.0000 |
| RL_haa_joint | 23.848 @ 1.1860 s / 0.743 m | [0.691, 0.089, 0.148] | 1.017 | 8.9625 / 1.1075 / 0.0000 | 0.0555 | 4.951 | 0.0000 / 0.0000 / 59.0680 / 0.0000 |
| RL_hfe_joint | 27.755 @ 0.4785 s / 0.332 m | [0.324, 0.060, 0.342] | -4.486 | 40.3905 / 0.2265 / 0.0000 | 0.1490 | 11.765 | 57.8600 / 0.0000 / 48.8135 / 0.0000 |
| RL_kfe_joint | 16.840 @ 13.3335 s / 1.992 m | [0.450, 0.094, 0.155] | -0.230 | 11.9860 / 0.6105 / 0.0000 | 0.0090 | 13.114 | 11.6250 / 0.0000 / 52.5850 / 0.0000 |
| RR_haa_joint | 19.470 @ 1.5185 s / 0.788 m | [0.686, 0.110, 0.153] | -0.378 | 8.4825 / 1.1485 / 0.0000 | 0.0560 | 2.519 | 0.0000 / 0.0000 / 56.8225 / 0.0000 |
| RR_hfe_joint | 28.562 @ 0.3535 s / 0.220 m | [0.213, 0.053, 0.383] | 2.094 | 39.7980 / 0.1975 / 0.0000 | 0.0535 | 17.427 | 57.1950 / 0.0000 / 48.9770 / 0.0000 |
| RR_kfe_joint | 16.797 @ 4.3335 s / 1.082 m | [0.637, 0.083, 0.154] | 0.005 | 12.1890 / 0.6755 / 0.0000 | 0.1705 | 18.883 | 12.6925 / 0.0000 / 52.3740 / 0.0000 |

## trot 1.00 m/s, yaw 0.00 rad/s

Duration 100.0s, RMS warmup 5.0s; travel 94.468m, lateral drift 24.405m; safety latch False, clamp events 12243, clamped cycles 30.6%, fault mask 6.

| Joint | Joint peak / RMS N·m | Motor peak / RMS N·m | Motor peak rad/s | Continuous % | Estimated final / steady °C |
|---|---:|---:|---:|---:|---:|
| FL_haa_joint | 9.65 / 3.26 | 9.65 / 3.26 | 2.71 | 41 | 20.9 / 24.0 |
| FL_hfe_joint | 20.82 / 5.31 | 20.82 / 5.31 | 13.05 | 66 | 22.3 / 30.5 |
| FL_kfe_joint | 24.98 / 8.68 | 13.15 / 4.57 | 28.19 | 57 | 21.7 / 27.8 |
| FR_haa_joint | 8.28 / 3.27 | 8.28 / 3.27 | 2.58 | 41 | 20.9 / 24.0 |
| FR_hfe_joint | 22.38 / 5.36 | 22.38 / 5.36 | 10.24 | 67 | 22.3 / 30.7 |
| FR_kfe_joint | 17.48 / 8.71 | 9.20 / 4.58 | 18.19 | 57 | 21.7 / 27.8 |
| RL_haa_joint | 13.89 / 2.36 | 13.89 / 2.36 | 4.41 | 30 | 20.5 / 22.1 |
| RL_hfe_joint | 28.15 / 5.09 | 28.15 / 5.09 | 9.66 | 64 | 22.1 / 29.7 |
| RL_kfe_joint | 18.92 / 10.19 | 9.96 / 5.36 | 22.03 | 67 | 22.3 / 30.7 |
| RR_haa_joint | 11.22 / 2.29 | 11.22 / 2.29 | 3.30 | 29 | 20.4 / 22.0 |
| RR_hfe_joint | 27.30 / 5.07 | 27.30 / 5.07 | 13.54 | 63 | 22.1 / 29.6 |
| RR_kfe_joint | 29.36 / 9.89 | 15.45 / 5.21 | 29.71 | 65 | 22.2 / 30.1 |

Applied motor-output torque at the physics rate. Time above 11 Nm is an overload comparison, not a substitute for the separate 8 Nm holding rating. The speed-envelope column counts the model's continuous motoring-torque reduction with speed, including below the speed ceiling; it is not a count of overspeed faults.

| Joint | Peak Nm @ time / travel | World xyz at peak m | Speed at peak rad/s | >8 / >11 / >36 Nm total s | >11 Nm longest s | Max speed while >11 rad/s | Safety / peak / speed-envelope / physics clipping s |
|---|---:|---:|---:|---:|---:|---:|---:|
| FL_haa_joint | 9.646 @ 0.0980 s / 0.023 m | [0.023, 0.006, 0.331] | 0.398 | 0.0945 / 0.0000 / 0.0000 | 0.0000 | 0.000 | 0.0000 / 0.0000 / 45.6630 / 0.0000 |
| FL_hfe_joint | 20.821 @ 1.3635 s / 1.229 m | [1.213, 0.097, 0.330] | -0.895 | 14.9805 / 5.7385 / 0.0000 | 0.1055 | 4.678 | 0.0000 / 0.0000 / 51.3105 / 0.0000 |
| FL_kfe_joint | 13.150 @ 0.2610 s / 0.151 m | [0.142, 0.051, 0.381] | 0.052 | 0.3475 / 0.0285 / 0.0000 | 0.0140 | 7.218 | 0.0000 / 0.0000 / 49.2670 / 0.0000 |
| FR_haa_joint | 8.282 @ 0.9080 s / 0.777 m | [0.762, 0.069, 0.339] | 2.035 | 0.0110 / 0.0000 / 0.0000 | 0.0000 | 0.000 | 0.0000 / 0.0000 / 46.6705 / 0.0000 |
| FR_hfe_joint | 22.375 @ 1.5910 s / 1.449 m | [1.431, 0.120, 0.334] | -0.827 | 15.0735 / 5.7975 / 0.0000 | 0.0645 | 4.841 | 0.0000 / 0.0000 / 51.3590 / 0.0000 |
| FR_kfe_joint | 9.198 @ 0.4035 s / 0.293 m | [0.280, 0.084, 0.344] | -4.886 | 0.0390 / 0.0000 / 0.0000 | 0.0000 | 0.000 | 0.0000 / 0.0000 / 49.5710 / 0.0000 |
| RL_haa_joint | 13.889 @ 0.4480 s / 0.339 m | [0.326, 0.089, 0.339] | -1.371 | 0.3595 / 0.0225 / 0.0000 | 0.0225 | 1.451 | 0.0000 / 0.0000 / 51.0145 / 0.0000 |
| RL_hfe_joint | 28.149 @ 0.6810 s / 0.571 m | [0.557, 0.087, 0.326] | -0.293 | 17.6485 / 3.6620 / 0.0000 | 0.0560 | 6.023 | 16.0325 / 0.0000 / 56.6270 / 0.0000 |
| RL_kfe_joint | 9.959 @ 0.4110 s / 0.301 m | [0.287, 0.085, 0.342] | 2.404 | 12.6585 / 0.0000 / 0.0000 | 0.0000 | 0.000 | 0.0000 / 0.0000 / 51.0215 / 0.0000 |
| RR_haa_joint | 11.222 @ 0.0010 s / 0.000 m | [0.000, -0.000, 0.322] | 0.000 | 0.1920 / 0.0005 / 0.0000 | 0.0005 | 0.000 | 0.0000 / 0.0000 / 51.2495 / 0.0000 |
| RR_hfe_joint | 27.295 @ 0.4535 s / 0.345 m | [0.331, 0.089, 0.340] | -0.066 | 16.5245 / 4.4665 / 0.0000 | 0.0740 | 6.324 | 14.5750 / 0.0000 / 53.9245 / 0.0000 |
| RR_kfe_joint | 15.453 @ 0.0155 s / 0.000 m | [0.000, 0.000, 0.322] | 0.264 | 4.2985 / 0.2395 / 0.0000 | 0.2090 | 17.794 | 0.0000 / 0.0000 / 50.6185 / 0.0000 |
