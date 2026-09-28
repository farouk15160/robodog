# Official Go2 model: shared-controller benchmark

This uses the official Unitree Go2 MJCF with robodog's gait generator, not Unitree's factory walking controller. The floating base is never forced. Model details and limitations are in [the source audit](go2_benchmark_sources.md). The separate [official viewer/SDK smoke test](go2_runner_smoke.md) records its successful runtime and unresolved shutdown segmentation fault.

Official revision: `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`. Model mass: **15.206408 kg**. MuJoCo 3.13.0.

Common environment: obstacle-free plane, gravity 9.81 m/s², 0.5 ms physics / 2.5 ms control, implicitfast, elliptic cone, impratio 10. Foot friction is 1.0 / 0.05 / 0.002, condim 6; floor friction is 0.9 / 0.02 / 0.001, condim 3. Foot priority 2, solref/solimp and zero foot/floor margins match robodog. These contact/solver overrides replace the official defaults solely for comparison.

The official link inertias, collisions, damping, armature and actuator torque limits remain unchanged. Official hip outputs clip at 23.7 N·m, knees at 45.43 N·m. This model supplies no calibrated continuous torque, thermal or torque-speed envelope; none is invented here. Its ideal torque envelope differs from robodog's RS06 model and safety limiter. The 8/11 N·m thresholds in JSON are comparison thresholds, not Go2 continuous ratings.

The velocity step starts immediately from a standing IK pose with 2 mm ground clearance. Exact floating-base feedback is available to both simulations.

Applied generalized joint torque (`qfrc_actuator`) is sampled every physics step, paired with pre-step joint velocity. Peaks include startup; RMS omits the warmup specified below. No external knee transmission is added to the Go2 model.

| Command m/s | Actual m/s | Max tilt ° | Min height m | Stable / tracks |
|---:|---:|---:|---:|---|
| 0.40 | 0.393 | 5.0 | 0.322 | True / True |
| 0.50 | 0.492 | 6.1 | 0.322 | True / True |

## 0.40 m/s command

Duration 20 s; RMS warmup 1 s; initial height 0.322 m; command delay 1 ms. Trot: 2.2 Hz, duty 0.5, lift 0.04 m, height 0.33 m. Stance PD 90/2; swing PD 60/1.5.

Torque clipping: 0 joint-substeps; target-limit cycles: 0.

| Joint group | Worst joint RMS N·m | Peak joint N·m |
|---|---:|---:|
| hip roll | 3.08 | 9.01 |
| hip pitch | 2.51 | 8.06 |
| knee | 7.95 | 25.59 |

| Joint | RMS N·m | Peak N·m | Speed at peak rad/s | Time above 8 / 11 N·m |
|---|---:|---:|---:|---:|
| FL_haa_joint | 3.06 | 7.80 | 0.17 | 0.000 / 0.000 s |
| FL_hfe_joint | 2.49 | 8.06 | -1.13 | 0.001 / 0.000 s |
| FL_kfe_joint | 7.83 | 20.21 | 0.00 | 9.453 / 8.007 s |
| FR_haa_joint | 3.08 | 5.02 | -0.16 | 0.000 / 0.000 s |
| FR_hfe_joint | 2.51 | 8.03 | -0.95 | 0.001 / 0.000 s |
| FR_kfe_joint | 7.86 | 13.27 | -0.65 | 9.431 / 8.111 s |
| RL_haa_joint | 2.81 | 8.77 | -0.84 | 0.003 / 0.000 s |
| RL_hfe_joint | 1.87 | 6.11 | -0.13 | 0.000 / 0.000 s |
| RL_kfe_joint | 7.95 | 16.64 | 1.15 | 9.335 / 5.340 s |
| RR_haa_joint | 2.80 | 9.01 | -0.00 | 0.003 / 0.000 s |
| RR_hfe_joint | 1.86 | 6.61 | 0.13 | 0.000 / 0.000 s |
| RR_kfe_joint | 7.91 | 25.59 | -0.00 | 9.319 / 5.526 s |

## 0.50 m/s command

Duration 20 s; RMS warmup 1 s; initial height 0.322 m; command delay 1 ms. Trot: 2.2 Hz, duty 0.5, lift 0.04 m, height 0.33 m. Stance PD 90/2; swing PD 60/1.5.

Torque clipping: 0 joint-substeps; target-limit cycles: 0.

| Joint group | Worst joint RMS N·m | Peak joint N·m |
|---|---:|---:|
| hip roll | 2.87 | 9.47 |
| hip pitch | 2.87 | 9.63 |
| knee | 8.02 | 26.94 |

| Joint | RMS N·m | Peak N·m | Speed at peak rad/s | Time above 8 / 11 N·m |
|---|---:|---:|---:|---:|
| FL_haa_joint | 2.85 | 8.04 | 0.24 | 0.011 / 0.000 s |
| FL_hfe_joint | 2.85 | 9.63 | -1.23 | 0.325 / 0.000 s |
| FL_kfe_joint | 7.89 | 20.23 | 0.00 | 9.314 / 8.215 s |
| FR_haa_joint | 2.87 | 4.91 | -0.20 | 0.000 / 0.000 s |
| FR_hfe_joint | 2.87 | 9.63 | -0.99 | 0.276 / 0.000 s |
| FR_kfe_joint | 7.92 | 14.23 | -0.86 | 9.285 / 8.307 s |
| RL_haa_joint | 2.51 | 9.47 | -0.91 | 0.005 / 0.000 s |
| RL_hfe_joint | 2.26 | 7.68 | -0.11 | 0.000 / 0.000 s |
| RL_kfe_joint | 8.02 | 16.37 | 0.09 | 9.233 / 5.437 s |
| RR_haa_joint | 2.49 | 9.40 | -0.00 | 0.004 / 0.000 s |
| RR_hfe_joint | 2.24 | 8.12 | 0.24 | 0.004 / 0.000 s |
| RR_kfe_joint | 7.97 | 26.94 | -0.00 | 9.236 / 5.633 s |

Reproduce after cloning the pinned official repository:

```bash
python3 tools/go2_benchmark.py --repo /tmp/robodog-unitree-mujoco --seconds 20 --speeds 0.4 0.5 --output docs/go2_benchmark
```
