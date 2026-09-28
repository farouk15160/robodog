# Official Go2 benchmark sources and comparison limits

Checked 2026-09-28. The official Go2 MuJoCo model supports a reproducible comparison of **simulated joint torque demand**. It does not include Unitree's factory walking controller or a motor thermal model, so a custom gait running on this model must not be described as measured Go2 hardware performance or factory gait performance.

## Source revisions

| Source | Pinned revision | Role |
|---|---|---|
| [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco/tree/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d) | `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d` | Official Go2 MJCF, meshes, simulator and low-level examples |
| [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym/tree/276801e46c5d433564f24658bac64f254b7d2d4b) | `276801e46c5d433564f24658bac64f254b7d2d4b` | Go2 RL training configuration; no bundled Go2 deployment policy in inspected tree |
| [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab/tree/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3) | `4960b84732b0c2ec593dccbfe963fda1bcd7b1e3` | Go2 training and deployment scaffold; requires a separately trained Go2 policy |

The checked Go2 source is `unitree_robots/go2/go2.xml`, SHA-256 `2014a3d76e30f17ab9447d8a67bd015291f74fa4d71ae30d005f1a32bd693d4b`. The upstream `unitree_mujoco` [LICENSE](https://github.com/unitreerobotics/unitree_mujoco/blob/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/LICENSE) is BSD-3-Clause and must accompany redistributed source/meshes from that repository; preserve the copyright and disclaimer. Its SHA-256 is `a5d73fc4aca9074e3e6fe0b1a0ba763cf9514b2249b7390ed20fe8d53630bf25`. Do not imply Unitree endorsement.

`unitree_rl_gym` also includes a [BSD-3-Clause license](https://github.com/unitreerobotics/unitree_rl_gym/blob/276801e46c5d433564f24658bac64f254b7d2d4b/LICENSE). `unitree_rl_lab` advertises Apache 2.0 in its README badge, but the inspected tree has no top-level LICENSE file; this audit only references its available controller structure and does not vendor its code.

## Official model properties

Values below come from the pinned [Go2 MJCF](https://github.com/unitreerobotics/unitree_mujoco/blob/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/go2/go2.xml). A read-only compilation using local MuJoCo **3.13.0** confirmed the mass, actuator gear and inherited joint parameters. This is a model inspection, not a locomotion result.

| Property | Value / interpretation |
|---|---|
| Total model mass | **15.206408 kg**, sum of explicit body masses |
| Base mass | 6.921 kg |
| Each hip / thigh / calf body | 0.678 / 1.152 / 0.241352 kg |
| Root | Floating base; `nq=19`, `nv=18`, twelve actuators |
| HAA output control limit | **±23.7 N·m** |
| HFE output control limit | **±23.7 N·m** |
| KFE output control limit | **±45.43 N·m** |
| Actuator transmission gear | 1 for every joint; these are joint-output torques, not rotor torques |
| Joint armature | 0.01 kg·m² at every joint |
| Joint damping | 0.1 N·m·s/rad at every joint |
| Joint friction loss | 0.2 N·m at every joint |
| Upper / lower leg offsets | 0.213 / 0.213 m |
| Lateral hip offset | ±0.0955 m |
| Body-to-hip origins | x = ±0.1934 m, y = ±0.0465 m |
| Foot collision | Sphere radius 0.022 m; `condim=6`, priority 1 |
| Foot friction | Sliding 0.4, torsional 0.02 m, rolling 0.01 m |
| Model contact cone / impedance ratio | Elliptic / 100 |
| Unmodified compiled timestep / integrator / solver | 0.002 s / Euler / Newton; benchmark should explicitly select common settings |

The model does not provide continuous thermal torque limits, winding resistance, torque constants, motor temperature dynamics or a torque–speed/voltage envelope. Its constant control caps cannot supply those missing ratings.

Body principal inertias are explicit, with separate orientations and centers of mass. For the base, principal moments are `[0.107027, 0.0980771, 0.0244531]` kg·m²; hip `[0.00088403, 0.000596003, 0.000479967]`; thigh `[0.00594973, 0.00584149, 0.000878787]`; calf `[0.0014901, 0.00146356, 0.0000531397]`. Retain their source frames and mirrored poses rather than substituting diagonal moments in joint frames. [Pinned Go2 MJCF.](https://github.com/unitreerobotics/unitree_mujoco/blob/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/go2/go2.xml)

The retail [Go2 product page](https://www.unitree.com/go2/) advertises approximately 15 kg with battery and approximately 45 N·m peak joint torque; its specification table identifies the latter under knee joint parameters. Neither is a statement that every joint continuously delivers 45 N·m. Compare measured simulated HAA/HFE demand with the model's 23.7 N·m cap and KFE demand with its 45.43 N·m cap; label these **fractions of model peak/control caps**, not continuous thermal utilization.

## Model integration pitfalls

- **Ordering differs:** joint positions/velocities traverse legs FL, FR, RL, RR, while actuator and sensor order is FR, FL, RR, RL. Map all channels by joint name, then group `_hip` as HAA, `_thigh` as HFE and `_calf` as KFE.
- **The shipped `scene.xml` is not flat-only:** it contains blocks/stairs starting at x=1.2 m. For a flat benchmark use a scene with only a plane and the source robot. Record the scene modification. [Source scene.](https://github.com/unitreerobotics/unitree_mujoco/blob/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/go2/scene.xml)
- **Floor friction alone is insufficient:** Go2 feet have priority 1, so their contact parameters can override a lower-priority floor. Match the effective foot–ground coefficients, contact dimension and relevant contact parameters, and record them. [MuJoCo contact parameter rules.](https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters)
- **`ctrl` is a request:** record applied joint torque after control clamping, preferably `qfrc_actuator` selected by joint DOF. With unit gear it corresponds to joint-output actuator torque. Do not report an unclamped PD command as actual motor torque. [MuJoCo actuator and gear reference.](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-general-gear)
- **Home control is not a position controller:** the home keyframe includes joint-angle-like `ctrl` values, but actuators are motors. Explicitly initialize the controller and overwrite commands; do not assume loading that keyframe makes the robot hold position.

## Available official controllers

The [unitree_mujoco README](https://github.com/unitreerobotics/unitree_mujoco/blob/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/readme.md) describes support for low-level commands/state. Its examples stand the robot up and lie it down. Receiving simulated `SportModeState` is state reporting, not availability of the closed factory locomotion service.

The pinned [RL Gym Go2 configuration](https://github.com/unitreerobotics/unitree_rl_gym/blob/276801e46c5d433564f24658bac64f254b7d2d4b/legged_gym/envs/go2/go2_config.py) describes a training setup: P control, stiffness 20, damping 0.5, action scale 0.25 and decimation 4. The pinned tree's `deploy/pre_train` and MuJoCo deployment configurations contain G1, H1 and H1_2, not Go2. Training configuration is not a pretrained Go2 controller.

RL Lab has a [Go2 deployment configuration](https://github.com/unitreerobotics/unitree_rl_lab/blob/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3/deploy/robots/go2/config/config.yaml), but its velocity state points to `../../../logs/rsl_rl/unitree_go2_velocity`. The [Go2 RL state implementation](https://github.com/unitreerobotics/unitree_rl_lab/blob/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3/deploy/robots/go2/src/State_RLBase.cpp) requires that directory's `params/deploy.yaml` and `exported/policy.onnx`. Neither Go2 policy artifact is present in the checked tree. Therefore this audit did not identify a bundled ready-to-run official Go2 velocity policy in these repositories.

A shared project gait generator with a Go2 geometry/sign adapter is a valid **custom-controller comparison on the official Go2 model**. It must carry that label. Differences from Robodog then include morphology, mass distribution, joint dynamics and how well the controller is tuned for each model. Matching command velocity and physics engine alone cannot separate these effects.

## Reporting a fair 0.4 / 0.5 m/s comparison

Use a common MuJoCo version, flat surface, effective friction, gravity, physics timestep, solver settings, simulated duration, startup procedure and analysis window. Record actual speed, distance, tilt and fall status alongside torque. A fallen or poorly tracking Go2 run is a failed custom-controller trial, not evidence that the physical product cannot walk.

Report all twelve joints, grouped as HAA/HFE/KFE, with applied torque RMS, absolute peak, speed, saturation time and the exact RMS/peak windows. Preserve per-leg values before any group summary. For Robodog report **joint output torque** for comparison with Go2, and additionally motor output torque before the 2:1 knee belt. Do not compare Robodog's motor-side knee torque directly with Go2's knee-output torque.

Keep natural masses as the primary comparison: 15.206408 kg Go2 model versus the current Robodog mass. An optional equal-mass payload experiment is a separate result with a declared payload location and inertia. RMS demand divided by a peak cap is useful as a numerical margin, but cannot establish continuous motor temperature or hardware feasibility without motor thermal data.
