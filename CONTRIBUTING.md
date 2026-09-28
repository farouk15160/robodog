# Contributing

Thanks for looking. This is a young project and the most useful contributions
right now are the ones listed under *Where help is most wanted* below.

## Ground rules

**The model is generated. Do not hand-edit it.** `robot_parameters.yaml`, the
meshes under `robodog_description/meshes/`, the MuJoCo models, the figures and
`docs/generated_facts.tex` are all build products. If a number is wrong, fix
the tool that produces it — `tools/cad_to_model.py`, `tools/prepare_meshes.py`,
`robodog_sim/mjcf.py`, `tools/make_diagrams.py` — and regenerate. A hand edit
will be silently overwritten and, worse, will make the URDF and the MuJoCo
model disagree.

**Keep the hardware boundary clean.** Nothing above `JointBackend` or
`CameraBackend` may import a simulator or a driver, check which backend is
active, or branch on `is_simulation` to change control behaviour. If you find
yourself wanting to, that usually means something belongs in the backend.

**Say what is estimated.** Several constants are engineering estimates, marked
`[ESTIMATE]` in the selected `robstride06.yaml` and `[VERIFY]` / `HW-CHECK` in the camera
and CAN code. If you replace one with a measured value, remove the marker and
say in the commit message how it was measured. If you add a new one, mark it.

## Tests

```bash
cd ros2_ws
colcon build --symlink-install && source install/setup.bash
MUJOCO_GL=egl python3 -m pytest src/*/test -q
```

Run the complete regression suite; do not reuse the old RS02 test count or
thermal percentages as current evidence. New behavior needs tests that check
physical invariants and observable behavior: URDF/MJCF consistency, transmission
mapping, per-motor torque limits, actual travel and telemetry. Run gait checks
with the same gains, actuator configuration and safety layer as the controller.

## Style

Follow what is already there. Comments explain **why**, not what — if a line
needs a comment saying what it does, the line is usually the problem. Prefer a
short paragraph at the top of a module explaining the design decision over
scattered inline notes.

## Where help is most wanted

1. **Locomotion.** Validate the new 19.72 kg RS06 model over terrain and turns,
   including contact detection, slip handling and motor-side RMS loads. Historic
   RS02 straight-line results do not establish current performance.
2. **The `ros2_control` C++ `SystemInterface`.** The interface contract is
   already declared in `urdf/ros2_control.xacro`; the runtime is Python.
3. **Hardware validation.** If you have RobStride RS06 actuators, the single most
   valuable contribution is confirming or correcting the CAN frame layout in
   `robodog_hardware/protocol/robstride06.py` against your firmware revision.
4. **A NUWA HP60C datasheet.** The camera parameters are plausible placeholders
   and every one of them is marked.
5. **State estimation.** The current estimator is a complementary filter plus
   leg odometry, chosen because an EKF's covariances would be invented numbers
   until the robot exists. Once it does, that reasoning expires.

## Commits and pull requests

Small, focused commits with a message that explains the reasoning, not just the
change. Reference the affected package. If a change alters a documented number,
regenerate the docs in the same commit so the two never disagree.

## Safety

This repository can drive real actuators capable of 36 N·m at the motor output, with an
additional 2:1 knee reduction. Anything touching
`robodog_control/safety.py`, `robodog_hardware/protocol/`, or the joint limits
gets extra scrutiny and needs a test demonstrating the envelope still holds.
Please do not weaken a limit to make a gait work.

The September RS06 specification distinguishes 8 N·m continuous stall from
11 N·m rotating rated torque at 100 rpm with a 200 × 200 mm heat sink. The
controller uses the 8 N·m holding reference. Published output inertia and line
resistance replace the former estimates; thermal impedance remains uncalibrated.
See [the versioned audit](docs/rs06_datasheet_audit.md) before changing ratings.
Simulation temperatures are estimates. Real IMU/base feedback is unavailable,
and travel commands are blocked until that integration supplies live feedback.
Hardware startup defaults to disabled; RS06 enable and motion require physically
commissioned joints marked `calibrated: true`.
