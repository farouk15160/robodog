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
`[ESTIMATE]` in `robstride02.yaml` and `[VERIFY]` / `HW-CHECK` in the camera
and CAN code. If you replace one with a measured value, remove the marker and
say in the commit message how it was measured. If you add a new one, mark it.

## Tests

```bash
cd ros2_ws
colcon build --symlink-install && source install/setup.bash
MUJOCO_GL=egl python3 -m pytest src/*/test -q
```

All 217 must pass. `test_trot_is_thermally_sustainable` was an expected failure
for most of this project, at 173 % of the continuous torque rating; it now
reads 90 %, and it did not get there by being tuned — see the locomotion
section of the README. New behaviour needs a test; the useful ones here are
cross-checks rather than unit tests — the MuJoCo model is asserted against the
URDF, the world geometry against the clearances the docstring claims, the CAN
codec against the datasheet. Several tests exist because a bug got past a
weaker one, and each of those says so in its docstring. Please keep that habit:
a test that records *why* it exists is worth several that do not.

## Style

Follow what is already there. Comments explain **why**, not what — if a line
needs a comment saying what it does, the line is usually the problem. Prefer a
short paragraph at the top of a module explaining the design decision over
scattered inline notes.

## Where help is most wanted

1. **Locomotion.** Standing is solid and the balance layer holds attitude well,
   but the robot does not yet track commanded velocity (see §6.6 of the
   documentation for the measured numbers). This needs contact detection rather
   than a fixed gait schedule, slip handling, and co-tuning of stance
   compliance against swing timing. The force-to-motion path is verified
   correct in isolation, so this is tuning, not a sign error.
2. **The `ros2_control` C++ `SystemInterface`.** The interface contract is
   already declared in `urdf/ros2_control.xacro`; the runtime is Python.
3. **Hardware validation.** If you have ROBSTRIDE02 actuators, the single most
   valuable contribution is confirming or correcting the CAN frame layout in
   `robodog_hardware/protocol/robstride02.py` against your firmware revision.
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

This repository can drive real actuators capable of 17 N·m. Anything touching
`robodog_control/safety.py`, `robodog_hardware/protocol/`, or the joint limits
gets extra scrutiny and needs a test demonstrating the envelope still holds.
Please do not weaken a limit to make a gait work.
