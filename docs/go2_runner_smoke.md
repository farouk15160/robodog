# Official Unitree Python runner: local smoke test

The upstream `simulate_python/unitree_mujoco.py` successfully opened its MuJoCo viewer under Xvfb, initialized the official DDS bridge on **loopback only, domain 42**, and advanced **781 steps / 3.905 simulation seconds**. The five-second wall-time run used the upstream Go2 scene and timestep. No hardware or motion commands were sent; this tests local rendering, physics and SDK initialization, not walking.

**Clean shutdown failed:** the process printed its completed-runtime marker, then exited with signal 11 (status 139). Explicitly stopping its three DDS publishing threads and closing its channels did not eliminate the segmentation fault. Its cause remains unresolved; the runner is not counted as a clean pass. The independently executed headless model benchmark exits normally and provides the walking/torque measurements in [go2_benchmark.md](go2_benchmark.md).

Revisions: `unitree_mujoco` `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`; `unitree_sdk2_python` `814556d15970dd2ecf1c9984e845ca02ab07e206`. Environment: Python 3.10.12, MuJoCo 3.13.0, CycloneDDS Python 0.10.2, pygame 2.6.1. Dependencies were installed in `/tmp/robodog-unitree-deps`; SDK source in `/tmp/robodog-unitree-sdk2-python`. No system or ROS dependency versions were replaced.

Reproduce the bounded runtime test after cloning those revisions and installing its dependencies:

```bash
PYGLFW_LIBRARY_VARIANT=x11 \
PYTHONPATH=/tmp/robodog-unitree-deps:/tmp/robodog-unitree-sdk2-python \
xvfb-run -a python3 tools/unitree_runner_smoke.py \
  --repo /tmp/robodog-unitree-mujoco
```

The wrapper disables the joystick requirement, leaves the elastic band disabled, confines DDS to `lo`, closes the viewer after five seconds and joins the two upstream simulation/viewer threads. It does not patch upstream physics or suppress the shutdown failure. Direct viewer startup inside the restricted sandbox could not connect to the virtual display; the recorded runtime test ran with permission outside that restriction.
