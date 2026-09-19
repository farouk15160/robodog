"""
Standalone MuJoCo viewer for inspecting a generated model.

    ros2 run robodog_sim viewer --world house
    ros2 run robodog_sim viewer --world flat --pose crouch

This does NOT run the control stack: it holds a keyframe so the model itself can
be checked (geometry, contact, mesh placement). To see the robot under control,
launch the stack with `mujoco_viewer:=true`, which opens a passive viewer inside
the backend that is already stepping the physics -- attaching a second viewer to
a running simulation is not possible, MuJoCo state lives in one process.
"""
from __future__ import annotations

import argparse
import os
import sys
import time


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="house", choices=["house", "flat", "robot"])
    ap.add_argument("--pose", default="stand", choices=["zero", "stand", "crouch", "rest"])
    ap.add_argument("--settle", action="store_true",
                    help="let the model fall under gravity with no actuation")
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])

    try:
        import mujoco
        import mujoco.viewer
    except ImportError:
        print("mujoco is not installed: pip install mujoco", file=sys.stderr)
        return 1
    from ament_index_python.packages import get_package_share_directory

    fn = {"house": "robodog_house.xml", "flat": "robodog_scene.xml",
          "robot": "robodog.xml"}[a.world]
    path = os.path.join(get_package_share_directory("robodog_sim"), "models", fn)
    if not os.path.exists(path):
        print(f"{path} is missing -- run: ros2 run robodog_sim generate_models",
              file=sys.stderr)
        return 1

    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    kid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, a.pose)
    if kid >= 0:
        mujoco.mj_resetDataKeyframe(m, d, kid)
    mujoco.mj_forward(m, d)
    print(f"{fn}: nbody={m.nbody} ngeom={m.ngeom} nq={m.nq} mass={m.body_subtreemass[1]:.3f} kg")
    print("close the window to exit")

    with mujoco.viewer.launch_passive(m, d) as v:
        while v.is_running():
            if a.settle:
                mujoco.mj_step(m, d)
            else:
                mujoco.mj_forward(m, d)
                time.sleep(m.opt.timestep)
            v.sync()
    return 0


if __name__ == "__main__":
    sys.exit(main())
