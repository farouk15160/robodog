"""
Generate every MuJoCo model this package ships.

    ros2 run robodog_sim generate_models
    python3 -m robodog_sim.generate_models --out <dir>

Outputs
    robodog.xml         robot alone, no ground: for inertia/kinematics checks
    robodog_scene.xml   robot on flat ground with the calibration course
    robodog_house.xml   robot in the five-room house

Regenerate after any change to robot_parameters.yaml or to a world in
robodog_sim/worlds/. The models are build artefacts, not source.
"""
from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET

import yaml

from .mjcf import add_assets, add_defaults, add_keyframes, build_robot, pretty
from .world_builder import add_world
from .world_spec import World
from .worlds.house import WORLDS


def _params() -> tuple[dict, dict, str]:
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("robodog_description")
    with open(os.path.join(share, "config", "robot_parameters.yaml")) as f:
        P = yaml.safe_load(f)
    with open(os.path.join(share, "config", "robstride02.yaml")) as f:
        RS = yaml.safe_load(f)
    return P, RS, os.path.join(share, "meshes")


def make_model(params: dict, rs: dict, world: World | None, *, timestep: float,
               meshdir: str, spawn=(0.0, 0.0), yaw=0.0) -> str:
    root = ET.Element("mujoco", {"model": "robodog" + (f"_{world.name}" if world else "")})
    # Pin the model extent so the camera clipping planes resolve to fixed
    # absolute distances regardless of how large the world is.
    extent = max(world.size) / 2.0 if world is not None else 1.0
    add_defaults(root, rs, timestep, meshdir, extent)
    add_assets(root, params)
    ET.SubElement(root, "worldbody")
    # Order matters: the robot's free joint must be the first in qpos so the
    # keyframes line up. See add_keyframes().
    build_robot(root, params, spawn=spawn, yaw=yaw)
    if world is not None:
        add_world(root, world)
    add_keyframes(root, params, world, spawn=spawn, yaw=yaw)
    return pretty(root)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None, help="output directory (default: package models/)")
    ap.add_argument("--timestep", type=float, default=0.0005,
                    help="physics step; 0.0005 s = 2 kHz, 5 substeps per 400 Hz control cycle")
    ap.add_argument("--meshdir", default=None,
                    help="mesh directory baked into the model (default: robodog_description)")
    ap.add_argument("--validate", action="store_true", help="load each model with MuJoCo")
    a = ap.parse_args(argv)

    P, RS, meshdir = _params()
    if a.meshdir:
        meshdir = a.meshdir
    out = a.out
    if out is None:
        here = os.path.dirname(os.path.abspath(__file__))
        out = os.path.abspath(os.path.join(here, "..", "models"))
    os.makedirs(out, exist_ok=True)

    jobs = [("robodog.xml", None, (0.0, 0.0), 0.0)]
    for name, build in WORLDS.items():
        w = build()
        start = next((p for p in w.waypoints if p.name == "start"), None)
        spawn = start.pos if start else (0.0, 0.0)
        yaw = start.yaw if start else 0.0
        fn = "robodog_scene.xml" if name == "flat" else f"robodog_{name}.xml"
        jobs.append((fn, w, spawn, yaw))

    for fn, world, spawn, yaw in jobs:
        xml = make_model(P, RS, world, timestep=a.timestep, meshdir=meshdir,
                         spawn=spawn, yaw=yaw)
        path = os.path.join(out, fn)
        with open(path, "w") as f:
            f.write(xml)
        n = len(world.prims) if world else 0
        print(f"  wrote {fn:22s} {len(xml)/1024:6.1f} KB  world objects: {n}")

    if a.validate:
        import mujoco
        print("\nvalidating with MuJoCo:")
        for fn, *_ in jobs:
            path = os.path.join(out, fn)
            m = mujoco.MjModel.from_xml_path(path)
            d = mujoco.MjData(m)
            mujoco.mj_forward(m, d)
            mass = sum(m.body_mass[1:1 + 14])
            print(f"  {fn:22s} nq={m.nq:3d} nv={m.nv:3d} nbody={m.nbody:3d} "
                  f"ngeom={m.ngeom:4d} nu={m.nu:2d} nsensor={m.nsensor:2d} "
                  f"timestep={m.opt.timestep}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
