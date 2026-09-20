#!/usr/bin/env python3
"""
Render preview images of the robot and the test world for the README.

Off-screen MuJoCo renders, so they are reproducible and stay in step with the
model. Needs a GL context: run with MUJOCO_GL=egl (or osmesa) if the default
backend is unavailable.

Run:  MUJOCO_GL=egl python3 tools/render_previews.py
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS = os.path.join(ROOT, "ros2_ws/src/robodog_sim/models")
OUT = os.path.join(ROOT, "docs/images")

# name, model, keyframe, (lookat xyz), distance, azimuth, elevation, size
SHOTS = [
    ("robot_stand", "robodog_scene.xml", "stand", (0.0, 0.0, 0.17), 1.05, 138, -16, (1500, 950)),
    ("robot_side", "robodog_scene.xml", "stand", (0.0, 0.0, 0.17), 0.95, 90, -6, (1400, 800)),
    ("robot_crouch", "robodog_scene.xml", "crouch", (0.0, 0.0, 0.13), 0.95, 138, -16, (1300, 800)),
    ("robot_rest", "robodog_scene.xml", "rest", (0.0, 0.0, 0.10), 0.90, 138, -14, (1300, 800)),
    ("house_overview", "robodog_house.xml", "stand", (6.0, 4.5, 0.0), 14.0, 90, -80, (1500, 1150)),
    ("proving_ground", "robodog_scene.xml", "stand", (0.0, -0.3, 0.0), 23.0, 90, -80, (1500, 1180)),
    ("terrain_rough", "robodog_scene.xml", "stand", (4.2, 4.2, 0.2), 6.5, 135, -22, (1500, 900)),
    ("terrain_stairs", "robodog_scene.xml", "stand", (0.0, 3.6, 0.3), 6.0, 120, -18, (1500, 900)),
    ("house_living", "robodog_house.xml", "stand", (2.8, 2.0, 0.4), 4.6, 135, -20, (1500, 900)),
    ("house_workshop", "robodog_house.xml", "stand", (10.0, 6.4, 0.4), 5.4, 215, -22, (1500, 900)),
]


def main() -> int:
    try:
        import mujoco
    except ImportError:
        print("mujoco is not installed: pip install mujoco", file=sys.stderr)
        return 1
    os.makedirs(OUT, exist_ok=True)
    try:
        from PIL import Image
    except ImportError:
        print("Pillow is required to write PNGs", file=sys.stderr)
        return 1

    cache: dict[str, tuple] = {}
    print(f"{'image':22s} {'model':22s} {'size':>12s}   file")
    print("-" * 74)
    for name, model_file, key, lookat, dist, azim, elev, size in SHOTS:
        if model_file not in cache:
            m = mujoco.MjModel.from_xml_path(os.path.join(MODELS, model_file))
            cache[model_file] = (m, mujoco.MjData(m))
        m, d = cache[model_file]
        kid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, key)
        if kid >= 0:
            mujoco.mj_resetDataKeyframe(m, d, kid)
        mujoco.mj_forward(m, d)

        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = lookat
        cam.distance = dist
        cam.azimuth = azim
        cam.elevation = elev

        w, h = size
        renderer = mujoco.Renderer(m, height=h, width=w)
        opt = mujoco.MjvOption()
        # Hide the red collision primitives: these are presentation images.
        opt.geomgroup[3] = 0
        renderer.update_scene(d, camera=cam, scene_option=opt)
        img = renderer.render().copy()
        renderer.close()

        path = os.path.join(OUT, f"{name}.png")
        Image.fromarray(img).save(path, optimize=True)
        kb = os.path.getsize(path) / 1024
        print(f"{name:22s} {model_file:22s} {w}x{h:<6d}   images/{name}.png  {kb:.0f} KB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
