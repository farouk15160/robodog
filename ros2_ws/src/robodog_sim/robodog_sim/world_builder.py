"""World spec -> MuJoCo worldbody geoms."""
from __future__ import annotations

import numpy as np

from .mjcf import _f, _quat, _sub
from .world_spec import World


def add_world(root, world: World) -> None:
    wb = root.find("worldbody")
    _sub(wb, "light", name="sun", directional="true", diffuse="0.75 0.75 0.75",
         specular="0.1 0.1 0.1", pos="0 0 6", dir="0 0 -1", castshadow="true")
    _sub(wb, "light", name="fill", directional="true", diffuse="0.25 0.25 0.28",
         pos="-4 -4 5", dir="0.5 0.5 -1", castshadow="false")
    w, h = world.size
    x0, y0, x1, y1 = world.bounds
    _sub(wb, "geom", name="ground", type="plane", material="grid",
         size=_f(max(w, h), max(w, h), 0.05),
         pos=_f((x0 + x1) / 2.0, (y0 + y1) / 2.0, 0),
         condim="3", friction="0.9 0.02 0.001", group="0")

    for p in world.prims:
        if p.movable:
            # a free body: it can be knocked over, which is what makes contact
            # estimation and recovery testable instead of assumed
            b = _sub(wb, "body", name=p.name, pos=_f(p.pos), quat=_quat(p.rpy))
            _sub(b, "freejoint", name=f"{p.name}_free")
            _sub(b, "inertial", pos="0 0 0", mass=p.mass or 1.0,
                 diaginertia=_f(*_box_inertia(p)))
            _geom(b, p, local=True)
        else:
            _geom(wb, p, local=False)


def _box_inertia(p) -> tuple[float, float, float]:
    m = p.mass or 1.0
    if p.type == "box":
        x, y, z = p.size
    else:
        r, l = p.size[0], p.size[1]
        x = y = 2 * r
        z = l
    k = m / 12.0
    return (k * (y * y + z * z), k * (x * x + z * z), k * (x * x + y * y))


def _geom(parent, p, *, local: bool) -> None:
    attrs = dict(name=p.name, rgba=_f(p.rgba), group="0",
                 friction=_f(p.friction, 0.02, 0.001),
                 condim="3", solref="0.004 1", solimp="0.95 0.99 0.001")
    if not local:
        # quat, not euler: the ramps carry a two-axis rotation (pitch + yaw),
        # which MuJoCo's intrinsic euler would interpret differently from the
        # extrinsic convention world_spec uses.
        attrs.update(pos=_f(p.pos), quat=_quat(p.rpy))
    else:
        attrs.update(pos="0 0 0")
    if p.type == "box":
        _sub(parent, "geom", type="box", size=_f(np.array(p.size) / 2.0), **attrs)
    elif p.type == "cylinder":
        _sub(parent, "geom", type="cylinder", size=_f(p.size[0], p.size[1] / 2.0), **attrs)
    elif p.type == "sphere":
        _sub(parent, "geom", type="sphere", size=_f(p.size[0]), **attrs)
    else:
        raise ValueError(f"unsupported primitive type '{p.type}'")
