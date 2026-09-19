"""
Test environment description -- the single source of truth for the world.

The same spec is consumed twice:
    world_builder.py      -> MuJoCo MJCF bodies (physics, contact, friction)
    world_markers_node.py -> RViz MarkerArray   (visualisation alongside TF)

Keeping one description means the obstacle the planner sees in RViz is
geometrically the obstacle the robot trips over in MuJoCo. Two hand-maintained
files would drift, and the drift would look like a perception bug.

Units: metres, radians. `size` is the FULL extent of a box (MuJoCo wants half
extents; the builder halves them). Poses are the geometric centre, except for
stairs and walls which have their own constructors.

Scale reference: the robot is 0.32 m tall at the nominal stance, its body is
0.235 x 0.120 m, and its feet span 0.442 x 0.289 m. Doors are 0.90 m,
"narrow" passages are 0.40-0.55 m, and the stair rise is 0.08 m -- a third of
the 0.24 m the leg can lift while keeping the body level.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

WALL_H = 1.20          # tall enough to block the robot and the camera frustum
WALL_T = 0.10
DOOR_W = 0.90
SILL_H = 0.02          # door thresholds: small but not negligible for a foot

# tag -> (r, g, b, a). Tags also drive RViz namespaces, so a whole class of
# object can be toggled off in the GUI.
PALETTE = {
    "floor":     (0.82, 0.80, 0.76, 1.0),
    "wall":      (0.90, 0.89, 0.86, 1.0),
    "door":      (0.55, 0.38, 0.24, 1.0),
    "furniture": (0.45, 0.33, 0.24, 1.0),
    "soft":      (0.30, 0.42, 0.52, 1.0),
    "obstacle":  (0.78, 0.45, 0.15, 1.0),
    "stair":     (0.62, 0.62, 0.66, 1.0),
    "platform":  (0.50, 0.55, 0.60, 1.0),
    "ramp":      (0.40, 0.52, 0.40, 1.0),
    "target":    (0.85, 0.25, 0.30, 1.0),
}


@dataclass
class Prim:
    name: str
    type: str                       # box | cylinder | sphere
    pos: tuple[float, float, float]  # geometric centre
    size: tuple[float, ...]          # box: full extents; cylinder: (radius, length)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tag: str = "obstacle"
    friction: float = 0.9
    movable: bool = False            # a free body in MuJoCo: can be pushed over
    mass: float = 0.0                # only meaningful when movable

    @property
    def rgba(self) -> tuple[float, float, float, float]:
        return PALETTE.get(self.tag, (0.6, 0.6, 0.6, 1.0))


@dataclass
class Waypoint:
    """A named place, used by the navigation test scripts and shown in RViz.

    `z` is the FLOOR height at the waypoint, not the robot's height. It is
    non-zero for places reached by climbing -- a stair landing, the top of a
    ramp -- and tells the clearance checker that a surface at or below that
    level is something to stand on rather than an obstacle to avoid.
    """
    name: str
    pos: tuple[float, float]
    yaw: float = 0.0
    note: str = ""
    z: float = 0.0


@dataclass
class World:
    name: str
    prims: list[Prim] = field(default_factory=list)
    waypoints: list[Waypoint] = field(default_factory=list)
    size: tuple[float, float] = (10.0, 8.0)
    #: True when the origin is the centre of the floor plan (open test courses),
    #: False when it is the south-west corner (rooms with walls).
    centred: bool = False

    def add(self, *p: Prim) -> None:
        self.prims.extend(p)

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """(x_min, y_min, x_max, y_max) of the floor plan."""
        w, h = self.size
        return (-w / 2.0, -h / 2.0, w / 2.0, h / 2.0) if self.centred else (0.0, 0.0, w, h)


# --------------------------------------------------------------------------- #
# constructors
# --------------------------------------------------------------------------- #
def wall(name: str, x0: float, y0: float, x1: float, y1: float,
         *, height: float = WALL_H, thickness: float = WALL_T,
         doors: list[tuple[float, float]] | None = None) -> list[Prim]:
    """A straight wall from (x0, y0) to (x1, y1), split around door openings.

    `doors` are (start, end) distances measured along the wall from its start.
    A door leaves a full-height gap plus a low sill, because a real doorway has
    a threshold and stepping over one is a distinct locomotion test.
    """
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    segments: list[tuple[float, float]] = []
    cursor = 0.0
    for a, b in sorted(doors or []):
        if a > cursor:
            segments.append((cursor, a))
        cursor = max(cursor, b)
    if cursor < length:
        segments.append((cursor, length))

    out: list[Prim] = []
    for i, (a, b) in enumerate(segments):
        if b - a < 1e-6:
            continue
        mid = (a + b) / 2.0
        cx = x0 + dx / length * mid
        cy = y0 + dy / length * mid
        out.append(Prim(f"{name}_s{i}", "box", (cx, cy, height / 2.0),
                        (b - a, thickness, height), (0.0, 0.0, yaw), "wall"))
    for i, (a, b) in enumerate(sorted(doors or [])):
        mid = (a + b) / 2.0
        cx = x0 + dx / length * mid
        cy = y0 + dy / length * mid
        out.append(Prim(f"{name}_sill{i}", "box", (cx, cy, SILL_H / 2.0),
                        (b - a, thickness, SILL_H), (0.0, 0.0, yaw), "door"))
    return out


def table(name: str, x: float, y: float, *, w: float = 1.20, d: float = 0.70,
          h: float = 0.45, leg: float = 0.05, yaw: float = 0.0) -> list[Prim]:
    """A table with real legs, so the robot can choose to go under or around.
    Default height 0.45 m clears the 0.32 m stance with 0.13 m to spare, which
    makes duck-under behaviour testable rather than merely blocked."""
    top = 0.04
    c, s = math.cos(yaw), math.sin(yaw)
    out = [Prim(f"{name}_top", "box", (x, y, h - top / 2.0), (w, d, top),
                (0, 0, yaw), "furniture")]
    for i, (lx, ly) in enumerate([(w / 2 - leg, d / 2 - leg), (-(w / 2 - leg), d / 2 - leg),
                                  (w / 2 - leg, -(d / 2 - leg)), (-(w / 2 - leg), -(d / 2 - leg))]):
        out.append(Prim(f"{name}_leg{i}", "box",
                        (x + lx * c - ly * s, y + lx * s + ly * c, (h - top) / 2.0),
                        (leg, leg, h - top), (0, 0, yaw), "furniture"))
    return out


def stairs(name: str, x: float, y: float, *, steps: int = 4, rise: float = 0.08,
           run: float = 0.26, width: float = 1.20, yaw: float = 0.0,
           landing: float = 0.8) -> list[Prim]:
    """A flight climbing in +x (before yaw), with a landing at the top.

    Rise 0.08 m is deliberately modest: the leg can lift 0.24 m, but a quadruped
    of this size loses body clearance long before that, so 0.08 m is a realistic
    first target and 0.12 m is the stretch case (see `stairs_steep`).
    """
    out: list[Prim] = []
    c, s = math.cos(yaw), math.sin(yaw)
    for i in range(steps):
        # Each tread is a solid block from the ground up, so there are no
        # overhangs for a foot to catch under.
        h = rise * (i + 1)
        lx = run * (i + 0.5)
        out.append(Prim(f"{name}_step{i}", "box",
                        (x + lx * c, y + lx * s, h / 2.0),
                        (run, width, h), (0, 0, yaw), "stair"))
    lx = run * steps + landing / 2.0
    out.append(Prim(f"{name}_landing", "box",
                    (x + lx * c, y + lx * s, rise * steps / 2.0),
                    (landing, width, rise * steps), (0, 0, yaw), "platform"))
    return out


def ramp(name: str, x: float, y: float, *, length: float = 1.6, width: float = 1.0,
         angle_deg: float = 12.0, yaw: float = 0.0) -> Prim:
    """A sloped plank. 12 deg is about the limit at which a 0.9 friction
    coefficient still holds a static stance without the feet sliding."""
    a = math.radians(angle_deg)
    return Prim(name, "box", (x, y, length * math.sin(a) / 2.0),
                (length, width, 0.04), (0.0, -a, yaw), "ramp", friction=0.9)
