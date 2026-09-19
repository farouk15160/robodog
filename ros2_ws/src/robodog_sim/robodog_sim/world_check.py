"""
Geometric validation of a test world.

The docstring in worlds/house.py claims specific clearances -- "0.45 m gap",
"0.41 m under-table clearance". Those numbers are the whole point of the test
environment, and a careless edit to a position turns a designed challenge into
either a wall or an open field without anyone noticing. This module measures
them from the spec so the claims are checked, not asserted.

Footprints are treated as rotated rectangles in the xy plane. Distances between
convex polygons are computed by rotating-calipers over the edge normals, which
is exact for the face-vertex case and a slight under-estimate for vertex-vertex
-- conservative in the direction that matters.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .world_spec import Prim, World


def footprint(p: Prim) -> np.ndarray:
    """Four xy corners of the primitive's footprint."""
    if p.type == "box":
        hx, hy = p.size[0] / 2.0, p.size[1] / 2.0
    else:
        hx = hy = p.size[0]
    c, s = math.cos(p.rpy[2]), math.sin(p.rpy[2])
    R = np.array([[c, -s], [s, c]])
    local = np.array([[hx, hy], [-hx, hy], [-hx, -hy], [hx, -hy]])
    return (local @ R.T) + np.array(p.pos[:2])


def z_range(p: Prim) -> tuple[float, float]:
    if p.type == "box":
        # a pitched box (a ramp) spans more in z than its thickness
        h = abs(p.size[2] * math.cos(p.rpy[1])) + abs(p.size[0] * math.sin(p.rpy[1]))
    else:
        h = p.size[1]
    return p.pos[2] - h / 2.0, p.pos[2] + h / 2.0


def polygon_gap(a: np.ndarray, b: np.ndarray) -> float:
    """Separation between two convex polygons; negative means overlapping."""
    best = -np.inf
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            e = poly[(i + 1) % n] - poly[i]
            axis = np.array([-e[1], e[0]])
            norm = np.linalg.norm(axis)
            if norm < 1e-12:
                continue
            axis = axis / norm
            pa, pb = a @ axis, b @ axis
            gap = max(pa.min() - pb.max(), pb.min() - pa.max())
            best = max(best, gap)
    return float(best)


@dataclass
class Gap:
    a: str
    b: str
    distance: float
    z_low: float
    z_high: float


def narrow_gaps(world: World, *, body_band=(0.05, 0.40), limit=0.90) -> list[Gap]:
    """Every pair of static objects closer than `limit` whose vertical extents
    both overlap the band the robot's body occupies while walking."""
    cand = [p for p in world.prims
            if not p.movable and _overlaps(z_range(p), body_band)]
    fps = {p.name: footprint(p) for p in cand}
    out: list[Gap] = []
    for i, p in enumerate(cand):
        for q in cand[i + 1:]:
            if p.name.rsplit("_", 1)[0] == q.name.rsplit("_", 1)[0]:
                continue                        # segments of the same wall/table
            d = polygon_gap(fps[p.name], fps[q.name])
            if 0.0 < d < limit:
                zr = (max(z_range(p)[0], z_range(q)[0]), min(z_range(p)[1], z_range(q)[1]))
                out.append(Gap(p.name, q.name, d, zr[0], zr[1]))
    return sorted(out, key=lambda g: g.distance)


def _overlaps(a, b) -> bool:
    return a[0] < b[1] and b[0] < a[1]


def overhead_clearances(world: World, *, min_gap=0.25) -> list[tuple[str, float]]:
    """Objects the robot could pass under: a top surface above `min_gap` with
    nothing solid below it. Table tops qualify, a wall does not."""
    out = []
    for p in world.prims:
        if p.type != "box" or p.movable:
            continue
        lo, hi = z_range(p)
        if lo < min_gap or (hi - lo) > 0.12:
            continue
        under = [q for q in world.prims
                 if q is not p and q.name.rsplit("_", 1)[0] == p.name.rsplit("_", 1)[0]]
        if under:                                # a table top with its own legs
            out.append((p.name, lo))
    return sorted(out, key=lambda t: t[1])


def overlapping_pairs(world: World, *, tol: float = 1e-6) -> list[tuple[str, str, float]]:
    """Static objects whose solids intersect. Overlapping geometry is never
    intentional: MuJoCo resolves it by pushing bodies apart at the first step,
    which silently moves the test environment out from under the experiment."""
    out = []
    prims = [p for p in world.prims if not p.movable]
    for i, p in enumerate(prims):
        zp = z_range(p)
        for q in prims[i + 1:]:
            if p.name.rsplit("_", 1)[0] == q.name.rsplit("_", 1)[0]:
                continue                       # parts of one table / wall / flight
            if p.tag == "wall" and q.tag == "wall":
                continue                       # corners and T-junctions overlap
                                               # by half a wall thickness by
                                               # construction; that is correct
            zq = z_range(q)
            if not _overlaps(zp, zq):
                continue
            d = polygon_gap(footprint(p), footprint(q))
            if d < -tol:
                out.append((p.name, q.name, d))
    return sorted(out, key=lambda t: t[2])


def waypoint_clearance(world: World, wp) -> tuple[float, str]:
    """Distance from a waypoint to the nearest solid object in the body band,
    and which object that is. Negative means the waypoint is inside something."""
    best, who = np.inf, ""
    pt = np.array([[wp.pos[0], wp.pos[1]]])
    floor = getattr(wp, "z", 0.0)
    band = (floor + 0.05, floor + 0.40)
    for p in world.prims:
        if p.movable or not _overlaps(z_range(p), band):
            continue
        if z_range(p)[1] <= floor + 0.02:
            continue                    # a surface to stand on, not an obstacle
        d = polygon_gap(pt, footprint(p))
        if d < best:
            best, who = d, p.name
    return float(best), who


def waypoints_inside_objects(world: World) -> list[str]:
    """Waypoints whose centre is inside a solid object: an unreachable goal.

    Only the centre is tested, not a robot-sized disc: several waypoints mark
    gates that are deliberately narrower than the robot's foot span, and the
    robot crosses those by placing its feet, not by fitting a circle.
    """
    return [f"{wp.name} is inside {who}"
            for wp in world.waypoints
            for d, who in [waypoint_clearance(world, wp)] if d < 0.0]


def report(world: World) -> str:
    lines = [f"world '{world.name}': {len(world.prims)} objects, "
             f"{len(world.waypoints)} waypoints, {world.size[0]} x {world.size[1]} m", ""]
    ov = overlapping_pairs(world)
    lines.append("overlapping geometry: " + ("none" if not ov else ""))
    for a, b, d in ov[:20]:
        lines.append(f"  {-d*1000:6.0f} mm INTO   {a} <-> {b}")
    lines.append("")
    lines.append("narrow passages (body band 0.05-0.40 m):")
    for g in narrow_gaps(world)[:14]:
        lines.append(f"  {g.distance*1000:6.0f} mm   {g.a} <-> {g.b}")
    lines.append("")
    lines.append("overhead clearances (pass-under candidates):")
    for name, h in overhead_clearances(world):
        lines.append(f"  {h*1000:6.0f} mm   {name}")
    lines.append("")
    lines.append("waypoint clearances:")
    for wp in world.waypoints:
        d, who = waypoint_clearance(world, wp)
        flag = "  INSIDE" if d < 0 else ("  tight" if d < 0.20 else "")
        lines.append(f"  {d*1000:7.0f} mm  {wp.name:24s} nearest: {who}{flag}")
    bad = waypoints_inside_objects(world)
    lines.append("")
    lines.append("waypoint check: " + ("OK" if not bad else "; ".join(bad)))
    return "\n".join(lines)


if __name__ == "__main__":
    import sys
    from .worlds.house import WORLDS
    name = sys.argv[1] if len(sys.argv) > 1 else "house"
    print(report(WORLDS[name]()))
