#!/usr/bin/env python3
"""
Mesh preparation for robodog_description.

1. Decimates the Onshape STL exports (40 MB, up to 236 k triangles each) by
   vertex clustering. These are VISUAL meshes only -- collision uses analytic
   primitives from robot_parameters.yaml -- so the watertightness lost by
   clustering does not matter.
2. Generates an RS06 envelope (88mm max diameter x49mm depth), in the supplied
   CAD motor interface frame. This is a clearance estimate, not a fit approval.

Run:  python3 tools/prepare_meshes.py
"""
from __future__ import annotations
import math, os, struct
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "cad/urdf/meshes")
DST = os.path.join(ROOT, "ros2_ws/src/robodog_description/meshes")

# target cell size (m) for vertex clustering, per mesh
CLUSTER = {
    "body.stl": 0.0035, "body_panel.stl": 0.0040, "handle.stl": 0.0025,
    "angle_motor_connect.stl": 0.0025, "upperleg_main.stl": 0.0030,
    "upperleg_cover.stl": 0.0030, "inline_connect.stl": 0.0025,
    "lower_leg.stl": 0.0030, "foot_ball.stl": 0.0022,
    "_61804_2RS.stl": 0.0012, "motor_pulley.stl": 0.0015,
}
SKIP = {"Part_1.stl", "XYZ_frame.stl"}          # replaced / dropped


def read_stl(path: str) -> np.ndarray:
    """Return an (n, 3, 3) array of triangle vertices."""
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        head = f.read(84)
        n = struct.unpack("<I", head[80:84])[0]
        if 84 + n * 50 == size:                                   # binary
            raw = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
            return raw[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)
    tok = open(path, "r", errors="ignore").read().split()
    v = [[float(tok[i + 1]), float(tok[i + 2]), float(tok[i + 3])]
         for i, t in enumerate(tok) if t == "vertex"]
    return np.asarray(v, dtype=np.float64).reshape(-1, 3, 3)


def write_stl(path: str, tris: np.ndarray) -> None:
    n = len(tris)
    e1 = tris[:, 1] - tris[:, 0]
    e2 = tris[:, 2] - tris[:, 0]
    nrm = np.cross(e1, e2)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 0)
    rec = np.zeros((n, 50), dtype=np.uint8)
    rec[:, 0:12] = nrm.astype("<f4").view(np.uint8).reshape(n, 12)
    rec[:, 12:48] = tris.astype("<f4").view(np.uint8).reshape(n, 36)
    with open(path, "wb") as f:
        f.write(b"robodog decimated visual mesh".ljust(80, b"\0"))
        f.write(struct.pack("<I", n))
        f.write(rec.tobytes())


def cluster_decimate(tris: np.ndarray, cell: float) -> np.ndarray:
    """Snap vertices to a grid, keep one representative per cell, drop
    triangles that collapse to an edge or point."""
    v = tris.reshape(-1, 3)
    key = np.floor(v / cell).astype(np.int64)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    rep = v[first]                       # representative vertex per occupied cell
    idx = np.asarray(inv).reshape(-1, 3)
    keep = (idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])
    return rep[idx[keep]]


# --------------------------------------------------------------------------- #
# procedural ROBSTRIDE06 envelope visual
# --------------------------------------------------------------------------- #
def revolve(profile, seg=48):
    """Revolve a closed (r, z) polyline about +z into a triangle soup."""
    th = np.linspace(0, 2 * math.pi, seg, endpoint=False)
    c, s = np.cos(th), np.sin(th)
    tris = []
    for (r0, z0), (r1, z1) in zip(profile, profile[1:]):
        for i in range(seg):
            j = (i + 1) % seg
            a = (r0 * c[i], r0 * s[i], z0)
            b = (r0 * c[j], r0 * s[j], z0)
            d = (r1 * c[i], r1 * s[i], z1)
            e = (r1 * c[j], r1 * s[j], z1)
            if r0 > 0 and r1 > 0:
                tris += [[a, b, e], [a, e, d]]
            elif r1 > 0:
                tris += [[a, e, d]]
            elif r0 > 0:
                tris += [[a, b, e]]
    return np.asarray(tris, dtype=np.float64)


def make_robstride06() -> np.ndarray:
    """Conservative RS06 flange envelope, not manufacturing CAD.

    Manufacturer outline: maximum diameter 88mm, axial length49mm.
    Supplied CAD placeholder has shaft along local y, output plane y=-4mm.
    """
    radius, depth = 0.044, 0.049
    triangles = revolve([(0, -0.004), (radius, -0.004),
                         (radius, depth - 0.004), (0, depth - 0.004)])
    return np.ascontiguousarray(triangles[:, :, [0, 2, 1]])


def main():
    os.makedirs(DST, exist_ok=True)
    tot_in = tot_out = 0
    print(f"{'mesh':26s} {'tris in':>9s} {'tris out':>9s} {'ratio':>7s} {'KB':>7s}")
    print("-" * 64)
    for fn in sorted(os.listdir(SRC)):
        if not fn.endswith(".stl") or fn in SKIP:
            continue
        tris = read_stl(os.path.join(SRC, fn))
        out = cluster_decimate(tris, CLUSTER.get(fn, 0.003))
        name = fn.lstrip("_").lower()
        path = os.path.join(DST, name)
        write_stl(path, out)
        kb = os.path.getsize(path) / 1024
        tot_in += len(tris)
        tot_out += len(out)
        print(f"{name:26s} {len(tris):9d} {len(out):9d} {len(out)/len(tris)*100:6.1f}% {kb:7.1f}")

    rs = make_robstride06()
    write_stl(os.path.join(DST, "robstride06.stl"), rs)
    print(f"{'robstride06.stl':26s} {'-':>9s} {len(rs):9d} {'gen':>7s} "
          f"{os.path.getsize(os.path.join(DST,'robstride06.stl'))/1024:7.1f}")

    src_mb = sum(os.path.getsize(os.path.join(SRC, f)) for f in os.listdir(SRC)) / 1e6
    dst_mb = sum(os.path.getsize(os.path.join(DST, f)) for f in os.listdir(DST)) / 1e6
    print("-" * 64)
    print(f"triangles {tot_in} -> {tot_out}  ({tot_out/tot_in*100:.1f}%)")
    print(f"on disk   {src_mb:.1f} MB -> {dst_mb:.1f} MB")


if __name__ == "__main__":
    main()
