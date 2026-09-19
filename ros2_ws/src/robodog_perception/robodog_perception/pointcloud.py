"""Depth image -> PointCloud2, with the range filtering the datasheet implies."""
from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from .backend import Intrinsics

_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
]
POINT_STEP = 16


def deproject(depth: np.ndarray, K: Intrinsics, *, decimation: int = 1,
              min_range: float = 0.0, max_range: float = np.inf) -> tuple[np.ndarray, np.ndarray]:
    """Depth image -> (N, 3) points in the OPTICAL frame (z forward, x right,
    y down) plus the pixel indices they came from.

    Range filtering happens here rather than downstream because a stereo
    camera's error grows with the square of range: beyond `max_usable_range_m`
    the points are not merely noisy, they are systematically wrong, and letting
    them into a map costs more than the coverage is worth.
    """
    d = depth[::decimation, ::decimation]
    h, w = d.shape
    vs, us = np.mgrid[0:h, 0:w]
    us = us * decimation
    vs = vs * decimation
    valid = np.isfinite(d) & (d > min_range) & (d < max_range)
    z = d[valid].astype(np.float32)
    x = (us[valid] - K.cx) * z / K.fx
    y = (vs[valid] - K.cy) * z / K.fy
    return np.stack([x, y, z], axis=1).astype(np.float32), np.stack([vs[valid], us[valid]], axis=1)


def _pack_rgb(rgb: np.ndarray) -> np.ndarray:
    packed = (rgb[:, 0].astype(np.uint32) << 16 |
              rgb[:, 1].astype(np.uint32) << 8 |
              rgb[:, 2].astype(np.uint32))
    return packed.view(np.float32) if packed.dtype == np.float32 else \
        packed.astype(np.uint32).view(np.float32)


def make_cloud(points: np.ndarray, colors: np.ndarray | None, frame_id: str,
               stamp) -> PointCloud2:
    n = len(points)
    buf = np.zeros((n, 4), dtype=np.float32)
    buf[:, 0:3] = points
    if colors is not None and len(colors) == n:
        buf[:, 3] = _pack_rgb(colors)
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n
    msg.fields = _FIELDS
    msg.is_bigendian = False
    msg.point_step = POINT_STEP
    msg.row_step = POINT_STEP * n
    msg.is_dense = True
    msg.data = buf.tobytes()
    return msg
