"""Persistent room-scan export from RTAB-Map's assembled map products."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
from typing import Any

from sensor_msgs.msg import PointField

from .mapping import ExportPlan


_PCD_TYPES = {
    PointField.INT8: (1, "I"),
    PointField.UINT8: (1, "U"),
    PointField.INT16: (2, "I"),
    PointField.UINT16: (2, "U"),
    PointField.INT32: (4, "I"),
    PointField.UINT32: (4, "U"),
    PointField.FLOAT32: (4, "F"),
    PointField.FLOAT64: (8, "F"),
}


@dataclass(frozen=True)
class ExportResult:
    manifest_path: Path
    manifest: dict[str, Any]


class RoomScanExporter:
    """Create one immutable export using adapters at the ROS/process boundary."""

    def __init__(self, *, database_path: Path, cloud_provider,
                 flush_database, save_octomap):
        self._database_path = Path(database_path)
        self._cloud_provider = cloud_provider
        self._flush_database = flush_database
        self._save_octomap = save_octomap

    @staticmethod
    def _write(path: Path, payload: bytes) -> None:
        temporary = path.with_name(f".{path.name}.tmp")
        with temporary.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)

    @staticmethod
    def _file_record(path: Path) -> dict[str, Any]:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        return {"bytes": path.stat().st_size, "sha256": digest.hexdigest()}

    def export(self, plan: ExportPlan) -> ExportResult:
        plan.output_root.mkdir(parents=True, exist_ok=True)
        # Reserving the final directory is the no-overwrite guarantee.  A
        # manifest is written only after every artifact and checksum exists.
        plan.session_directory.mkdir(mode=0o700, exist_ok=False)
        artifacts = {
            "cloud_map.pcd": plan.session_directory / "cloud_map.pcd",
            "octomap.ot": plan.session_directory / "octomap.ot",
            "rtabmap.db.back": plan.session_directory / "rtabmap.db.back",
        }
        manifest_path = plan.session_directory / "manifest.json"
        cleanup = list(artifacts.values()) + [manifest_path]
        cleanup += [path.with_name(f".{path.name}.tmp") for path in cleanup]
        cleanup.append(artifacts["octomap.ot"].with_name(".octomap.tmp.ot"))
        try:
            cloud = self._cloud_provider()
            pcd, cloud_metadata = pointcloud_to_pcd(cloud)
            self._write(artifacts["cloud_map.pcd"], pcd)

            octomap_tmp = artifacts["octomap.ot"].with_name(".octomap.tmp.ot")
            self._save_octomap(octomap_tmp)
            if not octomap_tmp.is_file() or octomap_tmp.stat().st_size == 0:
                raise RuntimeError("OctoMap saver did not produce a nonempty .ot file")
            octomap_tmp.replace(artifacts["octomap.ot"])

            source_backup = Path(self._flush_database())
            if not source_backup.is_file() or source_backup.stat().st_size == 0:
                raise RuntimeError(f"RTAB-Map backup is missing or empty: {source_backup}")
            database_tmp = artifacts["rtabmap.db.back"].with_name(".rtabmap.db.back.tmp")
            shutil.copyfile(source_backup, database_tmp)
            database_tmp.replace(artifacts["rtabmap.db.back"])

            files = {name: self._file_record(path) for name, path in artifacts.items()}
            stamp = cloud.header.stamp
            manifest = {
                "format": "robodog-room-scan-v1",
                "complete": True,
                "created_at": datetime.now(timezone.utc).isoformat(),
                "session_name": plan.session_name,
                "source_database": str(self._database_path),
                "point_cloud": {
                    **cloud_metadata,
                    "source_topic": "/robodog/mapping/cloud_map",
                    "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
                    "representation": "assembled RTAB-Map point cloud",
                },
                "octomap": {
                    "source_service": "/robodog/mapping/rtabmap/octomap_full",
                    "representation": "full colored occupancy tree",
                },
                "database": {
                    "source_service": "/robodog/mapping/rtabmap/backup",
                    "representation": "consistent RTAB-Map database backup",
                },
                "files": files,
            }
            encoded = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
            self._write(manifest_path, encoded)
            return ExportResult(manifest_path, manifest)
        except Exception:
            for path in cleanup:
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass
            try:
                plan.session_directory.rmdir()
            except OSError:
                pass
            raise


def pointcloud_to_pcd(message: Any) -> tuple[bytes, dict[str, Any]]:
    """Serialize a PointCloud2 room map as portable PCD v0.7 binary data."""
    points = int(message.width) * int(message.height)
    if points <= 0 or not message.data:
        raise ValueError("Cannot save an empty room point cloud")
    fields = [field for field in message.fields if field.name and field.name != "_"]
    names = [field.name for field in fields]
    if not {"x", "y", "z"}.issubset(names):
        raise ValueError("Room point cloud must contain x, y and z fields")
    try:
        layouts = [(field, *_PCD_TYPES[field.datatype]) for field in fields]
    except KeyError as exc:
        raise ValueError(f"Unsupported PointCloud2 datatype: {exc.args[0]}") from exc
    packed_step = sum(size * int(field.count) for field, size, _ in layouts)
    if message.point_step <= 0 or message.row_step < message.point_step * message.width:
        raise ValueError("Invalid PointCloud2 point or row stride")
    for field, size, _ in layouts:
        count = int(field.count)
        if count <= 0 or int(field.offset) < 0:
            raise ValueError("PointCloud2 fields need positive counts and offsets")
        if int(field.offset) + size * count > message.point_step:
            raise ValueError("PointCloud2 field extends past its point stride")
    required = message.row_step * message.height
    if len(message.data) < required:
        raise ValueError("PointCloud2 payload is truncated")

    payload = bytearray(points * packed_step)
    target = 0
    source = memoryview(message.data)
    for row in range(message.height):
        row_start = row * message.row_step
        for column in range(message.width):
            point_start = row_start + column * message.point_step
            for field, size, _ in layouts:
                for index in range(field.count):
                    offset = point_start + field.offset + index * size
                    item = source[offset:offset + size]
                    if len(item) != size:
                        raise ValueError("PointCloud2 field extends past its point stride")
                    payload[target:target + size] = item[::-1] if message.is_bigendian else item
                    target += size

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {' '.join(names)}\n"
        f"SIZE {' '.join(str(size) for _, size, _ in layouts)}\n"
        f"TYPE {' '.join(kind for _, _, kind in layouts)}\n"
        f"COUNT {' '.join(str(field.count) for field, _, _ in layouts)}\n"
        f"WIDTH {points}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {points}\n"
        "DATA binary\n"
    ).encode("ascii")
    metadata = {
        "frame_id": message.header.frame_id,
        "points": points,
        "fields": names,
    }
    return header + payload, metadata
