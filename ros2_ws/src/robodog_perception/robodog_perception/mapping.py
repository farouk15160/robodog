"""Side-effect-free validation of the supported RGB-D mapping pipeline."""
from dataclasses import dataclass
from datetime import datetime, timezone
import os
from pathlib import Path
import re


@dataclass(frozen=True)
class MappingPlan:
    enabled: bool
    database_path: Path | None = None


@dataclass(frozen=True)
class ExportPlan:
    """Validated destination for one immutable room-scan snapshot."""

    output_root: Path
    session_name: str
    session_directory: Path


_SESSION_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")


def export_plan(*, output_directory: str = "", name: str = "",
                ros_home: str | None = None, generate_name: bool = True,
                now: datetime | None = None) -> ExportPlan:
    """Plan a room-scan export without creating or modifying anything.

    Explicit output roots must be absolute so a GUI request cannot depend on
    the mapper process' working directory.  Session names are single safe path
    components and an existing target is never reused.
    """
    if output_directory:
        root = Path(output_directory).expanduser()
        if not root.is_absolute():
            raise ValueError("Map export output_directory must be absolute")
    else:
        ros_root = Path(ros_home or os.environ.get("ROS_HOME", "~/.ros")).expanduser()
        root = ros_root / "robodog/exports"
    root = root.absolute()
    session_name = name
    if not session_name and generate_name:
        stamp = now or datetime.now(timezone.utc)
        session_name = stamp.strftime("scan-%Y%m%d-%H%M%S")
    if not _SESSION_NAME.fullmatch(session_name):
        raise ValueError("Map export name must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}")
    destination = root / session_name
    if destination.exists():
        raise FileExistsError(f"Map export session already exists: {destination}")
    return ExportPlan(root, session_name, destination)


def mapping_plan(*, mode: str, backend: str, camera_backend: str,
                 use_camera: bool, world: str, database: str = "",
                 ros_home: str | None = None) -> MappingPlan:
    if mode == "none":
        return MappingPlan(False)
    if mode == "auto" and (backend != "mujoco" or camera_backend != "sim" or not use_camera):
        return MappingPlan(False)
    if mode not in ("auto", "rtabmap"):
        raise ValueError(f"Unsupported mapping mode: {mode}")
    if backend == "robstride06_can":
        raise ValueError("RTAB-Map currently supports simulation only. Real hardware "
                         "needs calibrated registered depth and odometry first.")
    if backend != "mujoco":
        raise ValueError("Mapping requires backend:=mujoco; the kinematic backend "
                         "does not provide moving base odometry or world spawn placement.")
    if camera_backend != "sim":
        raise ValueError("Mapping requires camera_backend:=sim; the real camera "
                         "depth stream is not integrated yet.")
    if not use_camera:
        raise ValueError("Mapping requires use_camera:=true for the optical TF frames.")
    if world not in ("flat", "house"):
        raise ValueError(f"Unsupported mapping world: {world}")
    root = Path(ros_home or os.environ.get("ROS_HOME", "~/.ros")).expanduser()
    path = Path(database).expanduser() if database else root / "robodog/maps" / f"{world}.db"
    path = path.absolute()
    if path.is_dir():
        raise ValueError(f"Mapping database must be a file, not a directory: {path}")
    return MappingPlan(True, path)
