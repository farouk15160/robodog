"""Side-effect-free validation of the supported RGB-D mapping pipeline."""
from dataclasses import dataclass
import os
from pathlib import Path


@dataclass(frozen=True)
class MappingPlan:
    enabled: bool
    database_path: Path | None = None


def mapping_plan(*, mode: str, backend: str, camera_backend: str,
                 use_camera: bool, world: str, database: str = "",
                 ros_home: str | None = None) -> MappingPlan:
    if mode == "none":
        return MappingPlan(False)
    if mode != "rtabmap":
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
