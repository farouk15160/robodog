"""Mapping startup must preserve maps and reject unavailable sensor pipelines."""
from pathlib import Path
import importlib.util

import pytest

from robodog_perception.mapping import mapping_plan


def plan(**kwargs):
    values = dict(mode="rtabmap", backend="mujoco", camera_backend="sim",
                  use_camera=True, world="house", database="", ros_home="/tmp/ros")
    return mapping_plan(**dict(values, **kwargs))


def test_disabled_mapping_needs_no_camera_or_odometry():
    result = plan(mode="none", backend="robstride06_can", camera_backend="none",
                  use_camera=False)
    assert not result.enabled
    assert result.database_path is None


def test_auto_mapping_enables_supported_simulation():
    assert plan(mode="auto").enabled


@pytest.mark.parametrize("kwargs", [
    {"backend": "kinematic"},
    {"backend": "robstride06_can"},
    {"camera_backend": "none"},
    {"camera_backend": "nuwa_hp60c"},
    {"use_camera": False},
])
def test_auto_mapping_disables_unsupported_sensor_pipelines(kwargs):
    result = plan(mode="auto", **kwargs)
    assert not result.enabled
    assert result.database_path is None


def test_simulation_maps_are_named_by_world_without_creating_files(tmp_path):
    flat = plan(world="flat", ros_home=str(tmp_path))
    house = plan(ros_home=str(tmp_path))
    assert flat.database_path == tmp_path / "robodog/maps/flat.db"
    assert house.database_path == tmp_path / "robodog/maps/house.db"
    assert not (tmp_path / "robodog").exists()


def test_explicit_database_is_preserved(tmp_path):
    database = tmp_path / "existing map.db"
    database.write_bytes(b"existing map")
    assert plan(database=str(database)).database_path == database
    assert database.read_bytes() == b"existing map"


@pytest.mark.parametrize("kwargs, message", [
    ({"backend": "robstride06_can"}, "depth and odometry"),
    ({"backend": "kinematic"}, "backend:=mujoco"),
    ({"camera_backend": "none"}, "camera_backend:=sim"),
    ({"camera_backend": "nuwa_hp60c"}, "camera_backend:=sim"),
    ({"use_camera": False}, "use_camera:=true"),
    ({"mode": "unknown"}, "mapping mode"),
    ({"world": "../outside"}, "world"),
])
def test_invalid_mapping_fails_before_any_nodes_start(kwargs, message):
    with pytest.raises(ValueError, match=message):
        plan(**kwargs)


def test_existing_directory_is_not_accepted_as_database(tmp_path):
    with pytest.raises(ValueError, match="file"):
        plan(database=str(tmp_path))


@pytest.fixture
def bringup(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "logs"))
    path = Path(__file__).parents[2] / "robodog_bringup/launch/robot.launch.py"
    spec = importlib.util.spec_from_file_location("mapping_bringup_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_context(bringup, **overrides):
    from launch import LaunchContext
    context = LaunchContext()
    context.launch_configurations.update({name: default for name, default, *_ in bringup.ARGS})
    context.launch_configurations.update(overrides)
    return context


def test_regular_bringup_never_looks_up_optional_rtabmap(bringup, monkeypatch):
    from launch.actions import IncludeLaunchDescription
    lookup = bringup.get_package_share_directory

    def without_rtabmap(package):
        assert package != "rtabmap_slam"
        return lookup(package)

    monkeypatch.setattr(bringup, "get_package_share_directory", without_rtabmap)
    actions = bringup._setup(launch_context(bringup))
    assert not any(isinstance(action, IncludeLaunchDescription) for action in actions)


def test_mujoco_bringup_enables_mapping_by_default(bringup, monkeypatch):
    from launch.actions import IncludeLaunchDescription
    lookup = bringup.get_package_share_directory
    monkeypatch.setattr(bringup, "get_package_share_directory",
                        lambda name: "/optional/rtabmap" if name == "rtabmap_slam" else lookup(name))
    actions = bringup._setup(launch_context(bringup, backend="mujoco"))
    assert any(isinstance(action, IncludeLaunchDescription) for action in actions)


def test_mujoco_mapping_can_be_explicitly_disabled(bringup, monkeypatch):
    from launch.actions import IncludeLaunchDescription
    lookup = bringup.get_package_share_directory

    def without_rtabmap(package):
        assert package != "rtabmap_slam"
        return lookup(package)

    monkeypatch.setattr(bringup, "get_package_share_directory", without_rtabmap)
    actions = bringup._setup(launch_context(bringup, backend="mujoco", mapping="none"))
    assert not any(isinstance(action, IncludeLaunchDescription) for action in actions)


def test_missing_mapping_runtime_fails_before_launch_actions(bringup, monkeypatch):
    def missing(package):
        assert package == "rtabmap_slam"
        raise bringup.PackageNotFoundError(package)

    monkeypatch.setattr(bringup, "get_package_share_directory", missing)
    with pytest.raises(RuntimeError, match="ros-humble-rtabmap-ros"):
        bringup._setup(launch_context(bringup, mapping="rtabmap", backend="mujoco"))


def test_mapping_launch_receives_explicit_database(bringup, monkeypatch, tmp_path):
    from launch.actions import IncludeLaunchDescription
    lookup = bringup.get_package_share_directory
    monkeypatch.setattr(bringup, "get_package_share_directory",
                        lambda name: "/optional/rtabmap" if name == "rtabmap_slam" else lookup(name))
    database = tmp_path / "custom.db"
    actions = bringup._setup(launch_context(bringup, mapping="rtabmap", backend="mujoco", world="house",
                                            mapping_database=str(database)))
    mapping_launch = next(action for action in actions if isinstance(action, IncludeLaunchDescription))
    assert dict(mapping_launch.launch_arguments)["mapping_database"] == str(database)
    assert dict(mapping_launch.launch_arguments)["world"] == "house"
    assert not database.exists()
