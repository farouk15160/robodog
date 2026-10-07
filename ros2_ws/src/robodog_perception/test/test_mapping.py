"""Mapping startup must preserve maps and reject unavailable sensor pipelines."""
from pathlib import Path
import importlib.util
import struct
import yaml

import pytest
from sensor_msgs.msg import PointCloud2, PointField

from robodog_perception.map_export import RoomScanExporter, pointcloud_to_pcd
from robodog_perception.map_export_node import require_fresh_cloud
from robodog_perception.save_map_cli import parse_args as parse_save_map_args
from robodog_perception.mapping import export_plan, mapping_plan


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


def test_rviz_profile_keeps_slam_and_octomap_topics_distinct():
    config = yaml.safe_load((Path(__file__).parents[1] / "config/mapping.rviz").read_text())
    displays = config["Visualization Manager"]["Displays"]
    topics = {display["Name"]: display.get("Topic", {}).get("Value")
              for display in displays}
    assert topics["SLAM 2D occupancy (/mapping/map)"] == "/robodog/mapping/map"
    assert topics["OctoMap 2D projection (/mapping/octomap_grid)"] == "/robodog/mapping/octomap_grid"
    assert topics["OctoMap occupied voxels"] == "/robodog/mapping/octomap_occupied_space"


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


def test_room_scan_export_uses_safe_unique_session_directory(tmp_path):
    result = export_plan(output_directory=str(tmp_path), name="kitchen-west")
    assert result.output_root == tmp_path
    assert result.session_name == "kitchen-west"
    assert result.session_directory == tmp_path / "kitchen-west"
    assert not result.session_directory.exists()


@pytest.mark.parametrize("name", ["../escape", "room/scan", "room scan", ".hidden", ""])
def test_room_scan_export_rejects_unsafe_or_missing_explicit_names(tmp_path, name):
    with pytest.raises(ValueError, match="name"):
        export_plan(output_directory=str(tmp_path), name=name, generate_name=False)


def test_room_scan_export_refuses_to_overwrite_an_existing_session(tmp_path):
    (tmp_path / "kitchen").mkdir()
    with pytest.raises(FileExistsError, match="already exists"):
        export_plan(output_directory=str(tmp_path), name="kitchen")


def test_room_scan_export_requires_an_absolute_output_directory(tmp_path):
    with pytest.raises(ValueError, match="absolute"):
        export_plan(output_directory="relative/maps", name="kitchen")


def test_assembled_room_cloud_is_exported_as_binary_pcd():
    cloud = PointCloud2()
    cloud.header.frame_id = "map"
    cloud.height = 1
    cloud.width = 2
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 16
    cloud.row_step = 32
    cloud.data = struct.pack("<ffffffff", 1.0, 2.0, 3.0, 4.0, -1.0, -2.0, -3.0, 8.0)

    pcd, metadata = pointcloud_to_pcd(cloud)

    header, payload = pcd.split(b"DATA binary\n", 1)
    assert b"FIELDS x y z rgb" in header
    assert b"WIDTH 2\nHEIGHT 1\n" in header
    assert b"POINTS 2" in header
    assert payload == bytes(cloud.data)
    assert metadata == {"frame_id": "map", "points": 2, "fields": ["x", "y", "z", "rgb"]}


def test_room_cloud_export_rejects_empty_or_non_xyz_clouds():
    cloud = PointCloud2()
    with pytest.raises(ValueError, match="empty"):
        pointcloud_to_pcd(cloud)


def test_room_cloud_export_rejects_fields_outside_point_stride():
    cloud = PointCloud2()
    cloud.header.frame_id = "map"
    cloud.height = 1
    cloud.width = 1
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="bad", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.point_step = 12
    cloud.row_step = 16
    cloud.data = struct.pack("<ffff", 1.0, 2.0, 3.0, 99.0)

    with pytest.raises(ValueError, match="extends past its point stride"):
        pointcloud_to_pcd(cloud)


def test_room_scan_export_publishes_complete_manifest_last(tmp_path):
    database = tmp_path / "house.db"
    database.write_bytes(b"live database")
    backup = Path(str(database) + ".back")
    cloud = PointCloud2()
    cloud.header.frame_id = "map"
    cloud.height = 1
    cloud.width = 1
    cloud.fields = [
        PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
        for name, offset in (("x", 0), ("y", 4), ("z", 8))
    ]
    cloud.point_step = 12
    cloud.row_step = 12
    cloud.data = struct.pack("<fff", 1.0, 2.0, 3.0)

    def flush_database():
        backup.write_bytes(b"consistent backup")
        return backup

    def save_octomap(path):
        path.write_bytes(b"# Octomap OcTree file\nid ColorOcTree\n")

    result = RoomScanExporter(
        database_path=database,
        cloud_provider=lambda: cloud,
        flush_database=flush_database,
        save_octomap=save_octomap,
    ).export(export_plan(output_directory=str(tmp_path), name="kitchen"))

    assert result.manifest_path == tmp_path / "kitchen/manifest.json"
    assert (tmp_path / "kitchen/cloud_map.pcd").read_bytes().endswith(struct.pack("<fff", 1, 2, 3))
    assert (tmp_path / "kitchen/octomap.ot").read_bytes().startswith(b"# Octomap")
    assert (tmp_path / "kitchen/rtabmap.db.back").read_bytes() == b"consistent backup"
    manifest = result.manifest
    assert manifest["complete"] is True
    assert manifest["point_cloud"]["source_topic"] == "/robodog/mapping/cloud_map"
    assert manifest["point_cloud"]["points"] == 1
    assert set(manifest["files"]) == {"cloud_map.pcd", "octomap.ot", "rtabmap.db.back"}
    assert all(len(details["sha256"]) == 64 for details in manifest["files"].values())


def test_save_map_cli_accepts_named_absolute_destination(tmp_path):
    args = parse_save_map_args(["--output-dir", str(tmp_path), "--name", "living-room"])
    assert args.output_directory == str(tmp_path)
    assert args.name == "living-room"


def test_save_map_cli_rejects_relative_destination():
    with pytest.raises(SystemExit):
        parse_save_map_args(["--output-dir", "relative/path"])


def test_room_scan_rejects_a_stale_assembled_cloud():
    cloud = PointCloud2()
    assert require_fresh_cloud(cloud, received_at=10.0, now=14.9,
                               max_age_s=5.0) is cloud
    with pytest.raises(RuntimeError, match="stale"):
        require_fresh_cloud(cloud, received_at=10.0, now=15.1,
                            max_age_s=5.0)


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
    export_directory = tmp_path / "exports"
    actions = bringup._setup(launch_context(
        bringup, mapping="rtabmap", backend="mujoco", world="house",
        mapping_database=str(database), mapping_export_directory=str(export_directory)))
    mapping_launch = next(action for action in actions if isinstance(action, IncludeLaunchDescription))
    assert dict(mapping_launch.launch_arguments)["mapping_database"] == str(database)
    assert dict(mapping_launch.launch_arguments)["mapping_export_directory"] == str(export_directory)
    assert dict(mapping_launch.launch_arguments)["world"] == "house"
    assert not database.exists()
