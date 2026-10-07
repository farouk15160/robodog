"""Optional RGB-D SLAM and graph-corrected OctoMap using existing odometry TF."""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robodog_perception.mapping import mapping_plan


def _setup(context):
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    plan = mapping_plan(mode="rtabmap", backend=cfg("backend"),
                        camera_backend=cfg("camera_backend"),
                        use_camera=cfg("use_camera").lower() == "true",
                        world=cfg("world"), database=cfg("mapping_database"))
    try:
        get_package_share_directory("rtabmap_slam")
    except PackageNotFoundError as exc:
        raise RuntimeError("Mapping needs RTAB-Map: install ros-humble-rtabmap-ros "
                           "and source /opt/ros/humble/setup.bash.") from exc
    try:
        get_package_share_directory("octomap_server")
    except PackageNotFoundError as exc:
        raise RuntimeError("Map export needs OctoMap server: install ros-humble-octomap-server.") from exc
    plan.database_path.parent.mkdir(parents=True, exist_ok=True)
    config = os.path.join(get_package_share_directory("robodog_perception"),
                          "config", "rtabmap.yaml")
    return [
        LogInfo(msg=f"RGB-D SLAM + OctoMap session: {plan.database_path}; "
                    "simulation ground-truth odometry, outputs /robodog/mapping/*"),
        Node(package="rtabmap_slam", executable="rtabmap", name="rtabmap",
             namespace="robodog/mapping", output="screen",
             parameters=[config, {"database_path": str(plan.database_path)}],
             remappings=[
                 ("rgb/image", "/robodog/camera/color/image_raw"),
                 ("depth/image", "/robodog/camera/depth/image_rect_raw"),
                 ("rgb/camera_info", "/robodog/camera/color/camera_info"),
             ]),
        Node(package="robodog_perception", executable="map_export_node", name="map_export",
             namespace="robodog/mapping", output="screen",
             parameters=[{"database_path": str(plan.database_path),
                          "output_directory": cfg("mapping_export_directory")}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="mujoco"),
        DeclareLaunchArgument("camera_backend", default_value="sim"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("world", default_value="flat"),
        DeclareLaunchArgument("mapping_database", default_value="",
                              description="Existing/new database; default creates a timestamped session"),
        DeclareLaunchArgument("mapping_export_directory", default_value="",
                              description="Absolute room-scan export root; default is under ROS_HOME"),
        OpaqueFunction(function=_setup),
    ])
