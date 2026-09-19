"""
Simulation-only launch: MuJoCo physics plus the RViz world markers.

Normally included by robodog_bringup/robot.launch.py; run it directly only to
check the simulation in isolation.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def _setup(context, *args, **kwargs):
    from ament_index_python.packages import get_package_share_directory
    world = LaunchConfiguration("world").perform(context)
    model = {"house": "robodog_house.xml", "flat": "robodog_scene.xml"}[world]
    path = os.path.join(get_package_share_directory("robodog_sim"), "models", model)
    if not os.path.exists(path):
        raise RuntimeError(
            f"{path} is missing. Generate the models first:\n"
            f"  ros2 run robodog_sim generate_models")
    return [
        Node(package="robodog_sim", executable="world_markers", name="robodog_world_markers",
             output="screen",
             parameters=[{"world": world,
                          "frame_id": LaunchConfiguration("world_frame")}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="house", choices=["house", "flat"]),
        DeclareLaunchArgument("world_frame", default_value="odom"),
        OpaqueFunction(function=_setup),
    ])
