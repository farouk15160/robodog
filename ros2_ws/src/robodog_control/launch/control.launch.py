"""Control node only. Normally included by robodog_bringup, not run directly."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("backend", default_value="kinematic",
                              choices=["kinematic", "mujoco", "robstride02_can"],
                              description="hardware backend; robstride02_can drives REAL motors"),
        DeclareLaunchArgument("mujoco_model", default_value=""),
        DeclareLaunchArgument("mujoco_viewer", default_value="false"),
        DeclareLaunchArgument("auto_enable", default_value="false"),
        DeclareLaunchArgument("auto_stand", default_value="false"),
        DeclareLaunchArgument("control_rate_hz", default_value="400.0"),
    ]
    node = Node(
        package="robodog_control", executable="control_node", name="robodog_control_node",
        output="screen", emulate_tty=True,
        parameters=[
            PathJoinSubstitution([FindPackageShare("robodog_control"), "config", "control.yaml"]),
            {"backend": LaunchConfiguration("backend"),
             "mujoco_model": LaunchConfiguration("mujoco_model"),
             "mujoco_viewer": LaunchConfiguration("mujoco_viewer"),
             "auto_enable": LaunchConfiguration("auto_enable"),
             "auto_stand": LaunchConfiguration("auto_stand"),
             "control_rate_hz": LaunchConfiguration("control_rate_hz")},
        ])
    return LaunchDescription(args + [node])
