"""Camera node only. Included by robodog_bringup."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_backend", default_value="sim",
                              choices=["sim", "nuwa_hp60c"],
                              description="nuwa_hp60c drives the REAL camera"),
        DeclareLaunchArgument("camera_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("mujoco_model", default_value=""),
        DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("publish_pointcloud", default_value="true"),
        Node(package="robodog_perception", executable="camera_node",
             name="robodog_camera_node", output="screen",
             parameters=[{
                 "backend": LaunchConfiguration("camera_backend"),
                 "rate_hz": LaunchConfiguration("camera_rate_hz"),
                 "mujoco_model": LaunchConfiguration("mujoco_model"),
                 "device": LaunchConfiguration("camera_device"),
                 "publish_pointcloud": LaunchConfiguration("publish_pointcloud"),
             }]),
    ])
