"""
Inspect the URDF alone: robot_state_publisher, the joint-state slider GUI and
RViz. No simulation, no control loop, no camera.

    ros2 launch robodog_bringup display.launch.py
    ros2 launch robodog_bringup display.launch.py fixed_base:=true

Use this to check geometry, joint limits, frames and mesh placement. With
`collision:=true` the RViz configuration also shows the collision primitives
the MuJoCo model is built from.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([FindPackageShare("robodog_description"),
                                  "urdf", "robodog.urdf.xacro"]),
            " fixed_base:=", LaunchConfiguration("fixed_base"),
            " use_camera:=", LaunchConfiguration("use_camera"),
        ]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("fixed_base", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[{"robot_description": desc}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             output="screen"),
        Node(package="rviz2", executable="rviz2", output="log",
             arguments=["-d", PathJoinSubstitution([FindPackageShare("robodog_description"),
                                                    "rviz", "robodog_model.rviz"])]),
    ])
