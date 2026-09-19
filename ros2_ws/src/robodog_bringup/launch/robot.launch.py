"""
Top-level launch for the robodog quadruped.

    # default: ideal joints, no physics -- fastest way to exercise the stack
    ros2 launch robodog_bringup robot.launch.py

    # full physics in the five-room house, with the web GUI and RViz
    ros2 launch robodog_bringup robot.launch.py backend:=mujoco world:=house

    # the real robot (requires calibration to have been signed off)
    ros2 launch robodog_bringup robot.launch.py backend:=robstride02_can \
        camera_backend:=nuwa_hp60c

The SAME node graph runs in every case. `backend` and `camera_backend` choose
implementations behind the hardware and camera boundaries; no topic, service,
frame or controller changes between simulation and hardware.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGS = [
    ("backend", "kinematic", ["kinematic", "mujoco", "robstride02_can"],
     "joint backend; robstride02_can drives REAL motors"),
    ("camera_backend", "sim", ["sim", "nuwa_hp60c", "none"],
     "camera backend; nuwa_hp60c drives the REAL camera"),
    ("world", "flat", ["flat", "house"], "simulation world"),
    ("rviz", "true", None, "start RViz"),
    ("web", "true", None, "start the web GUI server"),
    ("mujoco_viewer", "false", None, "open the MuJoCo viewer window"),
    ("auto_enable", "true", None, "energise the joints at start-up"),
    ("auto_stand", "true", None, "move to the stand pose at start-up"),
    ("control_rate_hz", "400.0", None, "control loop rate"),
    ("web_port", "8080", None, "web GUI port"),
    ("use_camera", "true", None, "include the camera in the robot model"),
]


def _setup(context, *a, **kw):
    cfg = lambda n: LaunchConfiguration(n).perform(context)
    backend = cfg("backend")
    world = cfg("world")
    cam_backend = cfg("camera_backend")
    is_sim = backend != "robstride02_can"

    model = os.path.join(get_package_share_directory("robodog_sim"), "models",
                         "robodog_house.xml" if world == "house" else "robodog_scene.xml")
    if backend == "mujoco" and not os.path.exists(model):
        raise RuntimeError(f"{model} is missing. Run: ros2 run robodog_sim generate_models")

    banner = (f"robodog: backend={backend} camera={cam_backend} world={world} "
              f"rate={cfg('control_rate_hz')} Hz")
    if not is_sim:
        banner += "\n  *** REAL HARDWARE: confirm the calibration in " \
                  "robodog_hardware/config/robstride_bus.yaml before enabling ***"

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([FindPackageShare("robodog_description"),
                                  "urdf", "robodog.urdf.xacro"]),
            " hardware:=", "sim" if is_sim else "real",
            " use_camera:=", LaunchConfiguration("use_camera"),
        ]), value_type=str)

    nodes = [
        LogInfo(msg=banner),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_description}]),
        Node(package="robodog_control", executable="control_node",
             name="robodog_control_node", output="screen", emulate_tty=True,
             parameters=[
                 PathJoinSubstitution([FindPackageShare("robodog_control"),
                                       "config", "control.yaml"]),
                 {"backend": backend,
                  "mujoco_model": model,
                  "mujoco_viewer": LaunchConfiguration("mujoco_viewer"),
                  "auto_enable": LaunchConfiguration("auto_enable"),
                  "auto_stand": LaunchConfiguration("auto_stand"),
                  "control_rate_hz": LaunchConfiguration("control_rate_hz")},
             ]),
    ]

    # The world markers describe the test environment. They are published in
    # simulation, and can also be published on the real robot to show the map it
    # was developed against around the live TF tree.
    if is_sim:
        nodes.append(Node(package="robodog_sim", executable="world_markers",
                          name="robodog_world_markers", output="screen",
                          parameters=[{"world": world, "frame_id": "odom"}]))

    if cam_backend != "none":
        nodes.append(Node(package="robodog_perception", executable="camera_node",
                          name="robodog_camera_node", output="screen",
                          parameters=[{"backend": cam_backend,
                                       "mujoco_model": model,
                                       "rate_hz": 15.0}]))

    # web.yaml holds a list of panel definitions, which is structured data
    # rather than ROS parameters; the server loads it by path. Only the
    # bind address is a parameter.
    nodes.append(Node(
        package="robodog_web", executable="web_server", name="robodog_web_server",
        output="screen", condition=IfCondition(LaunchConfiguration("web")),
        parameters=[{"port": LaunchConfiguration("web_port")}]))

    nodes.append(Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", PathJoinSubstitution([FindPackageShare("robodog_description"),
                                               "rviz", "robodog.rviz"])]))
    return nodes


def generate_launch_description():
    decls = [DeclareLaunchArgument(n, default_value=d, description=h,
                                   **({"choices": c} if c else {}))
             for n, d, c, h in ARGS]
    return LaunchDescription(decls + [OpaqueFunction(function=_setup)])
