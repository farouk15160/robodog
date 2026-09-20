#!/usr/bin/env python3
"""
Architecture diagrams for docs/robodog_architecture.tex.

Each diagram is described once here and rendered twice: an editable Draw.io
file and a vector PDF for LaTeX. See tools/diagram_lib.py.

Run:  python3 tools/make_diagrams.py
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from diagram_lib import Diagram, render  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# --------------------------------------------------------------------------- #
def system_architecture() -> Diagram:
    d = Diagram("system_architecture", "System architecture", w=1180, h=760,
                caption="Physical and computational stack. Everything above the red "
                        "boundary is identical in simulation and on hardware.")
    d.box("L1", 20, 20, 1140, 150, "OPERATOR", "off-board", kind="layer", z=0)
    d.box("gui", 60, 60, 240, 80, "Web GUI", "browser, any device", kind="external")
    d.box("rviz", 330, 60, 200, 80, "RViz2", "3D state + sensors", kind="external")
    d.box("cli", 560, 60, 200, 80, "ros2 CLI / tools", "pose, joint_test, param", kind="external")
    d.box("rec", 790, 60, 330, 80, "rosbag2 / diagnostics", "recording and replay",
          kind="external", dashed=True)

    d.box("L2", 20, 200, 1140, 300, "ONBOARD COMPUTER", "ROS 2 Jazzy, Ubuntu 24.04",
          kind="layer", z=0)
    d.box("web", 60, 245, 230, 70, "robodog_web", "aiohttp + WebSocket", kind="package")
    d.box("ctrl", 320, 245, 300, 110, "robodog_control", "400 Hz control loop,\nsafety, gait, estimator",
          kind="package")
    d.box("perc", 650, 245, 230, 70, "robodog_perception", "RGB-D, PointCloud2", kind="package")
    d.box("desc", 910, 245, 210, 70, "robodog_description", "URDF/Xacro, TF", kind="package")
    d.box("hw", 320, 390, 300, 75, "robodog_hardware", "JointBackend + RS02 CAN codec",
          kind="package")
    d.box("sim", 650, 390, 230, 75, "robodog_sim", "MuJoCo model + worlds",
          kind="package", dashed=True)
    d.box("msgs", 910, 390, 210, 75, "robodog_msgs", "interface contract", kind="package")

    d.box("BND", 20, 520, 1140, 40, "HARDWARE / SOFTWARE BOUNDARY",
          "JointBackend  +  CameraBackend", kind="boundary", z=0, rounded=False)

    d.box("L3", 20, 590, 1140, 150, "PHYSICAL ROBOT", "10.0 kg, 12 DOF", kind="layer", z=0)
    d.box("can0", 60, 630, 200, 85, "CAN bus 0", "1 Mbit/s, front legs\n6 x RS02, 72% load",
          kind="hardware")
    d.box("can1", 285, 630, 200, 85, "CAN bus 1", "1 Mbit/s, rear legs\n6 x RS02, 72% load",
          kind="hardware")
    d.box("cam", 510, 630, 200, 85, "NUWA HP60C", "USB3, RGB + depth", kind="hardware")
    d.box("imu", 735, 630, 180, 85, "IMU", "accel + gyro", kind="hardware")
    d.box("pwr", 940, 630, 180, 85, "48 V battery", "12S, 700 g", kind="hardware")

    d.edge("gui", "web", "WebSocket JSON", style="thick")
    d.edge("rviz", "ctrl", "topics + TF")
    d.edge("cli", "ctrl", "services")
    d.edge("web", "ctrl", "cmd / telemetry")
    d.edge("ctrl", "hw", "read() / write()", style="thick")
    d.edge("perc", "sim", "render", style="dashed")
    d.edge("ctrl", "perc", "robot_state", style="dashed")
    d.edge("hw", "can0", "motion frames", style="thick")
    d.edge("hw", "can1", "motion frames", style="thick")
    d.edge("perc", "cam", "UVC", style="dashed")
    d.edge("ctrl", "imu", "fused by estimator", style="dashed")
    return d


def software_architecture() -> Diagram:
    d = Diagram("software_architecture", "Software architecture", w=1160, h=700,
                caption="ROS 2 package graph. Arrows point from dependant to dependency; "
                        "no package above depends on a simulation package.")
    d.box("bring", 430, 30, 300, 65, "robodog_bringup", "launch composition only")
    d.box("web", 60, 150, 230, 70, "robodog_web", "GUI server + protocol", kind="package")
    d.box("ctrl", 330, 150, 250, 70, "robodog_control", "loop, safety, gait, IK", kind="package")
    d.box("perc", 620, 150, 230, 70, "robodog_perception", "camera + point cloud", kind="package")
    d.box("sim", 890, 150, 210, 70, "robodog_sim", "MJCF + worlds", kind="package")
    d.box("hw", 330, 300, 250, 70, "robodog_hardware", "backends + RS02 codec", kind="package")
    d.box("desc", 620, 300, 230, 70, "robodog_description", "URDF, meshes, RViz", kind="package")
    d.box("msgs", 380, 440, 210, 65, "robodog_msgs", "msg / srv definitions", kind="package")

    d.box("gen", 60, 440, 300, 65, "tools/cad_to_model.py", "CAD -> robot_parameters.yaml",
          kind="store", dashed=True)
    d.box("cad", 60, 560, 300, 65, "cad/robot/onshape_export", "12 DOF, 85 parts", kind="store")
    d.box("params", 620, 440, 230, 65, "robot_parameters.yaml", "single source of truth",
          kind="store")
    d.box("rs02", 890, 440, 210, 65, "robstride02.yaml", "actuator datasheet", kind="store")

    for a in ("web", "ctrl", "perc", "sim"):
        d.edge("bring", a)
    d.edge("ctrl", "hw")
    d.edge("ctrl", "msgs")
    d.edge("web", "msgs")
    d.edge("perc", "msgs")
    d.edge("hw", "msgs")
    d.edge("ctrl", "desc")
    d.edge("sim", "desc")
    d.edge("perc", "desc")
    d.edge("desc", "params", style="dashed")
    d.edge("desc", "rs02", style="dashed")
    d.edge("sim", "params", style="dashed")
    d.edge("hw", "rs02", style="dashed")
    d.edge("cad", "gen", "parsed by")
    d.edge("gen", "params", "generates")
    return d


def node_topic_graph() -> Diagram:
    d = Diagram("node_topic_graph", "ROS node and topic graph", w=1260, h=820,
                caption="Nodes (green), topics (amber), services (red). "
                        "The graph is identical in simulation and on hardware.")
    d.box("ctrl", 480, 330, 290, 90, "robodog_control_node", "400 Hz loop", kind="node")
    d.box("rsp", 480, 60, 290, 60, "robot_state_publisher", kind="node")
    d.box("camn", 60, 330, 250, 70, "robodog_camera_node", "15 Hz", kind="node")
    d.box("webn", 940, 330, 260, 70, "robodog_web_server", "20 Hz push", kind="node")
    d.box("wm", 60, 60, 250, 60, "robodog_world_markers", kind="node")
    d.box("rviz", 940, 60, 260, 60, "rviz2", kind="external")

    d.box("t_js", 480, 190, 290, 46, "/joint_states", "sensor_msgs/JointState  100 Hz", kind="topic")
    d.box("t_tf", 830, 190, 250, 46, "/tf", "odom -> base_link", kind="topic")
    d.box("t_wm", 60, 190, 250, 46, "/robodog/world_markers", "MarkerArray (latched)", kind="topic")

    # inputs on the left, outputs on the right, services in the middle
    d.box("t_jc", 60, 470, 270, 46, "/robodog/joint_command", "JointCommandArray", kind="topic")
    d.box("t_gc", 60, 530, 270, 46, "/robodog/gait_command", "GaitCommand", kind="topic")
    d.box("t_cv", 60, 590, 270, 46, "/cmd_vel", "geometry_msgs/Twist", kind="topic")
    d.box("t_col", 60, 660, 270, 42, "camera/color/image_raw", kind="topic")
    d.box("t_dep", 60, 710, 270, 42, "camera/depth/image_rect_raw", kind="topic")
    d.box("t_pc", 60, 760, 270, 42, "camera/depth/points", "PointCloud2  10 Hz", kind="topic")

    d.box("t_rs", 480, 470, 290, 46, "/robodog/robot_state", "RobotState  50 Hz", kind="topic")
    d.box("t_ss", 830, 470, 250, 46, "/robodog/safety_status", "SafetyStatus", kind="topic")
    d.box("t_imu", 830, 530, 250, 46, "/robodog/imu", "sensor_msgs/Imu", kind="topic")

    d.box("srv", 480, 580, 290, 150, "Services",
          "set_named_pose\nset_gait\nset_control_mode\nenable_joints\nemergency_stop",
          kind="service")

    d.edge("ctrl", "t_js", route="v")
    d.edge("t_js", "rsp", route="v")
    d.edge("rsp", "t_tf")
    d.edge("ctrl", "t_rs", route="v")
    d.edge("ctrl", "t_ss")
    d.edge("ctrl", "t_imu")
    d.edge("t_rs", "webn")
    d.edge("t_rs", "camn", "sim pose only", style="dashed")
    d.edge("t_jc", "ctrl")
    d.edge("t_gc", "ctrl")
    d.edge("t_cv", "ctrl")
    d.edge("webn", "t_jc", style="dashed")
    d.edge("webn", "t_cv", style="dashed")
    d.edge("webn", "srv", style="dashed")
    d.edge("srv", "ctrl", route="v")
    d.edge("camn", "t_col", route="v")
    d.edge("camn", "t_dep", route="v")
    d.edge("camn", "t_pc", route="v")
    d.edge("wm", "t_wm", route="v")
    d.edge("t_wm", "rviz", style="dashed")
    d.edge("t_tf", "rviz")
    d.edge("t_pc", "rviz", style="dashed")
    return d


def control_architecture() -> Diagram:
    d = Diagram("control_architecture", "Control architecture", w=1140, h=790,
                caption="Rates fall by roughly an order of magnitude per layer. "
                        "The innermost loop runs in the actuator, not on the host.")
    d.box("l4", 40, 30, 1060, 110, "LAYER 4  BEHAVIOUR", "on demand", kind="layer", z=0)
    d.box("op", 80, 70, 300, 55, "Operator / autonomy", "cmd_vel, gait, named pose")
    d.box("traj", 410, 70, 300, 55, "Trajectory", "minimum jerk, speed limited")
    d.box("test", 740, 70, 320, 55, "Joint test", "sine / sweep / step / sequential")

    d.box("l3", 40, 170, 1060, 110, "LAYER 3  GAIT AND POSE", "400 Hz", kind="layer", z=0)
    d.box("gait", 80, 210, 330, 55, "Gait generator", "phase, duty, swing / stance")
    d.box("ik", 440, 210, 280, 55, "Leg IK + Jacobian", "closed form, knee-back branch")
    d.box("ff", 750, 210, 310, 55, "Gravity feed-forward", "tau = -J^T f, per stance leg")

    d.box("l2", 40, 310, 1060, 110, "LAYER 2  SAFETY", "400 Hz, every cycle", kind="layer", z=0)
    d.box("lim", 80, 350, 250, 55, "Position / velocity", "soft limits + rate limit")
    d.box("tq", 360, 350, 250, 55, "Torque + I2t", "predict, scale kp/kd/tau")
    d.box("th", 640, 350, 200, 55, "Thermal", "warn / derate / fault")
    d.box("wd", 870, 350, 190, 55, "Watchdog + E-stop", "latching")

    d.box("l1", 40, 450, 1060, 110, "LAYER 1  JOINT INTERFACE", "400 Hz", kind="layer", z=0)
    d.box("be", 80, 490, 450, 55, "JointBackend.write(JointCommand)",
          "position, velocity, effort, kp, kd")
    d.box("st", 560, 490, 500, 55, "JointBackend.read() -> JointState",
          "position, velocity, effort, temperature, faults")

    d.box("l0", 40, 590, 1060, 170, "LAYER 0  ACTUATOR", "firmware, tens of kHz",
          kind="layer", z=0)
    d.box("can", 80, 630, 300, 55, "CAN motion-control frame", "1 command per joint per cycle",
          kind="hardware")
    d.box("foc", 410, 630, 340, 55, "RS02 impedance law + FOC",
          "tau = kp(q*-q) + kd(qd*-qd) + tau_ff", kind="hardware")
    d.box("enc", 780, 630, 280, 55, "2 x 14-bit encoders", "3.8e-4 rad output", kind="hardware")
    d.box("note", 80, 700, 980, 42,
          "The host sets SETPOINTS at 400 Hz; the servo loop is closed in the actuator. "
          "This is why 400 Hz is not a compromise.", kind="note", dashed=True)

    d.edge("op", "gait", route="v")
    d.edge("traj", "ik", route="v")
    d.edge("test", "ff", route="v")
    d.edge("gait", "lim", route="v")
    d.edge("ik", "tq", route="v")
    d.edge("ff", "th", route="v")
    d.edge("lim", "be", route="v")
    d.edge("tq", "be", route="v")
    d.edge("th", "be", route="v")
    d.edge("wd", "st", route="v")
    d.edge("be", "can", route="v", style="thick")
    d.edge("can", "foc", route="h")
    d.edge("foc", "enc", route="h")
    d.edge("enc", "st", route="v", style="thick")
    return d


def data_flow() -> Diagram:
    d = Diagram("data_flow", "Data flow, one control cycle", w=1280, h=430,
                caption="One 2.5 ms cycle. The safety monitor is unconditional: "
                        "nothing reaches the actuators without passing it.")
    y = 120
    d.box("read", 30, y, 180, 80, "read()", "JointState\n12 x pos/vel/tau/T")
    d.box("est", 235, y, 180, 80, "State estimator", "attitude + leg odometry")
    d.box("ctl", 440, y, 190, 80, "Active controller", "idle | joint | pose |\ngait | joint_test")
    d.box("saf", 655, y, 190, 80, "Safety monitor", "clamp, I2t, thermal,\nwatchdog, e-stop")
    d.box("wr", 870, y, 170, 80, "write()", "JointCommand")
    d.box("act", 1065, y, 185, 80, "Actuators", "CAN or MuJoCo", kind="hardware")

    d.box("pub1", 440, 290, 190, 60, "/joint_states", "every 4th cycle", kind="topic")
    d.box("pub2", 655, 290, 190, 60, "/robodog/robot_state", "every 8th cycle", kind="topic")
    d.box("pub3", 870, 290, 170, 60, "/tf", "odom->base_link", kind="topic")
    d.box("in", 440, 20, 190, 60, "commands in", "topics + services", kind="topic")

    d.edge("read", "est")
    d.edge("est", "ctl")
    d.edge("ctl", "saf")
    d.edge("saf", "wr")
    d.edge("wr", "act", style="thick")
    d.edge("act", "read", "next cycle", style="dashed", route="v")
    d.edge("in", "ctl", route="v")
    d.edge("ctl", "pub1", route="v", style="dashed")
    d.edge("saf", "pub2", route="v", style="dashed")
    d.edge("est", "pub3", style="dashed")
    return d


def hardware_boundary() -> Diagram:
    d = Diagram("hardware_boundary", "Hardware / software boundary", w=1180, h=700,
                caption="Replacing simulation with hardware is a launch argument. "
                        "Nothing above the boundary changes.")
    d.box("above", 30, 30, 1120, 160, "UNCHANGED BY THE SWAP", "", kind="layer", z=0)
    d.box("cn", 70, 75, 280, 85, "robodog_control_node", "loop, safety, gait, estimator")
    d.box("wn", 380, 75, 240, 85, "robodog_web_server", "GUI protocol")
    d.box("pn", 650, 75, 240, 85, "robodog_camera_node", "publishes RGB-D")
    d.box("rz", 920, 75, 200, 85, "RViz2 / rosbag2", "same topics", kind="external")

    d.box("B1", 30, 220, 550, 40, "JointBackend", "abstract: configure/enable/read/write/step",
          kind="boundary", z=0, rounded=False)
    d.box("B2", 610, 220, 540, 40, "CameraBackend", "abstract: configure/capture/intrinsics",
          kind="boundary", z=0, rounded=False)

    d.box("sim1", 60, 310, 230, 100, "KinematicBackend", "ideal 2nd-order joints\nno physics, runs in CI",
          kind="package", dashed=True)
    d.box("sim2", 320, 310, 230, 100, "MujocoBackend", "contact, armature,\n1 ms command delay",
          kind="package", dashed=True)
    d.box("cam1", 640, 310, 230, 100, "SimCameraBackend", "MuJoCo render + z^2\nrange noise",
          kind="package", dashed=True)
    d.box("real1", 60, 450, 490, 110, "RobStride02Backend", kind="hardware")
    d.box("real2", 640, 450, 230, 110, "NuwaHP60CBackend", kind="hardware")
    d.box("d1", 80, 490, 210, 55, "2 x CAN @ 1 Mbit/s", "python-can, 400 Hz", kind="hardware")
    d.box("d2", 315, 490, 215, 55, "RS02 frame codec", "pure, 29 unit tests", kind="hardware")
    d.box("d3", 660, 490, 190, 55, "UVC / vendor SDK", "USB 3.0", kind="hardware")

    d.box("map", 900, 310, 250, 250, "Per-joint mapping",
          "direction  +1 / -1\noffset_rad\nbus, motor_id\n\nThe ONLY place motor\n"
          "coordinates differ from\ncanonical joint coordinates.\n\nSet by\n"
          "calibrate_joint", kind="store")

    d.edge("cn", "B1", route="v", style="thick")
    d.edge("pn", "B2", route="v", style="thick")
    d.edge("B1", "sim1", route="v")
    d.edge("B1", "sim2", route="v")
    d.edge("B1", "real1", route="v", style="thick")
    d.edge("B2", "cam1", route="v")
    d.edge("B2", "real2", route="v", style="thick")
    d.edge("real1", "map", route="h", style="dashed")
    return d


def state_machine() -> Diagram:
    d = Diagram("state_machine", "Robot state machine", w=1080, h=620,
                caption="FAULT and E-STOP are reachable from every state. "
                        "Leaving E-STOP requires the cause to have cleared.")
    d.box("init", 60, 60, 180, 70, "INIT", "backend configuring", kind="state")
    d.box("idle", 300, 60, 180, 70, "IDLE", "powered, joints off", kind="state")
    d.box("ready", 540, 60, 180, 70, "READY", "enabled, holding", kind="state")
    d.box("stand", 540, 200, 180, 70, "STANDING", "pose reached", kind="state")
    d.box("move", 540, 340, 180, 70, "MOVING", "gait active", kind="state")
    d.box("fault", 830, 200, 190, 70, "FAULT", "limit exceeded", kind="state_bad")
    d.box("estop", 830, 340, 190, 70, "E-STOP", "output inhibited, latched", kind="state_bad")
    d.box("n1", 60, 200, 400, 120, "Transitions out of E-STOP",
          "clear_estop succeeds only when no\nhardware-protective fault is still true:\n"
          "over-temperature, over-current,\nunder-voltage, encoder, communication.",
          kind="note", dashed=True)
    d.box("n2", 60, 360, 400, 130, "Latching faults",
          "A silently stale joint is the failure mode\n"
          "most likely to break a leg, so a\n"
          "communication timeout latches rather\n"
          "than degrading quietly.", kind="note", dashed=True)

    d.edge("init", "idle", "backend ready")
    d.edge("idle", "ready", "enable_joints")
    d.edge("ready", "stand", "set_named_pose", route="v")
    d.edge("stand", "move", "set_gait", route="v")
    d.edge("move", "stand", "gait: stand", route="h")
    d.edge("ready", "idle", "disable", route="h")
    d.edge("stand", "fault", "limit exceeded")
    d.edge("fault", "estop", "protective fault", route="v")
    d.edge("move", "estop", "operator STOP")
    d.edge("estop", "idle", "clear_estop", route="v")
    return d


def domain_model() -> Diagram:
    d = Diagram("domain_model", "Domain model", w=1160, h=700,
                caption="Value types crossing the hardware boundary and the entities "
                        "that own them.")
    d.box("robot", 440, 30, 280, 80, "Robot", "12 joints, 4 legs\n10.0 kg, base_link")
    d.box("leg", 440, 170, 280, 100, "Leg  (FL FR RL RR)",
          "sx: front/rear   sy: left/right\nLegGeometry: L1 L2 offsets")
    d.box("joint", 100, 170, 260, 100, "Joint",
          "kind: haa | hfe | kfe\nlimits, axis, dynamics")
    d.box("act", 100, 330, 260, 110, "Actuator  RS02",
          "6 N.m cont / 17 N.m peak\n7.75:1, 2 x 14-bit\nKt = 1.22 N.m/Arms")
    d.box("foot", 800, 170, 260, 100, "Foot",
          "20 mm contact sphere\ncontact, normal force")
    d.box("jc", 100, 500, 260, 90, "JointCommand", "mode, position, velocity,\neffort, kp, kd",
          kind="store")
    d.box("js", 400, 500, 260, 90, "JointState", "position, velocity, effort,\ntemperature, faults",
          kind="store")
    d.box("bs", 700, 500, 260, 90, "BaseState", "pose, twist, contact,\nground_truth flag",
          kind="store")
    d.box("gp", 800, 330, 260, 110, "GaitParams",
          "gait, frequency, duty,\nstep height, stance height,\nvelocity")
    d.box("sl", 440, 330, 280, 110, "SafetyLimits",
          "position soft limits\nvelocity, continuous/peak torque\ntemperature thresholds")

    d.edge("robot", "leg", "4", route="v")
    d.edge("leg", "joint", "3")
    d.edge("leg", "foot", "1")
    d.edge("joint", "act", "1", route="v")
    d.edge("act", "jc", "consumes", route="v")
    d.edge("act", "js", "produces")
    d.edge("robot", "bs", "has", style="dashed")
    d.edge("leg", "gp", "driven by", style="dashed")
    d.edge("sl", "jc", "clamps", style="dashed")
    return d


def simulation_architecture() -> Diagram:
    d = Diagram("simulation_architecture", "Simulation architecture", w=1180, h=680,
                caption="The MJCF and the URDF are generated from the same parameters, "
                        "so the physics and the kinematics cannot drift apart.")
    d.box("cad", 40, 40, 250, 70, "Onshape CAD export", "85 parts, 12 revolute", kind="store")
    d.box("gen", 340, 40, 270, 70, "tools/cad_to_model.py", "FK, body aggregation, mass swap")
    d.box("par", 670, 40, 250, 70, "robot_parameters.yaml", "kinematics, inertia, visuals",
          kind="store")
    d.box("rs", 960, 40, 180, 70, "robstride02.yaml", "datasheet", kind="store")

    d.box("xac", 500, 170, 250, 70, "robodog.urdf.xacro", "RViz, TF, IK")
    d.box("mj", 800, 170, 250, 70, "robodog_sim/mjcf.py", "MJCF generator")
    d.box("mesh", 180, 170, 250, 70, "tools/prepare_meshes.py", "41 MB -> 2.3 MB", kind="store")

    d.box("world", 40, 300, 280, 100, "world_spec + worlds/house.py",
          "5 rooms, doors, stairs,\nnarrow gaps, movable props", kind="store")
    d.box("check", 40, 430, 280, 70, "world_check.py", "measures every documented gap")
    d.box("models", 800, 300, 250, 70, "models/*.xml", "robot, flat, house", kind="store")
    d.box("mark", 40, 560, 280, 70, "world_markers node", "same spec -> RViz MarkerArray")

    d.box("bk", 500, 300, 250, 100, "MujocoBackend",
          "impedance law at 2 kHz\narmature 4.8e-3\n1 ms command delay", kind="package")
    d.box("cam", 500, 440, 250, 90, "SimCameraBackend",
          "own model instance,\nrenders at 15 Hz", kind="package")
    d.box("note", 800, 420, 340, 190, "Fidelity choices that matter",
          "impedance evaluated at the PHYSICS\nrate, not the control rate\n\n"
          "armature = reflected rotor inertia,\nlarger than the calf's own\n\n"
          "znear/zfar pinned in metres via\nstatistic/extent\n\n"
          "depth noise grows as z^2", kind="note", dashed=True)

    d.edge("cad", "gen")
    d.edge("gen", "par")
    d.edge("par", "xac", route="v")
    d.edge("par", "mj", route="v")
    d.edge("rs", "mj", route="v")
    d.edge("cad", "mesh", route="v")
    d.edge("mesh", "xac")
    d.edge("mj", "models", route="v")
    d.edge("world", "mj")
    d.edge("world", "check", route="v")
    d.edge("world", "mark", route="v")
    d.edge("models", "bk", route="h")
    d.edge("models", "cam", style="dashed")
    return d


def perception_pipeline() -> Diagram:
    d = Diagram("perception_pipeline", "Perception pipeline", w=1250, h=500,
                caption="One configuration file drives both backends, so the simulated "
                        "camera has the real device's intrinsics and range.")
    d.box("cfg", 30, 30, 250, 70, "nuwa_hp60c.yaml", "resolution, FOV, range, baseline",
          kind="store")
    d.box("simb", 30, 160, 250, 90, "SimCameraBackend", "MuJoCo render\n+ range limits + z^2 noise",
          kind="package", dashed=True)
    d.box("realb", 30, 290, 250, 90, "NuwaHP60CBackend", "UVC / vendor SDK", kind="hardware")
    d.box("frame", 340, 210, 210, 90, "Frame", "colour uint8 RGB\ndepth float32 m, NaN",
          kind="store")
    d.box("node", 610, 210, 230, 90, "robodog_camera_node", "15 Hz")
    d.box("conv", 610, 60, 230, 90, "metres -> 16UC1 mm", "0 means invalid\n(ROS convention)")
    d.box("dep", 610, 340, 230, 90, "deproject + filter", "drop beyond\nmax_usable_range_m")

    d.box("t1", 920, 40, 290, 50, "camera/color/image_raw + info", kind="topic")
    d.box("t2", 920, 110, 290, 50, "camera/depth/image_rect_raw", kind="topic")
    d.box("t3", 920, 180, 290, 50, "camera/depth/points", kind="topic")
    d.box("fr", 920, 260, 290, 170, "Frame tree",
          "camera_link\n  camera_color_frame\n    camera_color_optical_frame\n"
          "  camera_depth_frame\n    camera_depth_optical_frame\n\n"
          "optical: z fwd, x right, y down", kind="note", dashed=True)

    d.edge("cfg", "simb", route="v")
    d.edge("cfg", "realb", route="v")
    d.edge("simb", "frame")
    d.edge("realb", "frame")
    d.edge("frame", "node")
    d.edge("node", "conv", route="v")
    d.edge("node", "dep", route="v")
    d.edge("conv", "t1")
    d.edge("conv", "t2")
    d.edge("dep", "t3")
    return d


DIAGRAMS = [system_architecture, software_architecture, node_topic_graph,
            control_architecture, data_flow, hardware_boundary, state_machine,
            domain_model, simulation_architecture, perception_pipeline]


def main() -> int:
    render([f() for f in DIAGRAMS], ROOT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
