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
                        "boundary shares commands and telemetry; hardware travel is gated.")
    d.box("L1", 20, 20, 1140, 150, "OPERATOR", "off-board", kind="layer", z=0)
    d.box("gui", 60, 60, 240, 80, "Web GUI", "joint RMS / peaks / thermal history", kind="external")
    d.box("rviz", 330, 60, 200, 80, "RViz2", "robot + 2D / 3D maps", kind="external")
    d.box("cli", 560, 60, 200, 80, "ros2 CLI / tools", "pose, joint_test, param", kind="external")
    d.box("rec", 790, 60, 330, 80, "rosbag2 / diagnostics", "recording and replay",
          kind="external", dashed=True)

    d.box("L2", 20, 200, 1140, 300, "ONBOARD COMPUTER", "ROS 2; Humble local validation",
          kind="layer", z=0)
    d.box("web", 60, 245, 230, 70, "robodog_web", "aiohttp + WebSocket", kind="package")
    d.box("ctrl", 320, 245, 300, 110, "robodog_control",
          "400 Hz loop, safety,\ngait, balance, estimator", kind="package")
    d.box("perc", 650, 245, 230, 70, "robodog_perception", "RGB-D, PointCloud2", kind="package")
    d.box("desc", 910, 245, 210, 70, "robodog_description", "URDF/Xacro, TF", kind="package")
    d.box("map", 60, 390, 230, 75, "RTAB-Map + OctoMap",
          "default for supported MuJoCo;\nmap -> odom", kind="package")
    d.box("hw", 320, 390, 300, 75, "robodog_hardware", "JointBackend + RS06 CAN codec",
          kind="package")
    d.box("sim", 650, 390, 230, 75, "robodog_sim", "MuJoCo model + worlds",
          kind="package", dashed=True)
    d.box("msgs", 910, 390, 210, 75, "robodog_msgs", "interface contract", kind="package")

    d.box("BND", 20, 520, 1140, 40, "HARDWARE / SOFTWARE BOUNDARY",
          "JointBackend  +  CameraBackend", kind="boundary", z=0, rounded=False)

    d.box("L3", 20, 590, 1140, 150, "PHYSICAL ROBOT", "19.72 kg, 12 DOF", kind="layer", z=0)
    d.box("can0", 60, 630, 200, 85, "CAN bus 0", "1 Mbit/s, front legs\n6 x RS06, 72% load",
          kind="hardware")
    d.box("can1", 285, 630, 200, 85, "CAN bus 1", "1 Mbit/s, rear legs\n6 x RS06, 72% load",
          kind="hardware")
    d.box("cam", 510, 630, 200, 85, "NUWA HP60C", "USB3 RGB; depth unavailable", kind="hardware")
    d.box("imu", 735, 630, 180, 85, "IMU", "integration unavailable", kind="hardware")
    d.box("pwr", 940, 630, 180, 85, "44.4 V battery", "2 x 6S series, 2.50 kg", kind="hardware")

    d.edge("gui", "web", "WebSocket JSON", style="thick")
    d.edge("rviz", "ctrl", "topics + TF")
    d.edge("cli", "ctrl", "services")
    d.edge("web", "ctrl", "cmd / telemetry")
    d.edge("ctrl", "hw", "read() / write()", style="thick")
    d.edge("perc", "sim", "render", style="dashed")
    d.edge("perc", "map", "registered RGB-D", style="dashed")
    d.edge("ctrl", "perc", "robot_state", style="dashed")
    d.edge("hw", "can0", "motion frames", style="thick")
    d.edge("hw", "can1", "motion frames", style="thick")
    d.edge("perc", "cam", "UVC", style="dashed")
    d.edge("ctrl", "imu", "not connected", style="dashed")
    return d


def software_architecture() -> Diagram:
    d = Diagram("software_architecture", "Software architecture", w=1160, h=700,
                caption="ROS 2 package graph. Arrows point from dependant to dependency. "
                        "RTAB-Map runs by default for the supported MuJoCo RGB-D pipeline.")
    d.box("bring", 430, 30, 300, 65, "robodog_bringup", "launch composition only")
    d.box("web", 60, 150, 230, 70, "robodog_web", "GUI server + protocol", kind="package")
    d.box("ctrl", 330, 150, 250, 70, "robodog_control",
          "loop, safety, gait,\nbalance, IK, estimator", kind="package")
    d.box("perc", 620, 150, 230, 70, "robodog_perception", "camera + point cloud\nmapping launch", kind="package")
    d.box("sim", 890, 150, 210, 70, "robodog_sim", "MJCF + worlds", kind="package")
    d.box("hw", 330, 300, 250, 70, "robodog_hardware", "backends + RS06 codec", kind="package")
    d.box("desc", 620, 300, 230, 70, "robodog_description", "URDF, meshes, RViz", kind="package")
    d.box("map", 890, 300, 210, 70, "rtabmap_slam",
          "SLAM + built-in OctoMap", kind="package")
    d.box("msgs", 380, 440, 210, 65, "robodog_msgs", "msg / srv definitions", kind="package")

    d.box("gen", 60, 440, 300, 65, "tools/cad_to_model.py", "CAD -> robot_parameters.yaml",
          kind="store", dashed=True)
    d.box("cad", 60, 560, 300, 65, "cad/urdf", "12 DOF, 264 links", kind="store")
    d.box("params", 620, 440, 230, 65, "robot_parameters.yaml", "single source of truth",
          kind="store")
    d.box("rs06", 890, 440, 210, 65, "robstride02.yaml", "actuator datasheet", kind="store")

    for a in ("web", "ctrl", "perc", "sim"):
        d.edge("bring", a)
    d.edge("ctrl", "hw")
    d.edge("ctrl", "msgs")
    d.edge("web", "msgs")
    d.edge("perc", "msgs")
    d.edge("perc", "map", "mapping launch")
    d.edge("hw", "msgs")
    d.edge("ctrl", "desc")
    d.edge("sim", "desc")
    d.edge("perc", "desc")
    d.edge("desc", "params", style="dashed")
    d.edge("desc", "rs06", style="dashed")
    d.edge("sim", "params", style="dashed")
    d.edge("hw", "rs06", style="dashed")
    d.edge("cad", "gen", "parsed by")
    d.edge("gen", "params", "generates")
    return d


def node_topic_graph() -> Diagram:
    d = Diagram("node_topic_graph", "ROS node and topic graph", w=1260, h=1080,
                caption="Nodes (green), topics (amber), services (red). "
                        "Mapping is automatic for the supported MuJoCo camera pipeline.")
    d.box("ctrl", 480, 330, 290, 90, "robodog_control_node", "400 Hz loop", kind="node")
    d.box("rsp", 480, 60, 290, 60, "robot_state_publisher", kind="node")
    d.box("camn", 60, 330, 250, 70, "robodog_camera_node", "15 Hz", kind="node")
    d.box("webn", 940, 330, 260, 70, "robodog_web_server", "20 Hz push", kind="node")
    d.box("wm", 60, 60, 250, 60, "robodog_world_markers", kind="node")
    d.box("rviz", 940, 60, 260, 60, "rviz2", kind="external")

    d.box("t_js", 480, 190, 290, 46, "/joint_states", "sensor_msgs/JointState  100 Hz", kind="topic")
    d.box("t_tf", 830, 185, 250, 60, "/tf",
          "control: odom -> base_link\nRTAB-Map: map -> odom", kind="topic")
    d.box("t_wm", 60, 190, 250, 46, "/robodog/world_markers", "MarkerArray (latched)", kind="topic")

    # inputs on the left, outputs on the right, services in the middle
    d.box("t_jc", 60, 470, 270, 46, "/robodog/joint_command", "JointCommandArray", kind="topic")
    d.box("t_gc", 60, 530, 270, 46, "/robodog/gait_command", "GaitCommand", kind="topic")
    d.box("t_cv", 60, 590, 270, 46, "/cmd_vel", "geometry_msgs/Twist", kind="topic")
    d.box("t_col", 60, 654, 270, 48, "camera/color/image_raw",
          "+ camera/color/camera_info", kind="topic")
    d.box("t_dep", 60, 710, 270, 42, "camera/depth/image_rect_raw", kind="topic")
    d.box("t_pc", 60, 760, 270, 42, "camera/depth/points", "PointCloud2  10 Hz", kind="topic")

    d.box("t_rs", 480, 470, 290, 46, "/robodog/robot_state", "RobotState  50 Hz", kind="topic")
    d.box("t_ss", 830, 470, 250, 46, "/robodog/safety_status", "SafetyStatus", kind="topic")
    d.box("t_imu", 830, 530, 250, 46, "/robodog/imu", "sensor_msgs/Imu", kind="topic")

    d.box("srv", 480, 580, 290, 150, "Services",
          "set_named_pose\nset_gait\nset_control_mode\nenable_joints\nemergency_stop",
          kind="service")

    d.box("mapn", 480, 850, 290, 75, "/robodog/mapping/rtabmap",
          "RGB-D SLAM + OctoMap, 2 Hz", kind="node")
    d.box("t_map", 60, 960, 330, 70, "/robodog/mapping/map",
          "2D occupancy, 5 cm cells", kind="topic")
    d.box("t_oct", 450, 960, 330, 70, "mapping/octomap_occupied_space",
          "PointCloud2; full / binary trees also available", kind="topic")
    d.box("mapdb", 870, 850, 330, 75, "Per-world map database",
          "preserved across restarts; .ot export", kind="store")
    d.edge("t_col", "mapn", style="dashed")
    d.edge("t_dep", "mapn", style="dashed")
    d.edge("mapn", "t_map", route="v")
    d.edge("mapn", "t_oct", route="v")
    d.edge("mapn", "mapdb")
    d.edge("mapn", "t_tf", style="dashed")
    d.edge("ctrl", "t_tf")
    d.edge("ctrl", "t_js", route="v")
    d.edge("t_js", "rsp", route="v")
    d.edge("rsp", "t_tf", "robot links")
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
    d.box("ff", 750, 210, 310, 55, "Balance",
          "body wrench -> per-foot force,\nfriction-cone projected")

    d.box("l2", 40, 310, 1060, 110, "LAYER 2  SAFETY", "400 Hz, every cycle", kind="layer", z=0)
    d.box("lim", 80, 350, 250, 55, "Position / velocity", "soft limits + rate limit")
    d.box("tq", 360, 350, 250, 55, "Torque + I2t", "predict, scale kp/kd/tau")
    d.box("th", 640, 350, 200, 55, "Thermal", "warn / derate / fault")
    d.box("wd", 870, 350, 190, 55, "Watchdog + E-stop", "latching")

    d.box("l1", 40, 450, 1060, 110, "LAYER 1  JOINT INTERFACE", "400 Hz", kind="layer", z=0)
    d.box("be", 80, 490, 450, 55, "JointBackend.write(JointCommand)",
          "joint coordinates; knee motor ratio 2:1")
    d.box("st", 560, 490, 500, 55, "JointBackend.read() -> JointState",
          "position, velocity, effort, temperature, faults")

    d.box("l0", 40, 590, 1060, 170, "LAYER 0  ACTUATOR", "firmware servo loop",
          kind="layer", z=0)
    d.box("can", 80, 630, 300, 55, "CAN motion-control frame", "1 command per joint per cycle",
          kind="hardware")
    d.box("foc", 410, 630, 340, 55, "RS06 impedance law + FOC",
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
    d.box("est", 235, y, 180, 80, "Base feedback", "sim truth; hardware absent")
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
                        "Hardware startup stays disabled; travel requires live base feedback.")
    d.box("above", 30, 30, 1120, 160, "UNCHANGED BY THE SWAP", "", kind="layer", z=0)
    d.box("cn", 70, 75, 280, 85, "robodog_control_node",
          "loop, safety, gait,\nbalance, estimator")
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
    d.box("real1", 60, 450, 490, 110, "RobStride06Backend", kind="hardware")
    d.box("real2", 640, 450, 230, 110, "NuwaHP60CBackend", kind="hardware")
    d.box("d1", 80, 490, 210, 55, "2 x CAN @ 1 Mbit/s", "python-can, 400 Hz", kind="hardware")
    d.box("d2", 315, 490, 215, 55, "RS06 frame codec", "model-specific scaling", kind="hardware")
    d.box("d3", 660, 490, 190, 55, "UVC / vendor SDK", "USB 3.0", kind="hardware")

    d.box("map", 900, 310, 250, 250, "Per-joint mapping",
          "direction, offset_rad\nratio, efficiency\nbus, motor_id, calibrated\n\nMaps motor output\n"
          "to canonical joints.\nKnee reduction 2:1.\n\nVerify with\n"
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
    d.box("move", 540, 340, 180, 70, "MOVING", "requires live base feedback", kind="state")
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
    d.edge("idle", "ready", "enable (calibrated)")
    d.edge("ready", "stand", "set_named_pose", route="v")
    d.edge("stand", "move", "set_gait + feedback", route="v")
    d.edge("move", "stand", "gait: stand", route="h")
    d.edge("ready", "idle", "disable", route="h")
    d.edge("stand", "fault", "limit exceeded")
    d.edge("fault", "estop", "protective fault", route="v")
    d.edge("move", "estop", "operator STOP")
    d.edge("estop", "idle", "clear_estop", route="v")
    return d


def domain_model() -> Diagram:
    d = Diagram("domain_model", "Domain model", w=1160, h=770,
                caption="Value types crossing the hardware boundary and the entities "
                        "that own them.")
    d.box("robot", 440, 30, 280, 80, "Robot", "12 joints, 4 legs\n19.72 kg, base_link")
    d.box("leg", 440, 170, 280, 100, "Leg  (FL FR RL RR)",
          "sx: front/rear   sy: left/right\nLegGeometry: L1 L2 offsets")
    d.box("joint", 100, 170, 260, 100, "Joint",
          "kind: haa | hfe | kfe\nlimits, axis, dynamics")
    d.box("act", 100, 330, 260, 110, "Actuator  RS06",
          "8 N.m stall / 11 N.m rotating\n36 N.m peak; knee belt 2:1\n9:1 internal; Kt = 1.1 N.m/Arms")
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
    d.box("bg", 100, 620, 260, 70, "BalanceGains",
          "height, roll, pitch, yaw,\nvelocity, friction cone", kind="store")

    d.edge("robot", "leg", "4", route="v")
    d.edge("leg", "joint", "3")
    d.edge("leg", "foot", "1")
    d.edge("joint", "act", "1", route="v")
    d.edge("act", "jc", "consumes", route="v")
    d.edge("act", "js", "produces")
    d.edge("robot", "bs", "has", style="dashed")
    d.edge("leg", "gp", "driven by", style="dashed")
    d.edge("sl", "jc", "clamps", style="dashed")
    d.edge("bg", "jc", "sizes tau_ff", style="dashed")
    return d


def simulation_architecture() -> Diagram:
    d = Diagram("simulation_architecture", "Simulation architecture", w=1180, h=680,
                caption="The MJCF and the URDF are generated from the same parameters, "
                        "so the physics and the kinematics cannot drift apart.")
    d.box("cad", 40, 40, 250, 70, "Onshape CAD export", "264 links, 12 revolute", kind="store")
    d.box("gen", 340, 40, 270, 70, "tools/cad_to_model.py", "FK, body aggregation, mass swap")
    d.box("par", 670, 40, 250, 70, "robot_parameters.yaml", "kinematics, inertia, visuals",
          kind="store")
    d.box("rs", 960, 40, 180, 70, "robstride02.yaml", "datasheet", kind="store")

    d.box("xac", 500, 170, 250, 70, "robodog.urdf.xacro", "RViz, TF, IK")
    d.box("mj", 800, 170, 250, 70, "robodog_sim/mjcf.py", "MJCF generator")
    d.box("mesh", 180, 170, 250, 70, "tools/prepare_meshes.py", "active CAD visual meshes", kind="store")

    d.box("world", 40, 300, 280, 100, "world_spec + worlds/house.py",
          "5 rooms, doors, stairs,\nnarrow gaps, movable props", kind="store")
    d.box("check", 40, 430, 280, 70, "world_check.py", "measures every documented gap")
    d.box("models", 800, 300, 250, 70, "models/*.xml", "robot, flat, house", kind="store")
    d.box("mark", 40, 560, 280, 70, "world_markers node", "same spec -> RViz MarkerArray")

    d.box("bk", 500, 300, 250, 100, "MujocoBackend",
          "impedance law at 2 kHz\narmature 0.012, knee x4\n1 ms command delay", kind="package")
    d.box("cam", 500, 440, 250, 90, "SimCameraBackend",
          "own model instance,\nrenders at 15 Hz", kind="package")
    d.box("note", 800, 420, 340, 190, "Fidelity choices that matter",
          "impedance evaluated at the PHYSICS\nrate, not the control rate\n\n"
          "published output equivalent inertia,\nknee reflects by ratio squared\n\n"
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
    d = Diagram("perception_pipeline", "RGB-D perception and mapping", w=1280, h=900,
                caption="Simulation uses renderer-derived calibration and exact base odometry. "
                        "Depth mapping tests do not validate hardware localization or loop closure.")
    d.box("cfg", 30, 30, 270, 80, "nuwa_hp60c.yaml",
          "range + noise + normal-mode sizes;\nreal calibration remains device-specific", kind="store")
    d.box("simb", 340, 30, 270, 80, "SimCameraBackend",
          "MuJoCo RGB-D render;\nK derived from rendered FOV", kind="package")
    d.box("realb", 30, 160, 270, 90, "NuwaHP60CBackend",
          "RGB capture only today;\ndepth / hardware odometry unavailable", kind="hardware")
    d.box("frame", 340, 170, 270, 100, "Registered simulation Frame",
          "mapping: RGB + depth 640 x 480\nsame origin, K and acquisition stamp\nRGB uint8; internal depth float32 m", kind="store")
    d.box("node", 680, 170, 260, 100, "robodog_camera_node",
          "15 Hz; depth -> 16UC1 millimetres\n0 depth means invalid", kind="node")
    d.box("topics", 680, 340, 260, 110, "ROS camera outputs",
          "color/image_raw + camera_info\ndepth/image_rect_raw + camera_info\ndepth/points (PointCloud2)", kind="topic")
    d.box("optical", 980, 170, 270, 190, "Simulation camera frame",
          "RGB, depth and point cloud all use\ncamera_color_optical_frame\n\noptical: z forward, x right, y down\n\nNo depth-frame baseline is applied\nto the co-located simulated images.", kind="note")
    d.box("policy", 30, 340, 580, 110, "mapping:=auto (default)",
          "RTAB-Map on: MuJoCo + sim camera + use_camera:=true\nOff for unsupported backends / cameras; mapping:=none disables\nExplicit mapping:=rtabmap rejects unsupported pipelines", kind="note")
    d.box("odom", 30, 520, 290, 90, "Control node owns odometry",
          "odom -> base_link\nMuJoCo ground truth; wall timestamps", kind="node")
    d.box("map", 390, 520, 290, 110, "RTAB-Map + built-in OctoMap",
          "exact RGB-D synchronization, 2 Hz\n5 cm cells; 4 m depth range\nowns map -> odom", kind="node")
    d.box("rviz", 790, 520, 350, 110, "Mapping RViz view",
          "map: 2D occupancy grid\noctomap_occupied_space: 3D cells\nrobot + world markers in map frame", kind="external")
    d.box("store", 390, 710, 290, 105, "Timestamped session database",
          "$ROS_HOME/robodog/maps/\n<world>-<UTC timestamp>.db\nexplicit path resumes a saved session", kind="store")
    d.box("export", 790, 710, 350, 105, "Backup and export",
          "rtabmap/backup: database .back\noctomap_saver_node: full ColorOcTree .ot\nNo separate mapping server required", kind="store")
    d.edge("cfg", "simb")
    d.edge("simb", "frame", route="v")
    d.edge("frame", "node")
    d.edge("node", "topics", route="v")
    d.edge("topics", "map", route="v")
    d.edge("odom", "map")
    d.edge("map", "rviz")
    d.edge("map", "store", route="v")
    d.edge("store", "export")
    return d


def gait_pipeline() -> Diagram:
    d = Diagram("gait_pipeline", "Gait generator: one control cycle",
                w=1180, h=900,
                caption="The generator plans FEET, not torques. Everything it "
                        "returns is a position, a velocity or a force; the "
                        "impedance law runs two layers below.")

    d.box("cmd", 40, 40, 300, 62, "cmd_vel -> GaitParams",
          "vx, vy, wz, gait,\nfrequency, duty")
    d.box("cfg", 370, 40, 280, 62, "gaits.yaml",
          "offsets, duty, step height,\nstance height", kind="store")
    d.box("fb", 680, 40, 460, 62, "BodyFeedback",
          "height, vz, roll, pitch, omega, v_xy")

    d.box("phase", 40, 150, 400, 68, "Phase clock",
          "phi += f dt  (mod 1)\nphi_i = (phi + offset_i) mod 1")
    d.box("raib", 680, 150, 460, 68, "Raibert foot placement",
          "p = v_meas T/2 + k_v (v_meas - v*)\nclamped to max_placement_m")

    d.box("sel", 40, 262, 400, 52, "Per leg:  phi_i < duty ?", "stance : swing",
          kind="state")

    d.box("stc", 40, 350, 540, 214, "STANCE  -  the foot is fixed in the world",
          kind="layer", z=0)
    d.box("st1", 62, 394, 496, 48, "Touchdown",
          "resume from the LAST COMMANDED position")
    d.box("st2", 62, 452, 496, 48, "Retract",
          "p -= (v_meas + g_sweep (v* - v_meas)) dt")
    d.box("st3", 62, 510, 496, 48, "Settle",
          "decay touchdown_depth over the first 1/6")

    d.box("swc", 600, 350, 540, 214, "SWING  -  interpolate two latched ends",
          kind="layer", z=0)
    d.box("sw1", 622, 394, 496, 48, "Lift-off",
          "latch p0 = where stance actually left the foot")
    d.box("sw2", 622, 452, 496, 48, "Land",
          "p1 tracks Raibert through the swing")
    d.box("sw3", 622, 510, 496, 48, "Arc",
          "p0 + (p1-p0) sigma(u),   z += h sin^2(pi u)")

    d.box("ik", 40, 600, 540, 62, "Leg IK + Jacobian",
          "q = IK(p),   qd = J^+ v")
    d.box("bal", 600, 600, 540, 62, "Balance",
          "wrench -> per-foot force -> tau = -J^T f")

    d.box("out", 40, 690, 1100, 104, "GaitOutput", kind="layer", z=0)
    d.box("outk", 62, 730, 496, 50, "Joint targets",
          "q[12]   qd[12]   foot_target[4]")
    d.box("outf", 622, 730, 496, 50, "Forces and schedule",
          "tau_ff[12]   foot_force[4]   contact[4]")

    d.box("note", 40, 815, 1100, 62,
          "Continuity is the invariant: neither transition may step.\n"
          "Rebuilding the swing arc from the nominal foot position instead "
          "cost 41 mm per lift-off, and stopped the robot walking.",
          kind="note", dashed=True)

    d.edge("cmd", "phase", route="v")
    d.edge("cfg", "phase", route="v")
    d.edge("fb", "raib", route="v")
    d.edge("phase", "sel", route="v")
    d.edge("sel", "st1", route="v")
    d.edge("sel", "sw1", route="h")
    d.edge("raib", "sw2", route="v")
    d.edge("st1", "st2", route="v")
    d.edge("st2", "st3", route="v")
    d.edge("sw1", "sw2", route="v")
    d.edge("sw2", "sw3", route="v")
    d.edge("st3", "ik", route="v")
    d.edge("sw3", "bal", route="v")
    d.edge("ik", "outk", route="v")
    d.edge("bal", "outf", route="v")
    d.edge("fb", "bal", "body state", style="dashed", route="v")
    return d


DIAGRAMS = [system_architecture, software_architecture, node_topic_graph,
            control_architecture, gait_pipeline, data_flow, hardware_boundary,
            state_machine, domain_model, simulation_architecture,
            perception_pipeline]


def main() -> int:
    render([f() for f in DIAGRAMS], ROOT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
