"""
robodog_control_node -- the single real-time loop of the robot.

Structure
---------
    read backend -> estimate state -> active controller -> safety -> write backend

The loop runs in its own thread at `control_rate_hz` (400 Hz by default, chosen
to match the CAN bus budget so that gains tuned in simulation transfer without
a rate change). ROS callbacks run on the executor and only ever set fields that
the loop reads; nothing in a callback touches the backend.

Publishing is decimated. Running RobotState at 400 Hz would cost more than the
control it reports on, and no consumer needs it: the GUI redraws at 30 Hz and
RViz at 30.

Controllers are selected by mode, not by composition, because only one thing
may drive the joints at a time and making that exclusive by construction is
worth more than the flexibility of a chain.
"""
from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
import rclpy
import yaml
from rclpy._rclpy_pybind11 import RCLError
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Twist, Vector3
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger  # noqa: F401  (typing aid)
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from robodog_hardware.registry import create_backend
from robodog_hardware.types import JOINT_NAMES, NJ, ControlMode, JointCommand
from robodog_msgs.msg import (ControllerState, FootState, GaitCommand, JointCommandArray,
                              JointTelemetry, RobotState, SafetyStatus, SimulationState)
from robodog_msgs.srv import (EmergencyStop, EnableJoints, SetControlMode, SetGait, SetNamedPose)

from .balance import roll_pitch_from_quat
from .gait import (BodyFeedback, GaitGenerator, GaitParams, JointTestGenerator,
                   JointTestParams)
from .kinematics import LEGS, LegGeometry, forward_in_base, jacobian
from .safety import SafetyLimits, SafetyMonitor
from .state_estimator import StateEstimator
from .trajectory import JointTrajectory

RELIABLE = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=10)
LATCHED = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)

STATE_NAMES = {RobotState.STATE_INIT: "INIT", RobotState.STATE_IDLE: "IDLE",
               RobotState.STATE_READY: "READY", RobotState.STATE_STANDING: "STANDING",
               RobotState.STATE_MOVING: "MOVING", RobotState.STATE_FAULT: "FAULT",
               RobotState.STATE_ESTOP: "ESTOP"}


def _load(pkg: str, *parts: str) -> dict:
    import os
    with open(os.path.join(get_package_share_directory(pkg), *parts)) as f:
        return yaml.safe_load(f)


class RoboDogControlNode(Node):
    def __init__(self) -> None:
        super().__init__("robodog_control_node")

        # ---------------- parameters ----------------
        # Launch-time selection
        self.declare_parameter("backend", "kinematic")
        self.declare_parameter("mujoco_model", "")
        self.declare_parameter("mujoco_viewer", False)
        self.declare_parameter("auto_enable", False)
        self.declare_parameter("auto_stand", False)
        self.declare_parameter("publish_odom_tf", True)
        # Tuning, from config/control.yaml; changeable at runtime with
        # `ros2 param set`, which is why they are parameters and not a
        # hand-loaded config dict.
        self.declare_parameter("control_rate_hz", 400.0)
        self.declare_parameter("state_publish_rate_hz", 50.0)
        self.declare_parameter("joint_state_rate_hz", 100.0)
        self.declare_parameter("pose_transition_max_velocity_rad_s", 1.0)
        self.declare_parameter("default_travel_gait", "trot")
        self.declare_parameter("gains.position_kp", 80.0)
        self.declare_parameter("gains.position_kd", 2.0)
        self.declare_parameter("gains.stance_kp", 120.0)
        self.declare_parameter("gains.stance_kd", 2.5)
        self.declare_parameter("gains.swing_kp", 45.0)
        self.declare_parameter("gains.swing_kd", 1.2)
        self.declare_parameter("state_estimator.tau_attitude_s", 1.0)
        self.declare_parameter("state_estimator.contact_force_threshold_n", 8.0)

        gp = lambda n: self.get_parameter(n).value
        self.rate = float(gp("control_rate_hz"))
        self.dt = 1.0 / self.rate
        backend_name = str(gp("backend"))

        # ---------------- static configuration ----------------
        # Generated model and datasheet: plain YAML loaded by path, because
        # they are structured data, not tunable parameters.
        self.P = _load("robodog_description", "config", "robot_parameters.yaml")
        self.RS = _load("robodog_description", "config", "robstride02.yaml")
        self.GAITS = _load("robodog_control", "config", "gaits.yaml")["gaits"]

        self.geom = LegGeometry.from_params(self.P)
        self.mass = float(self.P["mass_budget"]["total_kg"])
        jl = self.P["joint_limits"]
        kinds = [n.split("_")[1] for n in JOINT_NAMES]
        self.q_lower = np.array([jl[k]["lower"] for k in kinds])
        self.q_upper = np.array([jl[k]["upper"] for k in kinds])
        self.poses = {k: np.array([v["haa"], v["hfe"], v["kfe"]] * 4)
                      for k, v in self.P["named_poses"].items()}

        # ---------------- backend ----------------
        self.backend = create_backend(backend_name, self._backend_config(backend_name))
        self.backend.configure()
        self.get_logger().info(
            f"backend '{self.backend.name}' ready "
            f"({'simulation' if self.backend.is_simulation else 'REAL HARDWARE'})")

        # ---------------- control components ----------------
        self.safety = SafetyMonitor(SafetyLimits.from_config(self.P, self.RS, list(JOINT_NAMES)),
                                    list(JOINT_NAMES), self.rate)
        self.estimator = StateEstimator(
            self.geom,
            tau_attitude_s=float(gp("state_estimator.tau_attitude_s")),
            contact_force_threshold_n=float(gp("state_estimator.contact_force_threshold_n")))
        self.gait = GaitGenerator(self.geom, self.poses["stand"][:3], self.mass)
        self.jtest = JointTestGenerator(self.poses["stand"], self.q_lower, self.q_upper)

        self.kp_default = float(gp("gains.position_kp"))
        self.kd_default = float(gp("gains.position_kd"))
        self.kp_stance = float(gp("gains.stance_kp"))
        self.kd_stance = float(gp("gains.stance_kd"))
        self.kp_swing = float(gp("gains.swing_kp"))
        self.kd_swing = float(gp("gains.swing_kd"))

        # ---------------- mutable control state ----------------
        self._lock = threading.Lock()
        self.controller = "idle"
        self.mode = ControlMode.IDLE
        self.active_pose = ""
        self.active_gait = "stand"
        self.trajectory: JointTrajectory | None = None
        self.external_cmd: JointCommand | None = None
        self._external_stamp = 0.0
        self.enabled = False
        self.state = RobotState.STATE_INIT
        self._pending_gait: GaitParams | None = None

        self._last_state = self.backend.read()
        self._q_hold = self._last_state.position.copy()
        self._loop_period = self.dt
        self._loop_jitter = 0.0
        self._last_report: Any = None
        self._last_cmd: JointCommand = JointCommand()
        self._base = None

        # ---------------- ROS interfaces ----------------
        cb = ReentrantCallbackGroup()
        self.pub_js = self.create_publisher(JointState, "joint_states", RELIABLE)
        self.pub_state = self.create_publisher(RobotState, "robodog/robot_state", RELIABLE)
        self.pub_safety = self.create_publisher(SafetyStatus, "robodog/safety_status", RELIABLE)
        self.pub_imu = self.create_publisher(Imu, "robodog/imu", RELIABLE)
        self.pub_info = self.create_publisher(String, "robodog/backend_info", LATCHED)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(JointCommandArray, "robodog/joint_command",
                                 self._on_joint_command, RELIABLE, callback_group=cb)
        self.create_subscription(GaitCommand, "robodog/gait_command",
                                 self._on_gait_command, RELIABLE, callback_group=cb)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, RELIABLE, callback_group=cb)

        srv = MutuallyExclusiveCallbackGroup()
        self.create_service(SetControlMode, "robodog/set_control_mode", self._srv_mode, callback_group=srv)
        self.create_service(SetNamedPose, "robodog/set_named_pose", self._srv_pose, callback_group=srv)
        self.create_service(SetGait, "robodog/set_gait", self._srv_gait, callback_group=srv)
        self.create_service(EmergencyStop, "robodog/emergency_stop", self._srv_estop, callback_group=srv)
        self.create_service(EnableJoints, "robodog/enable_joints", self._srv_enable, callback_group=srv)

        info = String()
        info.data = yaml.safe_dump({"backend": self.backend.name,
                                    "simulation": self.backend.is_simulation,
                                    "control_rate_hz": self.rate,
                                    "joints": list(JOINT_NAMES)})
        self.pub_info.publish(info)

        self._decim_state = max(1, int(self.rate / float(gp("state_publish_rate_hz"))))
        self._decim_js = max(1, int(self.rate / float(gp("joint_state_rate_hz"))))
        self._tick = 0
        self._publish_tf = bool(gp("publish_odom_tf"))

        self._running = True
        self._thread = threading.Thread(target=self._control_loop, name="robodog-control", daemon=True)
        self._thread.start()

        if bool(gp("auto_enable")):
            self._enable(True)
        if bool(gp("auto_stand")):
            self._start_pose("stand", 0.0)

    # ------------------------------------------------------------------ #
    def _backend_config(self, name: str) -> dict:
        rs, th, jd = self.RS["performance"], self.RS["thermal"], self.RS["joint_dynamics"]
        el = self.RS["electrical"]
        cfg = {
            "peak_torque_nm": rs["peak_torque_nm"],
            "no_load_speed_rad_s": rs["no_load_speed_rad_s"],
            "torque_constant_nm_per_arms": el["torque_constant_nm_per_arms"],
            "phase_resistance_ohm": el["phase_resistance_ohm"],
            "thermal_resistance_k_per_w": th["thermal_resistance_k_per_w"],
            "thermal_capacitance_j_per_k": th["thermal_capacitance_j_per_k"],
            "ambient_temp_c": th["ambient_temp_c"],
            "damping_nms_per_rad": jd["damping_nms_per_rad"],
            "coulomb_friction_nm": jd["coulomb_friction_nm"],
            "armature_kgm2": jd["armature_kgm2"],
            "base_height_m": self.P["named_poses"]["stand"]["base_height_m"],
            "initial_position": np.array([self.P["named_poses"]["stand"][k]
                                          for k in ("haa", "hfe", "kfe")] * 4),
        }
        if name == "mujoco":
            path = self.get_parameter("mujoco_model").value
            if not path:
                import os
                path = os.path.join(get_package_share_directory("robodog_sim"),
                                    "models", "robodog_scene.xml")
            cfg.update(model_path=path, keyframe="stand",
                       viewer=bool(self.get_parameter("mujoco_viewer").value))
        elif name == "robstride02_can":
            cfg.update(_load("robodog_hardware", "config", "robstride_bus.yaml"))
        return cfg

    # ------------------------------------------------------------------ #
    #  control loop
    # ------------------------------------------------------------------ #
    def _control_loop(self) -> None:
        next_t = time.perf_counter()
        while self._running and rclpy.ok():
            t0 = time.perf_counter()
            try:
                self._cycle()
            except Exception as e:
                # Ctrl-C tears the rclpy context down from the signal handler
                # while this thread is mid-cycle. A publish then fails with
                # RCLError, which is shutdown, not a fault -- reporting it as an
                # e-stop buried the real shutdown path in a traceback.
                if not rclpy.ok() or isinstance(e, RCLError):
                    break
                self.get_logger().error("control cycle failed, engaging e-stop",
                                        throttle_duration_sec=1.0)
                self.get_logger().error(_tb(), throttle_duration_sec=1.0)
                self.safety.engage_estop("control-loop exception")
                try:
                    self.backend.write(JointCommand())
                except Exception:
                    pass
            next_t += self.dt
            slack = next_t - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                next_t = time.perf_counter()      # fell behind: resynchronise
            now = time.perf_counter()
            self._loop_jitter = max(abs((now - t0) - self.dt), self._loop_jitter * 0.999)
            self._loop_period = now - t0

    def _cycle(self) -> None:
        st = self.backend.read()
        self._last_state = st

        base = self.backend.base_state()
        if base is None:                      # real hardware: estimate it
            contact = self.estimator.contact_from_torque(st.position, st.effort)
            base = self.estimator.update(st.position, st.velocity,
                                         np.array([0.0, 0.0, 9.81]), np.zeros(3),
                                         self.dt, contact)
        self._base = base

        with self._lock:
            req, age = self._compute_request(st)
        cmd, report = self.safety.apply(st, req, age)
        self._last_cmd, self._last_report = cmd, report

        self.backend.write(cmd)
        if self.backend.is_simulation:
            self.backend.step(self.dt)

        self._tick += 1
        if not rclpy.ok():
            return
        if self._tick % self._decim_js == 0:
            self._publish_joint_states(st)
        if self._tick % self._decim_state == 0:
            self._publish_robot_state(st, cmd, report, base)

    def _body_feedback(self) -> BodyFeedback | None:
        """Pack the base estimate into what the balance layer needs."""
        b = self._base
        if b is None:
            return None
        roll, pitch = roll_pitch_from_quat(b.orientation)
        return BodyFeedback(
            height=float(b.position[2]), vz=float(b.linear_velocity[2]),
            roll=roll, pitch=pitch,
            omega=(float(b.angular_velocity[0]), float(b.angular_velocity[1]),
                   float(b.angular_velocity[2])),
            v_xy=(float(b.linear_velocity[0]), float(b.linear_velocity[1])))

    def _compute_request(self, st) -> tuple[JointCommand, float]:
        """Whichever controller is active produces the request. Exactly one."""
        c = JointCommand()
        age = 0.0
        if not self.enabled or self.controller == "idle":
            # Carry the measured position even while idle, so whatever runs next
            # starts from where the robot actually is rather than from zero.
            c.mode[:] = int(ControlMode.IDLE)
            c.position[:] = st.position
            self._q_hold = st.position.copy()
            return c, age

        if self.controller == "joint" and self.external_cmd is not None:
            age = time.monotonic() - self._external_stamp
            return self.external_cmd.copy(), age

        if self.controller == "pose":
            if self.trajectory is not None:
                q, qd = self.trajectory.step(self.dt)
                if self.trajectory.done:
                    self._q_hold = q.copy()
                    self.trajectory = None
                    self.state = (RobotState.STATE_STANDING if self.active_pose == "stand"
                                  else RobotState.STATE_READY)
            else:
                q, qd = self._q_hold, np.zeros(NJ)
            c.mode[:] = int(ControlMode.IMPEDANCE)
            c.position[:] = q
            c.velocity[:] = qd
            c.kp[:] = self.kp_default
            c.kd[:] = self.kd_default
            return c, age

        if self.controller == "gait":
            if self._pending_gait is not None:
                self.gait.set_params(self._pending_gait)
                self._pending_gait = None
            # The gait layer cannot balance without knowing where the body is.
            # `base` is ground truth in simulation and the estimator's output on
            # hardware; either way the gait sees the same structure.
            out = self.gait.update(self.dt, self._body_feedback())
            c.mode[:] = int(ControlMode.IMPEDANCE)
            c.position[:] = out.q
            c.velocity[:] = out.qd
            c.effort[:] = out.tau_ff
            # Stance legs need stiffness to carry the body; swing legs want to
            # be compliant so an unexpected contact does not lever the robot.
            per_leg_kp = np.where(out.contact, self.kp_stance, self.kp_swing)
            per_leg_kd = np.where(out.contact, self.kd_stance, self.kd_swing)
            c.kp[:] = np.repeat(per_leg_kp, 3)
            c.kd[:] = np.repeat(per_leg_kd, 3)
            self._q_hold = out.q.copy()
            return c, age

        if self.controller == "joint_test":
            q, qd, done = self.jtest.update(self.dt)
            if done:
                self.controller = "pose"
                self.trajectory = JointTrajectory.with_speed_limit(
                    self._last_state.position, self.poses["stand"], 1.0)
            c.mode[:] = int(ControlMode.IMPEDANCE)
            c.position[:] = q
            c.velocity[:] = qd
            c.kp[:] = self.kp_default
            c.kd[:] = self.kd_default
            return c, age

        c.mode[:] = int(ControlMode.IDLE)
        return c, age

    # ------------------------------------------------------------------ #
    #  publishing
    # ------------------------------------------------------------------ #
    def _publish_joint_states(self, st) -> None:
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(JOINT_NAMES)
        m.position = st.position.tolist()
        m.velocity = st.velocity.tolist()
        m.effort = st.effort.tolist()
        self.pub_js.publish(m)

    def _publish_robot_state(self, st, cmd, report, base) -> None:
        now = self.get_clock().now().to_msg()
        msg = RobotState()
        msg.header.stamp = now
        msg.header.frame_id = "base_link"

        cont = self.RS["operational_limits"]["continuous_torque_nm"]
        kt = self.RS["electrical"]["torque_constant_nm_per_arms"]
        for i, name in enumerate(JOINT_NAMES):
            j = JointTelemetry()
            j.name = name
            j.position = float(st.position[i])
            j.velocity = float(st.velocity[i])
            j.effort = float(st.effort[i])
            j.current = float(abs(st.effort[i]) / kt)
            j.temperature = float(st.temperature[i])
            j.position_command = float(cmd.position[i])
            j.velocity_command = float(cmd.velocity[i])
            j.effort_command = float(cmd.effort[i])
            j.kp = float(cmd.kp[i])
            j.kd = float(cmd.kd[i])
            j.torque_utilisation = float(abs(st.effort[i]) / cont)
            j.mode = int(cmd.mode[i])
            j.fault_flags = int(report.faults[i])
            j.enabled = bool(st.enabled[i])
            msg.joints.append(j)

        for i, leg in enumerate(LEGS):
            q = st.position[3 * i:3 * i + 3]
            p = forward_in_base(self.geom, leg, q)
            v = jacobian(self.geom, leg, q) @ st.velocity[3 * i:3 * i + 3]
            f = FootState()
            f.name = leg
            f.position_in_base = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            f.velocity_in_base = Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))
            f.contact = bool(base.foot_contact[i]) if base is not None else False
            f.normal_force = float(base.foot_force[i]) if base is not None else 0.0
            msg.feet.append(f)

        if base is not None:
            msg.base_pose.position = Point(x=float(base.position[0]), y=float(base.position[1]),
                                           z=float(base.position[2]))
            q = base.orientation
            msg.base_pose.orientation.x, msg.base_pose.orientation.y = float(q[0]), float(q[1])
            msg.base_pose.orientation.z, msg.base_pose.orientation.w = float(q[2]), float(q[3])
            msg.base_twist.linear = Vector3(x=float(base.linear_velocity[0]),
                                            y=float(base.linear_velocity[1]),
                                            z=float(base.linear_velocity[2]))
            msg.base_twist.angular = Vector3(x=float(base.angular_velocity[0]),
                                             y=float(base.angular_velocity[1]),
                                             z=float(base.angular_velocity[2]))
            msg.base_height_m = float(base.position[2])
            if self._publish_tf:
                self._publish_odom_tf(base, now)
            self._publish_imu(base, now)

        msg.battery_voltage_v = float(self.RS["electrical"]["rated_voltage_v"])
        msg.estimated_power_w = float(np.sum(
            3.0 * (st.effort / kt) ** 2 * self.RS["electrical"]["phase_resistance_ohm"]
            + np.abs(st.effort * st.velocity)))

        self.state = self._derive_state(report)
        msg.state = self.state
        msg.state_name = STATE_NAMES[self.state]

        cs = ControllerState()
        cs.header.stamp = now
        cs.active_controller = self.controller
        cs.control_mode = int(self.mode)
        cs.active_gait = self.active_gait
        cs.active_pose = self.active_pose
        cs.trajectory_active = self.trajectory is not None
        cs.trajectory_progress = self.trajectory.progress if self.trajectory else 0.0
        cs.update_rate_hz = 1.0 / max(self._loop_period, 1e-9)
        cs.available_controllers = ["idle", "joint", "pose", "gait", "joint_test"]
        msg.controller = cs

        ss = SafetyStatus()
        ss.header.stamp = now
        ss.estop_engaged = bool(report.estop)
        ss.estop_latched = bool(report.latched)
        ss.estop_source = report.source
        ss.active_faults = int(np.bitwise_or.reduce(report.faults)) if len(report.faults) else 0
        ss.messages = list(report.messages)
        hottest = int(np.argmax(st.temperature))
        ss.max_temperature_c = float(st.temperature[hottest])
        ss.hottest_joint = JOINT_NAMES[hottest]
        loaded = int(np.argmax(report.torque_utilisation))
        ss.max_torque_utilisation = float(report.torque_utilisation[loaded])
        ss.most_loaded_joint = JOINT_NAMES[loaded]
        ss.clamp_events = int(self.safety.clamp_events)
        ss.watchdog_ok = report.command_age_s <= self.safety.lim.command_timeout_s
        ss.command_age_s = float(report.command_age_s)
        ss.loop_period_s = float(self._loop_period)
        ss.loop_jitter_s = float(self._loop_jitter)
        msg.safety = ss
        self.pub_safety.publish(ss)

        sim = SimulationState()
        sim.header.stamp = now
        stats = self.backend.stats()
        sim.active = bool(self.backend.is_simulation)
        sim.backend = self.backend.name if self.backend.is_simulation else "none"
        sim.world = str(stats.get("world", ""))
        sim.sim_time_s = float(stats.get("sim_time", 0.0))
        sim.wall_time_s = float(stats.get("wall_time", 0.0))
        sim.realtime_factor = float(stats.get("realtime_factor", 0.0))
        sim.timestep_s = float(stats.get("timestep", self.dt))
        sim.steps = int(stats.get("steps", 0))
        msg.simulation = sim

        self.pub_state.publish(msg)

    def _derive_state(self, report) -> int:
        if report.latched:
            return RobotState.STATE_ESTOP
        if report.faults.any():
            return RobotState.STATE_FAULT
        if not self.enabled:
            return RobotState.STATE_IDLE
        if self.controller == "gait" and self.active_gait != "stand":
            return RobotState.STATE_MOVING
        if self.controller in ("pose", "gait") and self.trajectory is None:
            return RobotState.STATE_STANDING
        return RobotState.STATE_READY

    def _publish_odom_tf(self, base, stamp) -> None:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(base.position[0])
        t.transform.translation.y = float(base.position[1])
        t.transform.translation.z = float(base.position[2])
        q = base.orientation
        t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
        t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
        self.tf.sendTransform(t)

    def _publish_imu(self, base, stamp) -> None:
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = "imu_link"
        q = base.orientation
        m.orientation.x, m.orientation.y = float(q[0]), float(q[1])
        m.orientation.z, m.orientation.w = float(q[2]), float(q[3])
        m.angular_velocity = Vector3(x=float(base.angular_velocity[0]),
                                     y=float(base.angular_velocity[1]),
                                     z=float(base.angular_velocity[2]))
        m.linear_acceleration = Vector3(x=float(base.linear_acceleration[0]),
                                        y=float(base.linear_acceleration[1]),
                                        z=float(base.linear_acceleration[2]))
        self.pub_imu.publish(m)

    # ------------------------------------------------------------------ #
    #  subscriptions
    # ------------------------------------------------------------------ #
    def _on_joint_command(self, msg: JointCommandArray) -> None:
        with self._lock:
            c = self.external_cmd.copy() if self.external_cmd is not None else JointCommand()
            index = {n: i for i, n in enumerate(JOINT_NAMES)}
            for name, jc in zip(msg.names, msg.commands):
                i = index.get(name)
                if i is None:
                    self.get_logger().warn(f"unknown joint '{name}' in command",
                                           throttle_duration_sec=5.0)
                    continue
                c.mode[i] = jc.mode
                c.position[i] = jc.position
                c.velocity[i] = jc.velocity
                c.effort[i] = jc.effort
                c.kp[i] = jc.kp
                c.kd[i] = jc.kd
            self.external_cmd = c
            self._external_stamp = time.monotonic()
            if self.controller != "joint":
                self.controller = "joint"

    def _on_gait_command(self, msg: GaitCommand) -> None:
        with self._lock:
            ok, why = self._apply_gait(msg)
        if not ok:
            self.get_logger().warn(why)

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            if self.controller != "gait":
                return
            p = self.gait.params
            p.vx, p.vy, p.wz = msg.linear.x, msg.linear.y, msg.angular.z
            if self.active_gait == "stand" and (abs(p.vx) + abs(p.vy) + abs(p.wz)) > 1e-3:
                # walking was requested by velocity alone: promote to the
                # default travelling gait rather than silently ignoring it
                p.gait = self.active_gait = str(
                    self.get_parameter("default_travel_gait").value)
                self._merge_gait_defaults(p)
            self._pending_gait = p

    def _merge_gait_defaults(self, p: GaitParams) -> None:
        d = self.GAITS.get(p.gait, {})
        p.step_frequency_hz = float(d.get("step_frequency_hz", p.step_frequency_hz))
        p.step_height_m = float(d.get("step_height_m", p.step_height_m))
        p.duty_factor = float(d.get("duty_factor", p.duty_factor))
        p.stance_height_m = float(d.get("stance_height_m", p.stance_height_m))

    def _apply_gait(self, msg: GaitCommand) -> tuple[bool, str]:
        name = msg.gait or self.active_gait
        if name not in self.GAITS:
            return False, f"unknown gait '{name}'; configured: {sorted(self.GAITS)}"
        p = GaitParams(gait=name)
        self._merge_gait_defaults(p)
        if msg.step_frequency_hz > 0:
            p.step_frequency_hz = msg.step_frequency_hz
        if msg.step_height_m > 0:
            p.step_height_m = msg.step_height_m
        if msg.stance_height_m > 0:
            p.stance_height_m = msg.stance_height_m
        if msg.duty_factor > 0:
            p.duty_factor = msg.duty_factor
        p.vx, p.vy, p.wz = msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.angular.z
        self._pending_gait = p
        self.active_gait = name
        self.controller = "gait" if msg.enable else "pose"
        return True, f"gait '{name}' {'enabled' if msg.enable else 'staged'}"

    # ------------------------------------------------------------------ #
    #  services
    # ------------------------------------------------------------------ #
    def _srv_mode(self, req, res):
        if self.safety.latched:
            res.success, res.message = False, "e-stop latched, clear it first"
            return res
        with self._lock:
            self.mode = ControlMode(req.mode)
            if self.mode == ControlMode.IDLE:
                self.controller = "idle"
        res.success, res.message = True, f"mode set to {ControlMode(req.mode).name}"
        return res

    def _srv_pose(self, req, res):
        with self._lock:
            ok, msg, dur = self._start_pose(req.pose, req.duration_s)
        res.success, res.message, res.planned_duration_s = ok, msg, dur
        return res

    def _start_pose(self, name: str, duration: float) -> tuple[bool, str, float]:
        if name not in self.poses:
            return False, f"unknown pose '{name}'; have {sorted(self.poses)}", 0.0
        if self.safety.latched:
            return False, "e-stop latched, clear it first", 0.0
        goal = self.poses[name]
        start = self._last_state.position.copy()
        if duration and duration > 0:
            traj = JointTrajectory(start, goal, duration)
        else:
            traj = JointTrajectory.with_speed_limit(
                start, goal,
                float(self.get_parameter("pose_transition_max_velocity_rad_s").value))
        self.trajectory = traj
        self.controller = "pose"
        self.active_pose = name
        if not self.enabled:
            self._enable(True)
        return True, f"moving to '{name}'", float(traj.duration)

    def _srv_gait(self, req, res):
        with self._lock:
            res.success, res.message = self._apply_gait(req.command)
        return res

    def _srv_estop(self, req, res):
        if req.engage:
            self.safety.engage_estop(req.reason or "service")
            with self._lock:
                self.controller = "idle"
            self._enable(False)
            res.success, res.latched, res.message = True, True, "e-stop engaged"
        else:
            ok, msg = self.safety.clear_estop(self._last_state)
            res.success, res.latched, res.message = ok, self.safety.latched, msg
        return res

    def _srv_enable(self, req, res):
        if req.enable and self.safety.latched:
            res.success, res.message = False, "e-stop latched, clear it first"
            return res
        self._enable(req.enable, req.joints or None)
        res.success = True
        res.message = f"joints {'enabled' if req.enable else 'disabled'}"
        return res

    def _enable(self, on: bool, joints: list[str] | None = None) -> None:
        mask = None
        if joints:
            index = {n: i for i, n in enumerate(JOINT_NAMES)}
            mask = [False] * NJ
            for n in joints:
                if n in index:
                    mask[index[n]] = True
        (self.backend.enable if on else self.backend.disable)(mask)
        with self._lock:
            self.enabled = on
            if not on:
                self.controller = "idle"
            else:
                self._q_hold = self._last_state.position.copy()

    # ------------------------------------------------------------------ #
    def destroy_node(self) -> None:
        # Stop the control thread FIRST: it holds publishers that
        # Node.destroy_node() is about to invalidate.
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        try:
            self.backend.disable()
            self.backend.shutdown()
        except Exception:
            pass
        super().destroy_node()


def _tb() -> str:
    import traceback
    return traceback.format_exc()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = RoboDogControlNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
