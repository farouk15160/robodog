"""
robodog_web_server -- telemetry and control for the browser GUI.

Architecture
------------
One process holds both an rclpy node and an aiohttp server. rclpy spins in a
background thread; aiohttp owns the asyncio loop. They meet at two plain
dictionaries guarded by the GIL: the ROS callbacks write the latest state, the
web tasks read it. No queue, because a GUI only ever wants the newest value --
buffering telemetry for a browser that fell behind is how a monitoring tool
ends up showing the past.

Why not rosbridge
-----------------
rosbridge is a generic ROS-over-WebSocket bridge: it would re-serialise every
field of RobotState as JSON at the full publish rate and expose the whole graph
to the browser. This instead speaks a small, versioned, purpose-built protocol
(see PROTOCOL below), decimated to the rate a human display needs, and exposes
exactly the commands the GUI is allowed to send. That is less code, less
bandwidth, and a much smaller surface from a page that can drive a robot.

PROTOCOL (v1)
-------------
  server -> client   {"type": "state",  "seq": n, "t": secs, "data": {...}}
                     {"type": "info",   "data": {joints, limits, ...}}   once
                     {"type": "ack",    "action": "...", "ok": bool, "message": ""}
  client -> server   {"type": "cmd", "action": "estop"|"clear_estop"|"enable"|
                                               "pose"|"gait"|"cmd_vel"|"jog"|
                                               "mode", ...}
"""
from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from typing import Any

import numpy as np
import rclpy
import yaml
from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

from robodog_msgs.msg import GaitCommand, JointCommand, JointCommandArray, RobotState
from robodog_msgs.srv import (EmergencyStop, EnableJoints, SetControlMode, SetGait,
                              SetNamedPose)

PROTOCOL_VERSION = 1
SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)

STATE_NAMES = {0: "INIT", 1: "IDLE", 2: "READY", 3: "STANDING", 4: "MOVING",
               5: "FAULT", 6: "ESTOP"}
MODE_NAMES = {0: "IDLE", 1: "POSITION", 2: "VELOCITY", 3: "TORQUE", 4: "IMPEDANCE"}
FAULT_BITS = [(1, "POSITION_LIMIT"), (2, "VELOCITY_LIMIT"), (4, "TORQUE_LIMIT"),
              (8, "OVERTEMPERATURE"), (16, "COMMUNICATION"), (32, "ENCODER"),
              (64, "UNDERVOLTAGE"), (128, "OVERCURRENT"), (256, "NOT_ENABLED"),
              (512, "WATCHDOG")]


def decode_faults(flags: int) -> list[str]:
    return [name for bit, name in FAULT_BITS if flags & bit]


class WebBridgeNode(Node):
    """ROS side: caches the latest telemetry, forwards commands."""

    def __init__(self, cfg: dict) -> None:
        super().__init__("robodog_web_server")
        self.cfg = cfg
        self.state: dict[str, Any] | None = None
        self.state_seq = 0
        self.jpeg: bytes | None = None
        self.jpeg_seq = 0
        self.camera_meta: dict[str, Any] = {"available": False}
        self._events: list[dict] = []
        self._last_faults = 0

        cb = ReentrantCallbackGroup()
        self.create_subscription(RobotState, "robodog/robot_state", self._on_state, 10,
                                 callback_group=cb)
        self.create_subscription(Image, "robodog/camera/color/image_raw",
                                 self._on_image, SENSOR_QOS, callback_group=cb)

        self.pub_joint = self.create_publisher(JointCommandArray, "robodog/joint_command", 10)
        self.pub_gait = self.create_publisher(GaitCommand, "robodog/gait_command", 10)
        self.pub_vel = self.create_publisher(Twist, "cmd_vel", 10)

        self.cli = {
            "pose": self.create_client(SetNamedPose, "robodog/set_named_pose", callback_group=cb),
            "estop": self.create_client(EmergencyStop, "robodog/emergency_stop", callback_group=cb),
            "enable": self.create_client(EnableJoints, "robodog/enable_joints", callback_group=cb),
            "gait": self.create_client(SetGait, "robodog/set_gait", callback_group=cb),
            "mode": self.create_client(SetControlMode, "robodog/set_control_mode", callback_group=cb),
        }
        self._video_period = 1.0 / max(float(cfg.get("video_rate_hz", 10.0)), 0.1)
        self._last_video = 0.0
        self._encoder = _make_encoder(int(cfg.get("video_quality", 70)),
                                      int(cfg.get("video_max_width", 640)))

    # ------------------------------------------------------------------ #
    def _on_state(self, m: RobotState) -> None:
        joints = []
        for j in m.joints:
            joints.append({
                "name": j.name, "pos": j.position, "vel": j.velocity, "eff": j.effort,
                "cur": j.current, "temp": j.temperature,
                "pos_cmd": j.position_command, "vel_cmd": j.velocity_command,
                "eff_cmd": j.effort_command, "kp": j.kp, "kd": j.kd,
                "util": j.torque_utilisation, "mode": MODE_NAMES.get(j.mode, "?"),
                "faults": decode_faults(j.fault_flags), "enabled": j.enabled,
                "err": j.position_command - j.position,
            })
        feet = [{"name": f.name, "contact": f.contact, "force": f.normal_force,
                 "pos": [f.position_in_base.x, f.position_in_base.y, f.position_in_base.z]}
                for f in m.feet]
        s, c, sim = m.safety, m.controller, m.simulation
        self.state = {
            "state": STATE_NAMES.get(m.state, "?"),
            "state_code": m.state,
            "joints": joints,
            "feet": feet,
            "base": {
                "pos": [m.base_pose.position.x, m.base_pose.position.y, m.base_pose.position.z],
                "quat": [m.base_pose.orientation.x, m.base_pose.orientation.y,
                         m.base_pose.orientation.z, m.base_pose.orientation.w],
                "vel": [m.base_twist.linear.x, m.base_twist.linear.y, m.base_twist.linear.z],
                "omega": [m.base_twist.angular.x, m.base_twist.angular.y, m.base_twist.angular.z],
                "height": m.base_height_m,
            },
            "power": {"voltage": m.battery_voltage_v, "watts": m.estimated_power_w},
            "controller": {
                "active": c.active_controller, "mode": MODE_NAMES.get(c.control_mode, "?"),
                "gait": c.active_gait, "pose": c.active_pose,
                "traj_active": c.trajectory_active, "traj_progress": c.trajectory_progress,
                "rate_hz": c.update_rate_hz, "available": list(c.available_controllers),
            },
            "safety": {
                "estop": s.estop_engaged, "latched": s.estop_latched, "source": s.estop_source,
                "faults": decode_faults(s.active_faults),
                "messages": list(s.messages),
                "max_temp": s.max_temperature_c, "hottest": s.hottest_joint,
                "max_util": s.max_torque_utilisation, "most_loaded": s.most_loaded_joint,
                "clamps": s.clamp_events, "watchdog_ok": s.watchdog_ok,
                "command_age": s.command_age_s,
                "loop_period": s.loop_period_s, "loop_jitter": s.loop_jitter_s,
            },
            "simulation": {
                "active": sim.active, "backend": sim.backend, "world": os.path.basename(sim.world),
                "sim_time": sim.sim_time_s, "wall_time": sim.wall_time_s,
                "rtf": sim.realtime_factor, "timestep": sim.timestep_s, "steps": sim.steps,
            },
            "camera": self.camera_meta,
        }
        self.state_seq += 1
        self._track_events(s.active_faults, s.messages)

    def _track_events(self, faults: int, messages) -> None:
        """A rising fault edge is an event worth keeping; the steady state is
        already visible in the safety panel."""
        new = faults & ~self._last_faults
        if new:
            self._push_event("fault", ", ".join(decode_faults(new)))
        self._last_faults = faults
        for msg in list(messages)[:1]:
            if not self._events or self._events[0]["text"] != msg:
                self._push_event("info", msg)

    def _push_event(self, kind: str, text: str) -> None:
        self._events.insert(0, {"t": time.time(), "kind": kind, "text": text})
        del self._events[60:]

    def _on_image(self, m: Image) -> None:
        self.camera_meta = {"available": True, "width": m.width, "height": m.height,
                            "encoding": m.encoding, "frame": m.header.frame_id}
        now = time.monotonic()
        if now - self._last_video < self._video_period or self._encoder is None:
            return
        self._last_video = now
        try:
            arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
            if m.encoding == "bgr8":
                arr = arr[:, :, ::-1]
            self.jpeg = self._encoder(arr)
            self.jpeg_seq += 1
        except Exception:
            self.get_logger().warn("failed to encode a camera frame",
                                   throttle_duration_sec=10.0)

    # ------------------------------------------------------------------ #
    def call(self, which: str, request) -> tuple[bool, str]:
        cli = self.cli[which]
        if not cli.service_is_ready():
            return False, f"service '{which}' is not available"
        fut = cli.call_async(request)
        start = time.monotonic()
        while not fut.done() and time.monotonic() - start < 3.0:
            time.sleep(0.005)
        if not fut.done():
            return False, f"service '{which}' timed out"
        r = fut.result()
        return bool(getattr(r, "success", False)), getattr(r, "message", "")


def _make_encoder(quality: int, max_width: int):
    """JPEG encoder, Pillow preferred. Returns None if neither is installed, in
    which case the GUI shows the camera panel as a placeholder with metadata --
    which is still useful, and better than the page failing to load."""
    try:
        from PIL import Image as PILImage
        import io

        def encode(arr: np.ndarray) -> bytes:
            img = PILImage.fromarray(arr[:, :, :3])
            if img.width > max_width:
                img = img.resize((max_width, int(img.height * max_width / img.width)))
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=quality)
            return buf.getvalue()
        return encode
    except ImportError:
        pass
    try:
        import cv2

        def encode(arr: np.ndarray) -> bytes:
            bgr = arr[:, :, ::-1]
            if bgr.shape[1] > max_width:
                h = int(bgr.shape[0] * max_width / bgr.shape[1])
                bgr = cv2.resize(bgr, (max_width, h))
            ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
            return buf.tobytes() if ok else b""
        return encode
    except ImportError:
        return None


# --------------------------------------------------------------------------- #
#  HTTP / WebSocket
# --------------------------------------------------------------------------- #
class WebApp:
    def __init__(self, node: WebBridgeNode, cfg: dict, www: str, robot_info: dict) -> None:
        self.node = node
        self.cfg = cfg
        self.www = www
        self.info = robot_info
        self.clients: set[web.WebSocketResponse] = set()
        self.app = web.Application()
        self.app.add_routes([
            web.get("/", self.index),
            web.get("/api/config", self.api_config),
            web.get("/api/info", self.api_info),
            web.get("/api/state", self.api_state),
            web.get("/stream/color.mjpg", self.mjpeg),
            web.get("/ws", self.websocket),
            # follow_symlinks: colcon's --symlink-install makes every installed
            # asset a symlink into the source tree, and aiohttp refuses to serve
            # those by default -- the page would load with no CSS or JS.
            web.static("/static", www, follow_symlinks=True),
        ])
        self.app.on_startup.append(self._start_broadcast)
        self.app.on_cleanup.append(self._stop_broadcast)

    # ---------------- static + REST ----------------
    async def index(self, request):
        return web.FileResponse(os.path.join(self.www, "index.html"))

    async def api_config(self, request):
        panels = sorted([p for p in self.cfg["panels"] if p.get("enabled", True)],
                        key=lambda p: p.get("order", 0))
        return web.json_response({
            "protocol": PROTOCOL_VERSION,
            "panels": panels,
            "limits": self.cfg["limits"],
            "thresholds": self.cfg["thresholds"],
            "stream_rate_hz": self.cfg["stream_rate_hz"],
        })

    async def api_info(self, request):
        return web.json_response(self.info)

    async def api_state(self, request):
        if self.node.state is None:
            return web.json_response({"error": "no telemetry yet"}, status=503)
        return web.json_response(self.node.state)

    async def mjpeg(self, request):
        """Multipart JPEG stream. Separate from the WebSocket so a slow video
        consumer cannot stall telemetry, and so <img src> just works."""
        if self.node._encoder is None:
            raise web.HTTPServiceUnavailable(reason="no JPEG encoder installed")
        resp = web.StreamResponse(status=200, headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-store"})
        await resp.prepare(request)
        last = -1
        try:
            while True:
                if self.node.jpeg is not None and self.node.jpeg_seq != last:
                    last = self.node.jpeg_seq
                    frame = self.node.jpeg
                    await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(frame)).encode() +
                                     b"\r\n\r\n" + frame + b"\r\n")
                await asyncio.sleep(1.0 / max(self.cfg["video_rate_hz"], 1.0))
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp

    # ---------------- websocket ----------------
    async def websocket(self, request):
        ws = web.WebSocketResponse(heartbeat=20.0)
        await ws.prepare(request)
        self.clients.add(ws)
        await ws.send_json({"type": "info", "data": self.info})
        try:
            async for msg in ws:
                if msg.type is not WSMsgType.TEXT:
                    continue
                try:
                    payload = json.loads(msg.data)
                except json.JSONDecodeError:
                    await ws.send_json({"type": "ack", "ok": False,
                                        "message": "malformed JSON"})
                    continue
                ok, text = await asyncio.get_running_loop().run_in_executor(
                    None, self.handle_command, payload)
                await ws.send_json({"type": "ack", "action": payload.get("action"),
                                    "ok": ok, "message": text})
        finally:
            self.clients.discard(ws)
        return ws

    async def _start_broadcast(self, app):
        app["broadcast"] = asyncio.create_task(self._broadcast())

    async def _stop_broadcast(self, app):
        app["broadcast"].cancel()
        for ws in list(self.clients):
            await ws.close()

    async def _broadcast(self):
        period = 1.0 / max(float(self.cfg["stream_rate_hz"]), 1.0)
        last = -1
        while True:
            await asyncio.sleep(period)
            if not self.clients or self.node.state is None:
                continue
            if self.node.state_seq == last:
                continue          # nothing new: do not spend bandwidth on it
            last = self.node.state_seq
            payload = json.dumps({"type": "state", "seq": last, "t": time.time(),
                                  "data": self.node.state,
                                  "events": self.node._events[:20]})
            dead = []
            for ws in self.clients:
                try:
                    await ws.send_str(payload)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self.clients.discard(ws)

    # ---------------- commands ----------------
    def handle_command(self, p: dict) -> tuple[bool, str]:
        action = p.get("action")
        lim = self.cfg["limits"]
        try:
            if action == "estop":
                return self.node.call("estop", EmergencyStop.Request(
                    engage=True, reason=str(p.get("reason", "web GUI"))))
            if action == "clear_estop":
                return self.node.call("estop", EmergencyStop.Request(engage=False))
            if action == "enable":
                return self.node.call("enable", EnableJoints.Request(
                    enable=bool(p.get("enable", True)), joints=list(p.get("joints", []))))
            if action == "mode":
                return self.node.call("mode", SetControlMode.Request(
                    mode=int(p.get("mode", 0)), joints=[]))
            if action == "pose":
                return self.node.call("pose", SetNamedPose.Request(
                    pose=str(p.get("pose", "stand")),
                    duration_s=float(p.get("duration", 0.0))))
            if action == "gait":
                cmd = GaitCommand()
                cmd.gait = str(p.get("gait", "stand"))
                cmd.enable = bool(p.get("enable", True))
                cmd.step_frequency_hz = float(p.get("frequency", 0.0))
                cmd.step_height_m = float(p.get("height", 0.0))
                cmd.stance_height_m = float(p.get("stance", 0.0))
                cmd.duty_factor = float(p.get("duty", 0.0))
                return self.node.call("gait", SetGait.Request(command=cmd))
            if action == "cmd_vel":
                t = Twist()
                t.linear.x = _clamp(p.get("vx", 0.0), lim["max_linear_velocity"])
                t.linear.y = _clamp(p.get("vy", 0.0), lim["max_linear_velocity"])
                t.angular.z = _clamp(p.get("wz", 0.0), lim["max_angular_velocity"])
                self.node.pub_vel.publish(t)
                return True, f"vx={t.linear.x:.2f} vy={t.linear.y:.2f} wz={t.angular.z:.2f}"
            if action == "jog":
                if not lim.get("enable_joint_jog", False):
                    return False, ("joint jog is disabled in web.yaml; it bypasses the "
                                   "gait layer and can put a leg where the body "
                                   "cannot support it")
                return self._jog(p, lim)
            return False, f"unknown action '{action}'"
        except Exception as e:                                  # pragma: no cover
            return False, f"{type(e).__name__}: {e}"

    def _jog(self, p: dict, lim: dict) -> tuple[bool, str]:
        name = str(p.get("joint", ""))
        if name not in self.info["joints"]:
            return False, f"unknown joint '{name}'"
        if self.node.state is None:
            return False, "no telemetry yet"
        i = self.info["joints"].index(name)
        step = _clamp(p.get("delta", 0.0), lim["manual_joint_step_rad"])
        cur = self.node.state["joints"][i]["pos"]
        lo, hi = self.info["limits"][name]["lower"], self.info["limits"][name]["upper"]
        target = min(max(cur + step, lo), hi)
        msg = JointCommandArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.names = [name]
        msg.commands = [JointCommand(mode=JointCommand.MODE_IMPEDANCE, position=target,
                                     kp=float(self.info["default_kp"]),
                                     kd=float(self.info["default_kd"]))]
        self.node.pub_joint.publish(msg)
        return True, f"{name} -> {target:+.4f} rad"


def _clamp(v, limit: float) -> float:
    v = float(v)
    return max(-abs(limit), min(abs(limit), v))


# --------------------------------------------------------------------------- #
def _robot_info() -> dict:
    desc = get_package_share_directory("robodog_description")
    with open(os.path.join(desc, "config", "robot_parameters.yaml")) as f:
        P = yaml.safe_load(f)
    with open(os.path.join(desc, "config", "robstride02.yaml")) as f:
        RS = yaml.safe_load(f)
    # control.yaml is a ROS parameter file, so the gains live under the node
    # wildcard. Read defensively: the GUI must still start if the control
    # package is reconfigured, it just falls back to conservative jog gains.
    ctrl = get_package_share_directory("robodog_control")
    try:
        with open(os.path.join(ctrl, "config", "control.yaml")) as f:
            params = yaml.safe_load(f)
        gains = next(iter(params.values()))["ros__parameters"]["gains"]
    except (KeyError, StopIteration, TypeError, OSError):
        gains = {"position_kp": 60.0, "position_kd": 2.0}
    names = [f"{l}_{k}_joint" for l in ("FL", "FR", "RL", "RR")
             for k in ("haa", "hfe", "kfe")]
    return {
        "protocol": PROTOCOL_VERSION,
        "robot": "robodog",
        "actuator": RS["model"],
        "joints": names,
        "legs": ["FL", "FR", "RL", "RR"],
        "limits": {n: P["joint_limits"][n.split("_")[1]] for n in names},
        "operational": RS["operational_limits"],
        "poses": sorted(P["named_poses"]),
        "geometry": P["geometry"],
        "mass_kg": P["mass_budget"]["total_kg"],
        "default_kp": gains["position_kp"],
        "default_kd": gains["position_kd"],
    }


def main(argv=None) -> None:
    rclpy.init(args=argv)
    boot = Node("robodog_web_bootstrap")
    boot.declare_parameter("port", 8080)
    boot.declare_parameter("host", "0.0.0.0")
    boot.declare_parameter("config", "")
    port = int(boot.get_parameter("port").value)
    host = str(boot.get_parameter("host").value)
    cfg_path = boot.get_parameter("config").value or os.path.join(
        get_package_share_directory("robodog_web"), "config", "web.yaml")
    boot.destroy_node()

    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)["robodog_web"]
    www = os.path.join(get_package_share_directory("robodog_web"), "www")

    node = WebBridgeNode(cfg)
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, name="robodog-web-ros", daemon=True)
    spin.start()

    app = WebApp(node, cfg, www, _robot_info())
    node.get_logger().info(f"web GUI on http://{host}:{port}  "
                           f"(protocol v{PROTOCOL_VERSION}, "
                           f"{len([p for p in cfg['panels'] if p.get('enabled')])} panels)")
    try:
        web.run_app(app.app, host=host, port=port, print=None, handle_signals=True)
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
