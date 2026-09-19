"""
robodog_camera_node -- RGB, depth and PointCloud2, from either backend.

Topics (identical in simulation and on the real camera):
    robodog/camera/color/image_raw        sensor_msgs/Image        rgb8
    robodog/camera/color/camera_info      sensor_msgs/CameraInfo
    robodog/camera/depth/image_rect_raw   sensor_msgs/Image        16UC1, mm
    robodog/camera/depth/camera_info      sensor_msgs/CameraInfo
    robodog/camera/depth/points           sensor_msgs/PointCloud2

Depth is published as 16UC1 in millimetres, the ROS convention that
depth_image_proc and RViz expect. The backend works in float32 metres with NaN
for invalid, which is the representation that can express "no reading"; the
conversion to 0-means-invalid happens here, once, at the boundary.

The point cloud is built here rather than delegated to depth_image_proc so that
the range filtering implied by the datasheet (drop returns beyond
max_usable_range_m, where stereo error grows as z^2) is applied at the source.
Publishing points the camera cannot actually measure and filtering them later
wastes bandwidth and invites a map built from them.
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from robodog_msgs.msg import RobotState

from .backend import CameraBackendError, Intrinsics
from .pointcloud import deproject, make_cloud
from .registry import create_camera

# Sensor data is best-effort: a dropped frame must never block the publisher,
# and a late frame is worth less than the next one.
SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=2)


def camera_info(k: Intrinsics, frame_id: str, stamp) -> CameraInfo:
    m = CameraInfo()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.width, m.height = k.width, k.height
    m.distortion_model = k.distortion_model
    m.d = list(k.distortion)
    m.k = k.K
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.p = k.P
    return m


def image_msg(data: np.ndarray, encoding: str, frame_id: str, stamp) -> Image:
    m = Image()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.height, m.width = data.shape[0], data.shape[1]
    m.encoding = encoding
    m.is_bigendian = 0
    m.step = int(data.strides[0])
    m.data = data.tobytes()
    return m


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("robodog_camera_node")
        self.declare_parameter("backend", "sim")
        self.declare_parameter("config", "")
        self.declare_parameter("mujoco_model", "")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("publish_pointcloud", True)
        self.declare_parameter("device", "/dev/video0")

        cfg_path = self.get_parameter("config").value or os.path.join(
            get_package_share_directory("robodog_perception"), "config", "nuwa_hp60c.yaml")
        with open(cfg_path) as f:
            self.cfg = yaml.safe_load(f)
        self.cfg["device"] = self.get_parameter("device").value

        backend = str(self.get_parameter("backend").value)
        if backend == "sim":
            model = self.get_parameter("mujoco_model").value or os.path.join(
                get_package_share_directory("robodog_sim"), "models", "robodog_scene.xml")
            self.cfg["model_path"] = model
        self.camera = create_camera(backend, self.cfg)
        try:
            self.camera.configure()
        except CameraBackendError as e:
            self.get_logger().error(f"camera backend unavailable: {e}")
            raise
        self.color_k, self.depth_k = self.camera.intrinsics()
        self.get_logger().info(
            f"camera backend '{self.camera.name}' "
            f"({'simulation' if self.camera.is_simulation else 'REAL DEVICE'}): "
            f"colour {self.color_k.width}x{self.color_k.height} fx={self.color_k.fx:.1f}, "
            f"depth {self.depth_k.width}x{self.depth_k.height} fx={self.depth_k.fx:.1f}")

        pc = self.cfg["pointcloud"]
        self.pc_enabled = bool(self.get_parameter("publish_pointcloud").value and pc["enabled"])
        self.pc_decimation = int(pc["decimation"])
        self.pc_frame = pc["frame_id"]
        self.depth_scale = float(self.cfg["depth"]["depth_scale_m"])
        self.max_usable = float(self.cfg["depth"].get("max_usable_range_m",
                                                      self.cfg["depth"]["max_range_m"]))
        self.min_range = float(self.cfg["depth"]["min_range_m"])

        ns = "robodog/camera"
        self.pub_color = self.create_publisher(Image, f"{ns}/color/image_raw", SENSOR_QOS)
        self.pub_color_info = self.create_publisher(CameraInfo, f"{ns}/color/camera_info", SENSOR_QOS)
        self.pub_depth = self.create_publisher(Image, f"{ns}/depth/image_rect_raw", SENSOR_QOS)
        self.pub_depth_info = self.create_publisher(CameraInfo, f"{ns}/depth/camera_info", SENSOR_QOS)
        self.pub_points = self.create_publisher(
            __import__("sensor_msgs.msg", fromlist=["PointCloud2"]).PointCloud2,
            f"{ns}/depth/points", SENSOR_QOS)

        self._pc_decim = max(1, int(float(self.get_parameter("rate_hz").value) /
                                    max(float(pc["rate_hz"]), 1e-6)))
        self._tick = 0
        self._errors = 0

        if self.camera.is_simulation:
            # The simulated camera needs the robot placed in its own copy of the
            # scene; the real one does not care where the robot is.
            self.create_subscription(RobotState, "robodog/robot_state", self._on_state, 10)
        self.timer = self.create_timer(1.0 / float(self.get_parameter("rate_hz").value),
                                       self._tick_cb)

    def _on_state(self, msg: RobotState) -> None:
        p = msg.base_pose.position
        o = msg.base_pose.orientation
        self.camera.set_robot_state((p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
                                    [j.position for j in msg.joints])

    def _tick_cb(self) -> None:
        stamp = self.get_clock().now().to_msg()
        try:
            frame = self.camera.capture()
        except CameraBackendError as e:
            self._errors += 1
            self.get_logger().warn(f"capture failed: {e}", throttle_duration_sec=5.0)
            return

        if frame.color is not None:
            self.pub_color.publish(
                image_msg(np.ascontiguousarray(frame.color, dtype=np.uint8), "rgb8",
                          "camera_color_optical_frame", stamp))
            self.pub_color_info.publish(
                camera_info(self.color_k, "camera_color_optical_frame", stamp))

        if frame.depth is not None:
            # float32 metres (NaN = invalid) -> 16UC1 millimetres (0 = invalid)
            mm = np.nan_to_num(frame.depth / self.depth_scale, nan=0.0,
                               posinf=0.0, neginf=0.0)
            mm = np.clip(mm, 0, 65535).astype(np.uint16)
            self.pub_depth.publish(
                image_msg(np.ascontiguousarray(mm), "16UC1",
                          "camera_depth_optical_frame", stamp))
            self.pub_depth_info.publish(
                camera_info(self.depth_k, "camera_depth_optical_frame", stamp))

            if self.pc_enabled and self._tick % self._pc_decim == 0:
                pts, idx = deproject(frame.depth, self.depth_k,
                                     decimation=self.pc_decimation,
                                     min_range=self.min_range, max_range=self.max_usable)
                colors = None
                if frame.color is not None and len(pts):
                    sy = frame.color.shape[0] / frame.depth.shape[0]
                    sx = frame.color.shape[1] / frame.depth.shape[1]
                    cy = np.clip((idx[:, 0] * sy).astype(int), 0, frame.color.shape[0] - 1)
                    cx = np.clip((idx[:, 1] * sx).astype(int), 0, frame.color.shape[1] - 1)
                    colors = frame.color[cy, cx]
                self.pub_points.publish(make_cloud(pts, colors, self.pc_frame, stamp))
        self._tick += 1

    def destroy_node(self) -> None:
        try:
            self.camera.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    try:
        node = CameraNode()
    except Exception as e:
        print(f"camera node failed to start: {e}")
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
