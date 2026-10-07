"""ROS service that saves a complete, immutable RTAB-Map room scan."""
from __future__ import annotations

from pathlib import Path
import subprocess
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robodog_msgs.srv import SaveMap
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty

from .map_export import RoomScanExporter
from .mapping import export_plan


def require_fresh_cloud(cloud: PointCloud2 | None, *, received_at: float | None,
                        now: float, max_age_s: float) -> PointCloud2:
    """Return a recent assembled map, rejecting a stopped mapping stream."""
    if cloud is None or received_at is None:
        raise RuntimeError("No assembled RTAB-Map cloud is available")
    age = now - received_at
    if not 0.0 <= age <= max_age_s:
        raise RuntimeError(
            f"Assembled RTAB-Map cloud is stale ({age:.1f}s old; limit {max_age_s:.1f}s); "
            "wait for mapping to update before saving")
    return cloud


class MapExportNode(Node):
    def __init__(self):
        super().__init__("map_export")
        self.declare_parameter("database_path", "")
        self.declare_parameter("output_directory", "")
        self.declare_parameter("timeout_s", 30.0)
        self.declare_parameter("max_cloud_age_s", 5.0)
        self._database = Path(str(self.get_parameter("database_path").value)).expanduser().absolute()
        self._default_output = str(self.get_parameter("output_directory").value)
        self._timeout = float(self.get_parameter("timeout_s").value)
        self._max_cloud_age = float(self.get_parameter("max_cloud_age_s").value)
        if self._max_cloud_age <= 0.0:
            raise ValueError("max_cloud_age_s must be positive")
        self._latest_cloud = None
        self._cloud_received_at = None
        self._cloud_ready = threading.Event()
        self._cloud_lock = threading.Lock()
        self._export_lock = threading.Lock()
        callbacks = ReentrantCallbackGroup()
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, "/robodog/mapping/cloud_map",
                                 self._on_cloud, map_qos, callback_group=callbacks)
        self._backup = self.create_client(
            Empty, "/robodog/mapping/rtabmap/backup", callback_group=callbacks)
        self.create_service(SaveMap, "/robodog/mapping/save_map",
                            self._save, callback_group=callbacks)
        self.get_logger().info(
            "room-scan exporter ready: assembled cloud + full OctoMap + RTAB database backup")

    def _on_cloud(self, message: PointCloud2) -> None:
        with self._cloud_lock:
            self._latest_cloud = message
            self._cloud_received_at = time.monotonic()
        self._cloud_ready.set()

    def _wait_future(self, future, operation: str):
        ready = threading.Event()
        future.add_done_callback(lambda _future: ready.set())
        if not ready.wait(self._timeout):
            raise TimeoutError(f"Timed out waiting for {operation}")
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"{operation} failed: {exception}") from exception
        return future.result()

    def _cloud(self) -> PointCloud2:
        if not self._cloud_ready.wait(self._timeout):
            raise TimeoutError(
                "No assembled RTAB-Map cloud received on /robodog/mapping/cloud_map; "
                "move the robot through the room and wait for mapping to update")
        with self._cloud_lock:
            cloud = self._latest_cloud
            received_at = self._cloud_received_at
        return require_fresh_cloud(
            cloud, received_at=received_at, now=time.monotonic(),
            max_age_s=self._max_cloud_age)

    def _flush_database(self) -> Path:
        if not self._backup.wait_for_service(timeout_sec=self._timeout):
            raise TimeoutError("RTAB-Map backup service is unavailable")
        self._wait_future(self._backup.call_async(Empty.Request()), "RTAB-Map database backup")
        return Path(str(self._database) + ".back")

    def _save_octomap(self, path: Path) -> None:
        command = [
            "ros2", "run", "octomap_server", "octomap_saver_node", "--ros-args",
            "-p", "full:=true", "-p", f"octomap_path:={path}",
            "-r", "octomap_full:=/robodog/mapping/rtabmap/octomap_full",
        ]
        try:
            result = subprocess.run(command, capture_output=True, text=True,
                                    timeout=self._timeout, check=False)
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError("Timed out requesting the full colored OctoMap") from exc
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()[-1000:]
            raise RuntimeError(f"OctoMap saver failed ({result.returncode}): {detail}")

    def _save(self, request: SaveMap.Request, response: SaveMap.Response):
        if not self._export_lock.acquire(blocking=False):
            response.message = "A room-scan export is already in progress"
            return response
        try:
            output = request.output_directory or self._default_output
            plan = export_plan(output_directory=output, name=request.name)
            result = RoomScanExporter(
                database_path=self._database,
                cloud_provider=self._cloud,
                flush_database=self._flush_database,
                save_octomap=self._save_octomap,
            ).export(plan)
            response.success = True
            response.manifest_path = str(result.manifest_path)
            response.message = f"Room scan saved: {result.manifest_path}"
            self.get_logger().info(response.message)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"Room-scan export failed: {exc}")
        finally:
            self._export_lock.release()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapExportNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
