"""
Simulated depth camera: renders the MuJoCo scene through the robot's camera.

Design note -- why this loads its OWN MuJoCo model
--------------------------------------------------
The physics instance lives inside the control node's backend. Rather than reach
into it, this backend loads the same scene read-only and drives it from the
published robot state (base pose + joint angles), then calls mj_forward and
renders. Consequences, all of them wanted:

  * the camera is a separate node with a separate lifecycle, exactly as the
    real USB camera will be -- so the node graph does not change on the real
    robot, and a camera crash cannot take the control loop down;
  * rendering cost never steals time from the 400 Hz control loop;
  * the camera sees the world at the robot state it was told about, which makes
    latency explicit and measurable instead of hidden.

The cost is a second copy of the scene in memory, and rendering being one
control cycle behind. Both are the right trade for a 10-30 Hz sensor.
"""
from __future__ import annotations

import math
import copy
import time
from typing import Any

import numpy as np

from ..backend import CameraBackend, CameraBackendError, Frame, Intrinsics


class SimCameraBackend(CameraBackend):
    name = "mujoco_camera"
    is_simulation = True

    def __init__(self, config: dict[str, Any]) -> None:
        super().__init__(config)
        self.model_path = config.get("model_path")
        if not self.model_path:
            raise CameraBackendError("SimCameraBackend requires config['model_path']")
        self.camera_name = config.get("camera_name", "depth_camera")
        self.mapping_mode = bool(config.get("mapping_mode", False))
        cc, dc = self._stream_configs(config)
        # These provisional intrinsics keep the backend usable before configure().
        # configure() replaces them with values derived from the actual MuJoCo
        # camera fovy and render resolution.
        if self.mapping_mode:
            # Mapping mode is a simulated registered RGB-D camera. Before the
            # MuJoCo model is loaded, use one provisional pinhole model for
            # both streams so CameraInfo remains self-consistent.
            self.color_k = Intrinsics.from_fov(dc["width"], dc["height"],
                                               dc["hfov_deg"], dc["vfov_deg"])
            self.depth_k = Intrinsics.from_fov(dc["width"], dc["height"],
                                               dc["hfov_deg"], dc["vfov_deg"])
        else:
            self.color_k = Intrinsics.from_fov(cc["width"], cc["height"],
                                               cc["hfov_deg"], cc["vfov_deg"])
            self.depth_k = Intrinsics.from_fov(dc["width"], dc["height"],
                                               dc["hfov_deg"], dc["vfov_deg"])
        self.min_range = float(dc["min_range_m"])
        self.max_range = float(dc["max_range_m"])
        self.baseline = float(dc.get("baseline_m", 0.05))
        self.disp_noise = float(dc.get("disparity_noise_px", 0.0))
        self.add_noise = bool(config.get("simulate_noise", True))
        self._m = self._d = self._r = None
        self._rng = np.random.default_rng(0)
        self._frames = 0
        self._pose_stamp = None

    @staticmethod
    def _stream_configs(config: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        cc = copy.deepcopy(config["color"])
        dc = copy.deepcopy(config["depth"])
        if config.get("mapping_mode", False):
            cc["width"], cc["height"] = 640, 480
            dc["width"], dc["height"] = 640, 480
        return cc, dc

    def configure(self) -> None:
        try:
            import mujoco
        except ImportError as e:
            raise CameraBackendError("mujoco is not installed (pip install mujoco)") from e
        self._mj = mujoco
        self._m = mujoco.MjModel.from_xml_path(str(self.model_path))
        self._d = mujoco.MjData(self._m)
        # Place the model at its keyframe and run forward kinematics. Without
        # this every body sits at the origin, the camera ends up inside the
        # collapsed geometry, and the depth buffer comes back entirely invalid.
        kid = mujoco.mj_name2id(self._m, mujoco.mjtObj.mjOBJ_KEY,
                                str(self.config.get("keyframe", "stand")))
        if kid >= 0:
            mujoco.mj_resetDataKeyframe(self._m, self._d, kid)
        mujoco.mj_forward(self._m, self._d)
        cid = mujoco.mj_name2id(self._m, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)
        if cid < 0:
            raise CameraBackendError(f"camera '{self.camera_name}' not in {self.model_path}")
        self._cam_id = cid
        fovy_deg = float(self._m.cam_fovy[cid])
        render_w = max(self.color_k.width, self.depth_k.width)
        render_h = max(self.color_k.height, self.depth_k.height)
        render_k = Intrinsics.from_vfov(render_w, render_h, fovy_deg)
        self.color_k = render_k.scaled(self.color_k.width, self.color_k.height,
                                       distortion=(0.0, 0.0, 0.0, 0.0, 0.0))
        self.depth_k = render_k.scaled(self.depth_k.width, self.depth_k.height,
                                       distortion=(0.0, 0.0, 0.0, 0.0, 0.0))
        # Resolve qpos addresses by joint NAME. A world with movable props adds
        # free joints of its own, so the robot's state is not guaranteed to sit
        # at a fixed offset, and assuming one silently renders the wrong pose.
        free = mujoco.mj_name2id(self._m, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
        if free < 0:
            raise CameraBackendError("joint 'base_free' not found")
        self._base_adr = int(self._m.jnt_qposadr[free])
        self._joint_adr = []
        for leg in ("FL", "FR", "RL", "RR"):
            for k in ("haa", "hfe", "kfe"):
                j = mujoco.mj_name2id(self._m, mujoco.mjtObj.mjOBJ_JOINT, f"{leg}_{k}_joint")
                if j < 0:
                    raise CameraBackendError(f"joint '{leg}_{k}_joint' not found")
                self._joint_adr.append(int(self._m.jnt_qposadr[j]))
        # One renderer at the larger of the two resolutions; the colour and
        # depth images are derived from it, which keeps them perfectly
        # registered -- a real stereo pair is not, and the driver for the real
        # camera is where that difference belongs.
        w = max(self.color_k.width, self.depth_k.width)
        h = max(self.color_k.height, self.depth_k.height)
        try:
            self._r = mujoco.Renderer(self._m, height=h, width=w)
        except Exception as e:                       # pragma: no cover
            raise CameraBackendError(
                f"could not create a MuJoCo renderer ({e}). Off-screen rendering "
                "needs a GL context; try MUJOCO_GL=egl or MUJOCO_GL=osmesa.") from e
        self._rw, self._rh = w, h

    def shutdown(self) -> None:
        # EGL teardown raises spuriously on some drivers; a failure to release a
        # render context must not take the node down on shutdown.
        if self._r is not None:
            try:
                self._r.close()
            except Exception:
                pass
            self._r = None

    # ------------------------------------------------------------------ #
    def set_robot_state(self, base_pos, base_quat_xyzw, joint_positions, stamp=None) -> None:
        """Place the robot in the render scene. Called by the node from the
        latest RobotState; the simulated camera is otherwise stateless."""
        if self._d is None:
            return
        self._pose_stamp = stamp
        q = self._d.qpos
        a = self._base_adr
        q[a:a + 3] = base_pos
        x, y, z, w = base_quat_xyzw
        q[a + 3:a + 7] = (w, x, y, z)          # MuJoCo is wxyz, ROS is xyzw
        for adr, value in zip(self._joint_adr, joint_positions):
            q[adr] = value
        self._mj.mj_forward(self._m, self._d)

    def capture(self) -> Frame:
        if self._r is None:
            raise CameraBackendError("configure() has not been called")
        self._r.update_scene(self._d, camera=self._cam_id)
        self._r.disable_depth_rendering()
        rgb = self._r.render().copy()
        self._r.enable_depth_rendering()
        depth = self._r.render().copy().astype(np.float32)
        self._r.disable_depth_rendering()

        depth = self._apply_sensor_model(depth)
        self._frames += 1
        return Frame(color=self._resize(rgb, self.color_k),
                     depth=self._resize(depth, self.depth_k),
                     stamp=self._pose_stamp if self._pose_stamp is not None else time.time())

    def _apply_sensor_model(self, depth: np.ndarray) -> np.ndarray:
        """Turn a perfect z-buffer into something the real device could return.

        A pipeline tuned against noiseless, unbounded depth fails immediately on
        hardware, so the two effects that dominate a stereo camera are applied
        here: hard range limits, and noise that grows with the square of range.
        """
        out = depth.copy()
        out[(out < self.min_range) | (out > self.max_range)] = np.nan
        if self.add_noise and self.disp_noise > 0:
            f = self.depth_k.fx
            sigma = out ** 2 * self.disp_noise / (f * self.baseline)
            out = out + self._rng.normal(0.0, 1.0, out.shape).astype(np.float32) * sigma
        return out

    @staticmethod
    def _resize(img: np.ndarray, k: Intrinsics) -> np.ndarray:
        if img.shape[0] == k.height and img.shape[1] == k.width:
            return img
        ys = (np.linspace(0, img.shape[0] - 1, k.height)).astype(np.int32)
        xs = (np.linspace(0, img.shape[1] - 1, k.width)).astype(np.int32)
        return img[np.ix_(ys, xs)] if img.ndim == 2 else img[np.ix_(ys, xs)]

    def intrinsics(self) -> tuple[Intrinsics, Intrinsics]:
        return self.color_k, self.depth_k

    def stats(self) -> dict[str, Any]:
        return {"backend": self.name, "frames": self._frames,
                "render_size": (self._rw, self._rh) if self._r else None}
