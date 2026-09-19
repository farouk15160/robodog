"""
Yahboom NUWA HP60C backend: the real depth camera.

STATUS: structure complete, UNVALIDATED against the device. Everything that can
be written without the hardware is written; what cannot is marked HW-CHECK.
Nothing above the camera boundary changes when this becomes the active backend.

Two integration routes, chosen by `driver` in the config:

  "uvc"     Open the device as a UVC camera with OpenCV and split the frame.
            Works with no vendor SDK, which makes it the right first attempt.
            HW-CHECK: whether depth arrives as a second UVC stream, as a
            side-by-side frame, or only through the SDK.

  "sdk"     Use the vendor SDK/ROS driver if the UVC route cannot deliver
            registered depth. In that case this class becomes a thin adapter
            over the vendor's topics and the node graph still does not change.

Either way the contract is identical: `capture()` returns metres in a float32
array with NaN for invalid, and `intrinsics()` returns what the device reports.
"""
from __future__ import annotations

import time
from typing import Any

import numpy as np

from ..backend import CameraBackend, CameraBackendError, Frame, Intrinsics


class NuwaHP60CBackend(CameraBackend):
    name = "nuwa_hp60c"
    is_simulation = False

    def __init__(self, config: dict[str, Any]) -> None:
        super().__init__(config)
        self.driver = config.get("driver", "uvc")
        self.device = config.get("device", "/dev/video0")
        cc, dc = config["color"], config["depth"]
        self.depth_scale = float(dc.get("depth_scale_m", 0.001))
        self.min_range = float(dc["min_range_m"])
        self.max_range = float(dc["max_range_m"])
        # Until a calibration exists these come from the configured FOV, the
        # same as in simulation. `configure()` overwrites them if the device
        # reports its own.
        self.color_k = Intrinsics.from_fov(cc["width"], cc["height"],
                                           cc["hfov_deg"], cc["vfov_deg"],
                                           distortion=tuple(cc.get("distortion", [0.0] * 5)))
        self.depth_k = Intrinsics.from_fov(dc["width"], dc["height"],
                                           dc["hfov_deg"], dc["vfov_deg"])
        self._cap = None
        self._frames = 0
        self._drops = 0

    def configure(self) -> None:                         # pragma: no cover
        if self.driver != "uvc":
            raise CameraBackendError(
                f"driver '{self.driver}' is not implemented yet; see the module "
                "docstring for the SDK route")
        try:
            import cv2
        except ImportError as e:
            raise CameraBackendError("opencv-python is required for the uvc driver") from e
        self._cv2 = cv2
        cap = cv2.VideoCapture(self.device)
        if not cap.isOpened():
            raise CameraBackendError(f"cannot open {self.device}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.color_k.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.color_k.height)
        cap.set(cv2.CAP_PROP_FPS, self.config["color"]["fps"])
        self._cap = cap
        # HW-CHECK: confirm the negotiated format matches what was requested;
        # UVC silently falls back, and a silent fallback to 640x480 would make
        # every intrinsic in this file wrong by a factor of two.
        got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if got != (self.color_k.width, self.color_k.height):
            raise CameraBackendError(
                f"device negotiated {got}, not {(self.color_k.width, self.color_k.height)}; "
                "update config/nuwa_hp60c.yaml to a supported mode")

    def shutdown(self) -> None:                          # pragma: no cover
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def capture(self) -> Frame:                          # pragma: no cover
        if self._cap is None:
            raise CameraBackendError("configure() has not been called")
        ok, bgr = self._cap.read()
        if not ok:
            self._drops += 1
            raise CameraBackendError("frame drop")
        rgb = self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2RGB)
        # HW-CHECK: replace with the device's real depth stream. Returning None
        # rather than zeros is deliberate -- a downstream consumer must be able
        # to tell "no depth" from "everything is at 0 m".
        depth = self._read_depth()
        self._frames += 1
        return Frame(color=rgb, depth=depth, stamp=time.time())

    def _read_depth(self) -> np.ndarray | None:          # pragma: no cover
        """HW-CHECK: how depth is delivered is the one thing that cannot be
        determined without the device. Once known, convert to float32 metres
        and set out-of-range samples to NaN -- the rest of the pipeline already
        handles that representation."""
        return None

    def intrinsics(self) -> tuple[Intrinsics, Intrinsics]:
        return self.color_k, self.depth_k

    def stats(self) -> dict[str, Any]:
        return {"backend": self.name, "frames": self._frames, "drops": self._drops,
                "device": self.device, "driver": self.driver}
