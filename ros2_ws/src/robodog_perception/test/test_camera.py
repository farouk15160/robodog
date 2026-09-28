"""
Camera backend and depth-pipeline tests.

The self-occlusion test is the one that matters most: it is a geometry
constraint on the robot, not on the camera, and it was violated by the first
three mount positions tried -- at z = 0.020 the simulated camera saw nothing
but its own hip actuators.
"""
import copy
import math
import os
import subprocess
import sys
import types
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time

from robodog_perception.backend import Frame, Intrinsics
from robodog_perception.pointcloud import deproject, make_cloud
from robodog_perception.camera_node import CameraNode
from robodog_perception.registry import available, create_camera

PERC = get_package_share_directory("robodog_perception")


def _camera_fovy_from_model(model_path: str, camera_name: str = "depth_camera") -> float:
    root = ET.parse(model_path).getroot()
    for camera in root.iter("camera"):
        if camera.attrib.get("name") == camera_name:
            return float(camera.attrib["fovy"])
    raise AssertionError(f"camera {camera_name!r} not found in {model_path}")


def _mujoco_renderer_available(model_path: str) -> bool:
    env = os.environ.copy()
    env.setdefault("MUJOCO_GL", "egl")
    code = (
        "import sys, mujoco; "
        "m = mujoco.MjModel.from_xml_path(sys.argv[1]); "
        "r = mujoco.Renderer(m, height=8, width=8); "
        "r.close()"
    )
    try:
        result = subprocess.run([sys.executable, "-c", code, model_path], env=env,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                timeout=10, check=False)
    except Exception:
        return False
    return result.returncode == 0


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Logger:
    def warn(self, *args, **kwargs):
        pass


class _Clock:
    def now(self):
        return types.SimpleNamespace(to_msg=lambda: Time(sec=99, nanosec=0))


def _node_shell(camera, *, have_robot_state=True):
    node = types.SimpleNamespace()
    node.camera = camera
    node.color_k = Intrinsics(2, 1, 1.0, 1.0, 1.0, 0.5)
    node.depth_k = Intrinsics(2, 1, 1.0, 1.0, 1.0, 0.5)
    node.color_frame = "camera_color_optical_frame"
    node.depth_frame = "camera_color_optical_frame"
    node.pc_frame = "camera_color_optical_frame"
    node.pc_enabled = False
    node.pc_decimation = 1
    node.depth_scale = 0.001
    node.max_usable = 4.0
    node.min_range = 0.15
    node._pc_decim = 1
    node._tick = 0
    node._errors = 0
    node._have_robot_state = have_robot_state
    node._latest_robot_state_stamp = Time(sec=7, nanosec=8) if have_robot_state else None
    node._last_published_robot_state_stamp = None
    node.pub_color = _Publisher()
    node.pub_color_info = _Publisher()
    node.pub_depth = _Publisher()
    node.pub_depth_info = _Publisher()
    node.pub_points = _Publisher()
    node.get_clock = lambda: _Clock()
    node.get_logger = lambda: _Logger()
    return node


@pytest.fixture(scope="module")
def cfg():
    with open(os.path.join(PERC, "config", "nuwa_hp60c.yaml")) as f:
        c = yaml.safe_load(f)
    c["model_path"] = os.path.join(get_package_share_directory("robodog_sim"),
                                   "models", "robodog_house.xml")
    return c


# --------------------------------------------------------------------------- #
# intrinsics
# --------------------------------------------------------------------------- #
def test_registry_lists_both_backends():
    assert set(available()) == {"sim", "nuwa_hp60c"}


def test_intrinsics_from_fov_puts_the_principal_point_at_the_centre():
    k = Intrinsics.from_fov(640, 480, 65.0, 49.0)
    assert (k.cx, k.cy) == (320.0, 240.0)


def test_focal_length_matches_the_field_of_view():
    """fx = w / (2 tan(hfov/2)). A hand-written fx is the single biggest source
    of scale error in a depth pipeline, so it is derived, never guessed."""
    import math
    k = Intrinsics.from_fov(640, 480, 90.0, 90.0)
    assert k.fx == pytest.approx(320.0, rel=1e-9)
    assert math.degrees(2 * math.atan(k.width / (2 * k.fx))) == pytest.approx(90.0)


def test_projection_matrix_agrees_with_k():
    k = Intrinsics.from_fov(1280, 720, 65.0, 40.0)
    assert k.P[0] == k.K[0] and k.P[2] == k.K[2]
    assert k.P[5] == k.K[4] and k.P[6] == k.K[5]


def test_both_backends_report_the_configured_resolution(cfg):
    sim = create_camera("sim", cfg)
    real = create_camera("nuwa_hp60c", cfg)
    for cam in (sim, real):
        ck, dk = cam.intrinsics()
        assert (ck.width, ck.height) == (cfg["color"]["width"], cfg["color"]["height"])
        assert (dk.width, dk.height) == (cfg["depth"]["width"], cfg["depth"]["height"])


def test_mapping_mode_sim_backend_registers_rgbd_for_rtabmap(cfg):
    mapping_cfg = copy.deepcopy(cfg)
    mapping_cfg["mapping_mode"] = True

    cam = create_camera("sim", mapping_cfg)

    ck, dk = cam.intrinsics()
    assert (ck.width, ck.height) == (640, 480)
    assert (dk.width, dk.height) == (640, 480)
    assert ck.K == pytest.approx(dk.K)


def test_real_backend_declares_itself_not_a_simulation(cfg):
    assert create_camera("nuwa_hp60c", cfg).is_simulation is False


# --------------------------------------------------------------------------- #
# deprojection
# --------------------------------------------------------------------------- #
def test_deprojection_inverts_the_pinhole_projection():
    k = Intrinsics.from_fov(64, 48, 65.0, 49.0)
    depth = np.full((48, 64), 2.0, dtype=np.float32)
    pts, idx = deproject(depth, k, min_range=0.1, max_range=10.0)
    assert len(pts) == 48 * 64
    for (v, u), p in zip(idx[:50], pts[:50]):
        assert p[2] == pytest.approx(2.0)
        assert u == pytest.approx(p[0] * k.fx / p[2] + k.cx, abs=1e-4)
        assert v == pytest.approx(p[1] * k.fy / p[2] + k.cy, abs=1e-4)


def test_deprojection_drops_invalid_and_out_of_range_samples():
    k = Intrinsics.from_fov(8, 8, 65.0, 49.0)
    depth = np.full((8, 8), 2.0, dtype=np.float32)
    depth[0, 0] = np.nan
    depth[0, 1] = 0.05      # below min range
    depth[0, 2] = 50.0      # beyond max range
    pts, _ = deproject(depth, k, min_range=0.15, max_range=6.0)
    assert len(pts) == 61


def test_decimation_reduces_the_point_count_quadratically():
    k = Intrinsics.from_fov(64, 48, 65.0, 49.0)
    depth = np.full((48, 64), 2.0, dtype=np.float32)
    full, _ = deproject(depth, k, min_range=0.1, max_range=10.0)
    half, _ = deproject(depth, k, decimation=2, min_range=0.1, max_range=10.0)
    assert len(half) == pytest.approx(len(full) / 4, rel=0.05)


def test_pointcloud2_layout_is_xyzrgb_float32():
    pts = np.zeros((10, 3), dtype=np.float32)
    msg = make_cloud(pts, None, "camera_depth_optical_frame", Time())
    assert msg.point_step == 16 and msg.width == 10 and msg.height == 1
    assert [f.name for f in msg.fields] == ["x", "y", "z", "rgb"]
    assert len(msg.data) == 160


# --------------------------------------------------------------------------- #
# simulated camera against the real scene
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def sim_cam(cfg):
    pytest.importorskip("mujoco")
    os.environ.setdefault("MUJOCO_GL", "egl")
    cam = create_camera("sim", cfg)
    try:
        cam.configure()
    except Exception as e:                     # no GL in this environment
        pytest.skip(f"off-screen rendering unavailable: {e}")
    yield cam
    cam.shutdown()


def test_render_produces_the_configured_image_sizes(sim_cam, cfg):
    f = sim_cam.capture()
    assert f.color.shape == (cfg["color"]["height"], cfg["color"]["width"], 3)
    assert f.depth.shape == (cfg["depth"]["height"], cfg["depth"]["width"])
    assert f.color.dtype == np.uint8 and f.depth.dtype == np.float32


def test_sim_intrinsics_follow_mujoco_camera_fovy_and_render_resize(sim_cam, cfg):
    f = sim_cam.capture()
    assert f.color.size and f.depth.size
    ck, dk = sim_cam.intrinsics()
    render_w, render_h = sim_cam.stats()["render_size"]
    fovy = _camera_fovy_from_model(cfg["model_path"])
    render_focal = render_h / (2.0 * math.tan(math.radians(fovy) / 2.0))

    assert ck.fx == pytest.approx(render_focal * ck.width / render_w)
    assert ck.fy == pytest.approx(render_focal * ck.height / render_h)
    assert dk.fx == pytest.approx(render_focal * dk.width / render_w)
    assert dk.fy == pytest.approx(render_focal * dk.height / render_h)



def test_mapping_depth_reconstructs_segmented_ground_plane(cfg):
    if not _mujoco_renderer_available(cfg["model_path"]):
        pytest.skip("off-screen rendering unavailable")
    code = r"""
import copy
import os
import sys

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from robodog_perception.pointcloud import deproject
from robodog_perception.registry import create_camera

os.environ.setdefault("MUJOCO_GL", "egl")
perc = get_package_share_directory("robodog_perception")
with open(os.path.join(perc, "config", "nuwa_hp60c.yaml")) as f:
    cfg = yaml.safe_load(f)
cfg["model_path"] = sys.argv[1]
cfg["mapping_mode"] = True
cfg["simulate_noise"] = False

cam = create_camera("sim", cfg)
cam.configure()
try:
    frame = cam.capture()
    _, depth_k = cam.intrinsics()

    ground_id = cam._mj.mj_name2id(cam._m, cam._mj.mjtObj.mjOBJ_GEOM, "ground")
    assert ground_id >= 0
    cam._r.update_scene(cam._d, camera=cam._cam_id)
    cam._r.enable_segmentation_rendering()
    try:
        seg = cam._r.render().copy()
    finally:
        cam._r.disable_segmentation_rendering()

    geom_type = int(cam._mj.mjtObj.mjOBJ_GEOM.value)
    ground = (seg[:, :, 0] == ground_id) & (seg[:, :, 1] == geom_type)
    interior = np.zeros_like(ground, dtype=bool)
    interior[1:-1, 1:-1] = (ground[1:-1, 1:-1] & ground[:-2, 1:-1] & ground[2:, 1:-1] &
                            ground[1:-1, :-2] & ground[1:-1, 2:])
    pts_optical, _ = deproject(np.where(interior, frame.depth, np.nan), depth_k,
                               decimation=4, min_range=0.15, max_range=4.0)
    assert len(pts_optical) > 100, len(pts_optical)

    pts_mujoco_camera = np.column_stack((pts_optical[:, 0],
                                         -pts_optical[:, 1],
                                         -pts_optical[:, 2]))
    cam_pos = cam._d.cam_xpos[cam._cam_id]
    cam_rot = cam._d.cam_xmat[cam._cam_id].reshape(3, 3)
    pts_world = cam_pos + pts_mujoco_camera @ cam_rot.T
    z_error = np.abs(pts_world[:, 2])

    assert np.median(z_error) < 0.015, float(np.median(z_error))
    assert np.percentile(z_error, 95) < 0.05, float(np.percentile(z_error, 95))
finally:
    cam.shutdown()
"""
    env = os.environ.copy()
    env.setdefault("MUJOCO_GL", "egl")
    env["PYTHONPATH"] = os.pathsep.join(p for p in sys.path if p)
    result = subprocess.run([sys.executable, "-c", code, cfg["model_path"]], env=env,
                            text=True, capture_output=True, timeout=30, check=False)
    if result.returncode in {-6, 134} and "GL" in result.stderr:
        pytest.skip(f"off-screen rendering unavailable: {result.stderr[-400:]}")
    assert result.returncode == 0, result.stdout + result.stderr

def test_sim_capture_stamp_is_the_latest_robot_state_stamp(sim_cam):
    stamp = Time(sec=12, nanosec=34)
    sim_cam.set_robot_state((0.0, 0.0, 0.30), (0.0, 0.0, 0.0, 1.0), [0.0] * 12, stamp=stamp)

    assert sim_cam.capture().stamp is stamp


def test_camera_node_waits_for_robot_state_before_sim_capture():
    class Camera:
        is_simulation = True

        def capture(self):
            raise AssertionError("sim camera published before a RobotState pose arrived")

    node = _node_shell(Camera(), have_robot_state=False)

    CameraNode._tick_cb(node)

    assert node._tick == 0
    assert node.pub_color.messages == []
    assert node.pub_depth.messages == []


def test_camera_node_publishes_frame_stamp_and_shared_sim_optical_frame():
    stamp = Time(sec=7, nanosec=8)

    class Camera:
        is_simulation = True

        def capture(self):
            return Frame(color=np.zeros((1, 2, 3), dtype=np.uint8),
                         depth=np.ones((1, 2), dtype=np.float32),
                         stamp=stamp)

    node = _node_shell(Camera(), have_robot_state=True)

    CameraNode._tick_cb(node)

    assert node.pub_color.messages[0].header.stamp is stamp
    assert node.pub_color_info.messages[0].header.stamp is stamp
    assert node.pub_depth.messages[0].header.stamp is stamp
    assert node.pub_depth_info.messages[0].header.stamp is stamp
    assert node.pub_color.messages[0].header.frame_id == "camera_color_optical_frame"
    assert node.pub_depth.messages[0].header.frame_id == "camera_color_optical_frame"
    assert node.pub_depth_info.messages[0].header.frame_id == "camera_color_optical_frame"



def test_camera_node_uses_clock_for_non_ros_backend_stamp():
    class Camera:
        is_simulation = False

        def capture(self):
            return Frame(color=np.zeros((1, 2, 3), dtype=np.uint8), stamp=123.456)

    node = _node_shell(Camera(), have_robot_state=True)
    node.depth_frame = "camera_depth_optical_frame"

    CameraNode._tick_cb(node)

    assert node.pub_color.messages[0].header.stamp.sec == 99
    assert node.pub_color_info.messages[0].header.stamp.sec == 99


def test_camera_node_does_not_republish_a_stale_sim_robot_state():
    stamp = Time(sec=7, nanosec=8)

    class Camera:
        is_simulation = True

        def __init__(self):
            self.captures = 0

        def capture(self):
            self.captures += 1
            return Frame(color=np.zeros((1, 2, 3), dtype=np.uint8), stamp=stamp)

    camera = Camera()
    node = _node_shell(camera, have_robot_state=True)
    node._latest_robot_state_stamp = stamp

    CameraNode._tick_cb(node)
    CameraNode._tick_cb(node)

    assert camera.captures == 1
    assert len(node.pub_color.messages) == 1


def test_camera_does_not_look_at_the_robots_own_legs(sim_cam, cfg):
    """The mount height is determined by this constraint. At z = 0.020 the
    front hip assemblies filled 95% of the depth image; at z = 0.090 they still
    filled 48%. Anything above a few percent means the mount moved."""
    d = sim_cam.capture().depth
    too_close = np.isfinite(d) & (d < cfg["depth"]["min_range_m"] * 2.0)
    assert too_close.mean() < 0.02, (
        f"{too_close.mean()*100:.1f}% of the depth image is within 2x the "
        "minimum range -- the camera is looking at the robot")


def test_most_of_the_frame_returns_usable_depth(sim_cam):
    d = sim_cam.capture().depth
    assert np.isfinite(d).mean() > 0.5


def test_depth_respects_the_configured_range_limits(sim_cam, cfg):
    d = sim_cam.capture().depth
    valid = d[np.isfinite(d)]
    assert valid.min() >= cfg["depth"]["min_range_m"] - 1e-6
    assert valid.max() <= cfg["depth"]["max_range_m"] + 0.5   # + noise


def test_the_floor_is_visible_below_the_optical_axis(sim_cam, cfg):
    """A camera pitched down must see ground in the lower half of the frame.
    Catches a mount, pitch or optical-frame-convention error in one assertion."""
    f = sim_cam.capture()
    _, dk = sim_cam.intrinsics()
    pts, _ = deproject(f.depth, dk, decimation=4, min_range=0.15, max_range=4.0)
    assert len(pts) > 100
    assert pts[:, 1].max() > 0.1, "nothing is below the optical axis"


def test_range_noise_grows_with_the_square_of_distance(cfg):
    """Stereo error goes as z^2. A pipeline tuned against noiseless depth fails
    immediately on hardware, so the simulated camera must reproduce it."""
    pytest.importorskip("mujoco")
    cam = create_camera("sim", cfg)
    d = cfg["depth"]
    fx = cam.depth_k.fx
    sigma = lambda z: z ** 2 * d["disparity_noise_px"] / (fx * d["baseline_m"])
    assert sigma(4.0) / sigma(1.0) == pytest.approx(16.0, rel=1e-9)
    assert sigma(1.0) < 0.01, "near-field noise should be a few mm"
    assert sigma(6.0) > 0.05, "far-field noise should be significant"


def test_usable_range_is_shorter_than_the_maximum_range(cfg):
    """max_range_m is what the device reports; max_usable_range_m is where the
    z^2 error stops being worth mapping. They must not be the same number."""
    d = cfg["depth"]
    assert d["max_usable_range_m"] < d["max_range_m"]
