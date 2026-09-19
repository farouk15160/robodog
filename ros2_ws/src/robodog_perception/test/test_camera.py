"""
Camera backend and depth-pipeline tests.

The self-occlusion test is the one that matters most: it is a geometry
constraint on the robot, not on the camera, and it was violated by the first
three mount positions tried -- at z = 0.020 the simulated camera saw nothing
but its own hip actuators.
"""
import os

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from robodog_perception.backend import Intrinsics
from robodog_perception.pointcloud import deproject, make_cloud
from robodog_perception.registry import available, create_camera

PERC = get_package_share_directory("robodog_perception")


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
    msg = make_cloud(pts, None, "camera_depth_optical_frame", None)
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
