"""Greeting behavior at the motion-generator and ROS service seams."""
from types import SimpleNamespace
import threading

import numpy as np

from robodog_control.control_node import RoboDogControlNode
from robodog_control.greeting import GreetingMotion
from robodog_control.kinematics import LegGeometry, forward_in_base
from robodog_hardware.types import BaseState, JointState


STAND = np.array([0.0, 0.693417, -1.371545] * 4)
LOWER = np.array([-0.8, -1.4, -2.6] * 4)
UPPER = np.array([0.8, 2.4, 0.0] * 4)
GEOMETRY = LegGeometry(
    haa_x=0.231,
    haa_y=0.060,
    hfe_dx=0.060,
    hfe_dr=0.016,
    thigh=0.192,
    thigh_lat=0.068501,
    shank=0.195621,
    foot_radius=0.020,
)


def response():
    return SimpleNamespace(success=False, message="")


def node_shell(*, simulation=True, enabled=True, controller="pose",
               active_pose="stand", latched=False):
    node = object.__new__(RoboDogControlNode)
    node._lock = threading.RLock()
    node.backend = SimpleNamespace(is_simulation=simulation)
    node.enabled = enabled
    node.controller = controller
    node.active_pose = active_pose
    node.active_gait = "stand"
    node.trajectory = None
    node.greeting = None
    node._greeting_pending = False
    node._greeting_pending_until = 0.0
    node.safety = SimpleNamespace(latched=latched)
    node.poses = {"stand": STAND.copy()}
    node.P = {"named_poses": {"stand": {"base_height_m": 0.32}}}
    node.q_lower = LOWER.copy()
    node.q_upper = UPPER.copy()
    node._last_state = JointState(position=STAND.copy())
    node._base = BaseState(position=np.array([0.0, 0.0, 0.32]),
                           foot_contact=np.ones(4, dtype=bool))
    node._q_hold = STAND.copy()
    node.state = 0
    node._last_cmd_vel_stamp = None
    node.external_cmd = None
    node.get_parameter = lambda _: SimpleNamespace(value=1.0)
    node.get_logger = lambda: SimpleNamespace(warn=lambda *_args, **_kwargs: None)
    return node


def test_greeting_motion_coordinates_support_legs_and_returns_to_stand():
    motion = GreetingMotion(STAND, STAND, LOWER, UPPER)
    samples = []
    while not motion.done:
        q, qd, _ = motion.update(0.05)
        samples.append(q)
        assert np.all(q >= LOWER)
        assert np.all(q <= UPPER)
        assert np.all(np.isfinite(qd))

    samples = np.asarray(samples)
    # The front-left leg visibly folds and waves at the hip-abduction joint.
    assert np.ptp(samples[:, 0]) >= 0.23
    assert np.min(samples[:, 2]) <= -1.9
    # Each support leg participates in the slow body shift, but its motion is
    # smaller than the waving leg's motion.
    support_span = np.ptp(samples[:, 3:].reshape(-1, 3, 3), axis=0)
    assert np.all(np.max(support_span, axis=1) >= 0.10)
    assert np.all(np.max(support_span, axis=1) <= 0.60)
    assert np.allclose(samples[-1], STAND)


def _support_margin_at_origin(points: np.ndarray) -> float:
    """Signed distance from the origin to the closest triangle edge."""
    twice_area = sum(
        np.cross(points[i], points[(i + 1) % 3]) for i in range(3))
    orientation = np.sign(twice_area)
    distances = []
    for index, start in enumerate(points):
        edge = points[(index + 1) % 3] - start
        cross = np.cross(edge, -start)
        distances.append(orientation * cross / np.linalg.norm(edge))
    return float(min(distances))


def test_greeting_moves_center_inside_support_triangle_before_front_left_lift():
    motion = GreetingMotion(STAND, STAND, LOWER, UPPER)
    lifted_samples = []
    peak_speed = 0.0

    while not motion.done:
        q, qd, _ = motion.update(0.01)
        feet = np.asarray([
            forward_in_base(GEOMETRY, leg, q[3 * i:3 * i + 3])
            for i, leg in enumerate(("FL", "FR", "RL", "RR"))
        ])
        support_height = float(np.mean(feet[1:, 2]))
        clearance = feet[0, 2] - support_height
        if clearance >= 0.06:
            lifted_samples.append(feet)
        peak_speed = max(peak_speed, float(np.max(np.abs(qd))))

    assert lifted_samples, "front-left foot never achieved useful clearance"
    margins = [
        # Foot coordinates are expressed in the base frame, so the origin is
        # the body-centre projection whose support margin we need to measure.
        _support_margin_at_origin(feet[[1, 3, 2], :2])
        for feet in lifted_samples
    ]
    assert min(margins) >= 0.04
    assert peak_speed <= 1.25


def test_greeting_service_starts_nonblocking_motion_from_stable_sim_stand():
    node = node_shell()

    result = node._srv_greeting(SimpleNamespace(), response())

    assert result.success
    assert "accepted" in result.message
    assert node.controller == "greeting"
    assert isinstance(node.greeting, GreetingMotion)


def test_greeting_service_refuses_hardware_and_estop_but_queues_from_motion():
    hardware = node_shell(simulation=False)
    moving = node_shell(controller="gait", active_pose="")
    stopped = node_shell(latched=True)

    assert not hardware._srv_greeting(SimpleNamespace(), response()).success
    assert "simulation-only" in hardware._srv_greeting(SimpleNamespace(), response()).message
    result = moving._srv_greeting(SimpleNamespace(), response())
    assert result.success
    assert "automatically" in result.message
    assert moving.controller == "pose"
    assert moving.active_pose == "stand"
    assert moving._greeting_pending
    assert not stopped._srv_greeting(SimpleNamespace(), response()).success


def test_simulation_greeting_requires_upright_quiet_four_foot_stand():
    node = node_shell()

    assert node._greeting_readiness()[0]

    node._base.foot_contact[:] = False
    assert not node._greeting_readiness()[0]

    node._base.foot_contact[:] = True
    node._base.position[2] = 0.06
    assert not node._greeting_readiness()[0]

    node._base.position[2] = 0.32
    node._base.orientation[:] = [-1.0, 0.0, 0.0, 0.0]
    assert not node._greeting_readiness()[0]

    node._base.orientation[:] = [0.0, 0.0, 0.0, 1.0]
    node._base.angular_velocity[1] = 0.4
    assert not node._greeting_readiness()[0]

    node._base.angular_velocity[1] = float("nan")
    assert not node._greeting_readiness()[0]


def test_queued_greeting_starts_after_stand_settles():
    node = node_shell(controller="gait", active_pose="")
    result = node._srv_greeting(SimpleNamespace(), response())
    assert result.success and node._greeting_pending

    node.trajectory = None
    assert node._start_pending_greeting()
    assert not node._greeting_pending
    assert node.controller == "greeting"
    assert isinstance(node.greeting, GreetingMotion)


def test_new_pose_request_cancels_active_or_queued_greeting():
    node = node_shell()
    assert node._srv_greeting(SimpleNamespace(), response()).success
    node._last_state.position = STAND + 0.01
    node.get_parameter = lambda _: SimpleNamespace(value=1.0)

    ok, _, _ = node._start_pose("stand", 0.5)

    assert ok
    assert node.controller == "pose"
    assert node.greeting is None

    node.controller = "gait"
    result = node._srv_greeting(SimpleNamespace(), response())
    assert result.success and node._greeting_pending

    ok, _, _ = node._start_pose("stand", 0.5)

    assert ok
    assert not node._greeting_pending


def test_joint_command_cancels_a_queued_greeting():
    node = node_shell(controller="gait", active_pose="")
    assert node._srv_greeting(SimpleNamespace(), response()).success

    node._on_joint_command(SimpleNamespace(names=[], commands=[]))

    assert not node._greeting_pending
