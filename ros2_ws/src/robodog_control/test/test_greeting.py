"""Greeting behavior at the motion-generator and ROS service seams."""
from types import SimpleNamespace
import threading

import numpy as np

from robodog_control.control_node import RoboDogControlNode
from robodog_control.greeting import GreetingMotion
from robodog_hardware.types import BaseState, JointState


STAND = np.array([0.0, 0.693417, -1.371545] * 4)
LOWER = np.array([-0.8, -1.4, -2.6] * 4)
UPPER = np.array([0.8, 2.4, 0.0] * 4)


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
    return node


def test_greeting_motion_lifts_only_front_left_and_returns_to_stand():
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
    assert np.ptp(samples[:, 0]) >= 0.35
    assert np.min(samples[:, 2]) <= -1.9
    # The other three legs remain planted at the calibrated stand angles.
    assert np.allclose(samples[:, 3:], STAND[3:])
    assert np.allclose(samples[-1], STAND)


def test_greeting_service_starts_nonblocking_motion_from_stable_sim_stand():
    node = node_shell()

    result = node._srv_greeting(SimpleNamespace(), response())

    assert result.success
    assert "accepted" in result.message
    assert node.controller == "greeting"
    assert isinstance(node.greeting, GreetingMotion)


def test_greeting_service_refuses_hardware_and_unstable_states():
    hardware = node_shell(simulation=False)
    moving = node_shell(controller="gait", active_pose="")
    stopped = node_shell(latched=True)

    assert not hardware._srv_greeting(SimpleNamespace(), response()).success
    assert "simulation-only" in hardware._srv_greeting(SimpleNamespace(), response()).message
    assert not moving._srv_greeting(SimpleNamespace(), response()).success
    assert not stopped._srv_greeting(SimpleNamespace(), response()).success


def test_greeting_service_requires_four_quiet_contacts_before_lifting_leg():
    node = node_shell()
    node._base.foot_contact[0] = False
    assert not node._srv_greeting(SimpleNamespace(), response()).success
    node._base.foot_contact[:] = True
    node._base.angular_velocity[1] = 0.4
    assert not node._srv_greeting(SimpleNamespace(), response()).success
    node._base.angular_velocity[1] = float("nan")
    assert not node._srv_greeting(SimpleNamespace(), response()).success


def test_new_pose_request_cancels_an_active_greeting():
    node = node_shell()
    assert node._srv_greeting(SimpleNamespace(), response()).success
    node._last_state.position = STAND + 0.01
    node.get_parameter = lambda _: SimpleNamespace(value=1.0)

    ok, _, _ = node._start_pose("stand", 0.5)

    assert ok
    assert node.controller == "pose"
    assert node.greeting is None
