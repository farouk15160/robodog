"""cmd_vel policy at the control-node callback boundary."""
from types import SimpleNamespace
import threading

from geometry_msgs.msg import Twist

from robodog_control.control_node import RoboDogControlNode
from robodog_control.gait import GaitParams
from robodog_msgs.msg import GaitCommand
from robodog_hardware.types import BaseState


class Logger:
    def __init__(self):
        self.warnings = []

    def warn(self, msg, **_):
        self.warnings.append(msg)


def node_shell(*, simulation=True, enabled=True, controller="pose", active_pose="stand"):
    node = object.__new__(RoboDogControlNode)
    node._lock = threading.RLock()
    node._base = BaseState()
    node.enabled = enabled
    node.controller = controller
    node.active_pose = active_pose
    node.active_gait = "stand"
    node.trajectory = None
    node._pending_gait = None
    node.gait = SimpleNamespace(params=GaitParams(gait="stand"))
    node.GAITS = {
        "stand": {"step_frequency_hz": 1.0, "step_height_m": 0.0,
                  "stance_height_m": 0.32, "duty_factor": 1.0},
        "trot": {"step_frequency_hz": 1.5, "step_height_m": 0.03,
                 "stance_height_m": 0.32, "duty_factor": 0.78},
    }
    node.backend = SimpleNamespace(is_simulation=simulation)
    node.safety = SimpleNamespace(latched=False)
    node._logger = Logger()
    node.get_logger = lambda: node._logger
    params = {
        "default_travel_gait": "trot",
        "max_cmd_linear_velocity_sim_m_s": 2.0,
        "max_cmd_linear_velocity_hardware_m_s": 0.5,
        "max_cmd_yaw_rate_rad_s": 2.0,
        "cmd_vel_timeout_s": 0.35,
    }
    node.get_parameter = lambda name: SimpleNamespace(value=params[name])
    return node


def test_stale_velocity_is_zeroed_if_publisher_disappears():
    node = node_shell(controller="gait")
    node.gait.params = GaitParams(gait="trot", vx=1.0, vy=-0.2, wz=0.4)
    node._last_cmd_vel_stamp = 10.0

    assert not node._stop_stale_body_velocity(now=10.3)
    assert node._pending_gait is None
    assert node._stop_stale_body_velocity(now=10.36)
    assert (node._pending_gait.vx, node._pending_gait.vy, node._pending_gait.wz) == (0.0, 0.0, 0.0)
    assert node._last_cmd_vel_stamp is None


def twist(vx=0.0, vy=0.0, wz=0.0):
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = wz
    return msg


def test_cmd_vel_promotes_stable_sim_stand_pose_to_default_travel_gait():
    node = node_shell()

    node._on_cmd_vel(twist(vx=1.2))

    assert node.controller == "gait"
    assert node.active_gait == "trot"
    assert node._pending_gait.gait == "trot"
    assert node._pending_gait.vx == 1.2
    assert node._pending_gait.step_frequency_hz == 1.5
    assert node._pending_gait.duty_factor == 0.78


def test_back_to_back_cmd_vel_keeps_promoted_pending_travel_gait():
    node = node_shell()

    node._on_cmd_vel(twist(vx=0.2))
    node._on_cmd_vel(twist(vx=0.3))

    assert node.controller == "gait"
    assert node.active_gait == "trot"
    assert node._pending_gait.gait == "trot"
    assert node._pending_gait.vx == 0.3
    assert node._pending_gait.step_frequency_hz == 1.5
    assert node._pending_gait.duty_factor == 0.78


def test_cmd_vel_after_gait_service_keeps_pending_gait_before_cycle_consumes_it():
    node = node_shell()
    node.GAITS["walk"] = {"step_frequency_hz": 1.2, "step_height_m": 0.04,
                          "stance_height_m": 0.31, "duty_factor": 0.75}

    ok, why = node._apply_gait(gait_command(gait="walk", vx=0.1))
    assert ok, why
    node._on_cmd_vel(twist(vx=0.2))

    assert node.controller == "gait"
    assert node.active_gait == "walk"
    assert node._pending_gait.gait == "walk"
    assert node._pending_gait.vx == 0.2
    assert node._pending_gait.step_frequency_hz == 1.2
    assert node._pending_gait.duty_factor == 0.75


def test_cmd_vel_does_not_arm_disabled_or_hardware_pose():
    disabled = node_shell(enabled=False)
    disabled._on_cmd_vel(twist(vx=1.0))
    assert disabled.controller == "pose"
    assert disabled._pending_gait is None

    hardware = node_shell(simulation=False)
    hardware._on_cmd_vel(twist(vx=1.0))
    assert hardware.controller == "pose"
    assert hardware._pending_gait is None


def test_cmd_vel_rejects_mid_trajectory_and_joint_test():
    moving_pose = node_shell()
    moving_pose.trajectory = object()
    moving_pose._on_cmd_vel(twist(vx=1.0))
    assert moving_pose.controller == "pose"
    assert moving_pose._pending_gait is None

    joint_test = node_shell(controller="joint_test")
    joint_test._on_cmd_vel(twist(vx=1.0))
    assert joint_test.controller == "joint_test"
    assert joint_test._pending_gait is None


def test_cmd_vel_caps_and_rejects_nonfinite_values():
    node = node_shell()
    node._on_cmd_vel(twist(vx=9.0, vy=-9.0, wz=9.0))
    assert node._pending_gait.vx == 2.0
    assert node._pending_gait.vy == -2.0
    assert node._pending_gait.wz == 2.0

    invalid = node_shell()
    invalid._on_cmd_vel(twist(vx=float("nan")))
    assert invalid._pending_gait is None
    assert invalid.controller == "pose"


def gait_command(gait="trot", vx=0.0, vy=0.0, wz=0.0, enable=True):
    msg = GaitCommand()
    msg.gait = gait
    msg.enable = enable
    msg.velocity = twist(vx, vy, wz)
    return msg


def test_gait_service_caps_simulation_velocity_and_rejects_nonfinite():
    node = node_shell(controller="pose")

    ok, why = node._apply_gait(gait_command(vx=9.0, vy=-9.0, wz=9.0))

    assert ok, why
    assert node.controller == "gait"
    assert node._pending_gait.vx == 2.0
    assert node._pending_gait.vy == -2.0
    assert node._pending_gait.wz == 2.0

    invalid = node_shell(controller="pose")
    ok, why = invalid._apply_gait(gait_command(vx=float("inf")))
    assert not ok
    assert "finite" in why
    assert invalid._pending_gait is None


def test_gait_service_caps_hardware_velocity_independent_of_gui():
    node = node_shell(simulation=False, controller="pose")

    ok, why = node._apply_gait(gait_command(vx=9.0))

    assert ok, why
    assert node._pending_gait.vx == 0.5
