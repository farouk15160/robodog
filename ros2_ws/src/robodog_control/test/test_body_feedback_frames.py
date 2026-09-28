"""Base velocity handed to gait/balance is expressed in the body frame."""
import math
import numpy as np
import pytest

from robodog_control.control_node import RoboDogControlNode
from robodog_hardware.types import BaseState


def yaw_quat(yaw):
    return np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])


def test_body_feedback_rotates_world_velocity_into_body_frame():
    node = object.__new__(RoboDogControlNode)
    base = BaseState()
    base.orientation = yaw_quat(math.pi / 2.0)
    base.linear_velocity[:] = [0.0, 1.0, 0.0]
    node._base = base

    feedback = node._body_feedback()

    assert feedback.v_xy == pytest.approx((1.0, 0.0), abs=1e-9)
