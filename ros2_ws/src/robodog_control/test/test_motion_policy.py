"""Launching the simulation must not imply consent to energise real motors."""
import pytest

from robodog_control.motion_policy import startup_actions, travel_feedback_ready, joint_selection
from robodog_hardware.types import BaseState


def test_auto_start_is_enabled_only_for_simulation():
    assert startup_actions("mujoco", "auto", "auto") == (True, True)
    assert startup_actions("kinematic", "auto", "auto") == (True, True)
    assert startup_actions("robstride06_can", "auto", "auto") == (False, False)
    assert startup_actions("robstride06_can", "false", "false") == (False, False)


def test_explicit_startup_request_is_preserved_for_backend_calibration_gate():
    assert startup_actions("robstride06_can", "true", "false") == (True, False)
    with pytest.raises(ValueError):
        startup_actions("mujoco", "typo", "auto")


def test_travel_requires_an_actual_base_feedback_sample():
    assert not travel_feedback_ready(None)
    assert travel_feedback_ready(BaseState())


def test_joint_enable_selection_rejects_unknown_names():
    with pytest.raises(ValueError, match="not_a_joint"):
        joint_selection(("hip", "knee"), ["not_a_joint"])
    assert joint_selection(("hip", "knee"), ["knee"]) == [False, True]
    assert joint_selection(("hip", "knee"), None) is None
