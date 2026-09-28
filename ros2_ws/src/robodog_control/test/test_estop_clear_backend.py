"""Operator clear must clear backend-protective latches before safety clears."""
from types import SimpleNamespace
import threading
import time

import numpy as np

from robodog_control.control_node import RoboDogControlNode
from robodog_control.safety import SafetyLimits, SafetyMonitor
from robodog_hardware.types import Fault, JointState, NJ


NAMES = [f"J{i}" for i in range(NJ)]


class FakeBackend:
    is_simulation = False

    def __init__(self):
        self.disable_calls = 0
        self.clear_calls = 0
        self.fault_latched = True

    def disable(self, mask=None):
        self.disable_calls += 1

    def clear_faults(self):
        self.clear_calls += 1
        self.fault_latched = False

    def read(self):
        state = JointState()
        state.stamp = time.monotonic()
        if self.fault_latched:
            state.faults[0] = int(Fault.COMMUNICATION)
        return state


def node_shell(backend):
    node = object.__new__(RoboDogControlNode)
    node.backend = backend
    node.safety = SafetyMonitor(SafetyLimits(
        position_lower=np.full(NJ, -1.0),
        position_upper=np.full(NJ, 1.0)), NAMES, rate_hz=400.0)
    node.safety.engage_estop("fault")
    node._last_state = backend.read()
    node._lock = threading.RLock()
    node.controller = "gait"
    node.enabled = True
    return node


def response():
    return SimpleNamespace(success=False, latched=True, message="")


def test_clear_estop_sends_backend_clear_and_waits_for_clean_feedback():
    backend = FakeBackend()
    node = node_shell(backend)

    first = node._srv_estop(SimpleNamespace(engage=False), response())

    assert not first.success
    assert first.latched
    assert "awaiting fresh motor feedback" in first.message
    assert backend.clear_calls == 1
    assert backend.disable_calls == 1
    assert node.controller == "idle"
    assert not node.enabled

    node._last_state = backend.read()
    res = node._srv_estop(SimpleNamespace(engage=False), response())
    assert res.success
    assert not res.latched
    assert backend.clear_calls == 1
    assert backend.disable_calls == 2


def test_clear_estop_clear_transmit_failure_stays_latched_and_idle():
    backend = FakeBackend()

    def fail_clear():
        backend.clear_calls += 1
        raise RuntimeError("CAN transmit failed")

    backend.clear_faults = fail_clear
    node = node_shell(backend)

    res = node._srv_estop(SimpleNamespace(engage=False), response())

    assert not res.success
    assert res.latched
    assert "fault clear failed" in res.message
    assert node.safety.latched
    assert backend.clear_calls == 1
    assert backend.disable_calls == 1
    assert node.controller == "idle"


def test_clear_estop_disable_transmit_failure_stays_latched_and_idle():
    backend = FakeBackend()

    def fail_disable(mask=None):
        backend.disable_calls += 1
        raise RuntimeError("CAN stop failed")

    backend.disable = fail_disable
    node = node_shell(backend)

    res = node._srv_estop(SimpleNamespace(engage=False), response())

    assert not res.success
    assert res.latched
    assert "disable failed" in res.message
    assert node.safety.latched
    assert backend.disable_calls == 1
    assert backend.clear_calls == 0
    assert node.controller == "idle"
    assert node.enabled


def test_clear_estop_does_not_clear_safety_if_backend_fault_remains():
    backend = FakeBackend()
    backend.clear_faults = lambda: setattr(backend, "clear_calls", backend.clear_calls + 1)
    node = node_shell(backend)

    res = node._srv_estop(SimpleNamespace(engage=False), response())

    assert not res.success
    assert res.latched
    assert "awaiting fresh motor feedback" in res.message
    assert node.safety.latched
    assert backend.clear_calls == 1
    assert backend.disable_calls == 1
    assert node.controller == "idle"
    assert not node.enabled

    node._last_state = backend.read()
    second = node._srv_estop(SimpleNamespace(engage=False), response())
    assert not second.success
    assert "cannot clear, active faults" in second.message
    assert backend.clear_calls == 1
    assert backend.disable_calls == 2
