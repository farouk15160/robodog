"""Offline commissioning CLI checks; no CAN sockets or motors are used."""
from pathlib import Path

import numpy as np
import pytest

from robodog_hardware.tools import calibrate_joint
from robodog_hardware.types import JointState


class CalibrationBackend:
    direction = np.ones(12)
    ratio = np.array([1, 1, 2] * 4)
    offset = np.zeros(12)

    def __init__(self):
        self.enabled = []
        self.read_count = 0
        self.disabled = False

    def configure(self):
        pass

    def enable(self, mask, **kwargs):
        self.enabled.append((mask, kwargs))

    def read(self):
        state = JointState()
        state.position[:] = [0, .02, -.8][min(self.read_count, 2)]
        self.read_count += 1
        return state

    def write(self, cmd):
        pass

    def disable(self):
        self.disabled = True

    def shutdown(self):
        pass


@pytest.mark.parametrize("answer,expected", [("y", 0), ("", 1), ("maybe", 1)])
def test_only_explicit_direction_confirmation_can_produce_calibrated_mapping(monkeypatch, capsys, answer, expected):
    backend = CalibrationBackend()
    monkeypatch.setattr(calibrate_joint, "create_backend", lambda *_: backend)
    answers = iter(["y", answer, "y", "y"])
    monkeypatch.setattr(calibrate_joint, "_ask", lambda _: next(answers))
    base = Path(__file__).parents[2]
    args = ["--joint", "FL_kfe_joint", "--lower-limit", "-1",
            "--duration", ".001", "--config", str(base / "robodog_hardware/config/robstride_bus.yaml"),
            "--actuator-config", str(base / "robodog_description/config/robstride06.yaml")]
    assert calibrate_joint.main(args) == expected
    output = capsys.readouterr().out
    assert backend.enabled[0][1] == {"commissioning_joint": "FL_kfe_joint"}
    assert backend.disabled
    if expected == 0:
        assert "calibrated: true" in output
        assert "offset_rad: 0.4" in output
    else:
        assert "calibrated: true" not in output
