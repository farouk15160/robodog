"""The measurement boundary must observe physical transmission and saturation."""
from pathlib import Path

import pytest
import yaml

pytest.importorskip("mujoco")

from robodog_hardware.physics_metrics import PhysicsMetrics
from robodog_hardware.registry import create_backend
from robodog_hardware.transmission import backend_config
from robodog_hardware.types import JOINT_NAMES, JointCommand, ControlMode


@pytest.mark.parametrize("delay", [0, 2])
def test_command_delay_counts_physics_steps_not_control_writes(delay):
    src = Path(__file__).parents[2]
    spec = yaml.safe_load((src / "robodog_description/config/robstride06.yaml").read_text())
    backend = create_backend("mujoco", {
        **backend_config(spec, JOINT_NAMES), "command_delay_steps": delay,
        "model_path": str(src / "robodog_sim/models/robodog_scene.xml"),
    })
    backend.configure()
    try:
        backend.enable()
        command = JointCommand()
        command.mode[:] = int(ControlMode.TORQUE)
        command.effort[0] = 2
        # Repeated host writes must not consume physical transport delay.
        for _ in range(5 if delay else 1):
            backend.write(command)
        for _ in range(delay):
            backend.step(.0005)
            assert backend.read().effort[0] == 0
        backend.step(.0005)
        assert backend.read().effort[0] > 1.9
        backend.reset()
        for _ in range(delay + 1):
            backend.step(.0005)
            assert backend.read().effort[0] == 0
    finally:
        backend.shutdown()


def test_nonunit_mujoco_gear_and_ctrlrange_are_measured_as_applied_joint_torque():
    src = Path(__file__).parents[2]
    spec = yaml.safe_load((src / "robodog_description/config/robstride06.yaml").read_text())
    backend = create_backend("mujoco", {
        **backend_config(spec, JOINT_NAMES), "command_delay_steps": 0,
        "model_path": str(src / "robodog_sim/models/robodog_scene.xml"),
    })
    backend.configure()
    try:
        first = backend.aid[0]
        backend.m.actuator_gear[first, 0] = 2
        backend.m.actuator_ctrlrange[first] = [-1, 1]
        backend.enable()
        observer = PhysicsMetrics(JOINT_NAMES, backend.ratio, backend.efficiency)
        backend.set_physics_observer(observer.update)
        command = JointCommand()
        command.mode[:] = int(ControlMode.TORQUE)
        command.effort[0] = 100
        backend.write(command)
        backend.step(.0005)
        # actuator_force is1, qfrc_actuator is2 because gear=2; ctrl requests much more.
        assert backend.d.actuator_force[first] == pytest.approx(1)
        assert backend.read().effort[0] == pytest.approx(2)
        row = observer.summary()[0]
        assert row["joint_peak_nm"] == pytest.approx(2)
        assert row["clipping"]["peak"]["steps"] == 1
        assert row["clipping"]["physics"]["steps"] == 1
        assert observer.samples == 1
    finally:
        backend.shutdown()
