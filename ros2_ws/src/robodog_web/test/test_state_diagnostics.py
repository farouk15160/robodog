"""ROS message -> browser serialization, without starting a ROS graph."""
import json
from types import SimpleNamespace

import pytest
from robodog_msgs.msg import JointTelemetry, RobotState
from robodog_web.telemetry import RollingTelemetry
from robodog_web.web_server import WebBridgeNode, _robot_info


def bridge():
    info = _robot_info()
    return SimpleNamespace(_rolling=RollingTelemetry(info["joint_actuators"]),
                           state_seq=0, camera_meta={}, _track_events=lambda *_: None)


def message(t=1.0, backend="mujoco"):
    m = RobotState()
    m.simulation.active = backend != "none"
    m.simulation.backend = backend
    m.simulation.sim_time_s = t
    m.header.stamp.sec = 100
    m.header.stamp.nanosec = int(t * 1e8)
    for name in _robot_info()["joints"]:
        m.joints.append(JointTelemetry(name=name, effort=19.0, velocity=2.0,
                                       temperature=30.0, position_command=0.1))
    return m


def test_state_uses_physics_applied_effort_and_preserves_existing_contract():
    node = bridge()
    WebBridgeNode._on_state(node, message(1.0))
    WebBridgeNode._on_state(node, message(1.1))
    payload = json.loads(json.dumps(node.state, allow_nan=False))
    knee = payload["joints"][2]
    assert knee["name"] == "FL_kfe_joint"
    assert knee["eff"] == 19
    assert knee["eff_cmd"] == 0
    assert knee["stats"]["motor_torque_rms_nm"] == pytest.approx(10)
    assert payload["diagnostics"]["covered_s"] == pytest.approx(0.1)
    assert payload["diagnostics"]["clock"] == "mujoco:sim_time"
    assert "applied" in payload["diagnostics"]["torque_source"]
    assert "estimated" in payload["diagnostics"]["current_source"]
    assert payload["diagnostics"]["received_seq"] == 2
    assert payload["diagnostics"]["source_time_s"] == 1.1


@pytest.mark.parametrize("backend", ["none", "kinematic"])
def test_non_mujoco_uses_ros_header_clock(backend):
    node = bridge()
    WebBridgeNode._on_state(node, message(1.0, backend))
    WebBridgeNode._on_state(node, message(1.1, backend))
    assert node.state["diagnostics"]["covered_s"] == pytest.approx(0.01)
    assert node.state["diagnostics"]["clock"] == f"{backend}:ros_header"


def test_nonfinite_values_are_json_null():
    node = bridge()
    m = message()
    m.joints[0].effort = float("nan")
    m.base_height_m = float("inf")
    WebBridgeNode._on_state(node, m)
    assert node.state["joints"][0]["eff"] is None
    assert node.state["base"]["height"] is None
    assert node.state["diagnostics"]["dropped_samples"] == 1
    json.dumps(node.state, allow_nan=False)


def test_info_contains_sourced_rating_and_model_assumptions():
    info = _robot_info()
    assert info["performance"]["rated_torque_nm"] == 11
    assert info["operational"]["continuous_torque_nm"] == 8
    assert not info["thermal"]["calibrated"]
    assert info["electrical"]["supply_voltage_v"] == 44.4
    assert info["mass_budget"]["total_kg"] == info["mass_kg"]
    assert info["stats_window_s"] == 20
