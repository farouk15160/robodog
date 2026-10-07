"""Motor sizing reports must account for the external knee transmission."""
import numpy as np
import pytest

from robodog_sim.evaluation import joint_metrics, operating_point_assessment


def test_report_distinguishes_joint_load_from_motor_load():
    # A 2:1, 95%-efficient belt delivers 19 Nm from 10 Nm motor output.
    result = joint_metrics(
        names=["hip", "knee"],
        torque=np.array([[10.0, 19.0], [-10.0, -19.0]]),
        velocity=np.array([[2.0, 2.0], [-2.0, -2.0]]),
        temperature=np.array([[20.0, 20.0], [21.0, 22.0]]),
        ratio=np.array([1.0, 2.0]), efficiency=np.array([1.0, 0.95]),
        continuous=11.0, peak=36.0, kt=2.0, resistance=0.1,
        ambient=20.0, thermal_resistance=2.0,
    )
    assert result[1]["joint_rms_nm"] == pytest.approx(19.0)
    assert result[1]["motor_rms_nm"] == pytest.approx(10.0)
    assert result[1]["motor_peak_speed_rad_s"] == pytest.approx(4.0)
    assert result[1]["continuous_utilisation"] == pytest.approx(10 / 11)
    assert result[1]["estimated_steady_temperature_c"] == pytest.approx(35.0)
    assert result[1]["estimated_final_temperature_c"] == pytest.approx(22.0)
    assert result[0]["motor_rms_nm"] == pytest.approx(10.0)


def test_report_rejects_empty_samples_instead_of_claiming_feasibility():
    with pytest.raises(ValueError, match="samples"):
        joint_metrics(
            names=["hip"], torque=np.empty((0, 1)), velocity=np.empty((0, 1)),
            temperature=np.empty((0, 1)), ratio=np.ones(1), efficiency=np.ones(1),
            continuous=11, peak=36, kt=2, resistance=0.1,
            ambient=20, thermal_resistance=2,
        )


def test_active_safety_limiting_rejects_comfortable_margin_verdict():
    passed, reasons = operating_point_assessment(
        completed=True, stable=True, tracks_velocity=True,
        within_continuous_rating=True, safety_clamped_cycle_fraction=.306,
        max_clamped_cycle_fraction=.01,
    )
    assert not passed
    assert reasons == ["safety limiting exceeded 1.0% of control cycles"]


@pytest.fixture
def simulation_case():
    pytest.importorskip("mujoco")
    import yaml
    from pathlib import Path

    src = Path(__file__).parents[2]
    params = yaml.safe_load((src / "robodog_description/config/robot_parameters.yaml").read_text())
    actuator = yaml.safe_load((src / "robodog_description/config/robstride06.yaml").read_text())
    gaits = yaml.safe_load((src / "robodog_control/config/gaits.yaml").read_text())["gaits"]
    controls = yaml.safe_load((src / "robodog_control/config/control.yaml").read_text())["/**"]["ros__parameters"]
    return params, actuator, gaits, controls, src / "robodog_sim/models/robodog_scene.xml"


def test_run_case_collects_applied_physics_rate_torques_during_turn(simulation_case):
    from robodog_sim.evaluation import run_case
    result = run_case(*simulation_case,
                      gait="walk", vx=.1, vy=.01, wz=.3, seconds=.02, warmup=.005)
    assert result["effort_source"] == "MuJoCo qfrc_actuator at joint DOFs"
    assert result["physics_samples"] == 40
    assert result["physics_timestep_s"] == pytest.approx(.0005)
    assert result["commanded_wz_rad_s"] == .3
    assert result["duration_s"] == pytest.approx(.02)
    assert len(result["joints"]) == 12
    for row in result["joints"]:
        assert row["threshold_metrics"]["11"]["longest_time_s"] <= .02
        assert "36" in row["threshold_metrics"]
        assert len(row["motor_peak_world_position_m"]) == 3
        assert row["motor_peak_travel_m"] >= 0
        assert "safety_torque_clip_events" in row
        assert "full_run_motor_rms_nm" in row


def test_numerical_failure_cannot_be_reported_as_success(simulation_case, monkeypatch):
    import json
    from robodog_sim.evaluation import run_case
    from robodog_hardware.backends.mujoco_backend import MujocoBackend

    def failed_step(self, dt):
        raise FloatingPointError("nonfinite physics state")

    monkeypatch.setattr(MujocoBackend, "step", failed_step)
    result = run_case(*simulation_case, seconds=.02, warmup=.005)
    assert not result["completed"] and not result["stable"]
    assert not result["tracks_velocity"] and not result["within_continuous_rating"]
    assert result["failure_reason"] == "nonfinite physics state"
    assert result["joints"] == []  # no fabricated zero-torque sizing result
    json.dumps(result, allow_nan=False)
