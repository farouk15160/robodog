"""Physics-rate load measurement keeps brief applied loads and threshold runs."""
import numpy as np
import pytest

from robodog_hardware.physics_metrics import PhysicsMetrics, body_planar_velocity


def test_world_velocity_rotates_into_command_frame_during_yaw_turn():
    assert body_planar_velocity([0, 0, np.sqrt(.5), np.sqrt(.5)], [0, 1, 0]) == pytest.approx([1, 0])


def test_substep_peak_survives_between_control_samples_and_warmup():
    metrics = PhysicsMetrics(["hip", "knee"], ratio=[1, 2], efficiency=[1, .95], warmup_s=.001)
    for torque in ([0, 0], [12, 22.8], [0, 0], [0, 0], [0, 0]):
        metrics.update(torque, [3, -2], .0005)
    rows = metrics.summary()
    assert rows[1]["motor_peak_nm"] == pytest.approx(12)
    assert rows[1]["joint_peak_nm"] == pytest.approx(22.8)
    assert rows[1]["motor_speed_at_peak_rad_s"] == -4
    assert rows[1]["motor_peak_time_s"] == pytest.approx(.0005)
    assert rows[1]["motor_rms_nm"] == 0  # warmup is excluded only from the selected RMS
    assert rows[1]["full_run_motor_rms_nm"] == pytest.approx(12 / np.sqrt(5))
    assert rows[1]["threshold_metrics"]["11"]["total_time_s"] == pytest.approx(.0005)
    assert rows[1]["peak_abs_motor_speed_when_above_11"] == 4


def test_peak_records_robot_world_position_and_distance_travelled():
    metrics = PhysicsMetrics(["hip", "knee"], ratio=[1, 2], efficiency=[1, .95])
    samples = [
        ([2, 3.8], [10.0, -2.0, .32]),
        ([4, 7.6], [10.3, -1.9, .31]),
        ([3, 5.7], [10.6, -1.9, .30]),
    ]
    for torque, position in samples:
        metrics.update(torque, [1, -1], .001, world_position=position)

    hip, knee = metrics.summary()
    assert hip["motor_peak_world_position_m"] == pytest.approx([10.3, -1.9, .31])
    assert knee["motor_peak_world_position_m"] == pytest.approx([10.3, -1.9, .31])
    assert hip["motor_peak_travel_m"] == pytest.approx(np.hypot(.3, .1))
    assert knee["motor_peak_travel_m"] == pytest.approx(np.hypot(.3, .1))


def test_total_duration_longest_run_and_clipping_events_are_distinct():
    metrics = PhysicsMetrics(["joint"], warmup_s=0)
    for torque, clipped in [(12, True), (12, True), (7, False), (9, True)]:
        metrics.update([torque], [2], .01, peak_clipped=[clipped])
    row = metrics.summary()[0]
    assert row["threshold_metrics"]["8"]["total_time_s"] == pytest.approx(.03)
    assert row["threshold_metrics"]["8"]["longest_time_s"] == pytest.approx(.02)
    assert row["threshold_metrics"]["11"]["total_time_s"] == pytest.approx(.02)
    assert row["clipping"]["peak"] == pytest.approx({"events": 2, "steps": 3, "duration_s": .03})


def test_rms_integrates_time_including_partial_warmup_step():
    metrics = PhysicsMetrics(["joint"], warmup_s=.005)
    metrics.update([10], [0], .01)
    metrics.update([0], [0], .01)
    assert metrics.summary()[0]["motor_rms_nm"] == pytest.approx(np.sqrt(100 / 3))


def test_nonfinite_or_empty_measurements_cannot_report_success():
    metrics = PhysicsMetrics(["joint"])
    with pytest.raises(ValueError, match="samples"):
        metrics.summary()
    with pytest.raises(ValueError, match="finite"):
        metrics.update([float("nan")], [0], .001)
    with pytest.raises(ValueError, match="positive"):
        metrics.update([0], [0], 0)
