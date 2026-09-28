"""Rolling telemetry is a sampled observation, not a physics torque logger."""
import json
import math

import pytest

from robodog_web.telemetry import RollingTelemetry, finite_json


META = {"knee": {"ratio": 2.0, "torque_gain": 1.9,
                  "motor_continuous_torque_nm": 8.0,
                  "motor_vendor_rotating_torque_nm": 11.0}}


def reading(eff=19.0, vel=3.0, temp=25.0, err=0.1):
    return [{"name": "knee", "eff": eff, "vel": vel, "temp": temp, "err": err}]


def test_signed_unequal_time_rms_and_knee_output_conversion():
    stats = RollingTelemetry(META)
    stats.update(0.0, "sim_time", reading(19.0))
    stats.update(0.1, "sim_time", reading(-38.0, -4.0, 27.0, 0.2))
    joints, diagnostics = stats.update(0.3, "sim_time", reading(0.0))
    knee = joints[0]["stats"]
    assert knee["joint_torque_rms_nm"] == pytest.approx(math.sqrt((19**2 + 2*38**2)/3))
    assert knee["motor_torque_rms_nm"] == pytest.approx(math.sqrt(300))
    assert knee["motor_torque_peak_nm"] == pytest.approx(20)
    assert knee["joint_torque_peak_nm"] == 38
    assert knee["motor_speed_at_peak_rad_s"] == -8
    assert knee["above_continuous_s"] == pytest.approx(0.3)
    assert knee["above_vendor_rotating_s"] == pytest.approx(0.2)
    assert knee["tracking_error_rms_rad"] == pytest.approx(math.sqrt(0.03))
    assert knee["temperature_max_c"] == 27
    assert knee["mechanical_power_mean_w"] == pytest.approx((57 + 2*152)/3)
    assert diagnostics["covered_s"] == pytest.approx(0.3)
    assert diagnostics["samples"] == 3
    assert joints[0]["motor_eff_nm"] == 0
    assert joints[0]["motor_vel_rad_s"] == 6


def test_one_sample_has_peak_but_no_fabricated_rms():
    joints, diagnostics = RollingTelemetry(META).update(5.0, "sim_time", reading())
    assert joints[0]["stats"]["joint_torque_peak_nm"] == 19
    assert joints[0]["stats"]["joint_torque_rms_nm"] is None
    assert diagnostics["covered_s"] == 0
    assert diagnostics["sample_rate_hz"] is None


def test_window_clips_partial_intervals_and_expires_old_peaks():
    stats = RollingTelemetry(META, window_s=0.15)
    stats.update(0.0, "sim_time", reading(38))
    stats.update(0.1, "sim_time", reading(19))
    joints, diag = stats.update(0.2, "sim_time", reading(0))
    assert diag["covered_s"] == pytest.approx(0.15)
    assert joints[0]["stats"]["motor_torque_rms_nm"] == pytest.approx(math.sqrt(200))
    joints, _ = stats.update(0.3, "sim_time", reading(0))
    assert joints[0]["stats"]["motor_torque_peak_nm"] == 10


def test_duplicate_and_out_of_order_samples_do_not_change_history():
    stats = RollingTelemetry(META)
    stats.update(5, "sim_time", reading())
    stats.update(5.1, "sim_time", reading())
    _, diag = stats.update(5.1, "sim_time", reading(999))
    assert diag["dropped_samples"] == 1
    assert diag["samples"] == 2
    _, diag = stats.update(5.05, "sim_time", reading(999))
    assert diag["dropped_samples"] == 2
    joints, _ = stats.update(5.2, "sim_time", reading())
    assert joints[0]["stats"]["joint_torque_peak_nm"] == 19


def test_large_clock_reset_and_backend_switch_restart_window():
    stats = RollingTelemetry(META)
    stats.update(5.0, "sim_time", reading())
    stats.update(5.1, "sim_time", reading())
    _, diag = stats.update(0.0, "sim_time", reading())
    assert diag["resets"] == 1
    assert diag["covered_s"] == 0
    _, diag = stats.update(99.0, "ros_header", reading())
    assert diag["resets"] == 2
    assert diag["clock"] == "ros_header"


def test_gaps_and_nonfinite_frames_never_create_exposure_time():
    stats = RollingTelemetry(META)
    stats.update(0, "sim_time", reading())
    _, diag = stats.update(2, "sim_time", reading())
    assert diag["gaps"] == 1
    assert diag["covered_s"] == 0
    joints, diag = stats.update(2.1, "sim_time", reading(float("nan")))
    assert diag["dropped_samples"] == 1
    assert joints[0]["stats"]["motor_torque_rms_nm"] is None
    _, diag = stats.update(2.2, "sim_time", reading())
    assert diag["covered_s"] == 0
    _, diag = stats.update(float("nan"), "sim_time", reading())
    assert diag["dropped_samples"] == 2
    json.dumps(finite_json({"x": math.inf, "nested": [math.nan]}), allow_nan=False)


def test_memory_is_bounded_even_with_high_frequency_input():
    stats = RollingTelemetry(META, max_samples=8)
    for i in range(100):
        joints, diag = stats.update(i * 0.001, "sim_time", reading())
    assert diag["samples"] == 8
    assert diag["covered_s"] == pytest.approx(0.007)
    assert joints[0]["stats"]["window_s"] == 20


def test_missing_joint_cannot_leak_stale_statistics():
    stats = RollingTelemetry(META)
    stats.update(0, "sim_time", reading())
    _, diag = stats.update(0.1, "sim_time", [])
    assert diag["dropped_samples"] == 1
    _, diag = stats.update(0.2, "sim_time", reading())
    assert diag["covered_s"] == 0


@pytest.mark.parametrize("options", [{"window_s": 0}, {"max_gap_s": 0}, {"max_samples": 1}])
def test_invalid_window_configuration_is_rejected(options):
    with pytest.raises(ValueError):
        RollingTelemetry(META, **options)


def test_unknown_joint_has_no_misleading_conversion_or_statistics():
    stats = RollingTelemetry(META)
    joints, diag = stats.update(0, "sim_time", [{**reading()[0], "name": "unknown"}])
    assert joints[0]["stats"] is None
    assert "motor_eff_nm" not in joints[0]
    assert diag["dropped_samples"] == 1
