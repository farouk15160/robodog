"""Report rendering keeps the evidence needed to locate torque spikes."""
from argparse import Namespace

from torque_report import fault_names, flat_row, markdown, requested_cases


def _case():
    thresholds = {
        str(value): {
            "total_time_s": duration,
            "longest_time_s": duration,
            "peak_abs_motor_speed_rad_s": 2.0,
            "rms_motor_speed_rad_s": 1.0,
        }
        for value, duration in ((8, .4), (11, .2), (36, 0.0))
    }
    clipping = {
        kind: {"events": 0, "steps": 0, "duration_s": 0.0}
        for kind in ("peak", "speed", "physics")
    }
    joint = {
        "name": "FL_kfe_joint",
        "joint_peak_nm": 28.5,
        "joint_rms_nm": 9.5,
        "motor_peak_nm": 15.0,
        "motor_peak_signed_nm": -15.0,
        "motor_peak_time_s": 12.25,
        "motor_peak_world_position_m": [11.2, -.3, .31],
        "motor_peak_travel_m": 12.1,
        "motor_speed_at_peak_rad_s": -3.2,
        "motor_peak_speed_rad_s": 10.0,
        "motor_rms_nm": 5.0,
        "full_run_joint_rms_nm": 10.0,
        "full_run_motor_rms_nm": 5.2,
        "duration_s": 100.0,
        "rms_duration_s": 95.0,
        "continuous_utilisation": .625,
        "peak_utilisation": 15 / 36,
        "fraction_above_continuous": .004,
        "estimated_rms_current_a": 4.5,
        "estimated_final_temperature_c": 32.0,
        "estimated_steady_temperature_c": 40.0,
        "safety_torque_clipped_cycles": 0,
        "safety_torque_clip_events": 0,
        "safety_torque_clipped_duration_s": 0.0,
        "peak_abs_motor_speed_when_above_11": 2.0,
        "threshold_metrics": thresholds,
        "clipping": clipping,
    }
    return {
        "gait": "trot", "commanded_vx_m_s": 1.0, "commanded_wz_rad_s": 0.0,
        "actual_vx_m_s": .98, "max_tilt_deg": 5.0, "min_height_m": .3,
        "safety_clamped_cycle_fraction": 0.0, "stable": True, "tracks_velocity": True,
        "duration_s": 100.0, "warmup_s": 5.0, "travel_m": 98.0,
        "lateral_drift_m": .1, "safety_latched": False, "safety_clamp_events": 0,
        "fault_flags": 0, "completed": True, "stability_failure_reasons": [],
        "operating_point_pass": True, "margin_failure_reasons": [],
        "joints": [joint],
    }


def test_markdown_and_csv_row_include_spike_location_and_all_requested_thresholds():
    case = _case()
    report = {"mass_kg": 19.719, "supply_voltage_v": 44.4, "cases": [case]}

    rendered = markdown(report)
    row = flat_row(case, case["joints"][0])

    assert "12.2500 s / 12.100 m" in rendered
    assert "[11.200, -0.300, 0.310]" in rendered
    assert ">8 / >11 / >36 Nm total s" in rendered
    assert row["motor_peak_world_x_m"] == 11.2
    assert row["motor_peak_world_y_m"] == -.3
    assert row["motor_peak_world_z_m"] == .31
    assert row["above_36nm_total_time_s"] == 0.0
    assert row["operating_point_pass"] is True


def test_markdown_marks_a_stable_but_limited_case_as_margin_failure():
    case = _case()
    case.update(
        safety_clamped_cycle_fraction=.306,
        operating_point_pass=False,
        margin_failure_reasons=["safety limiting exceeded 1.0% of control cycles"],
    )
    rendered = markdown({"mass_kg": 19.719, "supply_voltage_v": 44.4,
                         "cases": [case]})
    assert "Motor-margin verdict: FAIL" in rendered
    assert "True / True / False" in rendered


def test_fault_mask_is_named_in_human_report():
    case = _case()
    case["fault_flags"] = 6

    rendered = markdown({"mass_kg": 28.0, "supply_voltage_v": 44.4,
                         "cases": [case]})

    assert fault_names(6) == ["VELOCITY_LIMIT", "TORQUE_LIMIT"]
    assert "fault mask 6 (VELOCITY_LIMIT, TORQUE_LIMIT)" in rendered


def test_walk_and_trot_can_share_one_100_second_report():
    args = Namespace(
        gait=None, gaits=["walk", "trot"], speeds=None, velocity=1.0,
        vy=0.0, wz=0.0, study=False,
    )
    assert requested_cases(args) == [
        ("walk", 1.0, 0.0, 0.0),
        ("trot", 1.0, 0.0, 0.0),
    ]


def test_markdown_compares_matching_tuned_case_with_hashed_baseline():
    current = _case()
    current["lateral_drift_m"] = .2
    report = {
        "study_label": "tuned_heading_hold",
        "mass_kg": 19.719,
        "supply_voltage_v": 44.4,
        "cases": [current],
        "baseline_reference": {
            "path": "docs/rs06_1ms_100s_baseline.json",
            "sha256": "abc123",
            "study_label": "baseline_before_heading_hold",
            "cases": [{
                "gait": "trot", "commanded_vx_m_s": 1.0,
                "commanded_wz_rad_s": 0.0, "actual_vx_m_s": .98,
                "lateral_drift_m": 24.4, "safety_clamped_cycle_fraction": .306,
                "worst_motor_rms_nm": 5.36, "peak_motor_nm": 28.15,
            }],
        },
    }

    rendered = markdown(report)

    assert "Baseline comparison" in rendered
    assert "`abc123`" in rendered
    assert "| Lateral drift m | 24.400 | 0.200 | -24.200 |" in rendered
