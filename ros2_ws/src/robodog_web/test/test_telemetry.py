"""The GUI metadata contract uses joint-side ratings and motor-output units."""
import json
from pathlib import Path

import pytest
import yaml

from robodog_web.telemetry import joint_actuator_info


def test_rs06_gui_metadata_exposes_all_joint_ratings_and_knee_transmission():
    source = Path(__file__).resolve().parents[2]
    spec = yaml.safe_load(
        (source / "robodog_description/config/robstride06.yaml").read_text())
    names = [f"{leg}_{axis}_joint" for leg in ("FL", "FR", "RL", "RR")
             for axis in ("haa", "hfe", "kfe")]
    # Round-trip the actual JSON contract, including numpy -> builtin conversion.
    info = json.loads(json.dumps(joint_actuator_info(spec, names)))
    assert list(info) == names
    assert info["FL_hfe_joint"]["continuous_torque_nm"] == 8.0
    assert "stall" in info["FL_hfe_joint"]["continuous_limit_basis"].lower()
    assert info["FL_hfe_joint"]["peak_torque_nm"] == 36.0
    assert info["RR_kfe_joint"]["ratio"] == 2.0
    assert info["RR_kfe_joint"]["torque_gain"] == 1.9
    assert info["RR_kfe_joint"]["continuous_torque_nm"] == pytest.approx(15.2)
    assert info["RR_kfe_joint"]["peak_torque_nm"] == pytest.approx(68.4)
