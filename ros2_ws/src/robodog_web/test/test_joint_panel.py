"""Exercise the browser's public panel with twelve live telemetry samples."""
from pathlib import Path
import subprocess


def test_joint_panel_displays_joint_and_motor_load_with_temperature_provenance():
    package = Path(__file__).resolve().parents[1]
    subprocess.run(
        ["node", str(package / "test" / "joint_panel.cjs"),
         str(package / "www" / "js" / "panels.js")],
        check=True,
        capture_output=True,
        text=True,
    )


def test_gait_panel_speed_range_follows_the_live_backend():
    package = Path(__file__).resolve().parents[1]
    subprocess.run(
        ["node", str(package / "test" / "gait_panel.cjs"),
         str(package / "www" / "js" / "panels.js")],
        check=True, capture_output=True, text=True,
    )


def test_diagnostics_panels_distinguish_live_commands_from_sampled_statistics():
    package = Path(__file__).resolve().parents[1]
    subprocess.run(
        ["node", str(package / "test" / "diagnostics_panel.cjs"),
         str(package / "www" / "js" / "panels.js")],
        check=True, capture_output=True, text=True,
    )
