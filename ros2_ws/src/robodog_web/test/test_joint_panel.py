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


def test_remote_control_panel_has_deadman_controls_and_live_feedback():
    package = Path(__file__).resolve().parents[1]
    subprocess.run(
        ["node", str(package / "test" / "remote_control_panel.cjs"),
         str(package / "www" / "js" / "panels.js")],
        check=True, capture_output=True, text=True,
    )


def test_page_navigation_can_hide_inactive_panels():
    package = Path(__file__).resolve().parents[1]
    css = (package / "www" / "css" / "app.css").read_text()
    app = (package / "www" / "js" / "app.js").read_text()
    assert ".panel[hidden]" in css
    assert "display: none" in css.split(".panel[hidden]", 1)[1].split("}", 1)[0]
    assert 'location.pathname === "/remote"' in app


def test_remote_control_has_phone_and_landscape_layouts():
    package = Path(__file__).resolve().parents[1]
    css = (package / "www" / "css" / "app.css").read_text()
    panels = (package / "www" / "js" / "panels.js").read_text()
    assert "grid-template-areas" in css
    assert "@media (max-width: 520px)" in css
    assert "orientation: landscape" in css
    assert "env(safe-area-inset-bottom)" in css
    assert "remote-feedback" in panels
    landscape = css.split("orientation: landscape", 1)[1]
    assert "grid-template-columns: minmax(0, .9fr) minmax(0, 1.1fr)" in landscape
    assert ".remote-speed { grid-template-columns: auto 1fr; }" in landscape
    assert ".remote-speed input { grid-column: 1 / -1; grid-row: 2; }" in landscape
