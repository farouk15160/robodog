"""Drive a running MuJoCo ROS GUI and inspect real rendered joint telemetry.

Usage: python3 tools/gui_smoke.py [http://127.0.0.1:8080] [/tmp/robodog_gui_smoke]
Requires Python Playwright and Google Chrome at /usr/bin/google-chrome; start the
ROS stack with backend:=mujoco first. Nothing is installed automatically.

Refuses hardware: verifies the live backend before sending commands. Exercises
Disable joints -> stand pose -> stand gait -> walk at 0.12 m/s, checks command
acks and all twelve torque/temperature rows, saves PNGs and JSON at the output
prefix, then requests zero velocity and stand. This is a short UI integration
check, not a thermal endurance or motor sizing validation.
"""
import json
import math
from pathlib import Path
import sys
import time

from playwright.sync_api import sync_playwright


url = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8080"
output = Path(sys.argv[2] if len(sys.argv) > 2 else "/tmp/robodog_gui_smoke")
output.parent.mkdir(parents=True, exist_ok=True)
frames = []
acks = []
errors = []


def received(payload):
    message = json.loads(payload)
    if message.get("type") == "state":
        frames.append(message["data"])
    elif message.get("type") == "ack":
        acks.append(message)


with sync_playwright() as playwright:
    browser = playwright.chromium.launch(
        executable_path="/usr/bin/google-chrome", headless=True,
        args=["--no-sandbox", "--disable-dev-shm-usage"],
    )
    page = browser.new_page(viewport={"width": 1920, "height": 1300})
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.on("console", lambda message: errors.append(message.text)
            if message.type == "error" else None)
    page.on("dialog", lambda dialog: dialog.accept())
    page.on("websocket", lambda socket: socket.on("framereceived", received))
    page.goto(url, wait_until="domcontentloaded")
    joint_panel = page.locator("section.panel").filter(
        has=page.locator(".ptitle", has_text="Joint Torque & Temperature"))
    stats_panel = page.locator("section.panel").filter(
        has=page.locator(".ptitle", has_text="Joint RMS, Peaks & Exposure"))
    robot_panel = page.locator("section.panel").filter(
        has=page.locator(".ptitle", has_text="Robot Model & Motor Ratings"))
    joint_panel.locator("tbody tr").last.wait_for(timeout=30000)
    page.wait_for_function(
        "document.querySelector('#top-backend').textContent === 'mujoco'", timeout=30000)
    assert "ROBSTRIDE06" in page.locator("#brand-sub").inner_text()
    assert joint_panel.locator("tbody tr").count() == 12
    assert stats_panel.locator(".joint-statistics tbody tr").count() == 12
    assert "19.719 kg" in robot_panel.inner_text()
    assert "384 × 220 × 120 mm" in robot_panel.inner_text()
    assert float(page.locator("#vx").get_attribute("max")) == 2.0
    gait_panel = page.locator("section.panel").filter(
        has=page.locator(".ptitle", has_text="Gait & Velocity"))
    pose_panel = page.locator("section.panel").filter(
        has=page.locator(".ptitle", has_text="Poses"))
    # Regression: pose service re-enables disabled joints while holding the
    # controller lock. Verify the real service responds, rather than deadlocks.
    page.get_by_role("button", name="Disable joints", exact=True).click()
    pose_panel.get_by_role("button", name="stand", exact=True).click()
    deadline = time.monotonic() + 6
    while not any(ack.get("action") == "pose" for ack in acks) and time.monotonic() < deadline:
        page.wait_for_timeout(100)
    assert any(ack.get("action") == "pose" and ack["ok"] for ack in acks), acks
    page.wait_for_timeout(2000)
    gait_panel.get_by_role("button", name="stand", exact=True).click()
    page.wait_for_timeout(1500)
    start = len(frames)
    gait_panel.get_by_role("button", name="walk", exact=True).click()
    page.locator("#vx").evaluate("element => { element.value = '0.12'; "
        "element.dispatchEvent(new Event('input')); element.dispatchEvent(new Event('change')); }")
    try:
        page.wait_for_timeout(8000)
        assert "vx=0.12" in page.locator("#command-status").inner_text()
        joint_panel.screenshot(path=str(output) + "_joints.png")
        stats_panel.locator("summary").click()
        stats_panel.screenshot(path=str(output) + "_statistics.png")
        page.screenshot(path=str(output) + ".png", full_page=True)
        rows = joint_panel.locator("tbody tr").evaluate_all(
            "rows => rows.map(row => [...row.cells].map(cell => cell.textContent))")
        for row in rows:
            for index in (5, 6, 9, 10):
                assert math.isfinite(float(row[index])), row
        caption = joint_panel.locator(".joint-note").inner_text()
        stats_rows = stats_panel.locator(".joint-statistics tbody tr").evaluate_all(
            "rows => rows.map(row => [...row.cells].map(cell => cell.textContent))")
        for row in stats_rows:
            assert all(math.isfinite(float(value)) for value in row[1:]), row
            assert float(row[2]) >= float(row[1]), row  # sampled abs peak >= RMS
        diagnostic = frames[-1]["diagnostics"]
        assert diagnostic["samples"] > 2, diagnostic
        assert 0 < diagnostic["covered_s"] <= diagnostic["window_s"], diagnostic
        assert "sampled peaks" in stats_panel.inner_text().lower()
        assert "not total requested or applied torque" in stats_panel.inner_text()
        assert "not a time-window RMS" in caption
        assert "thermal estimate (uncalibrated), not measured" in caption
        assert "Knee belt 2:1, 95% assumed efficiency" in caption
        assert not errors, errors
        assert not [ack for ack in acks if not ack["ok"]], acks
        walk = [state for state in frames[start:] if state["controller"]["gait"] == "walk"]
        assert len(walk) > 10, "No live walking telemetry"
        assert all(state["controller"]["mode"] == "IMPEDANCE" for state in walk)
        displacement = walk[-1]["base"]["pos"][0] - walk[0]["base"]["pos"][0]
        report = {
            "url": url, "frames": len(frames), "walk_frames": len(walk),
            "walk_displacement_x_m": displacement,
            "final_state": frames[-1]["state"], "caption": caption,
            "joint_rows": rows, "acks": acks, "page_errors": errors,
            "statistics_rows": stats_rows, "diagnostics": diagnostic,
            "robot_model": robot_panel.inner_text(),
            "peak_joint_torque_nm": max(abs(j["eff"]) for s in walk for j in s["joints"]),
            "temperature_range_c": [min(j["temp"] for s in walk for j in s["joints"]),
                                     max(j["temp"] for s in walk for j in s["joints"])],
        }
        Path(str(output) + ".json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        assert displacement > .03, "Walk did not advance the robot"
    finally:
        gait_panel.get_by_role("button", name="zero velocity", exact=True).click()
        gait_panel.get_by_role("button", name="stand", exact=True).click()
        page.wait_for_timeout(500)
        browser.close()
