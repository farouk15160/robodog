"""Keep the shipped GUI reachable from another device on the robot LAN."""
from pathlib import Path
import runpy

import yaml


def test_bringup_and_standalone_web_defaults_listen_on_lan_interfaces():
    workspace_src = Path(__file__).resolve().parents[2]
    launch = runpy.run_path(
        str(workspace_src / "robodog_bringup" / "launch" / "robot.launch.py")
    )
    defaults = {name: default for name, default, _choices, _help in launch["ARGS"]}
    assert defaults["web_host"] == "0.0.0.0"
    assert defaults["web_discovery"] == "true"
    assert defaults["device_name"] == "RoboDog"

    config = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "config" / "web.yaml").read_text()
    )
    assert config["robodog_web"]["host"] == "0.0.0.0"


def test_web_entrypoint_cache_busts_the_camera_route_fix():
    index = (Path(__file__).resolve().parents[1] / "www" / "index.html").read_text()

    assert '/static/js/panels.js?v=3' in index
