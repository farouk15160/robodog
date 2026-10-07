"""Web command translations at the WebApp public command seam."""
from pathlib import Path
from types import SimpleNamespace

from robodog_web.web_server import WebApp, WebBridgeNode


def app_with_calls(tmp_path):
    calls = []
    node = SimpleNamespace(
        state={"simulation": {"active": True}}, state_seq=0, _events=[],
        call=lambda name, request: calls.append((name, request)) or (True, "accepted"),
        pub_vel=SimpleNamespace(publish=lambda _: None),
    )
    cfg = {
        "host": "127.0.0.1", "stream_rate_hz": 20,
        "limits": {"max_linear_velocity": 0.5,
                   "simulation_max_linear_velocity": 2.0,
                   "max_angular_velocity": 1.2},
        "map_output_directory": str(tmp_path),
    }
    app = WebApp(node, cfg, str(Path(__file__).resolve().parents[1] / "www"), {})
    return app, calls


def test_greeting_command_calls_dedicated_service(tmp_path):
    app, calls = app_with_calls(tmp_path)

    assert app.handle_command({"action": "greeting"}) == (True, "accepted")
    assert calls[0][0] == "greeting"


def test_command_payload_must_be_a_json_object(tmp_path):
    app, calls = app_with_calls(tmp_path)

    assert app.handle_command(["cmd_vel"]) == (False, "command must be a JSON object")
    assert calls == []


def test_save_map_uses_server_output_root_and_validated_name(tmp_path):
    app, calls = app_with_calls(tmp_path)

    ok, message, data = app.handle_command(
        {"action": "save_map", "name": "ground-floor_01"})
    assert ok and message == "accepted"
    assert data["mapping"]["status"] == "saved"
    name, request = calls[0]
    assert name == "save_map"
    assert request.output_directory == str(tmp_path)
    assert request.name == "ground-floor_01"


def test_save_map_rejects_path_like_names(tmp_path):
    app, calls = app_with_calls(tmp_path)

    for name in ("../outside", "room/one", ".hidden", "room one"):
        ok, _ = app.handle_command({"action": "save_map", "name": name})
        assert not ok
    assert calls == []


def test_save_map_allows_empty_name_for_timestamped_session(tmp_path):
    app, calls = app_with_calls(tmp_path)

    assert app.handle_command({"action": "save_map"})[:2] == (True, "accepted")
    assert calls[0][1].name == ""


def test_save_map_result_keeps_manifest_for_websocket_ack(tmp_path):
    app, _ = app_with_calls(tmp_path)
    app.node.call = lambda *_: (
        True, "saved 1200 points", {"manifest_path": "/maps/room/manifest.json"})

    ok, message, data = app.handle_command({"action": "save_map", "name": "room.01"})

    assert ok
    assert message == "saved 1200 points"
    assert data["manifest_path"] == "/maps/room/manifest.json"


def test_drive_owner_disconnect_stops_motion_but_observer_disconnect_does_not(tmp_path):
    published = []
    app, _ = app_with_calls(tmp_path)
    app.node.pub_vel = SimpleNamespace(publish=published.append)
    owner, observer = object(), object()

    assert app.handle_command(
        {"action": "cmd_vel", "vx": 0.4}, owner)[0]
    assert not app.handle_command(
        {"action": "cmd_vel", "vx": 0.2}, observer)[0]
    assert not app.handle_command(
        {"action": "cmd_vel", "vx": 0.0}, observer)[0]
    assert app._drive_owner is owner
    assert not app._release_drive(observer)
    assert len(published) == 1
    assert app._release_drive(owner)
    assert published[-1].linear.x == 0.0
    assert published[-1].linear.y == 0.0
    assert published[-1].angular.z == 0.0


def test_expired_drive_lease_publishes_zero(tmp_path):
    published = []
    app, _ = app_with_calls(tmp_path)
    app.node.pub_vel = SimpleNamespace(publish=published.append)
    owner = object()
    assert app.handle_command({"action": "cmd_vel", "vx": 0.4}, owner)[0]

    assert app._expire_drive_lease(now=app._drive_last + 1.0)
    assert published[-1].linear.x == 0.0
    assert app._drive_owner is None


def test_save_map_service_can_finish_after_motion_service_timeout(monkeypatch):
    class DelayedFuture:
        def __init__(self):
            self.checks = 0

        def done(self):
            self.checks += 1
            return self.checks >= 6

        def result(self):
            return SimpleNamespace(success=True, message="saved",
                                   manifest_path="/maps/manifest.json")

    future = DelayedFuture()
    client = SimpleNamespace(service_is_ready=lambda: True,
                             call_async=lambda _: future)
    node = SimpleNamespace(cli={"save_map": client},
                           cfg={"map_save_timeout_s": 60.0})
    ticks = iter([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    monkeypatch.setattr("robodog_web.web_server.time.monotonic", lambda: next(ticks))
    monkeypatch.setattr("robodog_web.web_server.time.sleep", lambda _: None)

    result = WebBridgeNode.call(node, "save_map", object())

    assert result == (True, "saved", {"manifest_path": "/maps/manifest.json"})
