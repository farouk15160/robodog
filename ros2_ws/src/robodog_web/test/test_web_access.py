"""Command WebSockets accept the local GUI origin, not arbitrary websites."""
import asyncio
from pathlib import Path
from types import SimpleNamespace

import pytest
from aiohttp import WSServerHandshakeError
from aiohttp.test_utils import TestClient, TestServer

from robodog_web.web_server import WebApp


@pytest.mark.parametrize("origin", [None, "http://untrusted.example", "null"])
def test_command_socket_rejects_missing_or_foreign_origin(origin):
    async def check():
        app = WebApp(SimpleNamespace(state=None),
                     {"host": "127.0.0.1", "stream_rate_hz": 20},
                     str(Path(__file__).resolve().parents[1] / "www"), {})
        async with TestClient(TestServer(app.app)) as client:
            with pytest.raises(WSServerHandshakeError) as error:
                async with client.ws_connect("/ws", origin=origin):
                    pass
            assert error.value.status == 403
    asyncio.run(check())


def test_local_gui_origin_can_receive_telemetry():
    async def check():
        app = WebApp(SimpleNamespace(state=None),
                     {"host": "127.0.0.1", "stream_rate_hz": 20},
                     str(Path(__file__).resolve().parents[1] / "www"), {"robot": "robodog"})
        async with TestClient(TestServer(app.app)) as client:
            async with client.ws_connect("/ws", origin=str(client.make_url(""))) as ws:
                assert await ws.receive_json() == {"type": "info", "data": {"robot": "robodog"}}
    asyncio.run(check())


def test_command_socket_rejects_non_object_json_without_closing():
    async def check():
        app = WebApp(SimpleNamespace(state=None),
                     {"host": "127.0.0.1", "stream_rate_hz": 20},
                     str(Path(__file__).resolve().parents[1] / "www"), {})
        async with TestClient(TestServer(app.app)) as client:
            async with client.ws_connect("/ws", origin=str(client.make_url(""))) as ws:
                await ws.receive_json()
                await ws.send_json(["cmd_vel"])
                ack = await ws.receive_json()
                assert ack == {"type": "ack", "action": None, "ok": False,
                               "message": "command must be a JSON object"}
                assert not ws.closed
    asyncio.run(check())


def test_local_binding_rejects_foreign_host_even_with_matching_origin():
    """A rebound DNS name must not become an accepted localhost GUI origin."""
    async def check():
        app = WebApp(SimpleNamespace(state=None),
                     {"host": "127.0.0.1", "stream_rate_hz": 20},
                     str(Path(__file__).resolve().parents[1] / "www"), {})
        async with TestClient(TestServer(app.app)) as client:
            with pytest.raises(WSServerHandshakeError) as error:
                async with client.ws_connect("/ws", origin="http://untrusted.example",
                                             headers={"Host": "untrusted.example"}):
                    pass
            assert error.value.status == 403
    asyncio.run(check())


@pytest.mark.parametrize("simulation,expected", [(True, 2.0), (False, 0.5)])
def test_velocity_command_allows_two_only_in_simulation(simulation, expected):
    async def check():
        published = []
        node = SimpleNamespace(state={"simulation": {"active": simulation}},
                               state_seq=0, _events=[],
                               pub_vel=SimpleNamespace(publish=published.append))
        config = {"host": "127.0.0.1", "stream_rate_hz": 20,
                  "limits": {"max_linear_velocity": .5,
                             "simulation_max_linear_velocity": 2.,
                             "max_angular_velocity": 1.2}}
        app = WebApp(node, config, str(Path(__file__).resolve().parents[1] / "www"), {})
        async with TestClient(TestServer(app.app)) as client:
            async with client.ws_connect("/ws", origin=str(client.make_url(""))) as ws:
                await ws.receive_json()
                await ws.send_json({"type": "cmd", "action": "cmd_vel", "vx": 2.0})
                ack = await ws.receive_json()
                while ack["type"] != "ack":
                    ack = await ws.receive_json()
                assert ack["ok"], ack
                assert published[-1].linear.x == expected
                if not simulation:
                    assert "clamped" in ack["message"]
                    assert "hardware" in ack["message"]
    asyncio.run(check())


@pytest.mark.parametrize("state,value", [(None, 2.), ({"simulation": {"active": True}}, "nan")])
def test_velocity_rejects_unknown_backend_or_nonfinite_request(state, value):
    async def check():
        published = []
        node = SimpleNamespace(state=state, state_seq=0, _events=[],
                               pub_vel=SimpleNamespace(publish=published.append))
        config = {"host": "127.0.0.1", "stream_rate_hz": 20,
                  "limits": {"max_linear_velocity": .5,
                             "simulation_max_linear_velocity": 2.,
                             "max_angular_velocity": 1.2}}
        app = WebApp(node, config, str(Path(__file__).resolve().parents[1] / "www"), {})
        async with TestClient(TestServer(app.app)) as client:
            async with client.ws_connect("/ws", origin=str(client.make_url(""))) as ws:
                await ws.receive_json()
                await ws.send_json({"type": "cmd", "action": "cmd_vel", "vx": value})
                ack = await ws.receive_json()
                while ack["type"] != "ack":
                    ack = await ws.receive_json()
                assert not ack["ok"]
                assert not published
    asyncio.run(check())
