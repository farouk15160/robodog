"""Robot identity and DNS-SD discovery stay read-only and input-safe."""
import asyncio
import json
from pathlib import Path
from types import SimpleNamespace

from aiohttp.test_utils import TestClient, TestServer
import pytest

from robodog_web.discovery import (build_descriptor, DiscoveryAdvertiser,
                                   load_or_create_device_id)
from robodog_web.web_server import WebApp


def test_device_id_is_persistent_uuid_with_private_permissions(tmp_path):
    identity = tmp_path / 'state' / 'device.json'

    first = load_or_create_device_id(identity)
    second = load_or_create_device_id(identity)

    assert first == second
    assert len(first) == 36
    assert json.loads(identity.read_text()) == {'device_id': first}
    assert identity.stat().st_mode & 0o777 == 0o600
    assert identity.parent.stat().st_mode & 0o777 == 0o700


@pytest.mark.parametrize('value', ['', '../device.json', 'relative/device.json'])
def test_device_identity_path_must_be_absolute(value):
    with pytest.raises(ValueError, match='absolute'):
        load_or_create_device_id(Path(value))


def test_descriptor_truthfully_keeps_commands_same_origin():
    descriptor = build_descriptor(
        device_id='12345678-1234-4234-9234-123456789abc',
        device_name='RoboDog Lab',
        port=8080,
        camera_rate_hz=10.0,
    )

    assert descriptor['schema'] == 'robodog.discovery.v1'
    assert descriptor['device']['id'] == '12345678-1234-4234-9234-123456789abc'
    assert descriptor['telemetry']['state'] == '/api/state'
    assert descriptor['camera']['mjpeg'] == '/stream/color.mjpg'
    assert descriptor['ui']['remote_control'] == '/remote'
    assert descriptor['control'] == {
        'websocket': '/ws',
        'security': 'same-origin-local-network',
        'native_command_bypass': False,
    }


@pytest.mark.parametrize(
    'name,port',
    [('bad\nname', 8080), ('x' * 64, 8080), ('RoboDog', 0),
     ('RoboDog', 65536), ('RoboDog', True)],
)
def test_descriptor_rejects_unsafe_service_values(name, port):
    with pytest.raises(ValueError):
        build_descriptor(
            device_id='12345678-1234-4234-9234-123456789abc',
            device_name=name,
            port=port,
            camera_rate_hz=10.0,
        )


def test_advertiser_uses_argument_vector_and_stops_process():
    calls = []

    class Process:

        def __init__(self):
            self.returncode = None

        def poll(self):
            return self.returncode

        def terminate(self):
            calls.append('terminate')
            self.returncode = 0

        def wait(self, timeout):
            calls.append(('wait', timeout))

        def kill(self):
            calls.append('kill')

    def popen(argv, **kwargs):
        calls.append((argv, kwargs))
        return Process()

    advertiser = DiscoveryAdvertiser(
        executable='/usr/bin/avahi-publish-service',
        popen=popen,
    )
    assert advertiser.start(
        name='RoboDog Lab',
        port=8080,
        txt={'id': '12345678-1234-4234-9234-123456789abc', 'api': '1'},
    )
    advertiser.stop()

    argv, kwargs = calls[0]
    assert argv == [
        '/usr/bin/avahi-publish-service', 'RoboDog Lab', '_robodog._tcp', '8080',
        'api=1', 'id=12345678-1234-4234-9234-123456789abc',
    ]
    assert kwargs['shell'] is False
    assert kwargs['stdin'] is not None
    assert calls[1:] == ['terminate', ('wait', 2.0)]


@pytest.mark.parametrize(
    'txt',
    [{'bad key': '1'}, {'id': 'line\nbreak'}, {'id': 'x' * 253}],
)
def test_advertiser_rejects_invalid_txt_records(txt):
    advertiser = DiscoveryAdvertiser(executable='/bin/true')
    with pytest.raises(ValueError):
        advertiser.start(name='RoboDog', port=8080, txt=txt)


def test_advertiser_failure_is_nonfatal_when_optional_utility_cannot_start():
    def unavailable(*_args, **_kwargs):
        raise FileNotFoundError('avahi-publish-service disappeared')

    advertiser = DiscoveryAdvertiser(executable='/missing/avahi', popen=unavailable)

    assert not advertiser.start(name='RoboDog', port=8080, txt={'api': '1'})


def test_well_known_endpoint_is_read_only_and_no_store(tmp_path):
    async def check():
        cfg = {
            'host': '127.0.0.1',
            'port': 8080,
            'stream_rate_hz': 20,
            'video_rate_hz': 10,
            'discovery': {
                'enabled': False,
                'device_name': 'RoboDog Lab',
                'identity_file': str(tmp_path / 'device.json'),
            },
        }
        node = SimpleNamespace(state=None)
        app = WebApp(node, cfg, str(Path(__file__).resolve().parents[1] / 'www'), {})
        async with TestClient(TestServer(app.app)) as client:
            response = await client.get('/.well-known/robodog')
            payload = await response.json()
            assert response.status == 200
            assert response.headers['Cache-Control'] == 'no-store'
            assert payload['device']['name'] == 'RoboDog Lab'
            assert payload['control']['native_command_bypass'] is False
            assert 'token' not in json.dumps(payload).lower()
    asyncio.run(check())


def test_loopback_binding_does_not_advertise_identity_on_the_lan(tmp_path):
    cfg = {
        'host': '127.0.0.1', 'port': 8080, 'stream_rate_hz': 20,
        'discovery': {
            'enabled': True, 'device_name': 'RoboDog',
            'identity_file': str(tmp_path / 'device.json'),
        },
    }

    app = WebApp(SimpleNamespace(state=None), cfg,
                 str(Path(__file__).resolve().parents[1] / 'www'), {})

    assert not app._discovery_enabled
