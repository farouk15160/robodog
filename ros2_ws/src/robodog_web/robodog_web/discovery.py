"""Read-only LAN discovery for RoboDog operator clients.

Discovery intentionally advertises where the existing HTTP service lives.  It
does not create another command channel or weaken the WebSocket's same-origin
checks.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
from typing import Callable, Mapping
from uuid import UUID, uuid4


SERVICE_TYPE = '_robodog._tcp'
_SERVICE_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9 ._-]{0,62}$')
_TXT_KEY = re.compile(r'^[a-z][a-z0-9_-]{0,31}$')


def _valid_port(port: object) -> int:
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ValueError('discovery port must be an integer from 1 to 65535')
    return port


def _valid_name(name: object) -> str:
    if not isinstance(name, str) or not _SERVICE_NAME.fullmatch(name):
        raise ValueError('device name must be 1-63 safe display characters')
    return name


def _valid_device_id(device_id: object) -> str:
    if not isinstance(device_id, str):
        raise ValueError('device id must be a UUID string')
    try:
        parsed = UUID(device_id)
    except (ValueError, AttributeError) as error:
        raise ValueError('device id must be a UUID string') from error
    if str(parsed) != device_id.lower():
        raise ValueError('device id must use canonical UUID syntax')
    return str(parsed)


def default_identity_path(configured: str = '') -> Path:
    """Resolve an explicit path or the ROS state directory."""
    if configured:
        path = Path(configured).expanduser()
    else:
        ros_home = os.environ.get('ROS_HOME')
        base = Path(ros_home).expanduser() if ros_home else Path.home() / '.ros'
        path = base / 'robodog' / 'device.json'
    if not path.is_absolute():
        raise ValueError('device identity path must be absolute')
    return path


def _read_identity(path: Path) -> str:
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0)
    descriptor = os.open(path, flags)
    try:
        details = os.fstat(descriptor)
        if not stat.S_ISREG(details.st_mode):
            raise ValueError('device identity must be a regular file')
        with os.fdopen(descriptor, 'r', encoding='utf-8', closefd=False) as stream:
            payload = json.load(stream)
    finally:
        os.close(descriptor)
    if not isinstance(payload, dict) or set(payload) != {'device_id'}:
        raise ValueError('device identity file has an invalid schema')
    return _valid_device_id(payload['device_id'])


def load_or_create_device_id(path: Path) -> str:
    """Load a stable UUID, creating a private state file exactly once."""
    path = Path(path)
    if not path.is_absolute():
        raise ValueError('device identity path must be absolute')
    parent = path.parent
    parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    if parent.is_symlink() or not parent.is_dir():
        raise ValueError('device identity directory must be a real directory')
    os.chmod(parent, 0o700)
    try:
        device_id = _read_identity(path)
        os.chmod(path, 0o600, follow_symlinks=False)
        return device_id
    except FileNotFoundError:
        pass

    device_id = str(uuid4())
    payload = json.dumps({'device_id': device_id}, separators=(',', ':'))
    encoded = f'{payload}\n'.encode()
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0)
    try:
        descriptor = os.open(path, flags, 0o600)
    except FileExistsError:  # Another server process won the create race.
        return _read_identity(path)
    try:
        view = memoryview(encoded)
        while view:
            view = view[os.write(descriptor, view):]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return device_id


def build_descriptor(*, device_id: str, device_name: str, port: int,
                     camera_rate_hz: float) -> dict:
    """Return the public, non-secret service descriptor."""
    identifier = _valid_device_id(device_id)
    name = _valid_name(device_name)
    checked_port = _valid_port(port)
    if (isinstance(camera_rate_hz, bool)
            or not isinstance(camera_rate_hz, (int, float))
            or not math.isfinite(camera_rate_hz)
            or not 0.0 <= camera_rate_hz <= 120.0):
        raise ValueError('camera rate must be finite and between 0 and 120 Hz')
    return {
        'schema': 'robodog.discovery.v1',
        'device': {'id': identifier, 'name': name, 'model': 'robodog'},
        'service': {'port': checked_port, 'dns_sd_type': f'{SERVICE_TYPE}.local'},
        'api': {'protocol': 1, 'info': '/api/info', 'config': '/api/config'},
        'telemetry': {'state': '/api/state', 'websocket': '/ws'},
        'camera': {'mjpeg': '/stream/color.mjpg', 'max_rate_hz': camera_rate_hz},
        'ui': {'remote_control': '/remote'},
        'control': {
            'websocket': '/ws',
            'security': 'same-origin-local-network',
            'native_command_bypass': False,
        },
    }


def _txt_arguments(txt: Mapping[str, str]) -> list[str]:
    arguments = []
    for key, value in sorted(txt.items()):
        if not isinstance(key, str) or not _TXT_KEY.fullmatch(key):
            raise ValueError('DNS-SD TXT keys must use lowercase ASCII identifiers')
        if not isinstance(value, str) or any(ord(char) < 0x20 for char in value):
            raise ValueError('DNS-SD TXT values must be printable strings')
        record = f'{key}={value}'
        if len(record.encode('utf-8')) > 255:
            raise ValueError('DNS-SD TXT records cannot exceed 255 bytes')
        arguments.append(record)
    return arguments


class DiscoveryAdvertiser:
    """Lifecycle wrapper around Avahi's service publisher."""

    def __init__(self, *, executable: str | None = None,
                 popen: Callable[..., subprocess.Popen] = subprocess.Popen) -> None:
        self._executable = executable or shutil.which('avahi-publish-service')
        self._popen = popen
        self._process: subprocess.Popen | None = None

    @property
    def available(self) -> bool:
        return self._executable is not None

    def start(self, *, name: str, port: int, txt: Mapping[str, str]) -> bool:
        checked_name = _valid_name(name)
        checked_port = _valid_port(port)
        records = _txt_arguments(txt)
        if self._executable is None:
            return False
        if self._process is not None and self._process.poll() is None:
            return True
        argv = [self._executable, checked_name, SERVICE_TYPE, str(checked_port), *records]
        try:
            self._process = self._popen(
                argv,
                shell=False,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                close_fds=True,
            )
        except OSError:
            self._process = None
            return False
        return True

    def stop(self) -> None:
        process, self._process = self._process, None
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)
