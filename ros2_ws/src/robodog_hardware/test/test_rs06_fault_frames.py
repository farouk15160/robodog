"""RS06 protection frames reach the public backend state without real CAN I/O."""
from pathlib import Path
from types import SimpleNamespace
import struct

import pytest
import yaml

from robodog_hardware.protocol import robstride06 as rs
from robodog_hardware.registry import create_backend
from robodog_hardware.transmission import backend_config
from robodog_hardware.types import JOINT_NAMES, Fault


@pytest.fixture
def backend():
    base = Path(__file__).parents[2]
    spec = yaml.safe_load((base / "robodog_description/config/robstride06.yaml").read_text())
    bus = yaml.safe_load((base / "robodog_hardware/config/robstride_bus.yaml").read_text())
    instance = create_backend("robstride06_can", {**bus, **backend_config(spec, JOINT_NAMES)})
    instance._send = lambda *_: None
    return instance


def fault_frame(bits=0, warnings=0, motor=3, host=0xFD):
    return SimpleNamespace(arbitration_id=rs.make_id(rs.CommType.FAULT_FEEDBACK, motor, host),
                           data=struct.pack("<II", bits, warnings))


def normal_frame(motor=3):
    return SimpleNamespace(arbitration_id=rs.make_id(rs.CommType.FEEDBACK, motor | (2 << 14), 0xFD),
                           data=struct.pack(">HHHH", 32768, 32768, 32768, 250))


def deliver(backend, frames, bus_id=0):
    pending = iter(frames)

    class Bus:
        def recv(self, timeout):
            try:
                return next(pending)
            except StopIteration:
                backend._stop.set()
                return None

    backend._stop.clear()
    backend._rx_loop(bus_id, Bus())


def test_fault_payload_keeps_fault_and_warning_bytes_separate_without_guessing_endianness():
    frame = fault_frame((1 << 16) | 1, 0x02030405)
    decoded = rs.decode_fault_feedback(frame.arbitration_id, frame.data)
    assert decoded.motor_id == 3
    assert decoded.fault_bytes == bytes.fromhex("01000100")
    assert decoded.warning_bytes == bytes.fromhex("05040302")


@pytest.mark.parametrize("bits", [1, 1 << 2, 1 << 4, 1 << 5, 1 << 16, 1 << 7,
                                  1 << 9, 1 << 14, 1 << 31, 1 << 3])
def test_protection_event_survives_normal_feedback_and_explicitly_clears(backend, bits):
    expected = Fault.COMMUNICATION  # protection fallback: type21 byte order remains unverified
    deliver(backend, [normal_frame(), fault_frame(bits), normal_frame(), fault_frame(0)])
    state = backend.read()
    assert state.faults[2] & int(expected)
    assert state.effort[2] == pytest.approx(36 / 65535 * 1.9)  # no fictitious current/torque
    backend.clear_faults()
    assert backend.read().faults[2] & int(Fault.COMMUNICATION)
    deliver(backend, [normal_frame()])
    assert not backend.read().faults[2] & int(expected)


def test_warning_only_packet_does_not_set_fault(backend):
    deliver(backend, [normal_frame(), fault_frame(0, 1)])
    assert backend.read().faults[2] == 0
    assert backend.stats()["fault_warning_bytes"]["FL_kfe_joint"] == "01000000"


def test_packet_is_routed_to_correct_bus_even_for_reused_motor_ids(backend):
    deliver(backend, [normal_frame()])
    deliver(backend, [normal_frame(), fault_frame(1)], bus_id=1)
    assert backend.read().faults[8] & int(Fault.COMMUNICATION)
    assert backend.read().faults[2] == 0


def test_wrong_host_packet_is_ignored(backend):
    deliver(backend, [normal_frame(), fault_frame(1, host=1)])
    assert backend.read().faults[2] == 0


def test_failed_clear_does_not_forget_protection(backend):
    deliver(backend, [normal_frame(), fault_frame(1)])

    def failed_send(*_):
        raise RuntimeError("bus unavailable")

    backend._send = failed_send
    with pytest.raises(RuntimeError):
        backend.clear_faults()
    assert backend.read().faults[2] & int(Fault.COMMUNICATION)


def test_fresh_fault_during_clear_is_not_erased(backend):
    deliver(backend, [normal_frame(), fault_frame(1 << 14)])
    backend._send = lambda i, *_: deliver(backend, [fault_frame(1)]) if i == 2 else None
    backend.clear_faults()
    assert backend.read().faults[2] & int(Fault.COMMUNICATION)
    assert backend.stats()["fault_feedback_bytes"]["FL_kfe_joint"] == "01000000"


def test_clear_requires_new_normal_feedback_not_repeated_cached_reads(backend):
    deliver(backend, [normal_frame(), fault_frame(1)])
    backend.clear_faults()
    for _ in range(3):
        assert backend.read().faults[2] & int(Fault.COMMUNICATION)
    deliver(backend, [fault_frame()])
    assert backend.read().faults[2] & int(Fault.COMMUNICATION)
    deliver(backend, [normal_frame()])
    assert backend.read().faults[2] == 0
    # A fresh front-knee frame does not make a rear-knee frame fresh.
    assert backend.read().faults[8] & int(Fault.COMMUNICATION)


def test_normal_frame_during_clear_transmits_does_not_satisfy_post_clear_confirmation(backend):
    deliver(backend, [normal_frame()])
    backend._send = lambda *_: deliver(backend, [normal_frame()])
    backend.clear_faults()
    assert backend.read().faults[2] & int(Fault.COMMUNICATION)
    deliver(backend, [normal_frame()])
    assert backend.read().faults[2] == 0


def test_unknown_motor_and_all_zero_frame_cannot_fault_known_joint(backend):
    deliver(backend, [normal_frame(), fault_frame(1, motor=99), fault_frame()])
    assert backend.read().faults[2] == 0


def test_malformed_fault_packet_triggers_protective_fallback(backend):
    malformed = fault_frame(1)
    malformed.data = bytes(3)
    deliver(backend, [normal_frame(), malformed])
    assert backend.read().faults[2] & int(Fault.COMMUNICATION)
    assert backend.stats()["rx_errors"] == 1


def test_fault_packets_do_not_make_stale_position_feedback_fresh(backend):
    deliver(backend, [normal_frame()])
    backend._fb_time[2] = 0
    deliver(backend, [fault_frame(0, 1)])
    assert backend.read().faults[2] & int(Fault.COMMUNICATION)


def test_rs02_does_not_use_the_rs06_protection_decoder(backend):
    rs02 = create_backend("robstride02_can", {**backend.config, "motor_model": "ROBSTRIDE02"})
    deliver(rs02, [normal_frame(), fault_frame(1)])
    assert rs02.read().faults[2] == 0


@pytest.mark.parametrize("frame", [normal_frame(), SimpleNamespace(arbitration_id=rs.make_id(
    rs.CommType.FAULT_FEEDBACK, 3, 0xFD), data=bytes(7))])
def test_decoder_rejects_wrong_type_or_length(frame):
    with pytest.raises(ValueError):
        rs.decode_fault_feedback(frame.arbitration_id, frame.data)
