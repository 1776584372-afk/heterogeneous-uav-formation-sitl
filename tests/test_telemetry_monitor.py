import json
import socket
from unittest.mock import Mock

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from uav_formation.telemetry_monitor import TelemetryMonitor, monitor_settings


def test_udp_fleet_display_ids_preserve_control_message(tmp_path):
    config = tmp_path/'monitor.json'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(('127.0.0.1', 0))
        receiver.settimeout(1.)
        config.write_text(json.dumps(dict(enabled=True, host='127.0.0.1',
                                         port=receiver.getsockname()[1])))
        encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        encoder.seq = 42
        message = encoder.heartbeat_encode(2, 3, 0, 4, 3)
        original = message.pack(encoder)
        for display_id in (1, 2, 3):
            monitor = TelemetryMonitor(monitor_settings(display_id, config))
            try:
                monitor.forward(message)
                packet, _ = receiver.recvfrom(4096)
                received = mavlink.MAVLink(None).parse_buffer(packet)[0]
                assert received.get_srcSystem() == display_id
                assert received.get_srcComponent() == 1
                assert received.get_seq() == 42
                assert received.to_dict() == message.to_dict()
                assert bytes(message.get_msgbuf()) == original
                assert message.get_srcSystem() == 1
            finally:
                result = monitor.close()
            assert result['sent_packets'] == 1
            assert result['errors'] == 0


def test_send_error_does_not_interrupt_control():
    monitor = TelemetryMonitor(dict(enabled=False, host='127.0.0.1', port=14550,
                                   display_system_id=1))
    monitor.socket = Mock()
    monitor.socket.sendto.side_effect = BlockingIOError('test full UDP buffer')
    encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    message = encoder.heartbeat_encode(2, 3, 0, 4, 3)
    message.pack(encoder)
    monitor.forward(message)
    result = monitor.close()
    assert result['errors'] == 1
    assert result['sent_packets'] == 0
    assert 'full UDP buffer' in result['last_error']


def test_unserializable_message_copy_does_not_interrupt_control(monkeypatch):
    monitor = TelemetryMonitor(dict(enabled=False, host='127.0.0.1', port=14550,
                                   display_system_id=1))
    monitor.socket = Mock()
    encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    message = encoder.heartbeat_encode(2, 3, 0, 4, 3)
    monkeypatch.setattr('uav_formation.telemetry_monitor.copy.copy',
                        Mock(side_effect=NotImplementedError('unsupported message copy')))
    monitor.forward(message)
    result = monitor.close()
    assert result['errors'] == 1
    assert result['sent_packets'] == 0
    assert result['last_error'] == 'unsupported message copy'


def test_disabled_monitor_and_bad_configuration(tmp_path):
    config = tmp_path/'monitor.json'
    config.write_text(json.dumps(dict(enabled=False, host='127.0.0.1', port=14550)))
    monitor = TelemetryMonitor(monitor_settings(1, config))
    assert monitor.socket is None
    assert monitor.close()['sent_packets'] == 0
    with pytest.raises(ValueError):
        monitor_settings(255, config)
    config.write_text(json.dumps(dict(enabled=True, host='127.0.0.1', port=0)))
    with pytest.raises(ValueError):
        monitor_settings(1, config)
