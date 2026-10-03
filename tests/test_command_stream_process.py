from pathlib import Path
import tempfile
from unittest import mock

from pymavlink import mavutil

from uav_formation.command_stream_process import (
    normalize_transmission, send_normalized)
from uav_formation.paper_tracking import PlaneCourseCommand
from scripts.audit_v3_command_timing import read_packets


class FakeMav:
    def __init__(self):
        self.local = []
        self.command = []
        self.attitude = []

    def set_position_target_local_ned_send(self, *values):
        self.local.append(values)

    def command_int_send(self, *values):
        self.command.append(values)

    def set_attitude_target_send(self, *values):
        self.attitude.append(values)


class FakeLink:
    def __init__(self):
        self.mav = FakeMav()

    class _Port:
        pass

    port = _Port()


class BatchPort:
    def __init__(self):
        self.batches = []

    def send_batch(self, packets):
        self.batches.append([bytes(packet) for packet in packets])


class BatchLink:
    def __init__(self):
        self.port = BatchPort()
        self.mav = mavutil.mavlink.MAVLink(self)

    def write(self, payload):
        raise AssertionError('batch path must not use per-packet write')


def test_copter_stream_reuses_existing_ned_field_mapping():
    transmission = normalize_transmission((
        'copter', ((101., 202., .3), (11., 12.), (.4, .5), -.2, False)))
    link = FakeLink()
    descriptor = {'home_offset': [1., 2.], 'altitude': 60.}
    send_normalized(link, descriptor, transmission)
    assert len(link.mav.local) == 1
    values = link.mav.local[0]
    assert values[4] == 0
    assert values[5:8] == (200., 100., -60.)
    assert values[8:11] == (12., 11., 0.)
    assert values[11:14] == (.5, .4, 0.)
    assert values[14:] == (.3, -.2)


def test_plane_stream_reuses_airspeed_altitude_and_course_commands():
    command = PlaneCourseCommand(14., 91., 60., 2.5, 0., 0., 14.,
                                 False, 0., 0.)
    transmission = normalize_transmission(('plane', command))
    link = FakeLink()
    send_normalized(link, {}, transmission)
    assert [values[3] for values in link.mav.command] == [43000, 186, 43002]
    assert link.mav.command[0][7] == 14.
    assert link.mav.command[1][6:8] == (60., 3.)
    assert link.mav.command[2][7:9] == (91., 2.5)


def test_plane_stream_batches_mavlink_packets_when_batch_port_is_available():
    command = PlaneCourseCommand(14., 91., 60., 2.5, 0., 0., 14.,
                                 False, 0., 0.)
    link = BatchLink()
    send_normalized(link, {}, normalize_transmission(('plane', command)))
    assert len(link.port.batches) == 1
    assert len(link.port.batches[0]) == 3
    decoded = [mavutil.mavlink.MAVLink(None).parse_buffer(packet)[0]
               for packet in link.port.batches[0]]
    assert [message.command for message in decoded] == [43000, 186, 43002]
    assert [message.get_seq() for message in decoded] == [0, 1, 2]


def test_timing_audit_prefers_isolated_command_stream_journal():
    with tempfile.TemporaryDirectory() as folder:
        root = Path(folder)
        isolated = root/'command_stream_packets.jsonl'
        legacy = root/'outgoing_packets.jsonl'
        isolated.write_text('isolated')
        legacy.write_text('legacy')
        with mock.patch('scripts.audit_v3_command_timing.read_journal',
                        side_effect=lambda path: (Path(path).name, {})):
            packets, _ = read_packets(root)
        assert packets == isolated.name
