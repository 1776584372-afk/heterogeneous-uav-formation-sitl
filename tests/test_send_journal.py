import errno
import io
import json
import unittest
from unittest.mock import Mock

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from uav_formation.send_journal import JournaledSocket


class SendJournalTests(unittest.TestCase):
    def make_journal(self, socket, stream=None):
        self.stream = stream if stream is not None else io.StringIO()
        clock = iter([10., 10.002, 10.1, 10.102]).__next__
        return JournaledSocket(socket, self.stream, lambda: {'phase': 'track'},
                               origin=5., clock=clock)

    def test_real_mavlink_bytes_unchanged_and_independently_decodable(self):
        socket = Mock()
        socket.send.side_effect = lambda packet: len(packet)
        journal = self.make_journal(socket)
        class Writer:
            def write(self, data): return journal.send(data)
        mav = mavlink.MAVLink(Writer(), srcSystem=255)
        mav.set_position_target_local_ned_send(0, 1, 1, 1, 0,
            20., 10., -60., 0., 12., 0., 0., .1, 0., 1.57, -.2)
        journal.flush()
        event = json.loads(self.stream.getvalue())
        packed = bytes.fromhex(event['packet_hex'])
        self.assertEqual(packed, socket.send.call_args.args[0])
        decoded = mavlink.MAVLink(None).parse_char(packed)
        self.assertEqual(decoded.get_type(), 'SET_POSITION_TARGET_LOCAL_NED')
        self.assertEqual((decoded.x, decoded.y, decoded.z), (20., 10., -60.))
        self.assertEqual(decoded.type_mask, 0)
        self.assertAlmostEqual(decoded.vy, 12.)
        self.assertAlmostEqual(decoded.yaw_rate, -.2)
        self.assertEqual(event['status'], 'SOCKET_ACCEPTED_ALL')
        self.assertEqual(event['accepted_bytes'], len(packed))
        self.assertAlmostEqual(event['wall_before_s'], 5.)
        self.assertAlmostEqual(event['wall_after_s'], 5.002)
        journal.check_logging()

    def test_partial_write_is_not_retried_or_reported_as_complete(self):
        socket = Mock(); socket.send.return_value = 2
        journal = self.make_journal(socket)
        self.assertEqual(journal.send(b'abcd'), 2)
        journal.flush()
        socket.send.assert_called_once_with(b'abcd')
        event = json.loads(self.stream.getvalue())
        self.assertEqual(event['status'], 'SHORT_WRITE')
        self.assertEqual(event['accepted_bytes'], 2)

    def test_error_is_logged_and_original_exception_preserved(self):
        socket = Mock(); failure = OSError(errno.EPIPE, 'broken pipe')
        socket.send.side_effect = failure
        journal = self.make_journal(socket)
        with self.assertRaises(OSError) as raised:
            journal.send(b'abcd')
        journal.flush()
        self.assertIs(raised.exception, failure)
        event = json.loads(self.stream.getvalue())
        self.assertEqual(event['status'], 'ERROR')
        self.assertIsNone(event['accepted_bytes'])
        self.assertIn('broken pipe', event['error'])

    def test_logging_failure_cannot_masquerade_as_complete_evidence(self):
        socket = Mock(); socket.send.return_value = 4
        stream = Mock(); stream.write.side_effect = OSError('disk full')
        journal = self.make_journal(socket, stream)
        self.assertEqual(journal.send(b'abcd'), 4)
        with self.assertRaises(OSError):
            journal.flush()
        with self.assertRaisesRegex(RuntimeError, 'disk full'):
            journal.check_logging()

    def test_recv_close_and_fileno_still_use_underlying_socket(self):
        socket = Mock(); socket.recv.return_value = b'reply'; socket.fileno.return_value = 9
        journal = self.make_journal(socket)
        self.assertEqual(journal.recv(5), b'reply')
        self.assertEqual(journal.fileno(), 9)
        journal.close()
        socket.close.assert_called_once()
        self.assertEqual(self.stream.getvalue(), '')


if __name__ == '__main__':
    unittest.main()
import json
import socket

from uav_formation.send_journal import JournaledSocket


def test_send_batch_records_monotonic_packet_timestamps():
    left, right = socket.socketpair()
    try:
        stream = __import__('io').StringIO()
        journal = JournaledSocket(left, stream, lambda: {'phase': 'track'},
                                  origin=0., clock=lambda: 10.)
        journal.send_batch([b'one', b'two', b'three'])
        journal.flush()
        records = [json.loads(line) for line in stream.getvalue().splitlines()]
        assert [row['send_index'] for row in records] == [1, 2, 3]
        assert all(row['wall_before_s'] == row['wall_after_s'] for row in records)
        assert all(row['batch_write'] and row['batch_size'] == 3 for row in records)
        assert right.recv(32) == b'onetwothree'
    finally:
        left.close()
        right.close()
