"""Observe owned SITL socket writes without changing packets or retry policy.

A complete socket write means acceptance by the local TCP stack, not receipt
or execution by the aircraft. Associated boot clocks are latest observations,
not exact flight-controller processing times.
"""
import json
import time


class JournaledSocket:
    def __init__(self, socket, stream, context, *, origin, clock=time.monotonic):
        self.socket = socket
        self.stream = stream
        self.context = context
        self.origin = origin
        self.clock = clock
        self.sequence = 0
        self.logging_error = None
        self._closed = False
        self._records = []

    def __getattr__(self, name):
        return getattr(self.socket, name)

    def send(self, payload, *args, **kwargs):
        data = bytes(payload)
        context = self.context()
        self.sequence += 1
        before = self.clock()
        accepted = None
        error = None
        try:
            accepted = self.socket.send(payload, *args, **kwargs)
            return accepted
        except Exception as exc:
            error = repr(exc)
            raise
        finally:
            after = self.clock()
            if error is not None:
                status = 'ERROR'
            elif not isinstance(accepted, int):
                status = 'UNKNOWN_WRITE_COUNT'
            elif accepted == len(data):
                status = 'SOCKET_ACCEPTED_ALL'
            else:
                status = 'SHORT_WRITE'
            record = dict(schema_version=1, event='socket_send', send_index=self.sequence,
                monotonic_before_s=before, monotonic_after_s=after,
                wall_before_s=before-self.origin, wall_after_s=after-self.origin,
                requested_bytes=len(data), accepted_bytes=accepted, status=status,
                error=error, packet_hex=data.hex(), context=context)
            self._records.append(record)

    def send_batch(self, payloads):
        """Write a logical command batch in one socket operation.

        Plane guidance updates contain multiple MAVLink messages.  Recording
        and writing them as one batch prevents receiver back-pressure between
        the messages from looking like a missed guidance cycle.
        """
        packets = [bytes(payload) for payload in payloads]
        if not packets:
            return
        context = self.context()
        before = self.clock()
        error = None
        try:
            self.socket.sendall(b''.join(packets))
        except Exception as exc:
            error = repr(exc)
            raise
        finally:
            after = self.clock()
            status = 'SOCKET_ACCEPTED_ALL' if error is None else 'ERROR'
            for packet in packets:
                self.sequence += 1
                self._records.append(dict(
                    schema_version=1, event='socket_send',
                    send_index=self.sequence,
                    # A batch is one transport operation.  Give every
                    # decoded packet the operation completion timestamp so
                    # adjacent records remain monotonic in the legacy audit.
                    monotonic_before_s=after,
                    monotonic_after_s=after,
                    wall_before_s=after-self.origin,
                    wall_after_s=after-self.origin,
                    requested_bytes=len(packet),
                    accepted_bytes=(len(packet) if error is None else None),
                    status=status, error=error,
                    packet_hex=packet.hex(), context=context,
                    batch_size=len(packets), batch_write=True,
                    batch_monotonic_before_s=before,
                    batch_monotonic_after_s=after))

    def flush(self):
        try:
            for record in self._records:
                self.stream.write(json.dumps(record, allow_nan=False)+'\n')
            self._records.clear()
            self.stream.flush()
        except Exception as exc:
            self.logging_error = repr(exc)
            raise

    def check_logging(self):
        if self.logging_error is not None:
            raise RuntimeError('outgoing packet journal failed: '+self.logging_error)

    def close(self):
        if not self._closed:
            self.flush()
            self._closed = True
        self.socket.close()
