"""Best-effort outbound GCS telemetry, isolated from the experiment control link."""
import copy
import ipaddress
import json
import queue
import socket
import threading
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mavlink


CONFIG = Path(__file__).resolve().parents[1]/'config/telemetry_monitor.json'


def monitor_settings(system_id, path=CONFIG):
    settings = json.loads(Path(path).read_text())
    if not isinstance(settings.get('enabled'), bool):
        raise ValueError('monitor enabled must be boolean')
    host = str(ipaddress.IPv4Address(settings['host']))
    port = settings['port']
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError('invalid GCS UDP port')
    if type(system_id) is not int or not 1 <= system_id <= 254:
        raise ValueError('GCS display system id must be between 1 and 254')
    return dict(enabled=settings['enabled'], host=host, port=port,
        display_system_id=system_id, original_system_id=1,
        address_basis=settings.get('address_basis', 'configured IPv4 address'),
        direction='outbound telemetry only; no GCS commands forwarded to SITL',
        evidence='local UDP send success does not establish Mac reception')


class TelemetryMonitor:
    def __init__(self, settings):
        self.settings = dict(settings)
        self.sent_packets = 0
        self.sent_bytes = 0
        self.errors = 0
        self.last_error = None
        self.socket = None
        self._queue = queue.Queue()
        self._worker = None
        if settings['enabled']:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.socket.setblocking(False)
            except OSError as exc:
                self.errors += 1
                self.last_error = str(exc)

    def _start_worker(self):
        if self._worker is None:
            self._worker = threading.Thread(
                target=self._send_messages, name='gcs-telemetry-forwarder', daemon=True)
            self._worker.start()

    def _send_messages(self):
        encoders = {}
        while True:
            message = self._queue.get()
            try:
                if message is None:
                    return
                component = message.get_srcComponent()
                encoder = encoders.setdefault(component, mavlink.MAVLink(
                    None, srcSystem=self.settings['display_system_id'],
                    srcComponent=component))
                encoder.seq = message.get_seq()
                packet = message.pack(encoder)
                count = self.socket.sendto(
                    packet, (self.settings['host'], self.settings['port']))
                if count != len(packet):
                    raise OSError('partial telemetry datagram')
                self.sent_packets += 1
                self.sent_bytes += count
            except (OSError, ValueError, TypeError, AttributeError,
                    NotImplementedError) as exc:
                self.errors += 1
                self.last_error = str(exc)
            finally:
                self._queue.task_done()

    def forward(self, message):
        if self.socket is None or message.get_type() == 'BAD_DATA':
            return
        # Repack only a copy: fleet SITLs all use sysid=1 on isolated TCP links.
        # A unique display ID prevents the GCS from merging their telemetry.
        try:
            forwarded = copy.copy(message)
            self._start_worker()
            self._queue.put(forwarded)
        except (OSError, ValueError, TypeError, AttributeError, NotImplementedError) as exc:
            self.errors += 1
            self.last_error = str(exc)

    def close(self):
        if self._worker is not None:
            self._queue.join()
            self._queue.put(None)
            self._queue.join()
            self._worker.join()
            self._worker = None
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        return dict(**self.settings, sent_packets=self.sent_packets,
                    sent_bytes=self.sent_bytes, errors=self.errors, last_error=self.last_error)
