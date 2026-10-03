"""Forward a dedicated SITL MAVLink channel to the configured Mac GCS."""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import socket
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tcp-port', type=int, required=True)
    parser.add_argument('--host', required=True)
    parser.add_argument('--udp-port', type=int, required=True)
    parser.add_argument('--display-system-id', type=int, required=True)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--cpu', type=int)
    args = parser.parse_args()
    if args.cpu is not None:
        os.sched_setaffinity(0, {args.cpu})
    stopped = False

    def stop(_signum, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    sent_packets = sent_bytes = errors = 0
    last_error = None
    link = None
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setblocking(False)
    encoders = {}
    start = time.monotonic()
    try:
        while link is None and not stopped:
            try:
                link = mavutil.mavlink_connection(
                    'tcp:127.0.0.1:%d' % args.tcp_port, source_system=253)
            except OSError as exc:
                last_error = str(exc)
                if time.monotonic() - start > 30.:
                    raise
                time.sleep(.1)
        while not stopped:
            message = link.recv_match(blocking=True, timeout=.1)
            if message is None or message.get_type() == 'BAD_DATA':
                continue
            try:
                if message.get_srcSystem() == args.display_system_id:
                    packet = bytes(message.get_msgbuf())
                else:
                    try:
                        component = message.get_srcComponent()
                        encoder = encoders.setdefault(component, mavlink.MAVLink(
                            None, srcSystem=args.display_system_id,
                            srcComponent=component))
                        encoder.seq = message.get_seq()
                        packet = copy.copy(message).pack(encoder)
                    except (ValueError, TypeError, AttributeError,
                            NotImplementedError):
                        packet = bytes(message.get_msgbuf())
                count = udp.sendto(packet, (args.host, args.udp_port))
                if count != len(packet):
                    raise OSError('partial telemetry datagram')
                sent_packets += 1
                sent_bytes += count
            except (OSError, ValueError, TypeError, AttributeError,
                    NotImplementedError) as exc:
                errors += 1
                last_error = str(exc)
    finally:
        if link is not None:
            link.close()
        udp.close()
        args.summary.write_text(json.dumps({
            'enabled': True,
            'host': args.host,
            'port': args.udp_port,
            'display_system_id': args.display_system_id,
            'sent_packets': sent_packets,
            'sent_bytes': sent_bytes,
            'errors': errors,
            'last_error': last_error,
            'separate_process': True,
        }, indent=2)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
