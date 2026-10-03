"""GIL-isolated cached command stream for owned ArduPilot SITL links.

The control process still sends every newly computed command on its primary
MAVLink link.  This helper owns a second SITL serial link and repeats only the
latest validated command.  A separate process is required because CPython
threads on different CPUs still share one process-wide GIL.
"""
from dataclasses import asdict, is_dataclass
import gc
import json
import math
import multiprocessing
import os
from pathlib import Path
import queue
import time

from pymavlink import mavutil

from .send_journal import JournaledSocket


def normalize_transmission(transmission):
    """Convert the runner's cached command into spawn-safe primitives."""
    if transmission is None:
        return None
    kind, values = transmission
    if kind == 'copter':
        target, velocity, acceleration, yaw_rate, yaw_angle_only = values
        if not all(math.isfinite(float(value)) for value in
                   (*target, *velocity, *acceleration)):
            raise ValueError('nonfinite cached Copter command')
        if yaw_rate is not None and not math.isfinite(float(yaw_rate)):
            raise ValueError('nonfinite cached Copter yaw rate')
        return {
            'kind': 'copter',
            'target': [float(value) for value in target],
            'velocity': [float(value) for value in velocity],
            'acceleration': [float(value) for value in acceleration],
            'yaw_rate': None if yaw_rate is None else float(yaw_rate),
            'yaw_angle_only': bool(yaw_angle_only),
        }
    if kind == 'plane':
        command = asdict(values) if is_dataclass(values) else dict(vars(values))
        required = ('airspeed', 'bearing_deg', 'altitude',
                    'lateral_accel_limit')
        if not all(key in command and math.isfinite(float(command[key]))
                   for key in required):
            raise ValueError('invalid cached Plane command')
        return {
            'kind': 'plane',
            'airspeed': float(command['airspeed']),
            'bearing_deg': float(command['bearing_deg']),
            'altitude': float(command['altitude']),
            'lateral_accel_limit': float(command['lateral_accel_limit']),
            'bank': (None if command.get('bank') is None
                     else float(command['bank'])),
        }
    raise ValueError('unknown cached transmission')


def send_normalized(link, descriptor, transmission):
    """Emit exactly the fields used by ``SitlVehicle`` on a second link."""
    if transmission['kind'] == 'copter':
        east, north, yaw = transmission['target']
        ve, vn = transmission['velocity']
        ae, an = transmission['acceleration']
        yaw_rate = transmission['yaw_rate']
        mask = 2048
        if yaw_rate is not None and not transmission['yaw_angle_only']:
            mask &= ~2048
        else:
            yaw_rate = 0.
        home_east, home_north = descriptor['home_offset']
        link.mav.set_position_target_local_ned_send(
            0, 1, 1, 1, mask,
            north-home_north, east-home_east, -descriptor['altitude'],
            vn, ve, 0., an, ae, 0., yaw, yaw_rate)
        return
    if transmission['kind'] == 'plane':
        speed = transmission['airspeed']
        course = transmission['bearing_deg'] % 360.
        altitude = transmission['altitude']
        accel = transmission['lateral_accel_limit']
        batch_port = getattr(link, 'port', None)
        if not hasattr(batch_port, 'send_batch'):
            for code, p1, p2, p3, z in (
                    (43000, 0., speed, 0., 0.),
                    (186, altitude, 3., 0., 0.),
                    (43002, 0., course, accel, 0.)):
                link.mav.command_int_send(
                    1, 1, 3, code, 0, 0, p1, p2, p3, 0., 0, 0, z)
            bank = transmission.get('bank')
            if bank is not None:
                link.mav.set_attitude_target_send(
                    0, 1, 1, 254,
                    [math.cos(bank/2.), math.sin(bank/2.), 0., 0.],
                    0., 0., 0., 0.)
            return

        commands = []
        for code, p1, p2, p3, z in (
                (43000, 0., speed, 0., 0.),
                (186, altitude, 3., 0., 0.),
                (43002, 0., course, accel, 0.)):
            commands.append(link.mav.command_int_encode(
                1, 1, 3, code, 0, 0, p1, p2, p3, 0., 0, 0, z))
        bank = transmission.get('bank')
        if bank is not None:
            commands.append(link.mav.set_attitude_target_encode(
                0, 1, 1, 254,
                [math.cos(bank/2.), math.sin(bank/2.), 0., 0.],
                0., 0., 0., 0.))
        packets = []
        for message in commands:
            packets.append(message.pack(link.mav))
            link.mav.seq = (link.mav.seq + 1) % 256
            link.mav.total_packets_sent += 1
            link.mav.total_bytes_sent += len(packets[-1])
        batch_port.send_batch(packets)
        return
    raise ValueError('unknown normalized transmission')


def _worker(descriptor, receive, ready, errors, interval, cpu):
    # The worker retains an acyclic packet journal until shutdown.  Disabling
    # cyclic GC prevents synchronized stop-the-world collections in otherwise
    # independent per-vehicle stream processes.
    gc.disable()
    link = journal = stream = None
    status_path = Path(descriptor['directory'])/'command_stream_status.json'
    status = {
        'schema_version': 1,
        'status': 'STARTING',
        'interval_s': interval,
        'tcp_port': descriptor['tcp_port'],
        'cpu': cpu,
        'sends': 0,
    }
    try:
        if cpu is not None:
            os.sched_setaffinity(0, {int(cpu)})
        deadline = time.monotonic()+20.
        while link is None:
            try:
                link = mavutil.mavlink_connection(
                    'tcp:127.0.0.1:%d' % descriptor['tcp_port'],
                    source_system=252, source_component=descriptor['member']+1)
            except OSError:
                if time.monotonic() >= deadline:
                    raise TimeoutError('isolated command-stream connection')
                time.sleep(.05)
        stream = (Path(descriptor['directory'])/'command_stream_packets.jsonl').open(
            'x', buffering=4*1024*1024)
        context = dict(descriptor['initial_context'])
        journal = JournaledSocket(
            link.port, stream, lambda: context, origin=descriptor['origin'])
        link.port = journal
        latest = None
        stopped = False
        next_send = time.monotonic()+interval
        status['status'] = 'READY'
        ready.set()
        while not stopped:
            wait = max(0., min(interval, next_send-time.monotonic()))
            if receive.poll(wait):
                message = receive.recv()
                while receive.poll():
                    message = receive.recv()
                if message.get('event') == 'stop':
                    stopped = True
                    continue
                latest = message['transmission']
                context = message['context']
            now = time.monotonic()
            if latest is None or now < next_send:
                continue
            send_normalized(link, descriptor, latest)
            status['sends'] += 1
            next_send += interval
            if next_send < now-interval:
                next_send = now+interval
        status['status'] = 'STOPPED'
    except (BaseException, EOFError) as exc:
        status.update(status='FAILED', error=repr(exc))
        try:
            errors.put_nowait(repr(exc))
        except Exception:
            pass
    finally:
        ready.set()
        try:
            if link is not None:
                link.close()
            elif journal is not None:
                journal.close()
        except Exception as exc:
            status.setdefault('close_error', repr(exc))
        try:
            if stream is not None and not stream.closed:
                stream.close()
        except Exception as exc:
            status.setdefault('stream_close_error', repr(exc))
        status_path.write_text(json.dumps(status, indent=2)+'\n')


class ProcessCommandStreamer:
    """One GIL-independent cached-command process per SITL vehicle."""
    def __init__(self, vehicles, interval=.045, context=None):
        self.vehicles = list(vehicles)
        self.interval = float(interval)
        self.context = context or multiprocessing.get_context('spawn')
        self.processes = {}
        self.senders = {}
        self.errors = {}
        self.ready = {}

    def start(self):
        cpu_text = os.environ.get('UAV_STREAM_CPU')
        cpu = None if cpu_text is None else int(cpu_text)
        for member, vehicle in enumerate(self.vehicles):
            receive, send = self.context.Pipe(duplex=False)
            ready = self.context.Event()
            errors = self.context.Queue()
            descriptor = vehicle.command_stream_descriptor(member)
            process = self.context.Process(
                target=_worker,
                args=(descriptor, receive, ready, errors, self.interval, cpu),
                name='command-stream-%d' % member, daemon=True)
            process.start()
            receive.close()
            self.processes[member] = process
            self.senders[member] = send
            self.errors[member] = errors
            self.ready[member] = ready
        deadline = time.monotonic()+25.
        for member, ready in self.ready.items():
            if not ready.wait(max(0., deadline-time.monotonic())):
                raise TimeoutError('command streamer %d startup' % member)
        self.check()

    def update(self, member, transmission, context):
        self.check()
        self.senders[member].send({
            'event': 'update',
            'transmission': normalize_transmission(transmission),
            'context': context,
        })

    def check(self):
        for member, errors in self.errors.items():
            try:
                error = errors.get_nowait()
            except queue.Empty:
                error = None
            if error is not None:
                raise RuntimeError('command streamer %d failed: %s'
                                   % (member, error))
        for member, process in self.processes.items():
            if process.exitcode is not None and process.exitcode != 0:
                raise RuntimeError('command streamer %d exited: %s'
                                   % (member, process.exitcode))

    def stop(self):
        for member, sender in self.senders.items():
            try:
                sender.send({'event': 'stop'})
            except (BrokenPipeError, EOFError):
                pass
        for process in self.processes.values():
            process.join(timeout=5.)
            if process.is_alive():
                process.terminate()
                process.join(timeout=5.)
        self.check()
        for sender in self.senders.values():
            sender.close()
