#!/usr/bin/env python3
"""Start an isolated local SITL model and save ground-only parameter evidence.

There is deliberately no remote endpoint option and no arming command.
Run with the Python installation that provides pymavlink.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import socket
import subprocess
import time

from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vehicle', choices=['copter', 'plane'], required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--defaults')
    parser.add_argument('--output', required=True)
    parser.add_argument('--port', type=int, default=19680)
    args = parser.parse_args()
    directory = Path(args.output).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    for port in (args.port, args.port+2, args.port+3):
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', port))
    binary = Path.home() / 'ardupilot/build/sitl/bin' / ('ardu'+args.vehicle)
    command = [str(binary), '--model', args.model, '--speedup', '1', '--base-port', str(args.port),
               '-I', '0', '--sysid', '1', '--home', '30.893,120.087,10,90',
               '--serial0', 'tcp:%d' % args.port, '--serial1', 'tcp:%d' % (args.port+2),
               '--serial2', 'tcp:%d' % (args.port+3)]
    if args.defaults:
        command += ['--defaults', str(Path(args.defaults).resolve())]
    (directory/'command.txt').write_text(shlex.join(command)+'\n')
    report = dict(command=command, flight_validated=False, evidence='config_only',
                  binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), parameters={})
    connection = None
    process = None
    try:
        with (directory/'sitl.log').open('w') as stream:
            process = subprocess.Popen(command, cwd=directory, stdout=stream, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+20
            while connection is None:
                if process.poll() is not None:
                    raise RuntimeError('SITL process exited')
                try:
                    connection = mavutil.mavlink_connection('tcp:127.0.0.1:%d' % args.port, source_system=255)
                except OSError:
                    if time.monotonic() > deadline:
                        raise TimeoutError('SITL startup')
                    time.sleep(.2)
            connection.setup_logfile(str(directory/'ground.tlog'))
            heartbeat = connection.wait_heartbeat(timeout=15)
            if heartbeat is None:
                raise TimeoutError('heartbeat')
            report['armed'] = bool(heartbeat.base_mode & 128)
            if report['armed']:
                raise RuntimeError('unexpected armed state')
            names = ['SIM_WIND_SPD','SIM_WIND_TURB','FRAME_CLASS','FRAME_TYPE','WP_SPD',
                     'ATC_ANGLE_MAX','SIM_FRM_MASS','SIM_FRM_REF_SPD','SIM_FRM_REF_ANG'] if args.vehicle=='copter' else [
                     'AIRSPEED_MIN','AIRSPEED_MAX','ROLL_LIMIT_DEG','SIM_WIND_SPD','SIM_WIND_TURB',
                     'ARSPD_USE','STALL_PREVENTION']
            for name in names:
                value = None
                for attempt in range(3):
                    connection.mav.param_request_read_send(1,1,name.encode(),-1)
                    deadline = time.monotonic()+1
                    while time.monotonic() < deadline:
                        msg = connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=.2)
                        if msg is not None and msg.param_id == name:
                            value = msg.param_value
                            break
                    if value is not None:
                        break
                report['parameters'][name] = value
            report['status'] = 'GROUND_ONLY'
    except Exception as exc:
        report.update(status='FAILED', error=repr(exc))
    finally:
        if connection is not None:
            if connection.logfile:
                connection.logfile.close()
            connection.close()
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        (directory/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report['status']=='GROUND_ONLY' else 1


if __name__=='__main__':
    raise SystemExit(main())
