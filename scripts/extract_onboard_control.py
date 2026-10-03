#!/usr/bin/env python3
"""Export native controller logs, retaining their own boot clock and fields."""
import argparse
from bisect import bisect_right
import csv
import hashlib
import json
from pathlib import Path
import shutil

from pymavlink import mavutil


def file_hash(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def extract(root, output):
    root, output = Path(root).resolve(), Path(output).resolve()
    manifest = json.loads((root/'manifest.json').read_text())
    with (root/'telemetry.csv').open() as stream:
        telemetry = list(csv.DictReader(stream))
    stamps = [float(r['sim_s']) for r in telemetry]
    if not stamps or any(b < a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('missing or unordered telemetry boot timestamps')
    common = ['ATT', 'RATE', 'IMU', 'RCOU', 'SIM', 'SIM2', 'POS', 'XKF1']
    types = (common + ['TECS', 'ARSP', 'PIDR', 'PIDP', 'PIDY', 'CTUN', 'OFG']
             if manifest['vehicle'] == 'plane'
             else common + ['ANG', 'PIDR', 'PIDP', 'PIDY', 'CTUN', 'MOTB',
                            'PSCN', 'PSCE', 'PSCD'])
    bins = sorted((root/'logs').glob('*.BIN'))
    if not bins:
        raise ValueError('missing onboard logs')
    output.mkdir(parents=True, exist_ok=False)
    streams, writers, schemas, counts = {}, {}, {}, {}
    try:
        for path in bins:
            reader = mavutil.mavlink_connection(str(path))
            try:
                while True:
                    msg = reader.recv_match(type=types)
                    if msg is None:
                        break
                    kind = msg.get_type(); row = msg.to_dict()
                    stamp = row['TimeUS']/1e6
                    index = bisect_right(stamps, stamp)-1
                    row['boot_s'] = stamp
                    row['phase_asof'] = telemetry[index]['phase'] if 0 <= index and stamp <= stamps[-1] else 'unassigned'
                    row['phase_age_s'] = stamp-stamps[index] if row['phase_asof'] != 'unassigned' else None
                    row['source_bin'] = path.name
                    if kind not in streams:
                        streams[kind] = (output/(kind+'.csv')).open('x', newline='')
                        schemas[kind] = list(row)
                        writers[kind] = csv.DictWriter(streams[kind], fieldnames=schemas[kind])
                        writers[kind].writeheader(); counts[kind] = 0
                    if set(row) != set(schemas[kind]):
                        raise ValueError('onboard schema changed within run: '+kind)
                    writers[kind].writerow(row); counts[kind] += 1
            finally:
                reader.close()
    finally:
        for stream in streams.values():
            stream.close()
    source_root = Path.home()/'ardupilot'
    sources = [source_root/'libraries/AP_TECS/AP_TECS.cpp',
               source_root/'libraries/AC_AttitudeControl/LogStructure.h',
               source_root/'ArduPlane/Log.cpp', source_root/'ArduPlane/navigation.cpp',
               source_root/'ArduPlane/GCS_MAVLink_Plane.cpp', source_root/'ArduPlane/Plane.h']
    source_hashes = {}
    for path in sources:
        if path.is_file():
            shutil.copy2(path, output/path.name)
            source_hashes[str(path)] = file_hash(path)
    result = dict(schema_version=2, source=str(root), record_counts=counts,
        missing_types=sorted(set(types)-set(counts)), native_fields=schemas,
        time='native TimeUS/1e6; phase label from preceding telemetry ATTITUDE clock, not exact phase transition',
        meaning='raw native fields; TECS.spdem/sp are true airspeed, not ground speed or the external equivalent-airspeed command; TECS.dsp is its internal forward-acceleration estimate, not the paper horizontal-speed derivative',
        plane_fields='OFG.Arsp is m/s in this revision (writer multiplies target_airspeed_cm by 0.01, despite the cm comment); CTUN.E2T is EAS2TAS; OFG.Alt is the separate slew-command location, not command 186 altitude; native log presence is receiver-state evidence, not path-tracking qualification',
        limitations='native controller target/output does not certify a physical bound or isolate a causal fault',
        source_sha256={str(p): file_hash(p) for p in [*bins, root/'manifest.json', root/'telemetry.csv']},
        local_source_sha256=source_hashes, extractor_sha256=file_hash(Path(__file__)),
        ardupilot_commit_in_run=manifest['ardupilot_commit'], formal_qualification=False)
    (output/'index.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('vehicle_directory')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = extract(args.vehicle_directory, args.output)
    print(json.dumps({k: result[k] for k in ('record_counts', 'missing_types')}, indent=2))
