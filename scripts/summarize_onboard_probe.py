#!/usr/bin/env python3
"""Summarize exported native controller signals without changing flight metrics."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from scripts.extract_onboard_control import file_hash
from uav_formation.paper_metrics import signal_derivative


def yaw_coordinate_decomposition(roll, pitch, yaw):
    """ZYX Euler yaw = quaternion z-twist + tilt offset, away from singularity.

    The local controller composes a shortest thrust rotation with a z rotation.
    This identity diagnoses coordinate coupling; neither term is a replacement
    for the actual Euler-yaw constraint or proof of the controller's cause.
    """
    roll, pitch, yaw = (np.asarray(v, dtype=float) for v in (roll, pitch, yaw))
    if (roll.shape != pitch.shape or roll.shape != yaw.shape or roll.ndim != 1
            or not all(np.all(np.isfinite(v)) for v in (roll, pitch, yaw))
            or np.any(abs(roll) >= np.pi/2) or np.any(abs(pitch) >= np.pi/2)):
        raise ValueError('matching finite upright Euler angle samples required')
    offset = 2*np.arctan2(np.sin(roll/2)*np.sin(pitch/2),
                         np.cos(roll/2)*np.cos(pitch/2))
    return np.unwrap(yaw)-offset, offset


def summarize(kind, rows):
    times = np.array([float(row['boot_s']) for row in rows])
    if len(times) < 3 or not np.all(np.isfinite(times)) or np.any(np.diff(times) < 0):
        raise ValueError('at least three finite ordered native samples required')
    result = dict(samples=len(rows), boot_window=[float(times[0]), float(times[-1])], signals={})

    def add(name, values):
        values = np.asarray(values, dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError('nonfinite native signal: '+name)
        lo, hi = int(np.argmin(values)), int(np.argmax(values))
        result['signals'][name] = dict(minimum=float(values[lo]), maximum=float(values[hi]),
            minimum_boot_s=float(times[lo]), maximum_boot_s=float(times[hi]))

    def col(name):
        return np.array([float(row[name]) for row in rows])

    if kind == 'ANG':
        for key, label in [('DesYaw', 'desired'), ('Yaw', 'estimated')]:
            add(label+'_euler_yaw_rate_rad_s', signal_derivative(np.unwrap(np.radians(col(key))), times))
        for field in ('DesRoll', 'Roll', 'DesPitch', 'Pitch'):
            add(field+'_deg', col(field))
        for prefix, label in [('Des', 'desired'), ('', 'estimated')]:
            angles = [np.radians(col(prefix+field)) for field in ('Roll', 'Pitch', 'Yaw')]
            twist, offset = yaw_coordinate_decomposition(*angles)
            add(label+'_z_twist_rate_rad_s', signal_derivative(twist, times))
            add(label+'_tilt_yaw_offset_rate_rad_s', signal_derivative(offset, times))
            add(label+'_tilt_yaw_offset_deg', np.degrees(offset))
    elif kind == 'RATE':
        for field in ('YDes', 'Y'):
            add(field+'_body_yaw_rate_rad_s', np.radians(col(field)))
        add('YOut_native', col('YOut'))
    elif kind == 'TECS':
        for field in ('spdem', 'sp'):
            add(field+'_true_airspeed_m_s', col(field))
            add(field+'_derivative_m_s2', signal_derivative(col(field), times))
        for field, unit in [('dsp','m_s2'), ('dspdem','m_s2'), ('h','m'), ('hin','m'),
                            ('hdem','m'), ('th','fraction'), ('ph','rad')]:
            add(field+'_'+unit, col(field))
    elif kind == 'OFG':
        # This revision writes target_airspeed_cm * 0.01, despite its cm comment.
        add('received_equivalent_airspeed_m_s', col('Arsp'))
        add('received_equivalent_airspeed_derivative_m_s2', signal_derivative(col('Arsp'), times))
        add('guided_slew_location_alt_field_m', col('Alt'))
    elif kind == 'CTUN':
        add('eas_to_tas_ratio', col('E2T'))
    else:
        raise ValueError('unsupported native message: '+kind)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = dict(source=str(args.directory.resolve()), formal_qualification=False,
        procedure='native boot-clock unique-sample derivatives, yaw unwrap, no smoothing; track and complete step phases only',
        caveats=['body yaw rate is not Euler yaw derivative',
                 'Euler-yaw decomposition into z-twist and tilt offset is a coordinate identity, not changed acceptance or causal proof',
                 'TECS sp/spdem are true airspeed; external command and VFR_HUD are not substituted',
                 'TECS dsp is an internal estimate, not the paper horizontal-speed derivative',
                 'OFG.Arsp units follow the frozen Log.cpp writer, not its inconsistent cm comment',
                 'OFG.Alt is the separate guided-slew location field; DO_CHANGE_ALTITUDE (186) is not recorded there, and -0.01 m may encode its -1 cm sentinel; use TECS hin/hdem for this backend',
                 'phase boundaries inherit as-of telemetry labels, not exact controller transition times'],
        messages={}, source_sha256={}, missing_types=[])
    for kind in ('ANG', 'RATE', 'TECS', 'OFG', 'CTUN'):
        path = args.directory/(kind+'.csv')
        if not path.is_file():
            result['missing_types'].append(kind)
            continue
        with path.open() as stream:
            rows = [r for r in csv.DictReader(stream)
                    if r['phase_asof'] == 'track' or r['phase_asof'].startswith('step_')]
        result['messages'][kind] = {}
        for phase in sorted({r['phase_asof'] for r in rows}):
            selected = [r for r in rows if r['phase_asof'] == phase]
            try:
                result['messages'][kind][phase] = summarize(kind, selected)
            except (ValueError, KeyError) as exc:
                result['messages'][kind][phase] = dict(unavailable=str(exc))
        result['source_sha256'][str(path.resolve())] = file_hash(path)
    result['analysis_sha256'] = {str(path.resolve()): file_hash(path) for path in
        (Path(__file__), Path(__file__).resolve().parents[1]/'uav_formation/paper_metrics.py')}
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(args.output)


if __name__ == '__main__':
    main()
