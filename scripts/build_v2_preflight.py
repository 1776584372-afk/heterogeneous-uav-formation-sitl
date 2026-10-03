#!/usr/bin/env python3
"""Build reproducible v2 environment, clock, and command-consistency reports."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import scipy
import yaml

from scripts.audit_motion_contract import ppva_residual, stats, unique_samples
from scripts.audit_outgoing_packets import read_journal


ROOT = Path(__file__).resolve().parents[1]
ARDUPILOT = Path('/home/lzzy/ardupilot')


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def command(*args, cwd=None):
    return subprocess.check_output(args, cwd=cwd, text=True).strip()


def finite_column(rows, name):
    values = np.asarray([float(row[name]) for row in rows], float)
    if not np.all(np.isfinite(values)):
        raise ValueError(f'nonfinite {name}')
    return values


def sample_clock(values):
    values = np.asarray(values, float)
    if len(values) < 3 or not np.all(np.isfinite(values)):
        raise ValueError('clock needs at least three finite samples')
    delta = np.diff(values)
    unique = np.unique(values)
    unique_delta = np.diff(unique)
    return {
        'samples': int(len(values)),
        'unique_samples': int(len(unique)),
        'duplicates': int(len(values) - len(unique)),
        'backward_steps': int(np.count_nonzero(delta < 0)),
        'span_s': float(values[-1] - values[0]),
        'median_unique_period_s': float(np.median(unique_delta)) if len(unique_delta) else None,
        'p99_unique_period_s': float(np.percentile(unique_delta, 99)) if len(unique_delta) else None,
        'maximum_unique_period_s': float(np.max(unique_delta)) if len(unique_delta) else None,
    }


def affine_clock(source, target, expected_slope=1.0):
    source, target = np.asarray(source, float), np.asarray(target, float)
    if len(source) != len(target) or len(source) < 3:
        raise ValueError('clock pairs required')
    coefficients = np.polyfit(source - source[0], target - target[0], 1)
    estimate = target[0] + np.polyval(coefficients, source - source[0])
    residual = target - estimate
    return {
        'target_per_source_slope': float(coefficients[0]),
        'expected_slope': float(expected_slope),
        'relative_rate_error_ppm': float((coefficients[0] / expected_slope - 1.0) * 1e6),
        'residual_rms_s': float(np.sqrt(np.mean(residual ** 2))),
        'residual_p99_abs_s': float(np.percentile(np.abs(residual), 99)),
        'residual_max_abs_s': float(np.max(np.abs(residual))),
        'interpretation': 'receive-side mapping only; not an aircraft command-arrival timestamp',
    }


def read_rows(vehicle):
    with (vehicle / 'telemetry.csv').open() as stream:
        return list(csv.DictReader(stream))


def time_audit(vehicle):
    rows = read_rows(vehicle)
    manifest = json.loads((vehicle / 'manifest.json').read_text())
    expected_speedup = float(manifest['speedup'])
    columns = {
        'wall_s': finite_column(rows, 'wall_s'),
        'attitude_boot_s': finite_column(rows, 'sim_s'),
        'velocity_boot_s': finite_column(rows, 'velocity_sim_s'),
        'airspeed_associated_boot_s': finite_column(rows, 'airspeed_sim_s'),
    }
    result = {'clocks': {name: sample_clock(value) for name, value in columns.items()}}
    result['wall_to_attitude'] = affine_clock(
        columns['wall_s'], columns['attitude_boot_s'], expected_speedup)
    for name in ('velocity_boot_s', 'airspeed_associated_boot_s'):
        offset = columns[name] - columns['attitude_boot_s']
        result[name + '_minus_attitude'] = {
            'minimum_s': float(offset.min()), 'maximum_s': float(offset.max()),
            'mean_s': float(offset.mean()), 'p99_abs_s': float(np.percentile(abs(offset), 99)),
        }
    journal = vehicle / 'outgoing_packets.jsonl'
    if journal.is_file():
        packets, packet_summary = read_journal(journal)
    else:
        packets, packet_summary = [], {
            'unavailable': 'historical run has no outgoing_packets.jsonl',
            'evidence_consistent': None,
        }
    context_pairs = []
    for packet in packets:
        clocks = packet.get('context', {}).get('clocks', {})
        available = [clock.get('boot_s') for clock in clocks.values()
                     if clock.get('boot_s') is not None]
        if available:
            context_pairs.append((packet['wall_s'], max(available)))
    result['packet_journal'] = packet_summary
    result['packet_context_wall_to_boot'] = (affine_clock(
        *zip(*context_pairs), expected_slope=expected_speedup)
        if len(context_pairs) >= 3 else {'unavailable': 'fewer than three packet context pairs'})
    result['limitations'] = [
        'VFR_HUD has no native onboard timestamp; its clock is associated with the latest ATTITUDE received.',
        'Packet journal proves local socket acceptance, not flight-controller receipt or application.',
        'Each vehicle clock is audited independently; fleet receive-time synchronization is a separate metric.',
    ]
    return result, packets, rows


def local_command_audit(vehicle, packets):
    selected = []
    semantics = {'packets': 0, 'ppva_packets': 0, 'yaw_packets': 0, 'yaw_rate_packets': 0,
                 'frames': {}, 'masks': {}}
    for packet in packets:
        if packet.get('kind') != 'local_target' or not packet['complete_local_write']:
            continue
        semantics['packets'] += 1
        fields = packet['fields']
        frame, mask = int(fields['coordinate_frame']), int(fields['type_mask'])
        semantics['frames'][str(frame)] = semantics['frames'].get(str(frame), 0) + 1
        semantics['masks'][str(mask)] = semantics['masks'].get(str(mask), 0) + 1
        has_ppva = not mask & (1 | 2 | 8 | 16 | 64 | 128)
        semantics['ppva_packets'] += int(has_ppva)
        semantics['yaw_packets'] += int(not mask & 1024)
        semantics['yaw_rate_packets'] += int(not mask & 2048)
        calibration_phase = packet['phase'].startswith(
            ('accel_', 'decel_', 'settle_step_', 'steady_step_', 'turn_'))
        if (packet['phase'] != 'track' and not calibration_phase) or not has_ppva:
            continue
        clocks = packet['context']['clocks']
        available = [value['boot_s'] for value in clocks.values() if value['boot_s'] is not None]
        if not available:
            continue
        selected.append([max(available), fields['y'], fields['x'], fields['vy'], fields['vx'],
                         fields['afy'], fields['afx'], fields['yaw'], fields['yaw_rate']])
    result = {'semantics': semantics, 'formal_qualification': False}
    if len(selected) < 3:
        result['kinematic_consistency'] = {'unavailable': 'fewer than three track PPVA packets'}
        return result
    selected = np.asarray(selected, float)
    times, data = unique_samples(selected[:, 0], selected[:, 1:])
    defects = ppva_residual(times, data[:, :2], data[:, 2:4], data[:, 4:6])
    yaw_rate_from_yaw = np.gradient(np.unwrap(data[:, 6]), times)
    yaw_rate_error = yaw_rate_from_yaw - data[:, 7]
    result['kinematic_consistency'] = {
        key: stats(value) for key, value in defects.items()
    }
    result['yaw_consistency'] = {
        'yaw_derivative_minus_sent_yaw_rate_rad_s': stats(yaw_rate_error),
        'note': 'sampled consistency on packet context boot time; not a continuous-time guarantee',
    }
    return result


def plane_command_audit(packets):
    groups = {}
    for packet in packets:
        kind = packet.get('kind')
        if kind not in ('airspeed', 'altitude', 'course') or not packet['complete_local_write']:
            continue
        fields = packet['fields']
        value = fields['param2'] if kind in ('airspeed', 'course') else fields['param1']
        clocks = packet.get('context', {}).get('clocks', {})
        available = [clock.get('boot_s') for clock in clocks.values()
                     if clock.get('boot_s') is not None]
        boot_s = max(available) if available else None
        groups.setdefault(kind, []).append(
            (packet['wall_s'], boot_s, float(value), int(fields['command'])))
    result = {'formal_qualification': False, 'semantics': {
        'airspeed': 'COMMAND_INT 43000 param2, external equivalent-airspeed target in m/s',
        'altitude': 'MAV_CMD_GUIDED_CHANGE_ALTITUDE/186 param1, relative altitude in m',
        'course': 'COMMAND_INT 43002 param2, ground-course bearing in degrees',
    }}
    for kind, samples in groups.items():
        wall_times = np.asarray([sample[0] for sample in samples])
        values = np.asarray([sample[2] for sample in samples])
        boot_samples = [(sample[1], sample[2]) for sample in samples if sample[1] is not None]
        boot_slope = None
        unique_boot_count = 0
        if len(boot_samples) >= 3:
            boot_time, boot_value = unique_samples(
                [sample[0] for sample in boot_samples],
                [sample[1] for sample in boot_samples])
            unique_boot_count = len(boot_time)
            boot_slope = float(np.max(np.abs(np.gradient(boot_value, boot_time))))
        result[kind] = {
            'count': len(samples), 'minimum': float(values.min()), 'maximum': float(values.max()),
            'maximum_wall_gap_s': float(np.max(np.diff(wall_times))) if len(wall_times) > 1 else None,
            'unique_boot_samples': unique_boot_count,
            'maximum_target_slope_per_boot_s': boot_slope,
        }
    result['limitation'] = 'command values and local socket acceptance do not establish TECS application or aircraft response'
    return result


def environment_manifest(copter, plane):
    tracked = [
        *sorted((ROOT / 'config/v2').glob('*.yaml')),
        ROOT / 'uav_formation/measurement_v2.py',
        ROOT / 'uav_formation/sitl_vehicle.py',
        ROOT / 'scripts/run_paper_sitl.py',
        ROOT / 'scripts/calibrate_sitl.py',
        ROOT / 'scripts/calibrate_turns.py',
        ROOT / 'scripts/calibrate_coupled_turns.py',
        ROOT / 'scripts/analyze_v2_bin.py',
        ROOT / 'scripts/diagnose_v2_transients.py',
        ROOT / 'scripts/archive_v2_calibration_run.py',
        ROOT / 'scripts/reanalyse_v2_archive.py',
        ROOT / 'scripts/summarize_v2_calibration.py',
        ROOT / 'scripts/run_v2_calibration_batch.py',
        ROOT / 'scripts/build_v2_evidence_index.py',
        Path(__file__).resolve(),
    ]
    run_manifests = {name: json.loads((path / 'manifest.json').read_text())
                     for name, path in (('copter_evidence', copter), ('plane_evidence', plane))}
    artifacts = {}
    for name, manifest in run_manifests.items():
        artifacts[name] = {key: manifest.get(key) for key in (
            'vehicle', 'model', 'altitude', 'speedup', 'ardupilot_commit', 'binary_sha256',
            'defaults_source', 'defaults_sources')}
    return {
        'schema_version': 1, 'document_id': 'environment_manifest_v2',
        'status': 'PREFLIGHT_SNAPSHOT_NOT_CAPABILITY_FREEZE', 'created_at': '2026-09-27',
        'host': {'platform': platform.platform(), 'machine': platform.machine(),
                 'python': sys.version, 'numpy': np.__version__, 'scipy': scipy.__version__,
                 'yaml': yaml.__version__},
        'repository': {'path': str(ROOT), 'commit': command('git', 'rev-parse', 'HEAD', cwd=ROOT),
                       'status': command('git', 'status', '--short', cwd=ROOT)},
        'ardupilot': {'path': str(ARDUPILOT),
                      'commit': command('git', 'rev-parse', 'HEAD', cwd=ARDUPILOT)},
        'evidence_platforms': artifacts,
        'file_sha256': {str(path.relative_to(ROOT)): sha256(path) for path in tracked},
        'warning': 'Dirty/untracked source requires the file hashes and snapshots; Git commit alone is insufficient.',
    }


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--copter-vehicle', required=True, type=Path)
    parser.add_argument('--plane-vehicle', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    copter, plane = args.copter_vehicle.resolve(), args.plane_vehicle.resolve()
    environment = environment_manifest(copter, plane)
    time_reports, command_reports = {}, {}
    for name, vehicle in (('multicopter', copter), ('fixedwing', plane)):
        time_report, packets, _ = time_audit(vehicle)
        time_reports[name] = time_report
        command_reports[name] = (local_command_audit(vehicle, packets) if name == 'multicopter'
                                 else plane_command_audit(packets))
    write_json(output / 'environment_manifest_v2.json', environment)
    write_json(output / 'time_alignment_report.json', time_reports)
    write_json(output / 'command_consistency_report.json', command_reports)
    snapshot = output / 'protocol_snapshot'; snapshot.mkdir()
    for path in (ROOT / 'config/v2').glob('*.yaml'):
        shutil.copy2(path, snapshot / path.name)
    report = [
        '# v2 P0预检报告', '', '日期：2026-09-27。此报告不冻结能力数值，也不构成飞行资格。', '',
        '## 结果', '',
        '- 已保存当前环境、协议和分析代码哈希。当前工作树包含未提交/未跟踪文件，因此不能只用Git提交号定义版本。',
        '- 已分别量化多旋翼和固定翼遥测时钟、重复时间戳、采样间隔及wall/boot映射残差。',
        '- 多旋翼审计P/V/A与yaw/yaw-rate采样一致性；固定翼审计空速、航迹和高度命令语义及发送节奏。',
        '- socket日志只证明本地完整发送；飞控内部接收和执行仍须结合DataFlash OFG/TECS/ATT/RATE和SIM/SIM2。', '',
        '## 输出', '',
        '- `environment_manifest_v2.json`',
        '- `time_alignment_report.json`',
        '- `command_consistency_report.json`',
        '- `protocol_snapshot/`', '',
        '下一步是使用本报告确认的数据源和处理规则，对A组单机/耦合工况进行预注册标定。',
    ]
    (output / 'README.md').write_text('\n'.join(report) + '\n')
    print(output)


if __name__ == '__main__':
    main()
