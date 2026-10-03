#!/usr/bin/env python3
"""Audit calibration reference/state alignment and command-update timing.

The calibration CSV stores the latest transmitted target beside each telemetry
sample.  That convenience copy is not a synchronous reference measurement: a
new vehicle state can arrive immediately before the next target is generated.
This audit reconstructs reference samples from the outgoing packet journal and
compares them with position data on the same vehicle boot clock.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
import csv
import io
import json
import math
from pathlib import Path
import statistics
import sys
import tarfile
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.audit_outgoing_packets import read_journal


QUALIFICATION_PREFIXES = (
    "turn_coupled_accel_",
    "turn_coupled_high_",
    "turn_coupled_decel_",
    "turn_coupled_final_",
)


def _raw_bytes(root: Path, name: str) -> bytes:
    direct = root / name
    if direct.is_file():
        return direct.read_bytes()
    archive = root / "raw_evidence.tar.gz"
    if not archive.is_file():
        raise FileNotFoundError(f"missing {name} and raw_evidence.tar.gz in {root}")
    with tarfile.open(archive, "r:gz") as stream:
        member = stream.getmember(name)
        extracted = stream.extractfile(member)
        if extracted is None:
            raise FileNotFoundError(f"archive member is not a file: {name}")
        return extracted.read()


def _packets(root: Path):
    direct = root / "outgoing_packets.jsonl"
    if direct.is_file():
        return read_journal(direct)
    payload = _raw_bytes(root, "outgoing_packets.jsonl")
    with tempfile.NamedTemporaryFile() as stream:
        stream.write(payload)
        stream.flush()
        return read_journal(stream.name)


def _telemetry(root: Path):
    text = _raw_bytes(root, "telemetry.csv").decode("utf-8")
    return list(csv.DictReader(io.StringIO(text)))


def _number(value):
    if value in (None, ""):
        return None
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("nonfinite numeric evidence")
    return result


def _is_qualification_phase(phase: str) -> bool:
    return phase.startswith(QUALIFICATION_PREFIXES)


def _turn(phase: str) -> str:
    if phase.endswith("_-1"):
        return "left"
    if phase.endswith("_1"):
        return "right"
    raise ValueError(f"qualification phase has no turn sign: {phase}")


def _describe(values):
    values = list(values)
    if not values:
        return {"samples": 0}
    ordered = sorted(values)
    index = max(0, math.ceil(0.99 * len(ordered)) - 1)
    return {
        "samples": len(values),
        "minimum": min(values),
        "maximum": max(values),
        "mean": statistics.mean(values),
        "p99": ordered[index],
        "rms": math.sqrt(statistics.mean(value * value for value in values)),
    }


def _actual_series(rows):
    # The frozen duplicate policy retains the last sample at a source stamp.
    samples = {}
    for row in rows:
        stamp = _number(row.get("velocity_sim_s"))
        east = _number(row.get("east"))
        north = _number(row.get("north"))
        if stamp is not None and east is not None and north is not None:
            samples[stamp] = (east, north)
    if len(samples) < 2:
        raise ValueError("insufficient position telemetry")
    times = sorted(samples)
    return times, [samples[stamp] for stamp in times]


def _interpolate(times, values, stamp, maximum_gap_s):
    index = bisect_right(times, stamp)
    if index and times[index - 1] == stamp:
        return values[index - 1]
    if index == 0 or index == len(times):
        return None
    lower, upper = times[index - 1], times[index]
    if upper - lower > maximum_gap_s + 1e-12:
        return None
    fraction = (stamp - lower) / (upper - lower)
    return tuple(a + fraction * (b - a) for a, b in zip(values[index - 1], values[index]))


def audit(root, maximum_gap_s=0.10, position_gate_m=4.0):
    root = Path(root).resolve()
    packets, journal = _packets(root)
    telemetry = _telemetry(root)
    references = []
    for packet in packets:
        if (packet.get("kind") != "local_target" or not packet["complete_local_write"]
                or not _is_qualification_phase(packet["phase"])):
            continue
        context = packet["context"]
        clock = context.get("clocks", {}).get("LOCAL_POSITION_NED", {}).get("boot_s")
        target = context.get("target", {})
        east, north = target.get("ref_east"), target.get("ref_north")
        if clock is None or east is None or north is None:
            continue
        references.append({
            "boot_s": float(clock),
            "wall_s": float(packet["wall_s"]),
            "phase": packet["phase"],
            "turn": _turn(packet["phase"]),
            "east": float(east),
            "north": float(north),
            "send_index": packet["send_index"],
        })
    if not references:
        raise ValueError("no multicopter qualification references in packet journal")

    # Multiple packets can share the latest observed source stamp.  Keep the
    # last command at that stamp, matching the frozen duplicate policy.
    unique = {}
    for reference in references:
        unique[(reference["turn"], reference["boot_s"])] = reference
    references = sorted(unique.values(), key=lambda item: (item["turn"], item["boot_s"]))

    actual_times, actual_positions = _actual_series(telemetry)
    aligned = []
    for reference in references:
        actual = _interpolate(actual_times, actual_positions, reference["boot_s"], maximum_gap_s)
        if actual is None:
            continue
        error = math.hypot(actual[0] - reference["east"], actual[1] - reference["north"])
        aligned.append({**reference, "actual_east": actual[0], "actual_north": actual[1],
                        "position_error_m": error})

    command_gaps = []
    for turn in ("left", "right"):
        selected = [item for item in references if item["turn"] == turn]
        for previous, current in zip(selected, selected[1:]):
            gap = current["boot_s"] - previous["boot_s"]
            if gap > maximum_gap_s + 1e-12:
                command_gaps.append({
                    "turn": turn,
                    "start_boot_s": previous["boot_s"],
                    "end_boot_s": current["boot_s"],
                    "gap_s": gap,
                    "wall_gap_s": current["wall_s"] - previous["wall_s"],
                    "start_send_index": previous["send_index"],
                    "end_send_index": current["send_index"],
                })

    packet_walls = [packet["wall_s"] for packet in packets
                    if packet.get("kind") == "local_target" and packet["complete_local_write"]]
    packet_items = [packet for packet in packets
                    if packet.get("kind") == "local_target" and packet["complete_local_write"]]
    copied_errors = []
    stale_rows = []
    for row in telemetry:
        if not _is_qualification_phase(row.get("phase", "")):
            continue
        wall = _number(row.get("wall_s"))
        stamp = _number(row.get("velocity_sim_s"))
        east, north = _number(row.get("east")), _number(row.get("north"))
        ref_east, ref_north = _number(row.get("ref_east")), _number(row.get("ref_north"))
        if None in (wall, stamp, east, north, ref_east, ref_north):
            continue
        copied_error = math.hypot(east - ref_east, north - ref_north)
        copied_errors.append(copied_error)
        index = bisect_right(packet_walls, wall) - 1
        if index < 0:
            continue
        command_stamp = (packet_items[index]["context"].get("clocks", {})
                         .get("LOCAL_POSITION_NED", {}).get("boot_s"))
        if command_stamp is None:
            continue
        age = stamp - float(command_stamp)
        if age > maximum_gap_s + 1e-12:
            stale_rows.append({
                "wall_s": wall,
                "boot_s": stamp,
                "phase": row["phase"],
                "reference_age_s": age,
                "copied_position_error_m": copied_error,
                "preceding_send_index": packet_items[index]["send_index"],
            })

    by_turn = {}
    for turn in ("left", "right"):
        errors = [item["position_error_m"] for item in aligned if item["turn"] == turn]
        by_turn[turn] = _describe(errors)
        if errors:
            by_turn[turn]["maximum_gate_m"] = position_gate_m
            by_turn[turn]["passed"] = max(errors) <= position_gate_m

    legacy_peak = max(copied_errors, default=None)
    aligned_peak = max((item["position_error_m"] for item in aligned), default=None)
    peak_stale = bool(stale_rows and legacy_peak is not None and any(
        abs(item["copied_position_error_m"] - legacy_peak) <= 1e-9 for item in stale_rows))
    return {
        "schema_version": 1,
        "run": str(root),
        "scope": ("Reference/state alignment audit. Associated receive clocks bound packet timing; "
                  "they are not exact flight-controller command-arrival timestamps."),
        "maximum_native_gap_s": maximum_gap_s,
        "position_gate_m": position_gate_m,
        "journal_evidence_consistent": journal["evidence_consistent"],
        "reference_samples": len(references),
        "aligned_samples": len(aligned),
        "legacy_copied_reference": {
            **_describe(copied_errors),
            "peak_uses_stale_reference": peak_stale,
        },
        "time_aligned_reference": {
            "all": _describe(item["position_error_m"] for item in aligned),
            "turns": by_turn,
            "maximum_gate_m": position_gate_m,
            "passed": aligned_peak is not None and aligned_peak <= position_gate_m,
        },
        "stale_copied_reference_rows": stale_rows,
        "command_source_clock_gaps": command_gaps,
        "decision": {
            "legacy_tracking_failure_valid": not peak_stale,
            "time_aligned_tracking_passed": aligned_peak is not None and aligned_peak <= position_gate_m,
            "timing_conformance_passed": not command_gaps,
            "classification": ("INVALID_EVIDENCE" if peak_stale else
                               "TRACKING_FAILED" if aligned_peak is None or aligned_peak > position_gate_m else
                               "SYSTEM_QUALIFIED" if not command_gaps else "TIMING_NONCONFORMITY"),
        },
        "aligned_rows": aligned,
    }


def write_outputs(report, output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    aligned = report.pop("aligned_rows")
    (output / "audit.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    if aligned:
        with (output / "aligned_position.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(aligned[0]))
            writer.writeheader()
            writer.writerows(aligned)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--measurement", type=Path,
                        default=ROOT / "config/v2/measurement_definition_v2.yaml")
    parser.add_argument("--position-gate", type=float, default=4.0)
    args = parser.parse_args()
    measurement = yaml.safe_load(args.measurement.read_text())
    gap = float(measurement["processing"]["maximum_native_gap_s"])
    report = write_outputs(audit(args.run, gap, args.position_gate), args.output)
    print(json.dumps(report["decision"], indent=2))


if __name__ == "__main__":
    main()
