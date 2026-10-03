#!/usr/bin/env python3
"""Quantify excursions against historical limits without selecting v2 limits."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from uav_formation.measurement_v2 import threshold_events


QUALIFICATION_PREFIXES = ("accel_", "decel_", "settle_", "steady_", "turn_")
HISTORICAL_LIMITS = {"copter": (-0.4, 0.4), "plane": (-0.2, 0.2)}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_primary(path):
    with Path(path).open() as stream:
        rows = list(csv.DictReader(stream))
    return {
        "time": np.asarray([float(row["boot_s"]) for row in rows]),
        "value": np.asarray([float(row["derivative"]) for row in rows]),
        "valid": np.asarray([bool(int(row["valid"])) for row in rows]),
        "phase": np.asarray([row["phase"] for row in rows], object),
    }


def identify_platform(protocol, analysis_path):
    platform = protocol.get("platform")
    if platform in HISTORICAL_LIMITS:
        return platform
    controller = str(protocol.get("controller", "")).lower()
    if "arducopter" in controller:
        return "copter"
    if "arduplane" in controller:
        return "plane"
    analysis = json.loads(Path(analysis_path).read_text())
    if analysis.get("record_counts", {}).get("PSCE", 0):
        return "copter"
    if analysis.get("record_counts", {}).get("TECS", 0):
        return "plane"
    raise ValueError("platform cannot be established from protocol or DataFlash records")


def analyse_run(run):
    run = Path(run).resolve()
    result_path = run / "result.json"
    result = json.loads(result_path.read_text())
    protocol = result.get("protocol", {})
    analysis = run / "v2_analysis"
    analysis_path = analysis / "analysis.json"
    platform = identify_platform(protocol, analysis_path)
    signal = "sim2_ground_speed_m_s" if platform == "copter" else "sim2_airspeed_m_s"
    primary_path = analysis / f"{signal}_primary.csv"
    data = load_primary(primary_path)
    lower, upper = HISTORICAL_LIMITS[platform]
    phases = {}
    for phase in sorted(set(data["phase"])):
        if not phase.startswith(QUALIFICATION_PREFIXES):
            continue
        selected = data["phase"] == phase
        valid = data["valid"] & selected
        phase_values = data["value"][valid]
        if not len(phase_values):
            continue
        phases[phase] = {
            "minimum": float(np.min(phase_values)),
            "maximum": float(np.max(phase_values)),
            "lower_excursion": threshold_events(
                data["time"], data["value"], valid, lower=lower),
            "upper_excursion": threshold_events(
                data["time"], data["value"], valid, upper=upper),
        }
    return {
        "run": str(run), "platform": platform,
        "protocol_id": protocol.get("protocol_id"),
        "repeat_id": protocol.get("repeat_id"),
        "signal": signal, "primary_processing": "50 Hz, 0.5 s third-order Savitzky-Golay",
        "diagnostic_limit_source": "historical v1 planning value; not a v2 acceptance limit",
        "historical_lower": lower, "historical_upper": upper,
        "source_sha256": {str(result_path): sha256(result_path),
                          str(primary_path): sha256(primary_path)},
        "phases": phases,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    runs = [analyse_run(path) for path in args.runs]
    report = {
        "schema_version": 1,
        "scope": ("Diagnostic comparison with historical planning limits only. "
                  "This report neither freezes nor proposes v2 capability or acceptance limits."),
        "runs": runs,
        "script_sha256": sha256(Path(__file__)),
    }
    (output / "transient_diagnostics.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"runs": len(runs), "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
