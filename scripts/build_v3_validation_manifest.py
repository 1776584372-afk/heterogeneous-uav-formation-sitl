#!/usr/bin/env python3
"""Build a fail-closed manifest for v3 independent-validation evidence."""
import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def platform_from_name(name):
    if name.startswith("fw_"):
        return "fixedwing"
    if name.startswith("mc_"):
        return "multicopter"
    return "unknown"


def classify_run(run):
    result_path = run / "result.json"
    if not result_path.is_file():
        return None
    result = json.loads(result_path.read_text())
    record = {
        "run": str(run.resolve()),
        "platform": platform_from_name(run.name),
        "result_status": result.get("status"),
        "result_sha256": sha256(result_path),
    }
    if result.get("failure_classification") == "INVALID_EVIDENCE":
        record.update(classification="INVALID_EVIDENCE",
                      reason=result.get("error", "online timing gate rejected evidence"))
        return record
    if result.get("status") != "COMPLETED":
        record.update(classification="FAILED",
                      reason=result.get("error", "run did not complete"))
        return record
    audit_path = run / "v3_timing_audit.json"
    if not audit_path.is_file():
        record.update(classification="INVALID_EVIDENCE",
                      reason="completed run has no post-run v3 timing audit")
        return record
    audit = json.loads(audit_path.read_text())
    record.update(
        timing_audit_sha256=sha256(audit_path),
        maximum_wall_gap_s=audit.get("maximum_wall_gap_s"),
        maximum_source_gap_s=audit.get("maximum_source_gap_s"),
    )
    if not audit.get("passed", False):
        record.update(classification="INVALID_EVIDENCE",
                      reason="post-run packet timing audit failed",
                      timing_violations=audit.get("violations", []))
        return record
    record.update(classification="VALID", reason="completed and post-run timing audit passed")
    return record


def build_manifest(roots, required_per_platform=5):
    records = []
    for root in roots:
        root = Path(root)
        if not root.is_dir():
            raise FileNotFoundError(root)
        for run in sorted(path for path in root.iterdir() if path.is_dir()):
            record = classify_run(run)
            if record is not None:
                records.append(record)
    valid = [record for record in records if record["classification"] == "VALID"]
    invalid = [record for record in records if record["classification"] == "INVALID_EVIDENCE"]
    failed = [record for record in records if record["classification"] == "FAILED"]
    counts = {
        platform: sum(record["platform"] == platform for record in valid)
        for platform in ("fixedwing", "multicopter")
    }
    passed = all(counts[platform] >= required_per_platform
                 for platform in ("fixedwing", "multicopter")) and not failed
    return {
        "schema_version": 1,
        "decision_source": "result.json plus mandatory post-run v3_timing_audit.json",
        "policy": "completed online status is insufficient; post-run timing audit is fail-closed",
        "required_valid_repeats_per_platform": required_per_platform,
        "valid_counts": counts,
        "validation_passed": passed,
        "valid_runs": valid,
        "invalid_evidence": invalid,
        "failed_runs": failed,
        "roots": [str(Path(root).resolve()) for root in roots],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("roots", nargs="*", type=Path, default=[
        ROOT / "runs/v3_timing_requalification_20260928/group_B_revision_1",
        ROOT / "runs/v3_timing_requalification_20260928/group_B_revision_2",
        ROOT / "runs/v3_timing_requalification_20260928/group_B_revision_3",
        ROOT / "runs/v3_timing_requalification_20260928/group_B_revision_3_replacements",
    ])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--required-per-platform", type=int, default=5)
    args = parser.parse_args()
    manifest = build_manifest(args.roots, args.required_per_platform)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    print(json.dumps({key: manifest[key] for key in
                      ("validation_passed", "valid_counts")}, indent=2))
    return 0 if manifest["validation_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
