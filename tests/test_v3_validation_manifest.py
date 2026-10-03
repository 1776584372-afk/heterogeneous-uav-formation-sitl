import json

from scripts.build_v3_validation_manifest import build_manifest


def write_run(root, name, status="COMPLETED", audit_passed=True,
              failure_classification=None):
    run = root / name
    run.mkdir()
    result = {"status": status}
    if failure_classification:
        result.update(failure_classification=failure_classification, error="timing failure")
    (run / "result.json").write_text(json.dumps(result))
    if audit_passed is not None:
        audit = {"passed": audit_passed, "maximum_wall_gap_s": .1,
                 "maximum_source_gap_s": .1, "violations": [] if audit_passed else [{}]}
        (run / "v3_timing_audit.json").write_text(json.dumps(audit))
    return run


def test_post_run_audit_overrides_completed_online_status(tmp_path):
    write_run(tmp_path, "fw_rep00", audit_passed=False)
    manifest = build_manifest([tmp_path], required_per_platform=0)
    assert not manifest["validation_passed"] or not manifest["failed_runs"]
    assert manifest["invalid_evidence"][0]["classification"] == "INVALID_EVIDENCE"
    assert manifest["valid_counts"]["fixedwing"] == 0


def test_missing_post_run_audit_is_invalid(tmp_path):
    write_run(tmp_path, "mc_rep00", audit_passed=None)
    manifest = build_manifest([tmp_path], required_per_platform=1)
    assert not manifest["validation_passed"]
    assert "no post-run" in manifest["invalid_evidence"][0]["reason"]


def test_valid_counts_are_platform_specific(tmp_path):
    write_run(tmp_path, "fw_rep00")
    write_run(tmp_path, "mc_rep00")
    manifest = build_manifest([tmp_path], required_per_platform=1)
    assert manifest["validation_passed"]
    assert manifest["valid_counts"] == {"fixedwing": 1, "multicopter": 1}
