import json

from scripts.archive_v2_calibration_run import archive_run


def test_verified_archive_replaces_only_prunable_raw_files(tmp_path):
    run = tmp_path / 'run'; run.mkdir()
    (run / 'result.json').write_text('{}')
    (run / 'telemetry.csv').write_text('a\n1\n')
    logs = run / 'logs'; logs.mkdir(); (logs / 'one.BIN').write_bytes(b'raw')
    analysis = run / 'v2_analysis'; analysis.mkdir(); (analysis / 'analysis.json').write_text('{}')
    result = archive_run(run)
    assert (run / 'raw_evidence.tar.gz').is_file()
    assert not (run / 'telemetry.csv').exists()
    assert not logs.exists()
    assert (analysis / 'analysis.json').is_file()
    assert json.loads((run / 'raw_archive_manifest.json').read_text()) == result
    assert archive_run(run) == result
