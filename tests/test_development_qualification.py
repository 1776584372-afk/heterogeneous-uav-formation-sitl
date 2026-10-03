import json
from unittest.mock import patch

from scripts.audit_tracking_run import audit


def test_development_run_cannot_be_formally_qualified(tmp_path):
    cfg=dict(capability_evidence='measured', acceptance=dict(tracking_rmse_m=3.,
        tracking_max_m=8., altitude_error_max_m=2., maximum_constraint_violation_time_fraction=0.))
    (tmp_path/'config.json').write_text(json.dumps(cfg))
    (tmp_path/'flight_status.json').write_text(json.dumps(dict(completed=True,
        full_route_completed=True, exit_completed=True)))
    metric=dict(valid_telemetry=True, members={'0':dict(tracking_rmse=.1,
        tracking_max=.5, altitude_error_max=.1, speed_min=12., speed_max=14.,
        bank_abs_max_deg=10., violation_time_fraction={})})
    for kind, expected in [('development_fleet', False), ('paper_fleet', True)]:
        (tmp_path/'run_manifest.json').write_text(json.dumps(dict(kind=kind)))
        with patch('scripts.audit_tracking_run.compute', return_value=metric):
            result=audit(tmp_path)
        assert result['run_kind']==kind
        assert 'audit_error' not in result
        assert result['formal_qualified'] is expected
    del metric['members']['0']['speed_min']
    with patch('scripts.audit_tracking_run.compute', return_value=metric):
        result=audit(tmp_path)
    assert 'audit_error' in result
    assert not result['formal_qualified']
    assert not result['tracking_qualified']
    assert not result['measured_constraints_passed']
