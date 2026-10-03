from pathlib import Path

from scripts.derive_v3_system_task import derive


ROOT = Path(__file__).resolve().parents[1]


def test_single_fixedwing_precheck_preserves_frozen_limits():
    cfg = derive(ROOT / 'config/experiments/v3/s2.yaml', 'fixedwing')
    assert len(cfg['members']) == 1
    assert cfg['members'][0]['r_y'] == 0.
    assert cfg['members'][0]['a_t_max'] == .05
    assert cfg['fixedwing_controller_limits']['a_t_max'] == .2
    assert not cfg['system_precheck']['formal_result']
