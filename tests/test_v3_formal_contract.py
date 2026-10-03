from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from uav_formation.experiment_config import load_config, fixedwing_controller_member


ROOT = Path(__file__).resolve().parents[1]


def source():
    return yaml.safe_load((ROOT / 'config/experiments/v3/s3.yaml').read_text())


def write(tmp_path, cfg):
    path = tmp_path / 'task.yaml'
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    return path


def test_v3_formal_task_matches_frozen_contract():
    cfg, members = load_config(ROOT / 'config/experiments/v3/s3.yaml')
    assert cfg['frozen_contract']
    assert len(members) == 3
    planning = members[2]
    controller = fixedwing_controller_member(cfg, planning)
    assert planning.bank_max < controller.bank_max
    assert planning.a_t_max == .05
    assert controller.a_t_max == .2
    assert cfg['multicopter_execution']['mode'] == 'measured_feedback_nmpc_consistent_correction'


def test_v3_formal_task_rejects_hash_mismatch(tmp_path):
    cfg = source()
    cfg['frozen_contract']['validation']['sha256'] = '0' * 64
    with pytest.raises(ValueError, match='hash mismatch: validation'):
        load_config(write(tmp_path, cfg))


def test_v3_formal_task_rejects_relaxed_member_limit(tmp_path):
    cfg = deepcopy(source())
    cfg['members'][0]['a_t_max'] = .31
    with pytest.raises(ValueError, match='exceeds frozen v3'):
        load_config(write(tmp_path, cfg))


def test_v3_formal_task_rejects_changed_controller_authority(tmp_path):
    cfg = deepcopy(source())
    cfg['fixedwing_controller_limits']['bank_max_deg'] = 28.
    with pytest.raises(ValueError, match='controller limits differ'):
        load_config(write(tmp_path, cfg))
