import hashlib
import json
import tarfile

import pytest

from scripts.reanalyse_v2_archive import reanalyse


def test_reanalysis_rejects_archive_hash_mismatch(tmp_path):
    run = tmp_path / 'run'; run.mkdir()
    archive = run / 'raw_evidence.tar.gz'
    with tarfile.open(archive, 'w:gz'):
        pass
    (run / 'raw_archive_manifest.json').write_text(json.dumps({
        'archive_sha256': 'bad', 'source_sha256': {}}))
    with pytest.raises(RuntimeError, match='hash mismatch'):
        reanalyse(run)
