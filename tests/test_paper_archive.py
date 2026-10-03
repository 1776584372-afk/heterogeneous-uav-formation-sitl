from pathlib import Path
import tempfile
import unittest
import tarfile
from unittest.mock import patch

from scripts.paper_batch import archive_run, file_hash


class ArchiveTests(unittest.TestCase):
    def test_verified_archive_preserves_bytes(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); source=root/'source'; source.mkdir()
            payload=bytes(range(256))*100
            (source/'raw.bin').write_bytes(payload)
            archive_run(source,root/'archive')
            self.assertFalse(source.exists())
            with tarfile.open(root/'archive/raw_evidence.tar.gz') as archive:
                self.assertEqual(archive.extractfile('./raw.bin').read(),payload)
            self.assertTrue((root/'archive/archive_manifest.json').exists())

    def test_checksum_failure_retains_original(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); source=root/'source'; source.mkdir()
            (source/'raw.bin').write_bytes(b'original flight bytes')
            def corrupted_destination(path):
                return 'mismatch' if path.parent.name=='archive' else file_hash(path)
            with patch('scripts.paper_batch.file_hash',side_effect=corrupted_destination):
                with self.assertRaises(RuntimeError):
                    archive_run(source,root/'archive')
            self.assertEqual((source/'raw.bin').read_bytes(),b'original flight bytes')
