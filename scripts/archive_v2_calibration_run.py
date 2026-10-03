#!/usr/bin/env python3
"""Create and verify a recoverable archive before pruning direct raw copies."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import tempfile


PRUNABLE = (
    'logs', 'raw.tlog', 'outgoing_packets.jsonl', 'events.jsonl', 'telemetry.csv',
    'source_snapshot', 'eeprom.bin', 'sitl.log',
)


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def archive_run(directory):
    directory = Path(directory).resolve()
    if not (directory / 'result.json').is_file():
        raise ValueError('run has no result.json')
    manifest_path = directory / 'raw_archive_manifest.json'
    archive_path = directory / 'raw_evidence.tar.gz'
    if manifest_path.is_file() and archive_path.is_file():
        manifest = json.loads(manifest_path.read_text())
        if file_hash(archive_path) != manifest['archive_sha256']:
            raise RuntimeError('existing archive hash mismatch')
        return manifest
    sources = []
    for name in PRUNABLE:
        path = directory / name
        if path.is_file():
            sources.append(path)
        elif path.is_dir():
            sources.extend(item for item in path.rglob('*') if item.is_file())
    if not sources:
        raise ValueError('no direct raw evidence to archive')
    hashes = {str(path.relative_to(directory)): file_hash(path) for path in sources}
    temporary = Path(tempfile.mkstemp(prefix='.raw-evidence-', suffix='.tar.gz',
                                      dir=directory)[1])
    try:
        with tarfile.open(temporary, 'w:gz', compresslevel=1) as archive:
            for path in sources:
                archive.add(path, arcname=str(path.relative_to(directory)), recursive=False)
        verified = {}
        with tarfile.open(temporary, 'r:gz') as archive:
            for member in archive:
                if not member.isfile():
                    continue
                digest = hashlib.sha256()
                with archive.extractfile(member) as stream:
                    for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                        digest.update(chunk)
                verified[member.name] = digest.hexdigest()
        if verified != hashes:
            raise RuntimeError('archive content does not match direct evidence')
        shutil.move(temporary, archive_path)
    finally:
        if temporary.exists():
            temporary.unlink()
    result = {
        'schema_version': 1, 'archive': archive_path.name,
        'archive_sha256': file_hash(archive_path), 'source_sha256': hashes,
        'storage': 'verified byte-exact archive; direct prunable copies removed after verification',
        'recovery': f'tar -xzf {archive_path.name} -C {directory}',
    }
    manifest_path.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    for name in PRUNABLE:
        path = directory / name
        if path.is_dir():
            shutil.rmtree(path)
        elif path.exists():
            path.unlink()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    result = archive_run(args.directory)
    print(json.dumps({'archive': result['archive'],
                      'archive_sha256': result['archive_sha256'],
                      'files': len(result['source_sha256'])}, indent=2))


if __name__ == '__main__':
    main()

