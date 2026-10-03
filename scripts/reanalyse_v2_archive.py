#!/usr/bin/env python3
"""Create a versioned v2 analysis from an immutable verified raw archive."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import tempfile

from scripts.analyze_v2_bin import analyse, file_hash


def reanalyse(run, output_name='v2_analysis_event_clock'):
    run = Path(run).resolve()
    archive_path = run / 'raw_evidence.tar.gz'
    manifest_path = run / 'raw_archive_manifest.json'
    if not archive_path.is_file() or not manifest_path.is_file():
        raise ValueError('verified raw archive and manifest are required')
    manifest = json.loads(manifest_path.read_text())
    if file_hash(archive_path) != manifest['archive_sha256']:
        raise RuntimeError('raw archive hash mismatch')
    output = run / output_name
    if output.exists():
        raise FileExistsError(output)
    with tempfile.TemporaryDirectory(prefix='v2-reanalysis-') as temporary_name:
        temporary = Path(temporary_name)
        with tarfile.open(archive_path, 'r:gz') as archive:
            names = archive.getnames()
            bins = sorted(name for name in names if name.startswith('logs/') and name.endswith('.BIN'))
            required = [bins[0] if bins else None, 'telemetry.csv', 'events.jsonl']
            if any(name is None or name not in names for name in required):
                raise ValueError('archive lacks BIN, telemetry, or phase events')
            extracted = {}
            for index, name in enumerate(required):
                destination = temporary / f'{index}_{Path(name).name}'
                with archive.extractfile(name) as source, destination.open('wb') as target:
                    shutil.copyfileobj(source, target)
                expected = manifest['source_sha256'].get(name)
                if expected is None or file_hash(destination) != expected:
                    raise RuntimeError(f'archived source hash mismatch: {name}')
                extracted[name] = destination
        report = analyse(extracted[required[0]], extracted['telemetry.csv'], output,
                         events=extracted['events.jsonl'])
    provenance = {
        'schema_version': 1, 'raw_archive': archive_path.name,
        'raw_archive_sha256': manifest['archive_sha256'],
        'raw_archive_manifest_sha256': file_hash(manifest_path),
        'analysis_sha256': report['analysis_sha256'],
        'phase_clock': report['phase_clock'],
        'note': 'versioned reanalysis; original analysis directory was not modified',
    }
    (output / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    return provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output-name', default='v2_analysis_event_clock')
    args = parser.parse_args()
    print(json.dumps(reanalyse(args.run, args.output_name), indent=2))


if __name__ == '__main__':
    main()
