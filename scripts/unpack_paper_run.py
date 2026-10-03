#!/usr/bin/env python3
"""Restore a verified complete run archive into a new local directory."""
import argparse
import json
from pathlib import Path
import tarfile

from scripts.paper_batch import file_hash


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run'); parser.add_argument('destination')
    args=parser.parse_args(); root=Path(args.run).resolve(); destination=Path(args.destination).resolve()
    manifest=json.loads((root/'archive_manifest.json').read_text())
    packed=root/manifest['archive_file']
    if file_hash(packed)!=manifest['archive_sha256']:
        raise ValueError('archive checksum mismatch')
    if destination.exists():
        raise ValueError('destination must be a new directory')
    with tarfile.open(packed,'r:gz') as archive:
        members=archive.getmembers()
        for member in members:
            if not (member.isdir() or member.isfile()) or not (destination/member.name).resolve().is_relative_to(destination):
                raise ValueError('unsafe archive entry '+member.name)
        destination.mkdir(parents=True)
        archive.extractall(destination,members=members)
    for name,expected in manifest['sha256'].items():
        if file_hash(destination/name)!=expected:
            raise ValueError('restored file mismatch: '+name)
    print('Verified complete run:',destination)


if __name__=='__main__':
    main()
