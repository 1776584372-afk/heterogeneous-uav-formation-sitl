#!/usr/bin/env python3
"""Export the actual dirty source tree and its diff, without staging or reverting."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

from uav_formation.experiment_artifacts import ROOT,write_json


def export(output):
    output.mkdir(parents=True,exist_ok=False)
    selected=[]
    for folder,patterns in (('uav_formation',('*.py',)),('scripts',('*.py','*.sh')),
                            ('tests',('*.py',)),('config',('*.yaml',)),('docs',('*.md',))):
        for pattern in patterns:
            selected.extend(p for p in (ROOT/folder).rglob(pattern) if '__pycache__' not in p.parts)
    selected.extend([ROOT/'requirements-paper.txt',ROOT/'实验结果请先看.md'])
    selected.extend(p for p in (ROOT/'runs/paper_hetero/frozen_v1').iterdir() if p.is_file())
    selected=sorted(set(selected))
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in selected}
    tracked=subprocess.check_output(['git','ls-files','-z'],cwd=ROOT).decode().split('\0')
    with (output/'working_tree_vs_HEAD.patch').open('wb') as stream:
        for path in selected:
            relative=str(path.relative_to(ROOT))
            cmd=['git','diff','--binary','HEAD','--',relative] if relative in tracked else [
                'git','diff','--binary','--no-index','--','/dev/null',relative]
            result=subprocess.run(cmd,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            if result.returncode not in (0,1):
                raise RuntimeError(result.stderr.decode())
            stream.write(result.stdout)
    archive=output/'engineering_source.tar.gz'
    with tarfile.open(archive,'w:gz') as package:
        for path in selected:
            package.add(path,arcname=str(path.relative_to(ROOT)))
    write_json(output/'manifest.json',dict(
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT).decode().strip(),
        attribution='actual selected working-tree source versus HEAD; may include pre-existing user changes; not a claim of single-session authorship',
        scope='source and frozen experiment inputs; not a standalone VM image; existing ROS/message builds, ArduPilot binaries, Python dependencies and raw evidence are external',
        archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),files=hashes))
    (output/'git_status.txt').write_bytes(subprocess.check_output(['git','status','--short'],cwd=ROOT))
    print(output.resolve())


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('output',type=Path)
    export(parser.parse_args().output)
