#!/usr/bin/env python3
"""Generate and audit plots for already archived attempts in a frozen batch."""
import argparse
import hashlib
import json
from pathlib import Path

from scripts.plot_paper_run import plot
from scripts.audit_paper_figures import audit
from uav_formation.experiment_artifacts import write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('root',type=Path)
    parser.add_argument('--output',type=Path,help='optional local figure root, one subdirectory per run')
    args=parser.parse_args(); root=args.root.resolve()
    state=json.loads((root/'batch_status.json').read_text())
    plot_hash=hashlib.sha256(Path('scripts/plot_paper_run.py').read_bytes()).hexdigest()
    for entry in state['runs']:
        source=root/entry['id']; target=args.output/entry['id'] if args.output else source/'plots'
        stamp=target/'plot_source.json'
        if stamp.exists() and json.loads(stamp.read_text()).get('sha256')==plot_hash:
            print('SKIP',entry['id'],flush=True); continue
        plot(source,target)
        result=audit(target)
        write_json(target/'plot_source.json',dict(sha256=plot_hash,source='scripts/plot_paper_run.py'))
        print('PLOTS',entry['id'],'PASS' if result['automated_pass'] else 'REVIEW',flush=True)


if __name__=='__main__':
    main()
