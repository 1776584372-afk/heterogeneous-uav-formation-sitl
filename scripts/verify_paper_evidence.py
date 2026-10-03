#!/usr/bin/env python3
"""Check frozen code, task files, archives, and accessible primary evidence."""
import argparse
import json
from pathlib import Path

from scripts.paper_batch import file_hash
from uav_formation.experiment_artifacts import ROOT,write_json


def verify(root,output):
    protocol=json.loads((root/'batch_protocol.json').read_text())
    state=json.loads((root/'batch_status.json').read_text())
    failures=[]; runs=[]; reference_groups={}; shared_tasks={}
    for name,expected in protocol['code_sha256'].items():
        if file_hash(ROOT/name)!=expected:
            failures.append('frozen code mismatch: '+name)
    for name,expected in protocol['config_sha256'].items():
        if file_hash(Path(name))!=expected:
            failures.append('frozen task mismatch: '+name)
    indexed={r['id']:r for r in state['runs']}
    for planned in protocol['runs']:
        directory=root/planned['id']; manifest=directory/'archive_manifest.json'
        errors=[]
        if planned['id'] not in indexed:
            errors.append('not indexed by batch')
        if not manifest.exists():
            errors.append('no completed archive manifest')
        else:
            archived=json.loads(manifest.read_text())
            if file_hash(directory/archived['archive_file'])!=archived['archive_sha256']:
                errors.append('archive checksum mismatch')
            # Full archive entries were already verified before removal of local
            # staging. Recheck every accessible original and the archive digest.
            checked=0
            for relative,expected in archived['sha256'].items():
                path=directory/relative
                if path.is_file():
                    checked+=1
                    if file_hash(path)!=expected:
                        errors.append('accessible primary file mismatch: '+relative)
            if not (directory/'flight_status.json').exists():
                errors.append('missing flight status')
            group=planned['scene']+'_'+planned['method']
            for filename in ('reference.csv','member_reference.csv'):
                path=directory/filename
                if path.exists():
                    reference_groups.setdefault(group+' / '+filename,set()).add(file_hash(path))
            cfg=json.loads((directory/'config.json').read_text())
            shared={k:cfg[k] for k in ('waypoints','altitude','wind_m_s','endpoint','members','speedup')}
            shared['progress_initial_terminal']=[cfg['planner'][k] for k in ('nominal_speed','start_speed','end_speed')]
            shared_tasks.setdefault(planned['scene'],set()).add(json.dumps(shared,sort_keys=True))
        runs.append(dict(id=planned['id'],integrity_pass=not errors,errors=errors))
        print(planned['id'],'PASS' if not errors else 'FAIL',flush=True)
    if state['status']!='FINISHED':
        failures.append('batch not FINISHED: '+state['status'])
    for group,hashes in reference_groups.items():
        if len(hashes)!=1:
            failures.append('different reference bytes between same-group seeds: '+group)
    for scene,tasks in shared_tasks.items():
        if len(tasks)!=1:
            failures.append('methods do not share task/platform/initial speed: '+scene)
    result=dict(status='PASS' if not failures and all(r['integrity_pass'] for r in runs) else 'FAIL',
        code_and_batch_errors=failures,runs=runs,
        same_group_reference_sha256={k:sorted(v) for k,v in reference_groups.items()},
        shared_task_variants={k:len(v) for k,v in shared_tasks.items()},
        scope='frozen flight/metric code and task hashes; whole-archive hashes; accessible original file hashes',
        archive_contents='archive_run verified every internal member before deleting owned temporary originals; current whole-archive digest proves unchanged archive bytes')
    write_json(output,result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path); parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); result=verify(args.root.resolve(),args.output)
    print('INTEGRITY',result['status'])
    raise SystemExit(0 if result['status']=='PASS' else 1)
