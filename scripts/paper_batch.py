#!/usr/bin/env python3
"""Freeze or execute a reproducible, sequential SITL repeat matrix."""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import tempfile
import tarfile

import yaml

from uav_formation.experiment_artifacts import ROOT, freeze_run, write_json
from uav_formation.experiment_config import load_config, coordinator_from_config
from uav_formation.paper_metrics import compute
from scripts.audit_v3_formal_run import audit as audit_v3


def file_hash(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):
            digest.update(chunk)
    return digest.hexdigest()


def archive_run(staging, output):
    hashes={str(p.relative_to(staging)):file_hash(p) for p in staging.rglob('*') if p.is_file()}
    packed=staging.parent/'raw_evidence.tar.gz'
    with tarfile.open(packed,'w:gz',compresslevel=1) as archive:
        archive.add(staging,arcname='.')
    verified=set()
    with tarfile.open(packed,'r:gz') as archive:
        for member in archive:
            if member.isfile():
                name=Path(member.name).as_posix()
                digest=hashlib.sha256()
                with archive.extractfile(member) as stream:
                    for chunk in iter(lambda:stream.read(1024*1024),b''):
                        digest.update(chunk)
                if digest.hexdigest()!=hashes[name]:
                    raise RuntimeError('packed evidence mismatch; original retained at '+str(staging))
                verified.add(name)
    if verified!=set(hashes):
        raise RuntimeError('incomplete archive')
    packed_hash=file_hash(packed)
    output.mkdir(parents=True,exist_ok=False)
    shutil.copy2(packed,output/packed.name)
    if file_hash(output/packed.name)!=packed_hash:
        raise RuntimeError('archive copy checksum mismatch; original retained at '+str(staging))
    # Keep compact summaries directly accessible. Large CSV inputs and full raw
    # logs remain byte-exact in the verified archive and can be unpacked on demand.
    for source in staging.iterdir():
        if source.is_file() and source.suffix in ('.json','.txt','.npz'):
            shutil.copy2(source,output/source.name)
    for name in ('config_snapshot','raw'):
        if (staging/name).exists():
            shutil.copytree(staging/name,output/name)
    for name,expected in hashes.items():
        accessible=output/name
        if accessible.is_file() and file_hash(accessible)!=expected:
            raise RuntimeError('analysis copy mismatch; original retained at '+str(staging))
    write_json(output/'archive_manifest.json',dict(original_temporary_path=str(staging),
        storage='local recording; verified complete tar.gz archive plus accessible analysis inputs',
        archive_file=packed.name,archive_sha256=packed_hash,sha256=hashes))
    packed.unlink()
    shutil.rmtree(staging)


def prepare(root, capabilities, prechecks, speedup):
    configs={scene:yaml.safe_load((capabilities/(scene+'.yaml')).read_text()) for scene in ('s1','s2','s3')}
    checks={}
    for scene in configs:
        path=Path(prechecks[scene]).resolve()
        status=json.loads((path/'flight_status.json').read_text())
        if not status.get('completed') or not status.get('exit_completed',True):
            raise ValueError('precheck did not complete: '+scene)
        v3_acceptance=path/'v3_formal_acceptance.json'
        if v3_acceptance.is_file():
            acceptance=json.loads(v3_acceptance.read_text())
            source=json.loads((path/'config.json').read_text())
            expected={'s1':{'multirotor'},'s2':{'fixedwing'},'s3':{'multirotor','fixedwing'}}[scene]
            if (not acceptance.get('system_precheck_qualified')
                    or acceptance.get('invalid_reasons') or acceptance.get('acceptance_failures')):
                raise ValueError('v3 system precheck acceptance failed: '+scene)
            if ({member['platform'] for member in source['members']}!=expected
                    or source.get('frozen_contract')!=configs[scene].get('frozen_contract')
                    or source.get('waypoints')!=configs[scene].get('waypoints')
                    or source.get('planner')!=configs[scene].get('planner')):
                raise ValueError('v3 system precheck task/contract mismatch: '+scene)
            checks[scene]=dict(path=str(path),status=status,
                acceptance=str(v3_acceptance),acceptance_sha256=file_hash(v3_acceptance),
                classification=acceptance['classification'])
            continue
        if not status.get('full_route_completed',False):
            raise ValueError('formal precheck requires explicit full-route completion: '+scene)
        acceptance_path=path/'acceptance.json'
        if not acceptance_path.is_file() or not json.loads(acceptance_path.read_text()).get('formal_qualified',False):
            raise ValueError('formal precheck needs measured tracking/constraint acceptance: '+scene)
        checks[scene]=dict(path=str(path),status=status,
                           acceptance=str(acceptance_path),acceptance_sha256=file_hash(acceptance_path))
    root=freeze_run(root,dict(prechecks=checks,capability_source=configs['s3']['capability_source']), 'frozen_repeat_matrix')
    config_dir=root/'tasks'; config_dir.mkdir()
    matrix=[]; planning={}; groups=[]
    for scene in ('s1','s2','s3'):
        loaded,members=load_config(capabilities/(scene+'.yaml'))
        a3=coordinator_from_config(loaded,members,nominal_only=True).plan(loaded['waypoints'])
        planning[scene]={'A3':{'feasible':a3.feasible,'selected_radius':a3.selected_radius}}
        groups.append((scene,'F'))
        if a3.feasible:
            groups.append((scene,'A3'))
        groups.append((scene,'C'))
    groups.append(('s3_swap','F'))
    for scene,method in groups:
        task=deepcopy(configs[scene.split('_')[0]])
        if scene.endswith('_swap'):
            task['members'][0]['r_y'],task['members'][2]['r_y']=task['members'][2]['r_y'],task['members'][0]['r_y']
        if method=='C':
            # Change only the radius/profile policy; retain the common 14.5 m/s
            # initial condition used by F and the frozen system prechecks.
            nominal=task['planner']['nominal_speed']
            task['planner'].update(nominal_radius=320.,radii=[320.],
                                   start_speed=nominal,end_speed=nominal)
        task['name']=scene.upper()+'_Freestyle_Plane'
        filename=config_dir/(scene+'_'+method+'.yaml')
        filename.write_text(yaml.safe_dump(task,sort_keys=False))
        for seed in range(5):
            matrix.append(dict(id='%s_%s_seed%d'%(scene,method,seed),scene=scene,method=method,seed=seed,
                               config=str(filename),base_port=24000+len(matrix)*40))
    protocol=dict(schema_version=2,created_unix=time.time(),speedup=speedup,runs=matrix,
        prechecks=checks,replicate_definition='five seeded +/-0.2 m initial home-position perturbations; fixed SITL model/noise configuration; technical repeats',
        planning_screen=planning,
        baseline='C uses fixed R=320 m and constant nu=14.5 m/s, independently audited; same takeoff/join and task initial speed as F/A3',
        protected_baselines='A0/A1/A2 and infeasible A3 S2/S3 remain offline; never label protected/offline traces as flight results',
        statistics='report every attempt; completion count over planned and attempted; successful task metrics mean and sample SD; failed partial metrics separate; no significance claims',
        metric_definition='paper_metrics.py frozen source; entire track phase, unique-source-time unsmoothed derivatives, no posthoc exclusions',
        plotting='every completed/partial task gets raw trace plots; seed 0 chosen in advance for illustrative traces; all seeds in summary',
        host_scheduling=dict(control_cpu=3,sitl_cpu=2,stream_cpu=1,aux_cpu=1,background_cpu=0),
        code_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [*sorted((ROOT/'uav_formation').glob('*.py')), ROOT/'scripts/run_paper_sitl.py',
                      ROOT/'scripts/audit_v3_formal_run.py',ROOT/'scripts/analyze_v2_bin.py',
                      ROOT/'scripts/paper_batch.py',ROOT/'scripts/host_cpu_isolation.py',
                      ROOT/'scripts/experiment_env.sh',ROOT/'requirements-paper.txt']},
        config_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in config_dir.glob('*.yaml')},
        consecutive_failure_stop=2,minimum_free_bytes=700*1024**2)
    protocol['storage']='owned local /tmp staging during flight; verified full archive after all SITL processes exit; compact JSON summaries directly accessible and CSV/raw evidence unpacked on demand'
    write_json(root/'batch_protocol.json',protocol)
    write_json(root/'batch_status.json',dict(status='PREPARED',runs=[]))
    print(root,flush=True)


def execute_isolated(root):
    protocol=json.loads((root/'batch_protocol.json').read_text())
    for relative,expected in protocol['code_sha256'].items():
        if hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()!=expected:
            raise ValueError('flight/metric source changed since freeze: '+relative)
    for path,expected in protocol['config_sha256'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=expected:
            raise ValueError('task changed since freeze: '+path)
    state=json.loads((root/'batch_status.json').read_text())
    done={r['id'] for r in state['runs']}
    state['status']='RUNNING'; failures=0
    for entry in protocol['runs']:
        if entry['id'] in done:
            continue
        if shutil.disk_usage(root).free<protocol['minimum_free_bytes']:
            state.update(status='STOPPED_STORAGE',error='less than reserved free space')
            write_json(root/'batch_status.json',state); return 2
        output=root/entry['id']
        if output.exists():
            raise ValueError('unindexed existing run; preserve and inspect before resuming: '+str(output))
        if shutil.disk_usage('/tmp').free<protocol['minimum_free_bytes']:
            raise RuntimeError('insufficient local staging storage')
        staging_parent=Path(tempfile.mkdtemp(prefix='nvidia-paper-'))
        staging=staging_parent/entry['id']
        scheduling=protocol['host_scheduling']
        cmd=['taskset','-c',str(scheduling['control_cpu']),sys.executable,str(ROOT/'scripts/run_paper_sitl.py'),'--config',entry['config'],'--output',str(staging),
             '--base-port',str(entry['base_port']),'--seed',str(entry['seed']),'--method',entry['method'],
             '--speedup',str(protocol['speedup'])]
        if entry['scene'].startswith('s3'):
            cmd.extend(['--rotor-precheck',protocol['prechecks']['s1']['path']])
        environment=dict(os.environ,UAV_SITL_CPU=str(scheduling['sitl_cpu']),
                         UAV_AUX_CPU=str(scheduling['aux_cpu']),
                         UAV_CONTROL_CPU=str(scheduling['control_cpu']),
                         UAV_STREAM_CPU=str(scheduling.get('stream_cpu', 1)))
        state['active_run']=entry['id']; write_json(root/'batch_status.json',state)
        print('START',entry['id'],flush=True)
        began=time.monotonic()
        interrupted=False
        console_path=staging_parent/'console.log'
        with console_path.open('w') as console:
            process=subprocess.Popen(cmd,cwd=ROOT,stdout=console,stderr=subprocess.STDOUT,
                                     env=environment)
            try:
                returncode=process.wait()
            except KeyboardInterrupt:
                interrupted=True
                process.send_signal(signal.SIGINT)
                returncode=process.wait(timeout=180)
        record=dict(entry,returncode=returncode,wall_seconds=time.monotonic()-began)
        shutil.copy2(console_path,root/(entry['id']+'.console.log'))
        if returncode==0 and staging.exists():
            try:
                record['formal_audit']=audit_v3(staging,False)
            except Exception as exc:
                record['formal_audit_error']=repr(exc)
        if staging.exists():
            try:
                record['metrics']=compute(staging)
            except Exception as exc:
                record['metrics_error']=repr(exc)
        if staging.exists():
            archive_run(staging,output)
        console_path.unlink(); staging_parent.rmdir()
        state['runs'].append(record); state.pop('active_run',None)
        success=(returncode==0
                 and record.get('formal_audit',{}).get('classification')=='SYSTEM_QUALIFIED')
        failures=0 if success else failures+1
        print('END',entry['id'],'PASS' if success else 'FAIL',flush=True)
        if interrupted:
            state['status']='STOPPED_REQUEST'; write_json(root/'batch_status.json',state); return 130
        if failures>=protocol['consecutive_failure_stop']:
            state.update(status='STOPPED_FAILURES',error='two consecutive failed runs; inspect before continuing')
            write_json(root/'batch_status.json',state); return 1
        write_json(root/'batch_status.json',state)
    state['status']='FINISHED'; write_json(root/'batch_status.json',state)
    return 0


def execute(root):
    isolation=root/'host_cpu_isolation.json'
    subprocess.run([sys.executable,str(ROOT/'scripts/host_cpu_isolation.py'),'apply',
                    '--state',str(isolation),'--background-cpus','0'],cwd=ROOT,check=True)
    try:
        return execute_isolated(root)
    finally:
        subprocess.run([sys.executable,str(ROOT/'scripts/host_cpu_isolation.py'),'restore',
                        '--state',str(isolation)],cwd=ROOT,check=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['prepare','execute','launch'])
    parser.add_argument('--root',required=True)
    parser.add_argument('--capabilities',default='runs/paper_hetero/frozen_v1')
    parser.add_argument('--precheck',action='append',default=[],help='scene=directory')
    parser.add_argument('--speedup',type=float,default=4.)
    args=parser.parse_args(); root=Path(args.root).resolve()
    if args.action=='prepare':
        prepare(root,Path(args.capabilities).resolve(),dict(s.split('=',1) for s in args.precheck),args.speedup)
        return 0
    if args.action=='launch':
        pid_file=root/'batch_process.json'
        if pid_file.exists():
            old=json.loads(pid_file.read_text())
            try:
                import os
                os.kill(old['pid'],0)
            except ProcessLookupError:
                pass
            else:
                raise RuntimeError('recorded batch process still exists; do not launch twice')
        with (root/'batch_console.log').open('a') as console:
            process=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'execute','--root',str(root)],
                cwd=ROOT,stdin=subprocess.DEVNULL,stdout=console,stderr=subprocess.STDOUT,start_new_session=True)
        write_json(pid_file,dict(pid=process.pid,started_unix=time.time(),root=str(root)))
        print('Batch PID',process.pid,'status',root/'batch_status.json')
        return 0
    return execute(root)


if __name__=='__main__':
    raise SystemExit(main())
