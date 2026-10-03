#!/usr/bin/env python3
"""Read-only provenance audit of historical acceleration planning limits.

Reproduces the legacy window fits and separately reports raw source-clock
accelerations. Neither calculation certifies or replaces a capability bound.
"""
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path
import shutil
import statistics
from xml.etree import ElementTree as ET
from zipfile import ZipFile

import numpy as np
import yaml

from scripts.diagnose_calibration_truth import acceleration_stats


def digest(path):
    result=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):
            result.update(chunk)
    return result.hexdigest()


def step_segments(rows):
    # Historical runs reused step_16 and step_8 names on deceleration.
    # Keep contiguous occurrences separate, just as the original audit did.
    return [(phase,list(group)) for phase,group in itertools.groupby(rows,key=lambda r:r['phase'])
            if phase.startswith('step_')]


def legacy_fits(rows, field):
    fits=[]
    for index,(phase,group) in enumerate(step_segments(rows)):
        unique=list({float(r['sim_s']):r for r in group}.values())
        moving=[r for a,r in zip(unique,unique[1:])
                if abs(float(r['ref_speed'])-float(a['ref_speed']))>1e-4]
        if not moving:
            continue
        lo=float(moving[0]['sim_s'])+2.
        hi=float(moving[-1]['sim_s'])-2.
        middle=[r for r in unique if lo<=float(r['sim_s'])<=hi]
        if len(middle)<20:
            continue
        fit=statistics.linear_regression([float(r['sim_s']) for r in middle],
                                         [float(r[field]) for r in middle])
        fits.append(dict(segment=index,phase=phase,window=[lo,hi],samples=len(middle),
            measured_slope=fit.slope,
            max_speed_error=max(abs(float(r[field])-float(r['ref_speed'])) for r in middle)))
    return fits


def window_acceleration(samples, window, limit):
    data=np.asarray(samples,dtype=float)
    if data.ndim!=2 or data.shape[1]!=3:
        return {'unavailable':'timestamped horizontal velocity samples missing'}
    selected=data[(data[:,0]>=window[0]) & (data[:,0]<=window[1])]
    return acceleration_stats(selected,limit)


def velocity_log(path, message):
    from pymavlink import mavutil
    reader=mavutil.mavlink_connection(str(path))
    samples=[]
    try:
        while True:
            msg=reader.recv_match(type=message)
            if msg is None:
                break
            if message=='SIM2':
                samples.append((msg.TimeUS/1e6,msg.VE,msg.VN))
            else:
                samples.append((msg.time_boot_ms/1000.,msg.vy,msg.vx))
    finally:
        reader.close()
    return np.asarray(samples,dtype=float).reshape((-1,3))


def docx_paragraphs(path):
    w='http://schemas.openxmlformats.org/wordprocessingml/2006/main'
    m='http://schemas.openxmlformats.org/officeDocument/2006/math'
    with ZipFile(path) as archive:
        root=ET.fromstring(archive.read('word/document.xml'))
    return [''.join(node.text or '' for node in p.iter() if node.tag in ('{'+w+'}t','{'+m+'}t'))
            for p in root.findall('.//{'+w+'}body//{'+w+'}p')]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frozen',type=Path,required=True)
    p.add_argument('--manuscript',type=Path,action='append',default=[])
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    caps=yaml.safe_load(args.frozen.read_text())
    source=Path(__file__).resolve()
    freeze_script=source.with_name('prepare_paper_experiments.py')
    dependencies=[source,freeze_script,source.with_name('diagnose_calibration_truth.py'),
                  source.with_name('analyze_backend_dynamics.py'),
                  source.parents[1]/'uav_formation/paper_metrics.py',args.frozen,*args.manuscript]
    before={str(path.resolve()):digest(path) for path in dependencies}
    report=dict(schema_version=1,read_only_audit=True,limits_modified=False,
        capability_certified=False,formal_qualification=False,
        interpretation='historical fit reproducibility, not instantaneous constraint certification',
        acceleration_definition='d hypot(VE,VN)/dt from unique source clocks; no smoothing',
        phase_alignment='phase edges use historical ATTITUDE sim_s; not exact command-send timestamps',
        original_limits='retained for historical comparison; exceeding a lower planning value in a higher-demand calibration is not itself a calibration failure',
        platforms={},manuscripts=[],source_sha256=before)
    for platform,field,key in [('multirotor','heading_forward','copter_ramp'),
                                ('fixedwing','airspeed','plane_ramp')]:
        limits=caps[platform]['limits']
        limit=limits['tangential_acceleration']
        if limit['value']!=limits['tangential_deceleration']['value']:
            raise ValueError('this historical audit requires symmetric frozen bounds')
        run=Path(limit['raw_run'])
        paths=[run/name for name in ('result.json','protocol.json','manifest.json','telemetry.csv','raw.tlog')]
        paths.extend(sorted((run/'logs').glob('*.BIN')))
        paths.extend(sorted((run/'source_snapshot').glob('*.py')))
        for path in paths:
            value=digest(path)
            expected=caps['source_sha256'].get(str(path))
            if expected is not None and value!=expected:
                raise ValueError('historical source hash mismatch: '+str(path))
            before[str(path.resolve())]=value
        with (run/'telemetry.csv').open() as stream:
            rows=list(csv.DictReader(stream))
        fits=legacy_fits(rows,field)
        expected=caps['ramp_audits'][key]
        if len(fits)!=len(expected) or any(a[name]!=b[name]
                for a,b in zip(fits,expected)
                for name in ('phase','window','measured_slope','max_speed_error')):
            raise ValueError('legacy fit does not reproduce frozen record: '+platform)
        measured=velocity_log(run/'raw.tlog','LOCAL_POSITION_NED')
        bins=sorted((run/'logs').glob('*.BIN'))
        if len(bins)!=1:
            raise ValueError('need one unambiguous SIM2 clock per historical run')
        truth=velocity_log(bins[0],'SIM2')
        phases=[]
        for index,(phase,group) in enumerate(step_segments(rows)):
            window=[float(group[0]['sim_s']),float(group[-1]['sim_s'])]
            phases.append(dict(segment=index,phase=phase,window=window,
                telemetry_source_clock=window_acceleration(measured,window,limit['value']),
                simulator_truth=window_acceleration(truth,window,limit['value'])))
        report['platforms'][platform]=dict(run=str(run.resolve()),frozen_limit=limit,
            freeze_script_matches_extraction=digest(freeze_script)==limit['extraction_revision'],
            source_hashes_match_frozen=True,legacy_fits_match_frozen=True,
            legacy_fit_field=field,legacy_fits=fits,
            telemetry_clock='LOCAL_POSITION_NED.time_boot_ms from raw.tlog, not legacy CSV sim_s',
            phases=phases,protocol=json.loads((run/'protocol.json').read_text()),
            recorded_result=json.loads((run/'result.json').read_text()))
    terms=('切向加速度','运行边界','操作限值','实际飞行','约束违反','具体数值')
    for path in args.manuscript:
        paragraphs=docx_paragraphs(path)
        report['manuscripts'].append(dict(path=str(path.resolve()),
            index_definition='zero-based XML body paragraph index including table cells; not page numbers',
            excerpts=[dict(paragraph_index=i,text=t) for i,t in enumerate(paragraphs)
                      if any(term in t for term in terms)]))
    for path,value in before.items():
        if digest(path)!=value:
            raise ValueError('source changed during audit: '+path)
    report['all_inputs_unchanged']=True
    (out/'audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    shutil.copy2(source,out/source.name)
    print(out/'audit.json')
    for platform,item in report['platforms'].items():
        print(platform,'legacy fits reproduced:',len(item['legacy_fits']),
              'reference ramp:',item['protocol']['reference_ramp_m_s2'],
              'frozen limit:',item['frozen_limit']['value'])


if __name__=='__main__':
    main()
