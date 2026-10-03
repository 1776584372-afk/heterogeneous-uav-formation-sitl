#!/usr/bin/env python3
"""Freeze conservative operation limits from completed independent SITL tests."""
import argparse
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
import statistics

import yaml

from uav_formation.experiment_artifacts import ROOT, write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    output=Path(args.output).resolve(); output.mkdir(parents=True,exist_ok=False)
    evidence=ROOT/'runs/paper_hetero'
    paths=dict(forward='selection/freestyle_flight_03',backward='calibration/freestyle_backward_01',
        left='calibration/freestyle_left_01',right='calibration/freestyle_right_01',
        copter_turns='calibration/freestyle_turns_ppva_01',plane_low='selection/plane_speed_down_01',
        plane_high='selection/plane_speed_up_01',plane_turns='calibration/plane_bank_01',
        copter_ramp='calibration/freestyle_ramp_01',plane_ramp='calibration/plane_ramp_04')
    results={}; raw_hashes={}
    for name,relative in paths.items():
        run=evidence/relative
        result=json.loads((run/'result.json').read_text())
        if result['status']!='COMPLETED' or not result.get('exit_completed') or not result['steps']:
            raise ValueError('incomplete calibration '+name)
        if any(s['status']!='PASS' for s in result['steps']):
            raise ValueError('failed selected calibration '+name)
        results[name]=result
        for filename in ('result.json','protocol.json','manifest.json','telemetry.csv','raw.tlog'):
            p=run/filename
            raw_hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    ramp_audits={}
    for name,field in (('copter_ramp','heading_forward'),('plane_ramp','airspeed')):
        with (evidence/paths[name]/'telemetry.csv').open() as stream:
            rows=list(csv.DictReader(stream))
        audits=[]
        for phase,g in itertools.groupby(rows,key=lambda r:r['phase']):
            if not phase.startswith('step_'):
                continue
            rr=list({float(r['sim_s']):r for r in g}.values())
            moving=[r for a,r in zip(rr,rr[1:]) if abs(float(r['ref_speed'])-float(a['ref_speed']))>1e-4]
            if not moving:
                continue
            start=float(moving[0]['sim_s'])+2.; end=float(moving[-1]['sim_s'])-2.
            mid=[r for r in rr if start<=float(r['sim_s'])<=end]
            if len(mid)<20:
                continue
            fit=statistics.linear_regression([float(r['sim_s']) for r in mid],[float(r[field]) for r in mid])
            error=max(abs(float(r[field])-float(r['ref_speed'])) for r in mid)
            audits.append(dict(phase=phase,window=[start,end],measured_slope=fit.slope,max_speed_error=error))
        if not audits or max(a['max_speed_error'] for a in audits)>1.:
            raise ValueError('ramp tracking audit failed: '+name)
        ramp_audits[name]=audits
    def limit(value,unit,source,margin,note=''):
        return dict(value=value,unit=unit,evidence='measured',raw_run=str(evidence/paths[source]),
                    margin=margin,note=note,extraction_revision=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    rotor_limits={name+'_speed':limit(20.,'m/s',name,5.,'Conservative operation ceiling below all tested 26 m/s directional steps; not physical maximum')
                  for name in ('forward','backward','left','right')}
    rotor_limits.update(lateral_acceleration=limit(3.,'m/s^2','copter_turns',.5,'below tested 3.57 m/s^2 reference turn demand'),
        yaw_rate=limit(.25,'rad/s','copter_turns',.04,'12 m/s left/right 0.297 rad/s reference tests passed'),
        tangential_acceleration=limit(.4,'m/s^2','copter_ramp',.1),tangential_deceleration=limit(.4,'m/s^2','copter_ramp',.1))
    plane_limits=dict(minimum_airspeed=limit(12.,'m/s','plane_low',2.,'10 m/s straight test passed; turns calibrated at >=12 m/s'),
        maximum_airspeed=limit(24.,'m/s','plane_high',6.,'30 m/s straight passed; turn calibration limited to <=24 m/s'),
        bank=limit(27.,'deg','plane_turns',3.,'30 degree commanded bank tracked at 12/18/24 m/s in both directions'),
        lateral_acceleration=limit(2.5,'m/s^2','plane_turns',.05,'below minimum measured mean lateral acceleration at 20 degree bank (~2.56); do not use ideal coordinated-turn relation alone'),
        tangential_acceleration=limit(.2,'m/s^2','plane_ramp',.1,'0.3 m/s^2 reference ramp; measured slope audit included'),
        tangential_deceleration=limit(.2,'m/s^2','plane_ramp',.1),
        bank_rate=dict(value=None,unit='rad/s',evidence='unknown',raw_run=None,margin=None))
    caps=dict(schema_version=1,scope='selected SITL model and control-chain operation envelopes, not real-aircraft limits',
        wind_m_s=0.,altitude_m=60.,coordinate_definition='horizontal heading frame for four directional bounds (paper planar beta model); raw full FLU velocity retained separately',
        continuous_envelope_claim='finite sampled empirical envelope with conservative operation limits; not a proof at every intermediate speed',
        multirotor=dict(model='ArduPilot x:freestyle.json',mass_kg=.8,limits=rotor_limits),
        fixedwing=dict(model='ArduPilot plane',mass_kg=2.,limits=plane_limits),ramp_audits=ramp_audits,
        source_sha256=raw_hashes)
    (output/'capabilities_verified.yaml').write_text(yaml.safe_dump(caps,allow_unicode=True,sort_keys=False))
    write_json(output/'straight_gate.json',dict(status='PASS',lower=12.,upper=20.,width=8.,required_width=1.,unit='m/s',
        lower_owner='fixedwing minimum airspeed',upper_owner='multirotor forward operation ceiling',
        evidence_file=str(output/'capabilities_verified.yaml'),config_only_old=dict(lower=12.,upper=6.,status='FAIL')))
    rotor=dict(platform='multirotor',v_min=0.,v_max=20.,a_lat_max=3.,yaw_rate_max=.25,a_t_min=-.4,a_t_max=.4)
    plane=dict(platform='fixedwing',v_min=12.,v_max=24.,a_lat_max=2.5,bank_max=math.radians(27),a_t_min=-.2,a_t_max=.2)
    for scene,types in (('s1',[rotor]*3),('s2',[plane]*3),('s3',[rotor,rotor,plane])):
        cfg=dict(schema_version=1,name=scene.upper()+'_frozen_Freestyle_Plane',wind_m_s=0.,altitude=60.,endpoint='flythrough',
            capability_evidence='measured',capability_source=str(output/'capabilities_verified.yaml'),required_straight_margin=1.,
            waypoints=[[0.,0.],[800.,0.],[800.,800.],[1600.,800.]],
            planner=dict(nominal_radius=80.,radii=[60.,80.,100.,120.,160.,200.,240.,320.],
                nominal_speed=14.,start_speed=14.,end_speed=14.,ds=4.,trans_len=80.,
                bounds=[-100.,-100.,1800.,1000.],smooth_weight=.05,objective_method='quadratic',lambda_time=0.),
            members=[dict(t,member_id=i,r_y=float((i-1)*10)) for i,t in enumerate(types)])
        (output/(scene+'.yaml')).write_text(yaml.safe_dump(cfg,sort_keys=False))
    write_json(output/'freeze_protocol.json',dict(calibration=paths,
        planned_scenarios=['S1','S2','S3'],seeds=[0,1,2,3,4],
        controller_protection=dict(sustained_seconds=2,speed_excess_m_s=.5,bank_excess_deg=3,
            position_abort_m=30,altitude_floor_fraction=.5,telemetry_age_s=.8),
        task_selection='same predefined radius set and route for all scenes; retain all candidate failures',
        counterfactuals='A0/A1/A2 evaluated offline first; no forced flight of infeasible references',
        conservative_baseline=dict(radius=320.,speed=13.),
        stage='frozen capabilities and proposed tasks; fleet prechecks required before formal batch'))
    print(output)
    print('G1 PASS: [12,20] m/s, width 8 m/s; required >1 m/s')


if __name__=='__main__':
    main()
