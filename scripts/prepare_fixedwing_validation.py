#!/usr/bin/env python3
"""Create separate development tasks; never rewrite frozen capability evidence."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import yaml

from uav_formation.experiment_artifacts import ROOT, write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    frozen=ROOT/'runs/paper_hetero/frozen_v1'
    s2=yaml.safe_load((frozen/'s2.yaml').read_text())
    s3=yaml.safe_load((frozen/'s3.yaml').read_text())
    s1=yaml.safe_load((frozen/'s1.yaml').read_text())
    rotor=s1['members'][1]
    tasks={}
    def task(name,source,waypoints,radius=160.,radii=None,single=False,speed=14.):
        cfg=deepcopy(source);cfg['name']=name
        cfg['capability_evidence']='prior_backend_calibration_pending_revalidation'
        cfg['capability_source']=str(frozen/'capabilities_verified.yaml')
        cfg['waypoints']=waypoints
        cfg['fixedwing_guidance']=dict(lookahead=40.,progress_gain=.3,
            guidance_mode='curvature_ff',cross_track_gain=1.,
            course_to_bank_gain=0.8726646259971648)
        cfg['acceptance']=dict(tracking_rmse_m=3.,tracking_max_m=8.,altitude_error_max_m=2.,
            maximum_constraint_violation_time_fraction=0.,constraint_tolerance=1e-3)
        cfg['planner'].update(nominal_radius=radius,radii=radii or [radius],
            nominal_speed=speed,start_speed=speed,end_speed=speed,
            bounds=[-100.,-1800.,1800.,1800.])
        if single:
            cfg['members']=[deepcopy(cfg['members'][1])]
            cfg['members'][0].update(member_id=0,r_y=0.)
        tasks[name]=cfg
        return cfg
    straight=[[0.,0.],[900.,0.]]
    left=[[0.,0.],[600.,0.],[600.,600.],[1200.,600.]]
    right=[[x,-y] for x,y in left]
    task('s1_turns',s1,left,80.,[60.,80.,100.,120.,160.,200.,240.])
    task('plane_single_straight',s2,straight,single=True)
    task('plane_single_left_right',s2,left,single=True)
    task('plane_single_right_left',s2,right,single=True)
    task('s2_direct',s2,left,240.)
    task('s2_dynamic_nominal',s2,left,160.)
    task('s2_reconstruct',s2,left,80.,[60.,80.,100.,120.,160.,200.,240.])
    task('s2_bounded_failure',s2,left,60.,[60.,80.])
    for name,pts in [('left',left),('right',right)]:
        for slot in (-10.,10.):
            cfg=task('s3_%s_wing_%s'%(name,'negative' if slot<0 else 'positive'),s3,pts,80.,[60.,80.,100.,120.,160.,200.,240.])
            if slot<0:
                cfg['members'][0]['r_y'],cfg['members'][2]['r_y']=10.,-10.
    cfg=task('s3_ablation_left_outer',s3,left[:3],60.,[60.,80.,100.,120.,160.,200.,240.])
    cfg['members'][0]['r_y'],cfg['members'][2]['r_y']=10.,-10.
    for speed in (12.,14.):
        cfg=task('rotor_straight_%g'%speed,s2,straight,single=True,speed=speed)
        cfg['members']=[dict(rotor,member_id=0,r_y=0.)]
    for name,cfg in tasks.items():
        (args.output/(name+'.yaml')).write_text(yaml.safe_dump(cfg,sort_keys=False))
    write_json(args.output/'protocol.json',dict(tasks=list(tasks),
        purpose='development verification; no automatic capability transfer to a changed controller',
        matrix='single Plane then S2; S3 flight requires measured rotor straight common-speed gate',
        frozen_limits_unchanged=True,
        control='Plane tangent/cross-track guidance with curvature-to-course feedforward + bounded along-track airspeed correction + TECS',
        evaluation='raw measured constraints, all attempts retained; completion is separate from acceptance',
        repetition='qualify each group before freezing a repeated formal batch'))
    print(args.output.resolve())


if __name__=='__main__': main()
