#!/usr/bin/env python3
"""A0/A1/A2/A3/C/F reference-layer ablations; never flight-performance claims."""
import argparse
import csv
from dataclasses import asdict
from datetime import datetime
import json
import math
import numpy as np
from uav_formation.experiment_artifacts import freeze_run,write_json,export_plan
from uav_formation.experiment_config import load_config,coordinator_from_config
from uav_formation.capability_model import static_progress_interval,team_interval,dynamic_margin
from uav_formation.icc_traj import IccRoute


def integrate_individual_progress(directory,route,members,s,individual,dt=.05):
    """Kinematic reference-only ablation, including divergence of A1 progress.

    Continue through the endpoint; a faster fixed wing must not be frozen there.
    These are generated reference trajectories, never measured flight traces.
    """
    if any(np.min(nu)<=0 for nu in individual):
        raise ValueError('reference ablation needs positive through-flight progress')
    durations=[np.sum(2*np.diff(s)/(nu[:-1]+nu[1:])) for nu in individual]
    progress=np.zeros(len(members));spread=shape_max=0.
    with (directory/'kinematic_reference_trace.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['t','member','individual_s','x','y','reference_speed','progress_spread','shape_error_rms'])
        for t in np.arange(0,max(durations)+dt,dt):
            points=[]; speeds=[]; rates=[]
            for member,ss,nu in zip(members,progress,individual):
                rate=float(np.interp(ss,s,nu));rates.append(rate)
                end=min(ss,s[-1]);x,y,psi,_=route.eval_drone(end,0,member.r_y)
                x+=max(0,ss-s[-1])*math.cos(psi);y+=max(0,ss-s[-1])*math.sin(psi)
                points.append((x,y));speeds.append(rate*(1-member.r_y*route.curvature_at(end)))
            errors=[(math.dist(points[i],points[j])-abs(members[i].r_y-members[j].r_y))**2
                    for i in range(len(members)) for j in range(i+1,len(members))]
            shape=math.sqrt(sum(errors)/len(errors)) if errors else 0.
            current=float(np.ptp(progress));spread=max(spread,current);shape_max=max(shape_max,shape)
            for member,ss,point,speed in zip(members,progress,points,speeds):
                writer.writerow([t,member.member_id,ss,*point,speed,current,shape])
            progress+=np.asarray(rates)*dt
    return dict(max_progress_spread_m=spread,max_reference_shape_error_m=shape_max,
                time_step_s=dt,scope='kinematic reference integration, not flight error')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default='config/experiments/s3.yaml')
    parser.add_argument('--output')
    parser.add_argument('--conservative-radius',required=True,type=float)
    parser.add_argument('--conservative-speed',required=True,type=float)
    args=parser.parse_args(); cfg,members=load_config(args.config)
    cfg['comparison']=dict(conservative_radius=args.conservative_radius,conservative_speed=args.conservative_speed)
    root=freeze_run(args.output or 'runs/reference-ablation-'+datetime.now().strftime('%Y%m%d-%H%M%S'),cfg,'reference_ablation_only')
    summary={}
    for method in ('A0','A1','A2','A3','C','F'):
        directory=root/method; directory.mkdir()
        try:
            if method in ('A3','F'):
                plan=coordinator_from_config(cfg,members,nominal_only=method=='A3').plan(cfg['waypoints'])
                export_plan(directory,plan,members)
                summary[method]=dict(feasible=plan.feasible,selected_radius=plan.selected_radius,
                                     interpretation='verified discrete dynamic reference' if plan.feasible else 'no verified candidate')
                continue
            radius=args.conservative_radius if method=='C' else cfg['planner']['nominal_radius']
            nominal=args.conservative_speed if method=='C' else cfg['planner']['nominal_speed']
            route=IccRoute(cfg['waypoints'],radius,1.,trans_len=cfg['planner'].get('trans_len'),strict_geometry=True)
            data=route.sample(cfg['planner']['ds']); s=data['s']; k=data['kappa']; kp=data['kappa_prime']
            individual=[]
            for member in members:
                bounds=np.array([static_progress_interval(member,curvature)[:2] for curvature in k])
                if np.any(bounds[:,0]>bounds[:,1]) and method=='A1':
                    raise ValueError('member has an empty individual interval')
                individual.append(np.clip(nominal,bounds[:,0],bounds[:,1]) if method=='A1' else np.full(len(s),nominal))
            if method=='A2':
                intervals=np.array([team_interval(members,curvature)[:2] for curvature in k])
                if np.any(intervals[:,0]>intervals[:,1]): raise ValueError('empty team static interval')
                common=np.clip(nominal,intervals[:,0],intervals[:,1]); individual=[common for _ in members]
            violations={}; max_spread=float(np.max(np.ptp(np.array(individual),axis=0)))
            with (directory/'reference_constraints.csv').open('w',newline='') as stream:
                writer=csv.writer(stream); writer.writerow(['s','member','required_progress_rate','speed','margins_json'])
                for member,nu in zip(members,individual):
                    zp=np.gradient(nu**2,s)
                    count=0
                    for j,ss in enumerate(s):
                        margins=dynamic_margin(member,k[j],kp[j],nu[j]**2,zp[j])
                        count+=any(v < -1e-6 for v in margins.values())
                        writer.writerow([ss,member.member_id,nu[j],nu[j]*(1-member.r_y*k[j]),
                                         json.dumps({int(key):v for key,v in margins.items()})])
                    violations[member.member_id]=count/len(s)
            trace=integrate_individual_progress(directory,route,members,s,individual)
            summary[method]=dict(sample_violation_fraction=violations,max_common_progress_inconsistency=max_spread,
                integrated_reference=trace,
                interpretation='spatial sample fractions, not flight violation time; endpoint ramps excluded')
        except ValueError as exc:
            summary[method]=dict(feasible=False,reason=str(exc))
    write_json(root/'comparison.json',summary); print(root.resolve()); print(json.dumps(summary,indent=2))


if __name__=='__main__': main()
