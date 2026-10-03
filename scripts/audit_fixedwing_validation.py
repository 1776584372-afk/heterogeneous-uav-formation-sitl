#!/usr/bin/env python3
"""Evaluate the full predefined matrix and static/dynamic distinctions offline."""
import argparse
from dataclasses import asdict
from pathlib import Path
import numpy as np

from uav_formation.experiment_artifacts import freeze_run, export_plan, write_json
from uav_formation.experiment_config import load_config, coordinator_from_config
from uav_formation.capability_model import team_interval
from uav_formation.icc_traj import IccRoute
from scripts.certify_dynamic_conflicts import certificate


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tasks',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    summary={}
    for path in sorted(args.tasks.glob('*.yaml')):
        cfg,members=load_config(path)
        directory=freeze_run(args.output/path.stem,cfg,'offline_fixedwing_validation')
        lo,hi,lo_owner,hi_owner=team_interval(members,0.)
        result=coordinator_from_config(cfg,members).plan(cfg['waypoints'])
        export_plan(directory,result,members)
        records=[]
        for candidate in result.candidates:
            item=dict(radius=candidate.radius,status=candidate.status)
            try:
                route=IccRoute(cfg['waypoints'],candidate.radius,1.,trans_len=cfg['planner']['trans_len'],strict_geometry=True)
                data=route.sample(cfg['planner']['ds'])
                intervals=[team_interval(members,k) for k in data['kappa']]
                item['all_sampled_static_intervals_nonempty']=all(l<=u for l,u,_,_ in intervals)
                item['minimum_sampled_interval_width']=min(u-l for l,u,_,_ in intervals)
                conflicts=[]
                for seg in route._segs:
                    if seg[2]!=seg[3]:
                        ss=.5*(seg[0]+seg[1])
                        proof=certificate(members,route.curvature_at(ss),route.curvature_derivative_at(ss))
                        if proof.get('necessary_condition_contradiction'):
                            conflicts.append(dict(s=ss,**proof))
                item['dynamic_necessary_condition_conflicts']=conflicts
            except ValueError as exc:
                item['geometry_detail']=str(exc)
            records.append(item)
        entry=dict(straight_interval=[lo,hi],straight_nonempty=lo<=hi,
            lower_owner=lo_owner,upper_owner=hi_owner,feasible=result.feasible,
            selected_radius=result.selected_radius,candidates=records,
            evidence='offline reference only; static sampling is not continuous proof')
        write_json(directory/'classification.json',entry);summary[path.stem]=entry
        print(path.stem,result.feasible,result.selected_radius,flush=True)
    write_json(args.output/'summary.json',summary)


if __name__=='__main__': main()
