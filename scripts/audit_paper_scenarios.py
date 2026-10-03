#!/usr/bin/env python3
"""Evaluate predefined task variations without changing measured capabilities."""
import argparse
from copy import deepcopy
from pathlib import Path

import yaml

from uav_formation.experiment_config import load_config, coordinator_from_config
from uav_formation.experiment_artifacts import freeze_run, export_plan, write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    cfg,_=load_config(args.config)
    root=Path(args.output).resolve(); root.mkdir(parents=True,exist_ok=False)
    variations={
        'direct_wide':dict(radii=[320.],nominal_radius=320.),
        'dynamic_nominal':dict(radii=[80.],nominal_radius=80.),
        'bounded_reconstruction':dict(),
        'radius_cap_120':dict(radii=[60.,80.,100.,120.]),
        'short_legs_nonmonotonic':dict(),
        'slot_swap':dict(),
    }
    write_json(root/'scan_protocol.json',dict(base_config=cfg,variations=variations,
        short_leg_m=600.,slot_swap='exchange lateral slots of member 0 and member 2; platform capabilities fixed',
        scope='offline task variations; no flight claims'))
    summary={}
    for name,changes in variations.items():
        task=deepcopy(cfg); task['name']+='_'+name; task['planner'].update(changes)
        if name=='short_legs_nonmonotonic':
            task['waypoints']=[[0.,0.],[600.,0.],[600.,600.],[1200.,600.]]
        if name=='slot_swap':
            task['members'][0]['r_y'],task['members'][2]['r_y']=task['members'][2]['r_y'],task['members'][0]['r_y']
        path=root/(name+'.yaml'); path.write_text(yaml.safe_dump(task,sort_keys=False))
        task,members=load_config(path)
        directory=freeze_run(root/name,task,'offline_task_audit')
        result=coordinator_from_config(task,members).plan(task['waypoints'])
        export_plan(directory,result,members)
        summary[name]=dict(feasible=result.feasible,radius=result.selected_radius,
            candidates=[dict(radius=c.radius,status=c.status) for c in result.candidates])
        print(name,summary[name],flush=True)
    write_json(root/'summary.json',summary)


if __name__=='__main__':
    main()
