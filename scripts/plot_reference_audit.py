#!/usr/bin/env python3
"""Plot observed planner statuses and reference intervals from saved artifacts."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from scripts.plot_paper_run import save
from uav_formation.experiment_artifacts import write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',default='runs/paper_hetero')
    parser.add_argument('--output',required=True)
    args=parser.parse_args(); root=Path(args.root); out=Path(args.output)
    out.mkdir(parents=True,exist_ok=False)
    cases=[('S1 Rotors',root/'offline_final_s1/F'),('S2 Wings',root/'offline_final_s2/F'),
           ('S3 Mixed',root/'offline_final_s3/F'),('S3 Short legs',root/'development/task_audit_v1/short_legs_nonmonotonic')]
    records=[]
    for label,path in cases:
        selected=json.loads((path/'plan.json').read_text())['selected_radius']
        for candidate in json.loads((path/'candidates.json').read_text()):
            records.append(dict(scene=label,radius=candidate['radius'],status=candidate['status'],
                                selected=candidate['radius']==selected,source=str(path/'candidates.json')))
    with (out/'candidate_source.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(records[0])); writer.writeheader(); writer.writerows(records)
    write_json(out/'figure_contract.json',dict(backend='Python matplotlib',archetype='single-axis categorical comparison',
        conclusion='fixed capabilities produce different feasible radii; short legs invalidate the largest radius',
        evidence='all recorded candidates; no candidate excluded; solver-unresolved is distinct from infeasible',
        uncertainty='deterministic offline calculation, not repeat statistics',source='candidate_source.csv',
        scope='discrete reference-layer findings, not actual flight constraint guarantees'))
    styles={'FEASIBLE':('o','#009E73','Feasible'),
            'STATIC_CONFLICT':('s','#D55E00','Static conflict'),
            'DYNAMIC_DISCRETE_INFEASIBLE':('^','#CC79A7','Dynamic conflict'),
            'GEOMETRY_REJECTED':('X','#666666','Geometry rejected'),
            'SOLVER_UNRESOLVED':('D','#E69F00','Solver unresolved'),
            'AUDIT_REJECTED':('P','#882255','Audit rejected')}
    fig,ax=plt.subplots(figsize=(7.2,3.4))
    for code,(marker,color,label) in styles.items():
        points=[r for r in records if r['status']==code]
        if points:
            ax.scatter([r['radius'] for r in points],[[c[0] for c in cases].index(r['scene']) for r in points],
                       s=45,marker=marker,c=color,label=label,zorder=3)
    selected=[r for r in records if r['selected']]
    ax.scatter([r['radius'] for r in selected],[[c[0] for c in cases].index(r['scene']) for r in selected],
               s=110,facecolors='none',edgecolors='black',linewidths=.8,label='Selected',zorder=4)
    ax.set_yticks(range(len(cases))); ax.set_yticklabels([c[0] for c in cases])
    ax.set_xticks(sorted({r['radius'] for r in records}))
    ax.set(xlabel='Candidate center radius (m)',ylim=(-.5,len(cases)-.5))
    ax.invert_yaxis(); ax.grid(alpha=.15)
    ax.legend(loc='lower left',bbox_to_anchor=(0,1.03),ncol=3,borderaxespad=0.)
    save(fig,out,'candidate_feasibility')
    for label,path in cases[:3]:
        with (path/'reference.csv').open() as stream:
            rows=list(csv.DictReader(stream))
        def col(name):
            return np.array([float(r[name]) for r in rows])
        fig,ax=plt.subplots(figsize=(7.2,3.4))
        ax.fill_between(col('s'),col('lower'),col('upper'),color='#BBBBBB',alpha=.3,label='Team interval')
        ax.plot(col('s'),col('nu'),color='#0072B2',lw=1.2,label='Dynamic progress rate')
        ax.set(xlabel='Center arc length (m)',ylabel='Progress rate (m/s)')
        ax.legend(loc='lower left',bbox_to_anchor=(0,1.02),ncol=2,borderaxespad=0.)
        ax.grid(alpha=.15)
        save(fig,out,label.split()[0].lower()+'_progress')
    (out/'captions.md').write_text(
        'Candidate feasibility uses the same frozen platform bounds and full recorded radius set. '
        'The short-leg variation changes task geometry from 800 m to 600 m legs. '
        'Dynamic conflict denotes HiGHS infeasibility of the discretized linear constraints; '
        'solver nonconvergence is never assigned that category. '
        'The outlined marker is the selected radius. Progress plots show selected reference rates '
        'and static team intervals over physical center arc length; they contain no measured flight data.\n')


if __name__=='__main__':
    main()
