#!/usr/bin/env python3
"""Show every completed technical repeat and prespecified group statistics."""
import argparse
import csv
import json
from pathlib import Path
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from scripts.plot_paper_run import save
from scripts.audit_paper_figures import audit
from uav_formation.experiment_artifacts import write_json


def plot(root, output=None):
    with (root/'all_runs.csv').open() as stream:
        rows=list(csv.DictReader(stream))
    summary=json.loads((root/'summary.json').read_text())
    out=Path(output) if output is not None else root/'comparison_plots'
    out.mkdir(parents=True,exist_ok=True)
    quantities=dict(fleet_tracking_rmse_m='Fleet position RMS (m)',shape_rmse_m='Shape RMS (m)',
                    minimum_separation_m='Minimum separation (m)',duration_s='Recorded task duration (s)',
                    max_member_tangential_violation_fraction='Worst-member tangential violation (%)',
                    planning_seconds='Planning computation (s)')
    colors={'F':'#0072B2','A3':'#009E73','C':'#777777'}
    good=[r for r in rows if r.get('completed')=='True' and r.get('valid_telemetry')=='True' and not r.get('metric_error')]
    names=list(summary)
    for key,label in quantities.items():
        fig,ax=plt.subplots(figsize=(7.2,3.6))
        for i,name in enumerate(names):
            selected=[r for r in good if r['scene']+'_'+r['method']==name]
            values=np.array([float(r[key]) for r in selected])
            if key.endswith('fraction'):
                values*=100
            if not len(values):
                continue
            color=colors[selected[0]['method']]
            ax.scatter([i+(int(r['seed'])-2)*.055 for r in selected],values,color=color,s=15,alpha=.7)
            ax.errorbar(i+.20,statistics.mean(values),yerr=statistics.stdev(values) if len(values)>1 else None,
                        fmt='D',color=color,ms=4,capsize=3,lw=1.)
        ax.set_xticks(range(len(names)))
        ax.set_xticklabels([n.replace('s','S',1).replace('_',' ')+'\n%d/%d'%(summary[n]['completed'],summary[n]['planned']) for n in names])
        ax.set_ylabel(label); ax.set_xlabel('Group and completed/planned repeats')
        ax.grid(axis='y',alpha=.15); save(fig,out,key)
    paired=[]
    for scene in ('s1','s2','s3'):
        for seed in range(5):
            pair={r['method']:r for r in good if r['scene']==scene and int(r['seed'])==seed and r['method'] in ('F','C')}
            if set(pair)!= {'F','C'}:
                continue
            paired.append(dict(scene=scene,seed=seed,**{key:float(pair['F'][key])-float(pair['C'][key])
                          for key in ('fleet_tracking_rmse_m','duration_s')}))
    if paired:
        with (out/'paired_differences.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(paired[0])); writer.writeheader(); writer.writerows(paired)
        for key,label in (('fleet_tracking_rmse_m','F minus C position RMS (m)'),('duration_s','F minus C task duration (s)')):
            fig,ax=plt.subplots(figsize=(7.2,3.6))
            for i,scene in enumerate(('s1','s2','s3')):
                rr=[r for r in paired if r['scene']==scene]; values=[r[key] for r in rr]
                if values:
                    ax.scatter([i+(r['seed']-2)*.035 for r in rr],values,s=18,color='#0072B2')
                    ax.errorbar(i+.15,statistics.mean(values),yerr=statistics.stdev(values) if len(values)>1 else None,
                                fmt='D',color='#333333',ms=4,capsize=3,lw=1.)
            ax.axhline(0,color='#777777',ls='--',lw=.8)
            ax.set_xticks([0,1,2]); ax.set_xticklabels(['S1','S2','S3'])
            ax.set(xlabel='Same initial-position seed, different reference method',ylabel=label)
            ax.grid(axis='y',alpha=.15); save(fig,out,'paired_'+key)
    write_json(out/'figure_contract.json',dict(backend='Python matplotlib',source=str((root/'all_runs.csv').resolve()),
        question='How do reference methods compare over every planned technical repeat, including actual constraint violations?',
        statistics='dots: each completed valid run; diamond: mean; whiskers: sample SD, not CI; five planned initial-position seeds per group',
        exclusions='failed/missing runs are counted in labels and retained in all_runs.csv; never substituted by zeros',
        input_rows=len(rows),successful_rows=len(good),paired_rows=len(paired),
        pairing='F minus C matched on scene and initial-position seed; SITL scheduling/noise is not fully paired',
        inference='descriptive only; no significance test, independence or real-aircraft generalization',
        archetype='single-axis quantitative comparisons; no multi-panel alignment needed'))
    (out/'captions.md').write_text(
        'Dots include every completed, valid technical repeat; diamonds and whiskers show mean and sample standard deviation. '
        'Group labels show completed/planned counts. Technical repeats perturb initial home position only. '
        'Tangential violation is the per-run maximum of member lower-plus-upper violation time fractions, not a fleet union probability. '
        'Paired differences subtract C from F for the same scene and seed; simulator timing/noise is not fully controlled. '
        'C uses a larger radius and thus a different path length; duration differences are not pure controller-speed improvements.\n')
    audit(out)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('root',type=Path)
    parser.add_argument('--output',type=Path,help='optional local output directory')
    args=parser.parse_args()
    plot(args.root.resolve(),args.output)
