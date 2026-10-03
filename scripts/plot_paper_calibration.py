#!/usr/bin/env python3
"""Plot every tested steady speed step and the measured Plane turn relation."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from scripts.plot_paper_run import save, COLORS
from uav_formation.experiment_artifacts import write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',default='runs/paper_hetero'); parser.add_argument('--output',required=True)
    args=parser.parse_args(); root=Path(args.root); output=Path(args.output)
    output.mkdir(parents=True,exist_ok=False)
    paths=dict(Forward='selection/freestyle_flight_03',Backward='calibration/freestyle_backward_01',
               Left='calibration/freestyle_left_01',Right='calibration/freestyle_right_01')
    data=[]
    for direction,path in paths.items():
        for step in json.loads((root/path/'result.json').read_text())['steps']:
            data.append(dict(platform='Rotor',direction=direction,command=step['speed'],mean=step['mean'],
                             minimum=step['minimum'],maximum=step['maximum'],status=step['status'],source=str(root/path/'result.json')))
    fig,ax=plt.subplots(figsize=(7.2,3.6))
    for i,direction in enumerate(paths):
        rows=[r for r in data if r['direction']==direction]
        x=np.array([r['command'] for r in rows]); y=np.array([r['mean']-r['command'] for r in rows])
        bounds=np.array([[r['mean']-r['minimum'] for r in rows],[r['maximum']-r['mean'] for r in rows]])
        ax.errorbar(x+(i-1.5)*.25,y,yerr=bounds,fmt=('o','s','^','D')[i],ms=3.5,capsize=2,lw=.7,
                    color=(*COLORS,'#CC79A7')[i],label=direction)
    ax.axhline(0,color='#777777',lw=.6)
    ax.axvline(20,color='#777777',lw=.8,ls='--',label='Frozen ceiling')
    ax.set(xlabel='Commanded directional speed (m/s)',ylabel='Measured minus commanded speed (m/s)')
    ax.set_xticks([2,6,10,14,18,22,26]); ax.grid(alpha=.15)
    ax.legend(loc='lower left',bbox_to_anchor=(0,1.02),ncol=3,borderaxespad=0.)
    save(fig,output,'rotor_directional_calibration')
    plane=[]
    for path in ('selection/plane_speed_down_01','selection/plane_speed_up_01'):
        for step in json.loads((root/path/'result.json').read_text())['steps']:
            row=dict(platform='Wing',direction='Forward',command=step['speed'],mean=step['mean'],
                     minimum=step['minimum'],maximum=step['maximum'],status=step['status'],source=str(root/path/'result.json'))
            plane.append(row); data.append(row)
    plane.sort(key=lambda r:r['command'])
    fig,ax=plt.subplots(figsize=(7.2,3.6))
    ax.axvspan(12,24,color='#CCCCCC',alpha=.3,label='Frozen operating interval')
    ax.errorbar([r['command'] for r in plane],[r['mean']-r['command'] for r in plane],
                yerr=[[r['mean']-r['minimum'] for r in plane],[r['maximum']-r['mean'] for r in plane]],
                fmt='o',color='#0072B2',ms=4,capsize=2,lw=.8,label='Steady-window mean and range')
    ax.axhline(0,color='#777777',lw=.6)
    ax.set(xlabel='Commanded airspeed (m/s)',ylabel='Measured minus commanded airspeed (m/s)')
    ax.set_xticks([r['command'] for r in plane]); ax.grid(alpha=.15)
    ax.legend(loc='lower left',bbox_to_anchor=(0,1.02),ncol=2,borderaxespad=0.)
    save(fig,output,'wing_airspeed_calibration')
    turns=json.loads((root/'calibration/plane_bank_01/result.json').read_text())['steps']
    fig,ax=plt.subplots(figsize=(7.2,3.6))
    for i,speed in enumerate((12,18,24)):
        for sign in (1,-1):
            rr=[r for r in turns if r['speed']==speed and r['sign']==sign]
            ax.scatter([r['bank']+(i-1)*.35 for r in rr],[abs(r['measured_mean_lateral_accel']) for r in rr],
                       color=COLORS[i],marker='o' if sign==1 else 'x',s=25,
                       label='%d m/s %s'%(speed,'Positive turn' if sign==1 else 'Negative turn'))
    angles=np.linspace(8,32,100)
    ax.plot(angles,9.80665*np.tan(np.radians(angles)),ls='--',color='#555555',lw=.9,label='Ideal coordinated turn')
    ax.axhline(2.5,color='#555555',ls=':',lw=.9,label='Frozen lateral bound')
    ax.set(xlabel='Commanded bank magnitude (degrees)',ylabel='Lateral acceleration magnitude (m/s²)')
    ax.set_xticks([10,20,30]); ax.grid(alpha=.15)
    ax.legend(loc='lower left',bbox_to_anchor=(0,1.02),ncol=3,borderaxespad=0.)
    save(fig,output,'wing_turn_model_check')
    with (output/'speed_source.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(data[0])); writer.writeheader(); writer.writerows(data)
    write_json(output/'turn_source.json',turns)
    write_json(output/'figure_contract.json',dict(backend='Python matplotlib',archetype='single-axis calibration plots',
        conclusion='finite sampled operating envelope is supported by independent runs; the Plane turn model differs from ideal bank-only kinematics',
        exclusions='none within selected frozen calibration runs; all earlier failed development runs remain in selection/calibration directories',
        statistic='steady-window mean, whiskers are observed minimum/maximum, not confidence intervals; one continuous run per direction/sweep',
        offsets='rotor command-axis offsets +/-0.375 m/s and turn bank offsets +/-0.35 degrees separate coincident markers only; actual command values are in source data',
        uncertainty='finite sampled envelope, no claim of physical maximum or continuous-envelope proof'))
    (output/'captions.md').write_text(
        'Each speed point is a complete prespecified steady window; whiskers show the observed range, not a confidence interval. '
        'Small horizontal offsets separate coincident directional and bank markers and do not change the underlying command values. '
        'The Plane turn plot includes all 18 left/right, speed and bank conditions. '
        'The dashed ideal relationship overpredicts measured lateral acceleration for this simulator, motivating the additional empirical lateral bound. '
        'Frozen operating values remain below the tested limits; sampled evidence is not a real-aircraft or continuous-envelope proof.\n')


if __name__=='__main__':
    main()
