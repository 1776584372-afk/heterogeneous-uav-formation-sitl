#!/usr/bin/env python3
"""Plot measured flight traces from exported telemetry (no fabricated results)."""
import argparse
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('directory')
    args=parser.parse_args(); directory=Path(args.directory)
    with (directory/'telemetry.csv').open() as stream:
        rows=[r for r in csv.DictReader(stream) if r['phase']=='track']
    if not rows: raise ValueError('no tracking data to plot')
    fig,axes=plt.subplots(2,2,figsize=(10,8),constrained_layout=True)
    for i in sorted({int(r['member']) for r in rows}):
        rr=[r for r in rows if int(r['member'])==i]
        def v(key): return np.array([float(r[key]) for r in rr])
        color=('tab:blue','tab:red','tab:green','tab:orange')[i%4]
        t=v('t')-float(rows[0]['t'])
        axes[0,0].plot(v('x'),v('y'),color=color,label='UAV %d measured'%i)
        axes[0,0].plot(v('ref_x'),v('ref_y'),'--',color=color,alpha=.6)
        axes[0,1].plot(t,np.hypot(v('x')-v('ref_x'),v('y')-v('ref_y')),color=color,label='UAV %d'%i)
        axes[1,0].plot(t,v('speed'),color=color,label='UAV %d measured'%i)
        axes[1,0].plot(t,v('ref_speed'),'--',color=color,alpha=.6)
        if i==0: axes[1,1].plot(t,v('nu'),color='black')
    axes[0,0].set(xlabel='East (m)',ylabel='North (m)',title='Solid: measured; dashed: reference',aspect='equal')
    axes[0,1].set(xlabel='Tracking time (s)',ylabel='Planar reference error (m)')
    axes[1,0].set(xlabel='Tracking time (s)',ylabel='Speed (m/s)')
    axes[1,1].set(xlabel='Tracking time (s)',ylabel='Common progress rate (m/s)')
    for ax in axes.flat: ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8); axes[0,1].legend(fontsize=8)
    fig.savefig(directory/'flight_summary.png',dpi=180)
    fig.savefig(directory/'flight_summary.pdf')
    print(directory/'flight_summary.png')


if __name__=='__main__': main()
