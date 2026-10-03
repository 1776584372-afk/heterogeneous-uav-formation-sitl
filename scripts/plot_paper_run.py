#!/usr/bin/env python3
"""Traceable single-panel plots of measured fleet data, with no smoothing."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from uav_formation.experiment_config import load_config
from uav_formation.experiment_artifacts import write_json
from uav_formation.paper_metrics import signal_derivative


COLORS=('#0072B2','#D55E00','#009E73')
plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['DejaVu Sans'],'font.size':8,'axes.labelsize':8,
                     'legend.fontsize':7,'pdf.fonttype':42,'svg.fonttype':'none',
                     'axes.spines.right':False,'axes.spines.top':False,'legend.frameon':False})


def save(fig, directory, name):
    # Equal-aspect paths can change tick positions during the first draw.
    fig.canvas.draw()
    fig.tight_layout()
    fig.canvas.draw()
    fig.tight_layout()
    fig.canvas.draw()
    ax=fig.axes[0]
    write_json(directory/(name+'.alignment.json'),dict(status='NOT APPLICABLE',
        reason='single plot area',axes_rectangle=list(ax.get_position().bounds)))
    fig.savefig(directory/(name+'.pdf'))
    fig.savefig(directory/(name+'.svg'))
    fig.savefig(directory/(name+'.png'),dpi=600)
    plt.close(fig)


def plot(directory, output=None):
    root=Path(directory); cfg,members=load_config(root/'config.json')
    with (root/'flight.csv').open() as stream:
        all_rows=list(csv.DictReader(stream))
    rows=[r for r in all_rows if r['phase']=='track']
    output=Path(output) if output is not None else root/'plots'
    output.mkdir(parents=True,exist_ok=True)
    contract=dict(backend='Python matplotlib',archetype='single-panel quantitative traces',
        evidence='recorded measured telemetry and contemporaneous reference; no fabricated or interpolated flight samples',
        source=str((root/'flight.csv').resolve()),input_rows=len(all_rows),plotted_rows=len(rows),
        exclusion='join/exit outside predefined task window; original rows retained',
        statistics='one run; all task samples; no smoothing or repeat uncertainty on a single-run trace',
        claim='assess path tracking, errors, speed bounds and common-reference feasibility for this run',
        dimensions_inches=[7.2,3.6],formats=['PDF','SVG','PNG'],
        status=json.loads((root/'flight_status.json').read_text()))
    write_json(output/'figure_contract.json',contract)
    if not rows:
        return
    groups={m.member_id:[r for r in rows if int(r['member'])==m.member_id] for m in members}
    def col(group,key):
        return np.asarray([float(r[key]) for r in group])
    for name,ylabel in (('path','North (m)'),('tracking_error','Position error (m)'),('speed','Speed (m/s)')):
        fig,ax=plt.subplots(figsize=(7.2,3.6))
        for m in members:
            rr=groups[m.member_id]; color=COLORS[m.member_id%len(COLORS)]
            label=('Wing' if m.platform=='fixedwing' else 'Rotor')+' '+str(m.member_id)
            t=col(rr,'t')
            if name=='path':
                ax.plot(col(rr,'ref_x'),col(rr,'ref_y'),color=color,lw=.8,ls='--',alpha=.6)
                ax.plot(col(rr,'x'),col(rr,'y'),color=color,lw=1.,label=label)
            elif name=='tracking_error':
                ax.plot(t,np.hypot(col(rr,'x')-col(rr,'ref_x'),col(rr,'y')-col(rr,'ref_y')),color=color,lw=.8,label=label)
            else:
                actual=col(rr,'airspeed') if m.platform=='fixedwing' else np.hypot(col(rr,'vx'),col(rr,'vy'))
                ax.plot(t,actual,color=color,lw=.8,label=label)
                ax.plot(t,col(rr,'ref_speed'),color=color,lw=.7,ls='--',alpha=.7)
        ax.set(xlabel='East (m)' if name=='path' else 'Task time (s)',ylabel=ylabel)
        if name=='path':
            ax.set_aspect('equal',adjustable='datalim')
        if name=='speed':
            bounds=sorted({v for m in members for v in (m.v_min,m.v_max) if v>0})
            for v in bounds:
                ax.axhline(v,color='#666666',lw=.6,ls=':')
        ax.legend(loc='lower left',bbox_to_anchor=(0.,1.02),ncol=3,borderaxespad=0.)
        ax.grid(alpha=.15)
        save(fig,output,name)
    rr=groups[members[0].member_id]
    fig,ax=plt.subplots(figsize=(7.2,3.6))
    t=col(rr,'t')
    ax.fill_between(t,col(rr,'lower'),col(rr,'upper'),color='#BBBBBB',alpha=.3,label='Team interval')
    ax.plot(t,col(rr,'nu'),color='#0072B2',lw=1.,label='Common progress rate')
    ax.set(xlabel='Task time (s)',ylabel='Progress rate (m/s)')
    ax.legend(loc='lower left',bbox_to_anchor=(0.,1.02),ncol=2,borderaxespad=0.)
    ax.grid(alpha=.15)
    save(fig,output,'progress_rate')
    if len(members)>1:
        fig,ax=plt.subplots(figsize=(7.2,3.6))
        complete=True
        for i,m in enumerate(members):
            for other in members[i+1:]:
                aa,bb=groups[m.member_id],groups[other.member_id]
                if len(aa)!=len(bb) or not np.array_equal(col(aa,'t'),col(bb,'t')):
                    complete=False; continue
                distance=np.hypot(col(aa,'x')-col(bb,'x'),col(aa,'y')-col(bb,'y'))
                ax.plot(col(aa,'t'),distance-abs(m.r_y-other.r_y),lw=.8,
                        label='Pair %d-%d'%(m.member_id,other.member_id))
        if complete:
            ax.set(xlabel='Task time (s)',ylabel='Pairwise distance error (m)')
            ax.legend(loc='lower left',bbox_to_anchor=(0,1.02),ncol=3,borderaxespad=0.)
            ax.grid(alpha=.15); save(fig,output,'shape_error')
        else:
            plt.close(fig)
            write_json(output/'shape_plot_status.json',dict(status='INCOMPLETE_SYNCHRONIZED_ROWS'))
    wings=[m for m in members if m.platform=='fixedwing']
    if wings:
        fig,ax=plt.subplots(figsize=(7.2,3.6))
        for m in wings:
            rr=groups[m.member_id]; color=COLORS[m.member_id%len(COLORS)]
            ax.plot(col(rr,'t'),np.degrees(col(rr,'roll')),color=color,lw=.9,label='Wing %d measured'%m.member_id)
            if all(r['command_bank']!='' for r in rr):
                bank=col(rr,'command_bank')
            else:
                bank=-np.arctan(col(rr,'ref_speed')**2*col(rr,'ref_member_curvature')/9.80665)
            ax.plot(col(rr,'t'),np.degrees(bank),color=color,lw=.7,ls='--',alpha=.6)
        for bound in sorted({np.degrees(m.bank_max) for m in wings}):
            ax.axhline(bound,color='#777777',lw=.7,ls=':')
            ax.axhline(-bound,color='#777777',lw=.7,ls=':')
        ax.set(xlabel='Task time (s)',ylabel='Bank angle (degrees, NED)')
        ax.legend(loc='lower left',bbox_to_anchor=(0,1.02),ncol=3,borderaxespad=0.)
        ax.grid(alpha=.15); save(fig,output,'bank_angle')
    rotors=[m for m in members if m.platform=='multirotor']
    if rotors:
        fig,ax=plt.subplots(figsize=(7.2,3.6))
        for m in rotors:
            rr=groups[m.member_id];color=COLORS[m.member_id%len(COLORS)]
            t=col(rr,'t');source_t=col(rr,'attitude_t') if 'attitude_t' in rr[0] else t
            actual=signal_derivative(np.unwrap(col(rr,'yaw')),source_t)
            ax.plot(t,actual,color=color,lw=.7,label='Rotor %d actual'%m.member_id)
            ax.plot(t,col(rr,'ref_speed')*col(rr,'ref_member_curvature'),color=color,lw=.8,ls='--',
                    label='Rotor %d reference'%m.member_id)
            if cfg.get('copter_yaw_rate_feedforward') and all(r.get('command_yaw_rate','')!='' for r in rr):
                ax.plot(t,col(rr,'command_yaw_rate'),color='#D55E00',lw=.6,ls='-.',
                        label='Rotor %d sent'%m.member_id)
            if m.yaw_rate_max is not None:
                for bound in (-m.yaw_rate_max,m.yaw_rate_max):
                    ax.axhline(bound,color='#555555',lw=.7,ls=':')
        ax.set(xlabel='Task time (s)',ylabel='Heading rate (rad/s, ENU)')
        ax.legend(loc='lower left',bbox_to_anchor=(0,1.02),ncol=3,borderaxespad=0.)
        ax.grid(alpha=.15);save(fig,output,'yaw_rate')
    for name in ('tangential_acceleration','lateral_acceleration'):
        fig,ax=plt.subplots(figsize=(7.2,3.6))
        for m in members:
            rr=groups[m.member_id]; color=COLORS[m.member_id%len(COLORS)]
            t=col(rr,'t'); source_t=col(rr,'velocity_t') if 'velocity_t' in rr[0] else t
            vx,vy=col(rr,'vx'),col(rr,'vy'); speed=np.hypot(vx,vy)
            q=1-m.r_y*col(rr,'kappa')
            if name=='tangential_acceleration':
                actual=signal_derivative(speed,source_t)
                reference=q*col(rr,'nu_dot')-m.r_y*col(rr,'kappa_prime')*col(rr,'nu')**2
                bounds=(m.a_t_min,m.a_t_max)
            else:
                actual=np.abs(vx*signal_derivative(vy,source_t)-vy*signal_derivative(vx,source_t))/np.maximum(speed,.1)
                reference=np.abs(q*col(rr,'kappa')*col(rr,'nu')**2)
                bounds=(m.a_lat_max,)
            ax.plot(t,actual,color=color,lw=.7,label=('Wing' if m.platform=='fixedwing' else 'Rotor')+' '+str(m.member_id))
            ax.plot(t,reference,color=color,lw=.7,ls='--',alpha=.6)
            for bound in bounds:
                ax.axhline(bound,color=color,lw=.6,ls=':',alpha=.6)
        ax.set(xlabel='Task time (s)',ylabel=name.replace('_',' ').capitalize()+' (m/s²)')
        ax.legend(loc='lower left',bbox_to_anchor=(0,1.02),ncol=3,borderaxespad=0.)
        ax.grid(alpha=.15); save(fig,output,name)
    (output/'captions.md').write_text(
        'All plots show the entire recorded task window of one run. Join and exit are excluded by the frozen metric rule. '
        'Solid curves are measured; dashed curves in path/speed plots are reference values. '
        'Rotor speed is horizontal ground speed; wing speed is measured airspeed in the zero-wind simulation. '
        'Dotted speed lines are the distinct positive operation bounds, not fitted limits. '
        'Bank plots use measured roll (solid) and frozen limits (dotted); positive is right bank in NED. '
        'Dashed bank is the issued roll command for the external-roll backend, or the nominal coordinated-turn reference for the course backend (not a sent roll command). '
        'Shape plots use all pairwise distance residuals, with no time smoothing. '
        'Heading-rate plots differentiate unwrapped measured yaw on unique attitude timestamps without smoothing; '
        'dashed is path reference, dash-dot is logged sent rate, dotted is the frozen limit. '
        'Acceleration plots use the frozen unique-source-time derivative, identical to primary metrics; '
        'solid is actual, dashed is reference, and color-matched dotted lines are member limits. '
        'The common-progress plot shows reference quantities only. Completion/failure is recorded in figure_contract.json.\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('directory')
    plot(parser.parse_args().directory)
