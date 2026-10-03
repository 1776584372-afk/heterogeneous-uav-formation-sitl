#!/usr/bin/env python3
"""Export complete raw calibration traces and descriptive actuator ranges."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from scripts.plot_paper_run import save
from scripts.audit_paper_figures import audit
from uav_formation.experiment_artifacts import write_json


def plot_run(root):
    with (root/'telemetry.csv').open() as stream:
        rows=list(csv.DictReader(stream))
    if not rows:
        return
    manifest=json.loads((root/'manifest.json').read_text())
    protocol=json.loads((root/'protocol.json').read_text())
    result=json.loads((root/'result.json').read_text()) if (root/'result.json').exists() else {}
    out=root/'trace_plots'; out.mkdir(exist_ok=True)
    def col(key):
        return np.asarray([float(r.get(key) or 'nan') for r in rows])
    t=col('sim_s'); direction=protocol.get('direction','forward')
    active=np.asarray([r['phase'].startswith(('step_','turn_')) or r['phase']=='ramp_ready' for r in rows])
    wing=manifest.get('vehicle')=='plane'
    turn=any(r['phase'].startswith('turn_') for r in rows)
    horizontal_fields='heading_forward' in rows[0] and 'heading_left' in rows[0]
    forward=col('heading_forward' if horizontal_fields else 'forward')
    left=col('heading_left' if horizontal_fields else 'left')
    actual=col('airspeed') if wing else (
        np.hypot(col('vn'),col('ve')) if turn else
        (forward if direction in ('forward','backward') else left)*
        (-1 if direction in ('backward','right') else 1))
    for name in ('speed','altitude','attitude','actuators'):
        fig,ax=plt.subplots(figsize=(7.2,3.6))
        if name=='speed':
            ax.plot(t,actual,lw=.8,label='Measured')
            ax.plot(t,np.where(active,col('ref_speed'),np.nan),lw=.8,ls='--',label='Command')
            label='Airspeed (m/s)' if wing else 'Horizontal speed (m/s)' if turn else (
                'Horizontal directional speed (m/s)' if horizontal_fields else 'Legacy tilted-body directional speed (m/s)')
            ax.set_ylabel(label)
        elif name=='altitude':
            ax.plot(t,col('alt'),lw=.8,label='Measured')
            ax.axhline(manifest['altitude'],color='#777777',ls='--',lw=.8,label='Test altitude')
            ax.set_ylabel('Relative altitude (m)')
        elif name=='attitude':
            ax.plot(t,np.degrees(col('roll')),lw=.8,label='Roll')
            ax.plot(t,np.degrees(col('pitch')),lw=.8,label='Pitch')
            bank=np.where(active,np.degrees(col('ref_bank')),np.nan)
            if np.any(np.isfinite(bank)):
                ax.plot(t,bank,lw=.8,ls='--',label='Roll command')
            ax.set_ylabel('Attitude (degrees, NED)')
        else:
            for channel in range(1,5):
                ax.plot(t,col('servo'+str(channel)),lw=.6,label='Output '+str(channel))
            ax.set_ylabel('Recorded actuator PWM (microseconds)')
        ax.set_xlabel('Simulator time (s)'); ax.grid(alpha=.15)
        ax.legend(loc='lower left',bbox_to_anchor=(0,1.02),ncol=4,borderaxespad=0.)
        save(fig,out,name)
    windows=[]
    for step in result.get('steps',[]):
        if 'window' not in step:
            continue
        low,high=step['window']; mask=(t>=low)&(t<=high)
        ranges={}
        for key in ('throttle','servo1','servo2','servo3','servo4'):
            values=col(key)[mask]; values=values[np.isfinite(values)]
            ranges[key]=dict(min=float(min(values)),max=float(max(values))) if len(values) else None
        windows.append(dict(window=step['window'],condition={k:step[k] for k in ('speed','bank','sign','status') if k in step},
                            observed_ranges=ranges))
    write_json(out/'actuator_observations.json',dict(windows=windows,
        saturation_assessment='UNKNOWN: no calibrated normalized saturation threshold or motor-thrust headroom model; observed PWM/throttle only',
        original_pass_criteria='unchanged; saturation was recorded but was not an explicit threshold in the original protocol'))
    write_json(out/'figure_contract.json',dict(source='../telemetry.csv',input_rows=len(rows),plotted_rows=len(rows),
        backend='Python matplotlib',claim='document complete acquired calibration time histories including failed attempts',
        legacy_rotor_coordinates=not wing and not turn and not horizontal_fields,
        statistics='raw traces, no averaging or smoothing; command is displayed only during active test phases, because exit rows retain stale command fields',
        scope='takeoff/test/exit included; steady assessment windows and original PASS/FAIL remain in result.json',
        saturation='descriptive PWM traces only, not validated actuator saturation envelope',status=result.get('status','UNKNOWN')))
    (out/'captions.md').write_text(
        'Every acquired telemetry row is retained, including connection, takeoff, test and exit. '
        'Command overlays are restricted to active step/turn/ramp phases; retained command fields after mode exit are stale. '
        'Steady assessment windows, original errors and PASS/FAIL are in result.json. '
        'Current rotor directional runs use horizontal heading axes. Legacy files without heading columns show their original tilted-body FLU component, explicitly labelled; '
        'those legacy failures do not define the frozen directional bounds. '
        'Actuator PWM is descriptive; saturation headroom was not validated. No smoothing or fitted uncertainty is used.\n')
    audit(out)
    print(root.name,result.get('status','UNKNOWN'),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('runs/paper_hetero'))
    args=parser.parse_args()
    for group in ('selection','calibration'):
        for telemetry in sorted((args.root/group).glob('*/telemetry.csv')):
            if (telemetry.parent/'protocol.json').exists() and (telemetry.parent/'manifest.json').exists():
                plot_run(telemetry.parent)
