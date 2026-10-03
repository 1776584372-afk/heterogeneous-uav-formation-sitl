#!/usr/bin/env python3
"""Generate and independently audit reference artifacts, without launching a flight."""
import argparse
from datetime import datetime
from uav_formation.experiment_config import load_config, coordinator_from_config
from uav_formation.experiment_artifacts import freeze_run, export_plan


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default='config/experiments/s1.yaml')
    parser.add_argument('--output')
    parser.add_argument('--nominal-only',action='store_true',help='A3 geometry ablation')
    args=parser.parse_args()
    cfg,members=load_config(args.config)
    cfg['method']='A3' if args.nominal_only else 'F'
    directory=freeze_run(args.output or 'runs/plan-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'),cfg,'reference_only')
    result=coordinator_from_config(cfg,members,nominal_only=args.nominal_only).plan(cfg['waypoints'])
    export_plan(directory,result,members)
    print('Artifacts:',directory.resolve())
    print('Verified reference:',result.feasible,'selected radius:',result.selected_radius)
    for c in result.candidates:
        print('R=%g: %s' % (c.radius,c.status))
    return 0 if result.feasible else 2


if __name__ == '__main__': raise SystemExit(main())
