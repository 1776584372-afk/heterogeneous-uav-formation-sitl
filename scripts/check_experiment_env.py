#!/usr/bin/env python3
"""Read-only readiness check, including an actual numerical solver invocation."""
import argparse
import importlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--output')
    args=parser.parse_args()
    report=dict(python=sys.executable,architecture=platform.machine(),checks={},errors=[])
    for module in ('numpy','scipy','yaml','rclpy','rosbag2_py','quadrotor_msgs.msg','mavros_msgs.msg','matplotlib'):
        try:
            loaded=importlib.import_module(module)
            report['checks'][module]=dict(path=getattr(loaded,'__file__',''),version=getattr(loaded,'__version__','available'))
        except Exception as exc: report['errors'].append(module+': '+str(exc))
    try:
        from scipy.optimize import minimize,linprog
        assert minimize(lambda x:(x[0]-1)**2,[0.]).success
        assert linprog([1.],bounds=[(1.,2.)],method='highs').success
        report['checks']['numerical_solvers']='SLSQP import/optimizer and HiGHS LP passed'
    except Exception as exc: report['errors'].append('numerical solver: '+str(exc))
    for name in ('arducopter','arduplane'):
        binary=Path.home()/'ardupilot/build/sitl/bin'/name
        report['checks'][name]=str(binary) if binary.is_file() else 'missing'
        if not binary.is_file(): report['errors'].append(name+' missing')
    for cmd in ('ros2','gz'):
        report['checks'][cmd]=shutil.which(cmd)
    report['checks']['disk_free_gb']=round(shutil.disk_usage('.').free/1024**3,2)
    report['checks']['ardupilot_commit']=subprocess.check_output(['git','-C',str(Path.home()/'ardupilot'),'rev-parse','HEAD'],text=True).strip()
    text=json.dumps(report,ensure_ascii=False,indent=2)
    if args.output: Path(args.output).write_text(text+'\n')
    print(text)
    return bool(report['errors'])


if __name__=='__main__': raise SystemExit(main())
