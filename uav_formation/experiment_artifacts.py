"""Frozen inputs and full reference exports for reproducibility."""
from dataclasses import asdict
import csv
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

from .capability_model import Limit, G, member_demands, dynamic_margin

ROOT = Path(__file__).resolve().parents[1]


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def freeze_run(directory, cfg, kind):
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    write_json(directory/'config.json', cfg)
    sources = {}
    for pattern in ('uav_formation/*.py', 'scripts/*.py', 'scripts/*.sh', 'config/experiments/*', 'tests/test_*.py', 'requirements-paper.txt'):
        for path in sorted(ROOT.glob(pattern)):
            if path.is_file():
                sources[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    for relative in sources:
        destination=directory/'source_snapshot'/relative
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/relative,destination)
    snapshot=directory/'config_snapshot'
    snapshot.mkdir()
    write_json(snapshot/'task.json',cfg)
    if cfg.get('capability_source'):
        shutil.copy2(cfg['capability_source'],snapshot/'capabilities_verified.yaml')
    import numpy, scipy
    manifest = dict(kind=kind, created_unix=time.time(), python=sys.version,
                    architecture=platform.machine(), numpy=numpy.__version__, scipy=scipy.__version__,
                    source_sha256=sources,
                    git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                    git_status=subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True),
                    constraint_enum={x.name:int(x) for x in Limit},
                    verification='discrete reference audit; actual flight requires telemetry metrics')
    ap=Path.home()/'ardupilot'
    if (ap/'.git').exists():
        manifest['ardupilot_commit']=subprocess.check_output(['git','-C',str(ap),'rev-parse','HEAD'],text=True).strip()
    manifest['solver']=cfg.get('planner',{}).get('objective_method','quadratic')
    manifest['solver_detail']='HiGHS discrete feasibility; Clarabel paper Eq.17 convex QP for quadratic mode'
    manifest['audit']='12 subdivisions per planning interval; both one-sided derivatives at nodes'
    write_json(directory/'manifest.json',manifest)
    write_json(directory/'run_manifest.json',manifest)
    (directory/'source_revision.txt').write_text(manifest['git_commit']+'\n'+json.dumps(sources,indent=2)+'\n')
    return directory


def export_plan(directory, result, members):
    directory = Path(directory)
    write_json(directory/'constraint_semantics.json', dict(
        planning_bounds='reference profile uses planning_members() with optional reference_reserve fractions',
        physical_bounds='member_reference.csv margins use the original frozen MemberSpec values',
        telemetry_bounds='flight.csv and acceptance.json compare measured telemetry against original frozen MemberSpec values',
        reserve_is_not_capability_change=True,
        reference_csv_columns={'lower': 'planning lower progress bound',
                               'upper': 'planning upper progress bound',
                               'lower_member': 'active planning lower owner',
                               'upper_member': 'active planning upper owner'},
        units='SI: m, s, rad, m/s, m/s^2',
    ))
    write_json(directory/'candidates.json',[asdict(c) for c in result.candidates])
    write_json(directory/'feasibility.json',dict(feasible=result.feasible,
        selected_radius=result.selected_radius,candidates=[asdict(c) for c in result.candidates],
        interpretation='discretized constraints with dense audit; SOLVER_UNRESOLVED is not infeasibility'))
    write_json(directory/'plan.json',dict(feasible=result.feasible, selected_radius=result.selected_radius,
               duration=float(result.profile.time[-1]) if result.feasible else None))
    if not result.feasible:
        return
    route,p = result.route,result.profile
    with (directory/'reference.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['t','s','nu','nu_dot','kappa','kappa_prime','lower','upper',
                         'lower_member','lower_limit','upper_member','upper_limit','radius'])
        for j,s in enumerate(p.s):
            writer.writerow([p.time[j],s,p.nu[j],.5*p.slope_at(s),route.curvature_at(s),
                route.curvature_derivative_at(s),p.team_lower[j],p.team_upper[j],
                *p.active_lower[j],*p.active_upper[j],result.selected_radius])
    with (directory/'member_reference.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['t','s','member','x','y','course','speed','curvature','heading_rate','bank',
                         'tangential_accel','bank_rate','margins_json'])
        for j,s in enumerate(p.s):
            k,kp=route.curvature_at(s),route.curvature_derivative_at(s)
            z,zp=p.z[j],p.slope_at(s)
            for m in members:
                x,y,psi,_=route.eval_drone(s,0,m.r_y)
                v,ki,omega,phi=member_demands(m,k,p.nu[j])
                q=1-m.r_y*k
                at=.5*q*zp-m.r_y*kp*z
                bank_rate=p.nu[j]*((1-2*m.r_y*k)*kp*z+q*k*zp)/G/(1+(q*k*z/G)**2)
                margins=dynamic_margin(m,k,kp,z,zp)
                writer.writerow([p.time[j],s,m.member_id,x,y,psi,v,ki,omega,phi,at,bank_rate,
                                 json.dumps({int(key):value for key,value in margins.items()})])
