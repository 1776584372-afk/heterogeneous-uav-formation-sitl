#!/usr/bin/env python3
"""Necessary-condition contradictions independent of the trajectory optimizer."""
import argparse
from pathlib import Path

from uav_formation.capability_model import slot_scale, team_interval
from uav_formation.experiment_config import load_config
from uav_formation.experiment_artifacts import write_json
from uav_formation.icc_traj import IccRoute


def certificate(members,k,kp):
    lower,upper,_,_=team_interval(members,k)
    if lower>upper:
        return dict(static_feasible=False,lower=lower,upper=upper)
    zlo,zhi=lower**2,upper**2
    for a in members:
        qa=slot_scale(a.r_y,k)
        for b in members:
            qb=slot_scale(b.r_y,k)
            # L_a(z)-U_b(z) is affine; check its minimum on the entire
            # static z interval, not just at the nominal speed.
            intercept=2*a.a_t_min/qa-2*b.a_t_max/qb
            slope=2*kp*(a.r_y/qa-b.r_y/qb)
            gap=min(intercept+slope*zlo,intercept+slope*zhi)
            if gap>1e-8:
                return dict(static_feasible=True,progress_interval=[lower,upper],
                    necessary_condition_contradiction=True,lower_member=a.member_id,upper_member=b.member_id,
                    gap_minimum=gap,gap_units='m/s^2 for dz/ds bounds',
                    affine_gap=dict(intercept=intercept,slope=slope),z_interval=[zlo,zhi])
    return dict(static_feasible=True,progress_interval=[lower,upper],
                necessary_condition_contradiction=False,note='absence of this certificate is not a feasibility proof')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-root',default='runs/paper_hetero/frozen_v1')
    parser.add_argument('--output',required=True)
    args=parser.parse_args(); output=Path(args.output); output.mkdir(parents=True,exist_ok=False)
    results=[]
    for scene,radii in (('s1',[80.]),('s2',[100.,120.,160.,200.]),('s3',[80.,100.,120.,160.])):
        cfg,members=load_config(Path(args.config_root)/(scene+'.yaml'))
        for radius in radii:
            route=IccRoute(cfg['waypoints'],radius,1.,trans_len=cfg['planner']['trans_len'],strict_geometry=True)
            transition=next(seg for seg in route._segs if seg[2]!=seg[3])
            s=.5*(transition[0]+transition[1])
            k,kp=route.curvature_at(s),route.curvature_derivative_at(s)
            result=dict(scene=scene,radius=radius,s=s,kappa=k,kappa_prime=kp,
                        **certificate(members,k,kp))
            results.append(result)
            print(scene,radius,result.get('necessary_condition_contradiction'),flush=True)
    write_json(output/'certificates.json',results)
    (output/'README.md').write_text(
        '# 动态不可行必要条件核验\n\n'
        '由 a_t,i=(q_i/2) z\'−r_y,i kappa\' z 得到每机对 z\' 的上下界。'
        '对任一成员下界 L_a(z) 与另一成员上界 U_b(z)，若在完整静态 z 区间内始终 L_a>U_b，'
        '则该路径位置不存在共同进度加速度，整条候选不可行。差值为仿射函数，直接检查两个区间端点即可。\n\n'
        '本文件在首个五次曲率过渡中点核验上述矛盾，独立于 HiGHS/Clarabel 输出。'
        '数值为双精度计算；未找到证书不能反推整段可行，仍须完整规划与复核。\n')


if __name__=='__main__':
    main()
