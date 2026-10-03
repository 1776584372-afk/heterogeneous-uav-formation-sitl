#!/usr/bin/env python3
"""SITL constant-speed bank/curvature calibration with both turn signs."""
import argparse
import json
import math
import statistics

from uav_formation.sitl_vehicle import SitlVehicle, AP
from uav_formation.capability_model import MemberSpec
from uav_formation.paper_tracking import PlaneCourseTracker


def set_phase(vehicle, name):
    setter = getattr(vehicle, 'set_phase', None)
    if setter is None:
        vehicle.phase = name
    else:
        setter(name)


def turn_phase_name(stage, speed, bank, sign):
    if stage not in ('settle', 'steady'):
        raise ValueError('turn stage must be settle or steady')
    return 'turn_%s_%g_%g_%d' % (stage, speed, bank, sign)


def bank_tracking_metrics(actualbank, equivalent_bank, *, course_control):
    """A curvature label is not a bank setpoint in course-control mode."""
    deviation=max(abs(value-equivalent_bank) for value in actualbank)
    if course_control:
        return ({'bank_abs_deg':max(map(abs,actualbank))},
                {'equivalent_bank_deviation_deg':deviation})
    return {'bank_error_deg':deviation}, {}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--vehicle',choices=['plane','copter'],required=True)
    p.add_argument('--model')
    p.add_argument('--defaults',help='comma-separated parameter files, applied in order')
    p.add_argument('--plane-heading-period',type=float,default=0.)
    p.add_argument('--lookahead',type=float,default=40.,help='Plane path preview in metres')
    p.add_argument('--guidance-mode',choices=['preview','frenet','curvature_ff'],default='preview')
    p.add_argument('--cross-track-gain',type=float,default=1.)
    p.add_argument('--course-to-bank-gain',type=float,default=math.radians(50.))
    p.add_argument('--speeds',default='12,18,24')
    p.add_argument('--banks',default='10,20,30')
    p.add_argument('--controller',choices=['course','bank'],default='bank')
    p.add_argument('--accel-feedforward',action='store_true')
    p.add_argument('--yaw-rate-feedforward',action='store_true')
    p.add_argument('--path-guidance',action='store_true',help='Plane: use production course preview and airspeed tracker')
    p.add_argument('--handover-command-speed',action='store_true',help='Plane path entry: initialize slew from last sent airspeed target')
    p.add_argument('--continue-after-tracking-failure',action='store_true',help='record all planned points; flight protections still abort')
    p.add_argument('--output',required=True)
    p.add_argument('--port',type=int,default=21000)
    p.add_argument('--speedup',type=float,default=2.)
    p.add_argument('--turn-signs',default='left,right')
    p.add_argument('--dataset-group',choices=['PILOT','A','B'],default='PILOT')
    p.add_argument('--repeat-id',type=int,default=0)
    p.add_argument('--protocol-id',default='v2-single-vehicle-turn')
    args=p.parse_args()
    expected_model='plane' if args.vehicle=='plane' else 'x:freestyle.json'
    if args.model not in (None,expected_model) and not args.defaults:
        p.error('a candidate model requires explicit matching --defaults')
    speeds=[float(v) for v in args.speeds.split(',')]; banks=[float(v) for v in args.banks.split(',')]
    sign_names=[value.strip() for value in args.turn_signs.split(',') if value.strip()]
    if not sign_names or any(value not in ('left','right') for value in sign_names) or len(set(sign_names))!=len(sign_names):
        p.error('turn-signs must be a unique comma-separated subset of left,right')
    # Compass course increases clockwise: negative rate is a left turn.
    signs=[-1 if value=='left' else 1 for value in sign_names]
    if any(not 0<v<=30 for v in speeds) or any(not 0<b<=40 for b in banks):
        p.error('invalid calibration range')
    if not math.isfinite(args.lookahead) or args.lookahead<=0:
        p.error('lookahead must be finite and positive')
    if not math.isfinite(args.speedup) or args.speedup<=0 or args.repeat_id<0:
        p.error('speedup must be positive and repeat-id nonnegative')
    if args.handover_command_speed and not args.path_guidance:
        p.error('command-speed handover requires Plane path guidance')
    if args.path_guidance and (args.vehicle!='plane' or args.controller!='course'
            or any(not 12<=v<=24 for v in speeds) or any(9.80665*math.tan(math.radians(b))>2.5 for b in banks)):
        p.error('path guidance requires Plane course mode and frozen speed/lateral bounds')
    v=SitlVehicle(args.vehicle,args.model or ('plane' if args.vehicle=='plane' else 'x:freestyle.json'),args.output,args.port,
                  speedup=args.speedup,
                  defaults=args.defaults or (None if args.vehicle=='plane' else AP/'Tools/autotest/models/freestyle.parm'),
                  plane_heading_period=args.plane_heading_period)
    protocol=dict(protocol_version=6,platform=args.vehicle,altitude_m=60,wind_m_s=0,
                  speeds=speeds,banks=banks,
                  turn_signs=sign_names,dataset_group=args.dataset_group,repeat_id=args.repeat_id,
                  protocol_id=args.protocol_id,speedup=args.speedup,
                  settle_s=30,window_s=15,airspeed_error_m_s=1,altitude_error_m=2,
                  bank_error_deg=3,bank_abs_deg=27,course_error_deg=8,position_error_m=4,abort_roll_deg=65,
                  bank_check='actual <=27 deg in course mode; equivalent curvature bank is diagnostic, not a sent target',
                  controller=args.controller,accel_feedforward=args.accel_feedforward,
                  yaw_rate_feedforward=args.yaw_rate_feedforward,
                  plane_heading_period_sim_s=args.plane_heading_period,
                  path_guidance=args.path_guidance,lookahead_m=args.lookahead if args.path_guidance else None,
                  progress_gain=.05 if args.path_guidance else None,
                  handover_command_speed=args.handover_command_speed,
                  path_entry_gate=dict(minimum_s=25.,settled_s=5.,timeout_s=90.,
                      speed_error_m_s=.3,altitude_error_m=2.,course_error_deg=3.) if args.path_guidance else None,
                  continue_after_tracking_failure=args.continue_after_tracking_failure,
                  capability_evidence='only tested combinations; no extrapolation to untested low-speed bank limits')
    (v.directory/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    report=dict(status='FAILED',steps=[],protocol=protocol)
    try:
        v.start(); v.takeoff()
        print('AIRBORNE',args.vehicle,flush=True)
        for speed in speeds:
            failed=False
            for bank in banks:
                for sign in signs:
                    if args.path_guidance:
                        set_phase(v,'path_entry_%g_%g_%d'%(speed,bank,sign))
                        entry=v.row();entry_start=entry['sim_s'];settled=None
                        entry_course=math.atan2(entry['ve'],entry['vn'])
                        while True:
                            v.plane_reference(speed,math.degrees(entry_course),2.5,lateral_mode='course')
                            v.pump(.05/v.speedup);v.check_plane_commands()
                            row=v.row();elapsed=row['sim_s']-entry_start
                            error=math.atan2(math.sin(math.atan2(row['ve'],row['vn'])-entry_course),
                                             math.cos(math.atan2(row['ve'],row['vn'])-entry_course))
                            if row['mode']!='GUIDED' or not row['armed'] or row['alt']<30:
                                raise RuntimeError('path entry flight protection')
                            valid=abs(row['airspeed']-speed)<.3 and abs(row['alt']-60)<2 and abs(error)<math.radians(3)
                            if valid and settled is None:settled=row['sim_s']
                            if not valid:settled=None
                            if elapsed>=25 and settled is not None and row['sim_s']-settled>=5:break
                            if elapsed>90:raise RuntimeError('path entry did not settle; calibration point not started')
                    omega=sign*9.80665*math.tan(math.radians(bank))/speed
                    begin=v.row()['sim_s']; previous=begin
                    row=v.row(); course=math.atan2(row['ve'],row['vn'])
                    east,north=row['east'],row['north']; first=len(v.samples)
                    if args.path_guidance:
                        tracker=PlaneCourseTracker(MemberSpec(0,'fixedwing',0.,12.,24.,2.5,
                            bank_max=math.radians(27),a_t_min=-.2,a_t_max=.2),lookahead=args.lookahead,progress_gain=.05,
                            guidance_mode=args.guidance_mode,
                            cross_track_gain=args.cross_track_gain,
                            course_to_bank_gain=args.course_to_bank_gain,
                            initial_command_airspeed=v.target['ref_speed'] if args.handover_command_speed else None)
                    turn_stage=None
                    while v.row()['sim_s']-begin<protocol['settle_s']+protocol['window_s']:
                        now=v.row()['sim_s']; dt=max(0,now-previous); previous=now
                        elapsed=now-begin
                        next_stage='settle' if elapsed<protocol['settle_s'] else 'steady'
                        if next_stage!=turn_stage:
                            set_phase(v,turn_phase_name(next_stage,speed,bank,sign))
                            turn_stage=next_stage
                        old=course; course+=omega*dt
                        east+=speed/omega*(math.cos(old)-math.cos(course))
                        north+=speed/omega*(math.sin(course)-math.sin(old))
                        if args.vehicle=='plane':
                            if args.path_guidance:
                                if dt<=0:
                                    v.pump(.05/v.speedup)
                                    continue
                                row=v.row()
                                command=tracker.update((east,north,math.pi/2-course,speed,-omega/speed,60.),
                                    (row['east'],row['north']),row['airspeed'],dt)
                                v.plane_reference(command.airspeed,command.bearing_deg,
                                    command.lateral_accel_limit,lateral_mode='course',altitude=command.altitude)
                                v.target.update(ref_east=east,ref_north=north)
                            else:
                                v.plane_reference(speed,math.degrees(course)%360,9.80665*math.tan(math.radians(bank+5)),
                                    bank=sign*math.radians(bank) if args.controller=='bank' else None)
                        else:
                            v.copter_reference(east,north,speed*math.sin(course),speed*math.cos(course),course,
                                accel=(speed*omega*math.cos(course),-speed*omega*math.sin(course)) if args.accel_feedforward else None,
                                yaw_rate=omega if args.yaw_rate_feedforward else None)
                        v.target['ref_path_course']=math.degrees(course)
                        v.pump(.05/v.speedup)
                        if args.vehicle=='plane':
                            v.check_plane_commands()
                        row=v.row()
                        if row['mode']!='GUIDED' or not row['armed'] or abs(math.degrees(row['roll']))>65 or row['alt']<30:
                            raise RuntimeError('flight protection condition')
                        if max(row['attitude_age'],row['velocity_age'],row['airspeed_age'])>.8:
                            raise RuntimeError('stale telemetry')
                    rows=[r for r in v.samples[first:] if r['sim_s']>=begin+protocol['settle_s']]
                    if len(rows)<20:
                        raise RuntimeError('insufficient telemetry')
                    actualspeed=[r['airspeed'] if args.vehicle=='plane' else math.hypot(r['vn'],r['ve']) for r in rows]
                    actualbank=[math.degrees(r['roll']) for r in rows]
                    course_error=[abs(math.degrees(math.atan2(math.sin(math.atan2(r['ve'],r['vn'])-math.radians(r['ref_path_course'])),math.cos(math.atan2(r['ve'],r['vn'])-math.radians(r['ref_path_course']))))) for r in rows]
                    errors=dict(airspeed_error_m_s=max(abs(x-speed) for x in actualspeed),
                                altitude_error_m=max(abs(r['alt']-60) for r in rows))
                    diagnostics={}
                    if args.controller=='course' or args.vehicle=='copter':
                        errors['course_error_deg']=max(course_error)
                    if args.vehicle=='plane':
                        bank_errors,diagnostics=bank_tracking_metrics(actualbank,sign*bank,course_control=args.controller=='course')
                        errors.update(bank_errors)
                    if args.vehicle=='copter' or args.path_guidance:
                        errors['position_error_m']=max(math.hypot(r['east']-r['ref_east'],r['north']-r['ref_north']) for r in rows)
                    failures=[k for k,x in errors.items() if x>protocol[k]]
                    step=dict(speed=speed,bank=bank,sign=sign,target_lateral_accel=abs(speed*omega),
                              target_heading_rate=omega,mean_bank=statistics.mean(actualbank),
                              max_bank=max(map(abs,actualbank)),airspeed_min=min(actualspeed),
                              errors=errors,diagnostics=diagnostics,failures=failures,status='FAIL' if failures else 'PASS',
                              window=[rows[0]['sim_s'],rows[-1]['sim_s']])
                    heading=[math.atan2(r['ve'],r['vn']) for r in rows]
                    rates=[]
                    for i in range(1,len(rows)):
                        dt=rows[i]['sim_s']-rows[i-1]['sim_s']
                        if dt>.001:
                            d=math.atan2(math.sin(heading[i]-heading[i-1]),math.cos(heading[i]-heading[i-1]))
                            rates.append(d/dt)
                    step['measured_mean_heading_rate']=statistics.mean(rates)
                    step['measured_mean_lateral_accel']=statistics.mean(rates)*statistics.mean(actualspeed)
                    report['steps'].append(step)
                    (v.directory/'result.json').write_text(json.dumps(report,indent=2)+'\n')
                    print(json.dumps(step),flush=True)
                    if failures and not args.continue_after_tracking_failure:
                        failed=True
                        break
                if failed:
                    break
        report['status']='COMPLETED'
        report['all_tested_points_passed']=len(report['steps'])==len(signs)*len(speeds)*len(banks) and all(
            step['status']=='PASS' for step in report['steps'])
        report['envelope_certified']=False
    except Exception as exc:
        report['error']=repr(exc); print('FAILED',repr(exc),flush=True)
    finally:
        try:
            if v.link:
                v.finish(); report['exit_completed']=True
        except Exception as exc:
            report['exit_completed']=False; report['exit_error']=repr(exc)
        v.close()
        (v.directory/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if report.get('all_tested_points_passed') and report.get('exit_completed') else 1


if __name__=='__main__':
    raise SystemExit(main())
