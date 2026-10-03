#!/usr/bin/env python3
"""Independent single-vehicle calibration with frozen preflight criteria."""
import argparse
import json
import math
from pathlib import Path
import statistics

from uav_formation.sitl_vehicle import SitlVehicle, AP


def set_phase(vehicle,name):
    setter=getattr(vehicle,'set_phase',None)
    if setter is None:
        vehicle.phase=name
    else:
        setter(name)


def set_reference_acceleration(vehicle,value):
    target=getattr(vehicle,'target',None)
    if target is not None:
        target['ref_accel']=value


def quintic_speed(start, end, elapsed, rate):
    """C2 speed ramp whose maximum absolute acceleration equals rate."""
    if not all(math.isfinite(v) for v in (start,end,elapsed,rate)) or rate<=0 or elapsed<0:
        raise ValueError('finite speeds, positive rate and nonnegative time required')
    duration=1.875*abs(end-start)/rate
    if duration==0 or elapsed>=duration:
        return end,0.
    u=elapsed/duration
    return (start+(end-start)*(10*u**3-15*u**4+6*u**5),
            (end-start)*30*u*u*(1-u)**2/duration)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vehicle',choices=['copter','plane'],required=True)
    parser.add_argument('--model')
    parser.add_argument('--defaults')
    parser.add_argument('--plane-heading-period',type=float,default=0.,help='optional changed-target heading interval in simulation seconds; zero preserves streaming')
    parser.add_argument('--speeds',default='17')
    parser.add_argument('--output',required=True)
    parser.add_argument('--port',type=int,default=20660)
    parser.add_argument('--altitude',type=float,default=60.)
    parser.add_argument('--speedup',type=float,default=2.)
    parser.add_argument('--direction',choices=['forward','backward','left','right'],default='forward')
    parser.add_argument('--ramp-rate',type=float,default=0.,help='optional reference speed ramp in m/s^2')
    parser.add_argument('--approach-rate',type=float,default=1.,help='Copter-only pre-calibration approach peak acceleration in m/s^2')
    parser.add_argument('--approach-settle',type=float,default=10.,help='pre-calibration settling time in simulation seconds')
    parser.add_argument('--ppva',action='store_true',help='Copter position/velocity/acceleration with yaw-rate feedforward')
    parser.add_argument('--continuous-position',action='store_true',help='keep Copter position setpoint continuous between speed steps')
    parser.add_argument('--ramp-shape',choices=['linear','quintic'],default='linear')
    parser.add_argument('--dataset-group',choices=['PILOT','A','B'],default='PILOT')
    parser.add_argument('--repeat-id',type=int,default=0)
    parser.add_argument('--protocol-id',default='v2-single-vehicle-ramp')
    args=parser.parse_args()
    expected_model='plane' if args.vehicle=='plane' else 'x:freestyle.json'
    if args.model not in (None,expected_model) and not args.defaults:
        parser.error('a candidate model requires explicit matching --defaults')
    speeds=[float(v) for v in args.speeds.split(',')]
    if (not speeds or any(v<0 or not math.isfinite(v) for v in speeds)
            or (args.vehicle=='plane' and any(v==0 for v in speeds))
            or not math.isfinite(args.ramp_rate) or args.ramp_rate<0
            or not math.isfinite(args.approach_rate) or args.approach_rate<=0
            or not math.isfinite(args.approach_settle) or args.approach_settle<0
            or ((args.ppva or args.continuous_position) and args.vehicle!='copter')
            or (args.ramp_shape=='quintic' and args.ramp_rate==0) or args.repeat_id<0):
        parser.error('finite nonnegative Copter speeds, positive Plane speeds and valid ramp required')
    vehicle=SitlVehicle(args.vehicle,args.model or ('plane' if args.vehicle=='plane' else 'x:freestyle.json'),
        args.output,args.port,args.altitude,args.speedup,args.defaults or (str(AP/'Tools/autotest/models/freestyle.parm') if args.vehicle=='copter' else None),
        plane_heading_period=args.plane_heading_period)
    protocol=dict(kind='independent_SITL_calibration',platform=args.vehicle,
        altitude_m=args.altitude,wind_m_s=0,
        protocol_version=6,direction=args.direction,speeds=speeds,settle_sim_s=25.,steady_sim_s=15.,
        dataset_group=args.dataset_group,repeat_id=args.repeat_id,protocol_id=args.protocol_id,
        plane_heading_period_sim_s=args.plane_heading_period,
        speed_frame='horizontal heading frame for planar beta model; full 3D body FLU retained separately',
        speed_error_m_s=1.,altitude_error_m=2.,roll_deg=10. if args.vehicle=='plane' else 65.,course_error_deg=5.,
        position_error_m=4.,max_telemetry_age_s=.5,uncertainty_m_s=.5,
        controller='ArduPlane GUIDED course/airspeed/altitude' if args.vehicle=='plane' else 'ArduCopter GUIDED position+velocity feedforward',
        exit='LOITER then terminate owned simulation; not landing validation' if args.vehicle=='plane' else 'LAND and wait disarm',
        reference_ramp_m_s2=args.ramp_rate,
        ramp_shape=args.ramp_shape,
        ppva=args.ppva, yaw_rate_feedforward=args.ppva,continuous_position=args.continuous_position,
        position_clock='carry last command timestamp across steps' if args.continuous_position else 'reset at each step',
        ramp_ready_sim_s=40. if args.ramp_rate and args.vehicle=='plane' else 0.,
        approach_rate_m_s2=args.approach_rate if args.vehicle=='copter' else None,
        approach_settle_sim_s=args.approach_settle,
        qualification_phases=['accel_*','decel_*','settle_*','steady_*'],
        excluded_phases=['takeoff','approach_accel','approach_settle','ramp_ready','exit'],
        saturation='record servo/throttle; no physical-envelope extrapolation')
    (vehicle.directory/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    report=dict(status='FAILED',steps=[],protocol=protocol)
    try:
        vehicle.start(); vehicle.takeoff()
        print('AIRBORNE',args.vehicle,vehicle.row()['alt'],flush=True)
        east,north=vehicle.row()['east'],vehicle.row()['north']
        previous=vehicle.row()['sim_s']
        if args.vehicle=='copter' and speeds[0]>0:
            approach_start=previous
            while True:
                now=vehicle.row()['sim_s']; dt=max(0.,now-previous); previous=now
                issued,acceleration=quintic_speed(0.,speeds[0],now-approach_start,args.approach_rate)
                approach_complete=issued>=speeds[0]-1e-9
                set_phase(vehicle,'approach_accel')
                set_reference_acceleration(vehicle,acceleration)
                ve,vn={'forward':(issued,0.),'backward':(-issued,0.),'left':(0.,issued),'right':(0.,-issued)}[args.direction]
                de,dn={'forward':(1.,0.),'backward':(-1.,0.),'left':(0.,1.),'right':(0.,-1.)}[args.direction]
                east+=ve*dt; north+=vn*dt
                vehicle.copter_reference(east,north,ve,vn,
                    accel=(de*acceleration,dn*acceleration) if args.ppva else None,
                    yaw_rate=0. if args.ppva else None)
                vehicle.pump(.05/args.speedup)
                if approach_complete:
                    break
            settle_start=vehicle.row()['sim_s']
            while vehicle.row()['sim_s']-settle_start<args.approach_settle:
                now=vehicle.row()['sim_s']; dt=max(0.,now-previous); previous=now
                set_phase(vehicle,'approach_settle'); set_reference_acceleration(vehicle,0.)
                ve,vn={'forward':(speeds[0],0.),'backward':(-speeds[0],0.),'left':(0.,speeds[0]),'right':(0.,-speeds[0])}[args.direction]
                east+=ve*dt; north+=vn*dt
                vehicle.copter_reference(east,north,ve,vn,
                    accel=(0.,0.) if args.ppva else None,
                    yaw_rate=0. if args.ppva else None)
                vehicle.pump(.05/args.speedup)
        if protocol['ramp_ready_sim_s']:
            set_phase(vehicle,'ramp_ready'); began=vehicle.row()['sim_s']
            while vehicle.row()['sim_s']-began<protocol['ramp_ready_sim_s']:
                vehicle.plane_reference(speeds[0],90.)
                vehicle.pump(.05/args.speedup)
        # Preserve the last sent target after warm-up; measured sensor noise
        # must not introduce a new target jump at the first calibration step.
        issued=(speeds[0] if protocol['ramp_ready_sim_s'] else vehicle.row()['airspeed']) if args.vehicle=='plane' else speeds[0]
        for step_index,speed in enumerate(speeds):
            begin=vehicle.row()['sim_s']
            if (step_index==0 and args.vehicle=='plane') or not args.continuous_position:
                previous=begin
            # A continuous position target must also retain its integration clock.
            # Resetting it here drops the last pump interval at every step boundary.
            ramp_duration=abs(speed-issued)/args.ramp_rate if args.ramp_rate else 0.
            ramp_start=issued
            if args.ramp_shape=='quintic':
                ramp_duration*=1.875
            steady_start=ramp_duration+protocol['settle_sim_s']
            if (step_index==0 and args.vehicle=='plane') or not args.continuous_position:
                east,north=vehicle.row()['east'],vehicle.row()['north']
            step_label='step_%02d_%g'%(step_index,speed)
            first=len(vehicle.samples)
            while vehicle.row()['sim_s']-begin < steady_start+protocol['steady_sim_s']:
                now=vehicle.row()['sim_s']; dt=max(0,now-previous); previous=now
                if args.ramp_shape=='quintic':
                    issued,acceleration=quintic_speed(ramp_start,speed,now-begin,args.ramp_rate)
                elif args.ramp_rate:
                    before=issued
                    issued+=max(-args.ramp_rate*dt,min(args.ramp_rate*dt,speed-issued))
                    acceleration=(issued-before)/dt if dt>0 else 0.
                else:
                    issued=speed
                    acceleration=0.
                elapsed=now-begin
                if elapsed<ramp_duration-1e-9:
                    phase=('accel_' if speed>=ramp_start else 'decel_')+step_label
                elif elapsed<steady_start:
                    phase='settle_'+step_label
                else:
                    phase='steady_'+step_label
                set_phase(vehicle,phase)
                set_reference_acceleration(vehicle,acceleration)
                if args.vehicle=='plane':
                    vehicle.plane_reference(issued,90.)
                else:
                    ve,vn={'forward':(issued,0.),'backward':(-issued,0.),'left':(0.,issued),'right':(0.,-issued)}[args.direction]
                    east+=ve*dt; north+=vn*dt
                    de,dn={'forward':(1.,0.),'backward':(-1.,0.),'left':(0.,1.),'right':(0.,-1.)}[args.direction]
                    vehicle.copter_reference(east,north,ve,vn,
                        accel=(de*acceleration,dn*acceleration) if args.ppva else None,
                        yaw_rate=0. if args.ppva else None)
                vehicle.pump(.05/args.speedup)
                if args.vehicle=='plane':
                    vehicle.check_plane_commands()
                row=vehicle.row()
                if row['mode']!='GUIDED' or not row['armed']:
                    raise RuntimeError('left armed GUIDED')
                if max(row['attitude_age'],row['velocity_age'],row['airspeed_age'])>.8:
                    raise RuntimeError('stale telemetry')
                if row['alt']<args.altitude*.5 or abs(row['roll'])>math.radians(65):
                    raise RuntimeError('flight excursion')
            rows=[r for r in vehicle.samples[first:] if r['sim_s']>=begin+steady_start]
            if len(rows)<20:
                raise RuntimeError('insufficient steady samples')
            if args.vehicle=='plane':
                measured=[r['airspeed'] for r in rows]
            else:
                name,sign={'forward':('heading_forward',1),'backward':('heading_forward',-1),'left':('heading_left',1),'right':('heading_left',-1)}[args.direction]
                measured=[sign*r[name] for r in rows]
            errors=dict(speed_error_m_s=max(abs(v-speed) for v in measured),
                altitude_error_m=max(abs(r['alt']-args.altitude) for r in rows),
                roll_deg=max(abs(math.degrees(r['roll'])) for r in rows))
            if args.vehicle=='plane':
                errors['course_error_deg']=max(abs(math.degrees(math.atan2(r['vn'],r['ve']))) for r in rows)
            else:
                errors['position_error_m']=max(math.hypot(r['east']-r['ref_east'],r['north']-r['ref_north']) for r in rows)
            failures=[name for name,value in errors.items() if value>protocol[name]]
            step=dict(speed=speed,status='FAIL' if failures else 'PASS',failures=failures,errors=errors,
                      mean=statistics.mean(measured),minimum=min(measured),maximum=max(measured),
                      window=[rows[0]['sim_s'],rows[-1]['sim_s']],count=len(rows),
                      throttle_max=max(r['throttle'] for r in rows))
            report['steps'].append(step)
            (vehicle.directory/'result.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(step),flush=True)
            if failures:
                break
        report['status']='COMPLETED'
        report['all_tested_points_passed']=len(report['steps'])==len(speeds) and all(
            step['status']=='PASS' for step in report['steps'])
        report['envelope_certified']=False
    except Exception as exc:
        report['error']=repr(exc)
        print('FAILED',repr(exc),flush=True)
    finally:
        try:
            if vehicle.link:
                vehicle.finish()
                report['exit_completed']=True
        except Exception as exc:
            report['exit_completed']=False; report['exit_error']=repr(exc)
        vehicle.close()
        (vehicle.directory/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if report.get('all_tested_points_passed') and report.get('exit_completed') else 1


if __name__=='__main__':
    raise SystemExit(main())
