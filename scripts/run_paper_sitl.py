#!/usr/bin/env python3
"""Shared-reference fleet SITL development runner with raw flight evidence.

Owns all processes. Copters use measured-state DroneMPC with a GUIDED
position/velocity/acceleration interface. Planes use lateral guidance and
ArduPlane roll/TECS loops. This is a separate, recorded execution backend;
the existing ROS S1 launcher and its default bridge remain available.
"""
import argparse
from dataclasses import asdict
from concurrent.futures import ThreadPoolExecutor
import csv
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import random
import threading
import time

import numpy as np

from uav_formation.sitl_vehicle import SitlVehicle, AP
from uav_formation.experiment_config import (load_config, coordinator_from_config,
                                             planning_members, fixedwing_controller_member)
from uav_formation.experiment_artifacts import freeze_run, export_plan, write_json
from uav_formation.capability_model import team_interval
from uav_formation.egompc_formation import DroneMPC, N, DT, RATE
from uav_formation.paper_tracking import (plane_command, preview_member, PlaneCourseTracker,
                                        synchronous_copter_target, CopterExecutionGovernor, CopterVelocityFilter,
                                        CopterReferenceCorrection, member_reference)
from uav_formation.measured_feedback import FeedbackError, command_endpoint
from uav_formation.progress_profile import ProgressProfile, PlanningError, validate_profile
from uav_formation.command_stream_process import ProcessCommandStreamer


EVIDENCE_BUFFER_BYTES = 4 * 1024 * 1024


def pin_current_thread_from_env(variable):
    """Pin only the calling thread when an execution protocol requests it."""
    value = os.environ.get(variable)
    if value is None:
        return None
    cpu = int(value)
    os.sched_setaffinity(0, {cpu})
    return cpu


def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))


def update_join_origin(origin, rows, members, dt, nominal_speed):
    """Advance a stable rendezvous frame for mixed fleets during mission setup."""
    platforms={member.platform for member in members}
    if platforms=={'multirotor','fixedwing'}:
        origin[0]+=nominal_speed*dt
    else:
        origin[0]=sum(row['east'] for row in rows)/len(rows)


def rotor_precheck_planner_compatible(source, target):
    """Match execution-relevant speed conditions, not method geometry.

    A qualified rotor execution chain may support F and the more conservative
    C radius policy.  Radius candidates are reference-method inputs; changing
    the common speed or discretization would be a different execution test.
    """
    keys = ('nominal_speed', 'start_speed', 'end_speed', 'ds', 'trans_len')
    return all(source.get(key) == target.get(key) for key in keys)


def rotor_development_gate(paths, cfg, members, yaw_rate, synchronous=False, control_period=RATE, analytic_gradient=False, governor=False, held_control=False, tangent_yaw=False, velocity_filter=None, yaw_angle_only=False, consistent_correction=False):
    """Finite sampled straight tracking evidence, not a continuous capability proof."""
    evidence=[]; system_evidence=[]
    for path in paths:
        path=Path(path)
        source=json.loads((path/'config.json').read_text())
        backend_mismatch=(bool(source.get('copter_reference_only'))
                or bool(source.get('copter_yaw_rate_feedforward'))!=yaw_rate
                or bool(source.get('copter_synchronous_setpoint'))!=synchronous
                or source.get('feedback',{}).get('control_period_s',RATE)!=control_period
                or bool(source.get('copter_analytic_gradient'))!=analytic_gradient
                or bool(source.get('copter_command_governor'))!=governor
                or bool(source.get('copter_held_control'))!=held_control
                or bool(source.get('copter_tangent_yaw'))!=tangent_yaw
                or bool(source.get('copter_yaw_angle_only'))!=yaw_angle_only
                or bool(source.get('copter_consistent_correction'))!=consistent_correction
                or source.get('copter_velocity_filter')!=velocity_filter
                or source.get('copter_platform_candidate')!=cfg.get('copter_platform_candidate')
                or bool(source.get('copter_predicted_sequence'))
                or bool(source.get('copter_tangent_velocity'))
                or source.get('copter_yaw_cap') is not None)
        formal_path=path/'v3_formal_acceptance.json'
        if formal_path.is_file():
            assessment=json.loads(formal_path.read_text())
            required=[m.r_y for m in members if m.platform=='multirotor']
            available=[float(m['r_y']) for m in source['members']
                       if m['platform']=='multirotor']
            if (assessment.get('classification') not in ('SYSTEM_PRECHECK_QUALIFIED','SYSTEM_QUALIFIED')
                    or not assessment.get('system_precheck_qualified')
                    or assessment.get('invalid_reasons') or assessment.get('acceptance_failures')
                    or not assessment.get('lifecycle',{}).get('completed')
                    or not assessment.get('lifecycle',{}).get('exit_completed')):
                raise ValueError('rotor v3 system precheck did not qualify: '+str(path))
            if (any(m['platform']!='multirotor' for m in source['members']) or backend_mismatch
                    or source.get('frozen_contract')!=cfg.get('frozen_contract')
                    or source.get('waypoints')!=cfg.get('waypoints')
                    or not rotor_precheck_planner_compatible(
                        source.get('planner', {}), cfg.get('planner', {}))
                    or source.get('multicopter_execution')!=cfg.get('multicopter_execution')
                    or any(not any(abs(candidate-slot)<=1e-12 for candidate in available)
                           for slot in required)):
                raise ValueError('rotor v3 system precheck backend/task/contract mismatch: '+str(path))
            system_evidence.append(dict(path=str(path.resolve()),
                classification=assessment['classification'],members=len(source['members']),
                duration_s=assessment['lifecycle'].get('duration')))
            continue
        assessment=json.loads((path/'acceptance.json').read_text())
        if not assessment.get('tracking_qualified') or not assessment.get('full_route_completed'):
            raise ValueError('rotor precheck did not qualify: '+str(path))
        if (any(m['platform']!='multirotor' for m in source['members']) or len(source['waypoints'])!=2
                or backend_mismatch):
            raise ValueError('rotor precheck backend/task mismatch: '+str(path))
        evidence.append(dict(path=str(path.resolve()),speed=source['planner']['nominal_speed'],
                             measured_constraints_passed=assessment['measured_constraints_passed']))
    if system_evidence:
        if evidence:
            raise ValueError('legacy and v3 rotor precheck evidence cannot be mixed')
        return dict(status='V3_SYSTEM_PRECHECK_GATE',evidence=system_evidence,
                    scope='system execution prerequisite only; frozen independent capability evidence remains authoritative')
    speeds=sorted({e['speed'] for e in evidence})
    lower=max(m.v_min for m in members if m.platform=='fixedwing')
    nominal=cfg['planner']['nominal_speed']
    compatible=[speed for speed in speeds if speed>=lower]
    if len(compatible)<2 or compatible[-1]-compatible[0]<=cfg.get('required_straight_margin',1.) or not compatible[0]<=nominal<=compatible[-1]:
        raise ValueError('two compatible straight precheck speeds spanning the nominal task are required')
    return dict(status='DEVELOPMENT_TRACKING_GATE_ONLY',sampled_speeds=compatible,evidence=evidence,
                scope='not a certified continuous envelope; all actual constraints must still be measured')


def send_copter_setpoint(vehicle, target, velocity, acceleration, yaw_rate_ned, *, yaw_angle_only=False):
    """Suppress only the wire yaw-rate field; retain the complete spatial target."""
    effective_rate = None if yaw_angle_only else yaw_rate_ned
    vehicle.copter_reference(target[0], target[1], *velocity,
        math.pi/2-target[2], accel=acceleration, yaw_rate=effective_rate)
    return None if effective_rate is None else -effective_rate


def reference_copter_setpoint(reference):
    """Development control: transmit one internally consistent route sample."""
    x, y, psi, speed, curvature, tangential = reference
    c, s = math.cos(psi), math.sin(psi)
    velocity = (speed*c, speed*s)
    acceleration = (tangential*c-speed*speed*curvature*s,
                    tangential*s+speed*speed*curvature*c)
    return (x, y, psi), velocity, acceleration, -speed*curvature


def retransmit_fleet_reference(vehicle, transmission):
    """Repeat a valid command without advancing reference or measured state."""
    if transmission is None:
        return False
    kind, values = transmission
    if kind == 'plane':
        command = values
        vehicle.plane_reference(command.airspeed, command.bearing_deg,
                                command.lateral_accel_limit,
                                lateral_mode='course', altitude=command.altitude)
    elif kind == 'copter':
        target, velocity, acceleration, yaw_rate_ned, yaw_angle_only = values
        send_copter_setpoint(vehicle, target, velocity, acceleration, yaw_rate_ned,
                             yaw_angle_only=yaw_angle_only)
    elif kind == 'plane':
        command = values
        vehicle.plane_reference(command.airspeed, command.bearing_deg,
                                command.lateral_accel_limit,
                                lateral_mode='course', altitude=command.altitude)
    else:
        raise ValueError('unknown cached transmission')
    return True


class CopterCommandStreamer:
    """Keep the last valid copter command alive while optimization is busy."""
    def __init__(self, vehicles, transmissions, interval=.045):
        self.vehicles=vehicles
        self.transmissions=transmissions
        self.interval=interval
        self.next_send=time.monotonic()+interval
        self.stop_event=threading.Event()
        self.error=None
        self.thread=None
        self.threads=[]

    def step(self, now=None):
        now=time.monotonic() if now is None else now
        if now<self.next_send:
            return 0
        sent=0
        for member,vehicle in enumerate(self.vehicles):
            transmission=self.transmissions.get(member)
            if transmission is not None:
                sent+=bool(retransmit_fleet_reference(vehicle,transmission))
        self.next_send=now+self.interval
        return sent

    def _run(self):
        """Compatibility path used by the deterministic unit-test step."""
        try:
            pin_current_thread_from_env('UAV_STREAM_CPU')
            while not self.stop_event.wait(max(0.,self.next_send-time.monotonic())):
                self.step()
        except Exception as exc:
            self.error=exc

    def _run_member(self, member, vehicle):
        try:
            pin_current_thread_from_env('UAV_STREAM_CPU')
            next_send=time.monotonic()+self.interval
            while not self.stop_event.wait(max(0.,next_send-time.monotonic())):
                transmission=self.transmissions.get(member)
                if transmission is not None:
                    retransmit_fleet_reference(vehicle,transmission)
                next_send += self.interval
                now=time.monotonic()
                if next_send < now-self.interval:
                    next_send=now+self.interval
        except Exception as exc:
            self.error=exc

    def start(self):
        self.threads=[threading.Thread(target=self._run_member,args=(member,vehicle),
            name='copter-command-stream-%d'%member,daemon=True)
            for member,vehicle in enumerate(self.vehicles)]
        for thread in self.threads:
            thread.start()

    def check(self):
        if self.error is not None:
            raise RuntimeError('copter command streamer failed: '+repr(self.error))

    def stop(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=2.)
            if thread.is_alive():
                raise RuntimeError('copter command streamer did not stop')
        self.check()


def main():
    # The command journal intentionally retains immutable packet records until
    # shutdown.  It contains no reference cycles; disabling cyclic GC avoids
    # a stop-the-world collection pausing the single real-time streamer.
    gc.disable()
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--base-port',type=int,default=22000)
    parser.add_argument('--speedup',type=float,default=1.)
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--development',action='store_true')
    parser.add_argument('--copter-model',help='development only: explicit candidate SITL model')
    parser.add_argument('--copter-defaults',help='development only: matching comma-separated parameter files')
    parser.add_argument('--method',choices=['F','A3','C'],default='F')
    parser.add_argument('--track-seconds',type=float,help='development only: stop after this tracking duration')
    parser.add_argument('--plane-lateral',choices=['course','external_roll'],default='course')
    parser.add_argument('--copter-yaw-rate',action='store_true',help='development: current-time yaw plus yaw-rate feedforward')
    parser.add_argument('--copter-synchronous',action='store_true',help='development: same-time position, velocity and yaw commands')
    parser.add_argument('--copter-control-period',type=float,default=RATE,help='development: NMPC solve/hold period; default unchanged')
    parser.add_argument('--copter-analytic-gradient',action='store_true',help='development: exact gradient of unchanged NMPC objective')
    parser.add_argument('--copter-command-governor',action='store_true',help='development: continuous command trajectory with input reserves')
    parser.add_argument('--copter-held-control',action='store_true',help='development: optimize a constant first hold block instead of averaging differing controls')
    parser.add_argument('--copter-tangent-yaw',action='store_true',help='development: NMPC spatial commands with path-tangent yaw tracked by Copter attitude control')
    parser.add_argument('--copter-yaw-angle-only',action='store_true',help='development: keep tangent yaw angle but mask its angular-rate feedforward; spatial commands unchanged')
    parser.add_argument('--copter-reference-only',action='store_true',help='development: bypass NMPC and stream the common member reference directly for chain diagnosis')
    parser.add_argument('--copter-consistent-correction',action='store_true',help='development: use bounded NMPC-driven route correction with consistent PPVA')
    parser.add_argument('--copter-velocity-filter',type=float,help='development: velocity output smoothing time constant; requires tangent yaw')
    parser.add_argument('--plane-progress-gain',type=float,help='development: along-track airspeed correction gain')
    parser.add_argument('--plane-command-accel-scale',type=float,help='development: scale Plane command slew only')
    parser.add_argument('--rotor-precheck',action='append',default=[],help='qualified rotor execution evidence; v3 accepts a matching system precheck')
    args=parser.parse_args()
    cfg,members=load_config(args.config)
    formal_copter = cfg.get('multicopter_execution')
    if formal_copter:
        if any((args.copter_reference_only, args.copter_yaw_angle_only,
                args.copter_command_governor, args.copter_held_control,
                args.copter_velocity_filter is not None)):
            parser.error('CLI copter execution overrides are forbidden by the frozen task')
        args.copter_tangent_yaw = formal_copter['tangent_yaw']
        args.copter_yaw_rate = formal_copter['yaw_rate_feedforward']
        args.copter_synchronous = formal_copter['synchronous_setpoint']
        args.copter_analytic_gradient = formal_copter['analytic_gradient']
        args.copter_consistent_correction = True
    formal_copter_enabled = formal_copter is not None
    if args.copter_model is not None or args.copter_defaults is not None:
        if not (args.development and args.copter_model and args.copter_defaults):
            parser.error('candidate aircraft require --development and both model/defaults')
        paths=[Path(p).expanduser().resolve() for p in args.copter_defaults.split(',')]
        if not all(p.is_file() for p in paths):
            parser.error('candidate parameter file missing')
        cfg['copter_platform_candidate']=dict(model=args.copter_model,
            defaults=','.join(str(p) for p in paths),
            defaults_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
            scope='development screening, capabilities not certified')
        cfg['previous_capability_evidence']=cfg.get('capability_evidence')
        cfg['capability_evidence']='candidate_platform_uncalibrated'
    elif cfg.get('copter_platform_candidate') is not None:
        parser.error('candidate configuration requires explicit model/defaults flags')
    control_period=args.copter_control_period
    if args.copter_yaw_angle_only and not (args.development and args.copter_tangent_yaw):
        parser.error('yaw-angle-only requires development and tangent yaw')
    if args.copter_reference_only and (not args.development or not args.copter_tangent_yaw
            or args.copter_yaw_angle_only or args.copter_command_governor
            or args.copter_velocity_filter is not None or args.copter_held_control):
        parser.error('reference-only requires development/tangent yaw and excludes other copter execution experiments')
    if args.copter_reference_only and (len(members) != 1 or members[0].platform != 'multirotor'):
        parser.error('reference-only diagnostic is restricted to one multirotor')
    if args.copter_consistent_correction and (not (args.development or formal_copter_enabled) or not args.copter_tangent_yaw
            or args.copter_reference_only or args.copter_yaw_angle_only
            or args.copter_command_governor or args.copter_velocity_filter is not None):
        parser.error('consistent correction requires development/tangent yaw and excludes other copter output adapters')
    if args.copter_velocity_filter is not None and (not args.copter_tangent_yaw
            or not math.isfinite(args.copter_velocity_filter) or args.copter_velocity_filter<=0):
        parser.error('velocity filter requires tangent yaw and a positive finite time constant')
    if args.copter_tangent_yaw and (not ((args.development or formal_copter_enabled) and args.copter_yaw_rate and args.copter_synchronous)
                                   or args.copter_command_governor):
        parser.error('tangent yaw requires development/yaw-rate/synchronous and excludes command governor')
    if args.copter_held_control and not (args.development and args.copter_analytic_gradient):
        parser.error('held control requires development and analytic gradient')
    if args.copter_command_governor and not (args.development and args.copter_yaw_rate and args.copter_synchronous):
        parser.error('command governor requires development, yaw-rate and synchronous flags')
    if (not math.isfinite(control_period) or not DT<=control_period<=RATE
            or ((control_period!=RATE or args.copter_analytic_gradient)
                and not (args.development or formal_copter_enabled))):
        parser.error('NMPC timing/gradient experiments require development and DT <= period <= RATE')
    if args.copter_yaw_rate and not (args.development or formal_copter_enabled):
        parser.error('yaw-rate interface experiment requires --development')
    if args.copter_synchronous and not ((args.development or formal_copter_enabled) and args.copter_yaw_rate):
        parser.error('--copter-synchronous requires --development and --copter-yaw-rate')
    if cfg.get('reference_reserve') and not args.development:
        parser.error('reference reserve verification requires --development')
    if args.plane_progress_gain is not None and (not args.development or not math.isfinite(args.plane_progress_gain)
                                                  or args.plane_progress_gain < 0):
        parser.error('--plane-progress-gain requires development and a nonnegative finite value')
    if args.plane_command_accel_scale is not None and (not args.development or not math.isfinite(args.plane_command_accel_scale)
                                                        or not 0 < args.plane_command_accel_scale <= 1):
        parser.error('--plane-command-accel-scale requires development and a value in (0,1]')
    if args.track_seconds is not None and (not args.development or args.track_seconds<=0):
        parser.error('--track-seconds requires --development and a positive duration')
    if not args.development and cfg.get('capability_evidence')!='measured':
        parser.error('formal flights require frozen measured capabilities')
    lo,hi,_,_=team_interval(members,0.)
    if hi-lo<=cfg.get('required_straight_margin',1.):
        parser.error('straight common-speed margin insufficient; radius cannot fix it')
    yaw_rate_feedforward=args.copter_yaw_rate and not args.copter_yaw_angle_only
    if {m.platform for m in members}=={'multirotor','fixedwing'}:
        try:
            cfg['rotor_execution_gate']=rotor_development_gate(
                args.rotor_precheck,cfg,members,yaw_rate_feedforward,args.copter_synchronous,
                control_period,args.copter_analytic_gradient,args.copter_command_governor,args.copter_held_control,
                args.copter_tangent_yaw,args.copter_velocity_filter,args.copter_yaw_angle_only,
                args.copter_consistent_correction)
        except (ValueError,OSError,KeyError) as exc:
            parser.error(str(exc))
    cfg['execution_backend']=('owned MAVLink SITL; direct member reference+GUIDED PPVA, no NMPC'
        if args.copter_reference_only else
        'owned MAVLink SITL; measured-feedback DroneMPC+GUIDED PPVA and Plane '+args.plane_lateral+'+TECS')
    guidance=cfg.setdefault('fixedwing_guidance',dict(lookahead=40.,progress_gain=.3))
    if args.plane_progress_gain is not None:
        guidance['progress_gain']=args.plane_progress_gain
    if args.plane_command_accel_scale is not None:
        guidance['command_accel_scale']=args.plane_command_accel_scale
    cfg['plane_lateral']=args.plane_lateral
    cfg['copter_yaw_rate_feedforward']=yaw_rate_feedforward
    cfg['copter_yaw_angle_only']=args.copter_yaw_angle_only
    cfg['copter_reference_only']=args.copter_reference_only
    cfg['terminal_reference_policy']='straight flythrough extension by elapsed profile overshoot; unchanged exit modes'
    cfg['copter_consistent_correction']=args.copter_consistent_correction
    if args.copter_consistent_correction:
        cfg['copter_correction_settings']=dict(max_offset_m=1.,max_rate_m_s=.2,
            max_accel_m_s2=.1,response_s_inv=.5,
            scope='command correction only; actual vehicle acceleration/yaw still audited')
    cfg['copter_synchronous_setpoint']=args.copter_synchronous
    cfg['copter_analytic_gradient']=args.copter_analytic_gradient
    cfg['copter_command_governor']=args.copter_command_governor
    cfg['copter_held_control']=args.copter_held_control
    cfg['copter_tangent_yaw']=args.copter_tangent_yaw
    cfg['copter_velocity_filter']=args.copter_velocity_filter
    if args.copter_tangent_yaw:
        cfg['copter_yaw_execution']=(
            'path tangent and its derivative; direct geometric reference, no NMPC' if args.copter_reference_only
            else 'path tangent and its derivative; NMPC drives a bounded C1 spatial correction' if args.copter_consistent_correction
            else ('path tangent angle only, angular-rate field ignored' if args.copter_yaw_angle_only
                  else 'path tangent and its derivative')+'; NMPC w affects spatial commands, not physical yaw setpoint')
    if args.copter_command_governor:
        cfg['command_governor']=dict(reserve=.5,yaw_accel_rad_s2=.1,
            scope='execution only; does not change common reference, measured X0, NMPC model or physical limits')
    cfg['planning_members']=[asdict(m) for m in planning_members(cfg,members)]
    if (cfg.get('copter_tangent_velocity') or cfg.get('copter_yaw_cap') is not None
            or cfg.get('copter_predicted_sequence')):
        parser.error('retired experimental copter execution config; use the recorded source snapshot for reproduction')
    cfg['feedback']=dict(source='LOCAL_POSITION_NED + ATTITUDE',world_frame='ENU + per-vehicle home translation',
        alignment='interpolate/short measured-slope extrapolate to newer stream boot timestamp; reference time uses initial per-vehicle clock offsets',
        max_age_s=.5,max_extrapolation_s=.2,max_gap_s=.25,control_period_s=control_period)
    cfg['fixedwing_limit_semantics'] = {
        'planning': 'members: common-reference feasibility limits',
        'controller': 'fixedwing_controller_limits: A/B-matched inner tracking authority',
        'acceptance': 'frozen v3 actual-response thresholds; evaluated independently',
    }
    cfg['track_seconds']=args.track_seconds
    cfg['seed']=args.seed; cfg['method']=args.method; cfg['speedup']=args.speedup
    directory=freeze_run(args.output,cfg,'development_fleet' if args.development else 'paper_fleet')
    planning_start=time.monotonic()
    result=coordinator_from_config(cfg,members,nominal_only=args.method!='F').plan(cfg['waypoints'])
    if args.method=='C' and result.feasible:
        try:
            p=result.profile
            constant=ProgressProfile(p.s,np.full(len(p.s),cfg['planner']['nominal_speed']**2),
                                     p.team_lower,p.team_upper,p.active_lower,p.active_upper,{})
            constant.margins=validate_profile(result.route,planning_members(cfg,members),constant)
            result.profile=constant
            result.candidates[0].duration=float(constant.time[-1])
            result.candidates[0].margins=constant.margins
        except PlanningError as exc:
            result.candidates[0].status='CONSTANT_PROGRESS_REJECTED'
            result.candidates[0].detail=exc.detail
            result.route=result.profile=result.selected_radius=None
    export_plan(directory,result,members)
    if not result.feasible:
        write_json(directory/'flight_status.json',dict(status='REFERENCE_REJECTED',completed=False,
                   compute_seconds=time.monotonic()-planning_start))
        return 2
    route,profile=result.route,result.profile
    print('PLANNED',result.selected_radius,'duration',profile.time[-1],flush=True)
    rng=random.Random(args.seed)
    vehicles=[]; states={}; controls={}; solve_at={}; plane_speeds={}; violations_since={}
    last_transmissions={}; last_command_wall={}
    command_streamer=None
    governors={}; governor_times={}; velocity_filters={}; filter_times={}; corrections={}
    plane_controller_members={m.member_id:fixedwing_controller_member(cfg,m)
                              for m in members if m.platform=='fixedwing'}
    plane_trackers={m.member_id:PlaneCourseTracker(plane_controller_members[m.member_id],**guidance)
                    for m in members if m.platform=='fixedwing'}
    status=dict(status='FAILED',completed=False,seed=args.seed,method=args.method,
                planning_seconds=time.monotonic()-planning_start,full_route_completed=False)
    nominal=cfg['planner']['nominal_speed']
    fields=['t','attitude_t','velocity_t','airspeed_t','phase','member','s','nu','nu_dot','kappa','kappa_prime','radius','ref_x','ref_y','ref_yaw',
            'ref_speed','ref_member_curvature','lower','upper','lower_owner','upper_owner','lower_limit','upper_limit','x','y','alt',
            'vx','vy','airspeed','groundspeed','roll','pitch','yaw','command_speed','command_bank',
            'mode','armed','mpc_failures','controller_seconds','clock_drift',
            'command_x','command_y','command_yaw','command_yaw_rate','feedback_x','feedback_y','feedback_yaw','feedback_stamp',
            'optimizer_speed','optimizer_yaw_rate']
    flight_stream=(directory/'flight.csv').open('w',newline='',buffering=EVIDENCE_BUFFER_BYTES)
    flight_writer=csv.DictWriter(flight_stream,fieldnames=fields)
    flight_writer.writeheader()
    feedback_stream=(directory/'feedback_solves.jsonl').open('w',buffering=EVIDENCE_BUFFER_BYTES)
    guidance_stream=(directory/'fixedwing_commands.jsonl').open('w',buffering=EVIDENCE_BUFFER_BYTES)

    def write_json_record(stream, record):
        stream.write(json.dumps(record)+'\n')

    try:
        for m in members:
            plane=m.platform=='fixedwing'
            candidate=cfg.get('copter_platform_candidate') or {}
            vehicle=SitlVehicle('plane' if plane else 'copter','plane' if plane else candidate.get('model','x:freestyle.json'),
                directory/('vehicle_%d'%m.member_id),args.base_port+10*m.member_id,
                cfg['altitude'],args.speedup,None if plane else candidate.get('defaults',AP/'Tools/autotest/models/freestyle.parm'),
                home_offset=(rng.uniform(-.2,.2),m.r_y+rng.uniform(-.2,.2)),
                gcs_system_id=m.member_id+1)
            vehicles.append(vehicle)
        # Independent localhost links are serviced concurrently during startup
        # and takeoff so no airborne member waits through another's telemetry boot.
        with ThreadPoolExecutor(max_workers=len(vehicles)) as pool:
            for future in [pool.submit(v.start) for v in vehicles]:
                future.result()
        def ready(v):
            v.takeoff()
            if v.vehicle=='plane':
                v.mode('LOITER')
        with ThreadPoolExecutor(max_workers=len(vehicles)) as pool:
            for future in [pool.submit(ready,v) for v in vehicles]:
                future.result()
        for _ in range(20):
            for v in vehicles:
                v.pump(.01)
        for v in vehicles:
            v.mode('GUIDED')
        command_streamer=ProcessCommandStreamer(vehicles)
        command_streamer.start()
        # v6 execution runs the control loop and the cached-command streamer
        # on separate CPUs.  With no protocol setting this is a no-op, keeping
        # the legacy runner behavior unchanged.
        pin_current_thread_from_env('UAV_CONTROL_CPU')
        rows=[v.row() for v in vehicles]
        origin=[sum(r['east'] for r in rows)/len(rows),sum(r['north']-m.r_y for r,m in zip(rows,members))/len(rows)]
        starts=[r['sim_s'] for r in rows]
        previous=starts[0]; entry_start=previous; joined_since=None; tracking_start=None
        wall_deadline=time.monotonic()+900/args.speedup
        s=0.; phase='join'; last_print=-100.
        while True:
            command_streamer.check()
            for v in vehicles:
                v.pump(.003)
            now=vehicles[0].row()['sim_s']; dt=now-previous
            if dt<.05:
                wall_now=time.monotonic()
                # Once the dedicated streamer is active it is the sole
                # retransmission source.  Keeping this legacy fallback alive
                # would create two writers on each MAVLink socket and produce
                # bursts followed by an artificial timing gap.
                if command_streamer is None:
                    for i,v in enumerate(vehicles):
                        if (wall_now-last_command_wall.get(i,-math.inf)>=.045
                                and retransmit_fleet_reference(v,last_transmissions.get(i))):
                            last_command_wall[i]=time.monotonic()
                if time.monotonic()>wall_deadline:
                    raise TimeoutError('fleet deadline')
                continue
            if dt>1.:
                raise RuntimeError('controller simulation-time gap exceeds one second')
            previous=now
            if phase=='track':
                s=profile.advance(s,dt)
            terminal_elapsed=(max(0.,now-tracking_start-float(profile.time[-1]))
                if phase=='track' and cfg['endpoint']=='flythrough' else 0.)
            elapsed=now-entry_start
            if phase=='join':
                # Formation capture has no mission progress yet. Anchor its
                # homogeneous longitudinal center to its measured centroid.
                # A mixed fleet instead needs a stable moving rendezvous frame:
                # a centroid coupled to Plane's intercept orbit never settles.
                update_join_origin(origin,[v.row() for v in vehicles],members,dt,nominal)
            refs=[]
            for m in members:
                if phase=='join':
                    refs.append((origin[0],origin[1]+m.r_y,0.,nominal,0.,0.))
                else:
                    x,y,psi,speed,ki,at=member_reference(route,profile,s,m.r_y,terminal_elapsed)
                    refs.append((origin[0]+x,origin[1]+y,psi,speed,ki,at))
            errors=[]
            for i,(m,v,ref) in enumerate(zip(members,vehicles,refs)):
                began=time.monotonic(); row=v.row(); x,y,psi,speed,ki,at=ref
                error=math.hypot(row['east']-x,row['north']-y); errors.append(error)
                if not row['armed'] or row['mode']!='GUIDED':
                    raise RuntimeError('member %d left armed GUIDED'%i)
                if max(row['attitude_age'],row['velocity_age'],row['airspeed_age'])>.8:
                    raise RuntimeError('member %d stale telemetry'%i)
                if row['alt']<cfg['altitude']*.5:
                    raise RuntimeError('member %d altitude protection'%i)
                bank_command=None
                command_target=None; command_yaw_rate=None
                if m.platform=='fixedwing':
                    v.check_plane_commands()
                    if args.plane_lateral=='course':
                        if phase=='track':
                            (px,py,_),_=preview_member(route,profile,s,m.r_y,guidance['lookahead']/speed+terminal_elapsed,True)
                            reference=(x,y,psi,speed,ki,cfg['altitude'],origin[0]+px,origin[1]+py)
                        else:
                            reference=(x,y,psi,speed,ki,cfg['altitude'])
                        command=plane_trackers[i].update(reference,(row['east'],row['north']),row['airspeed'],dt)
                        command_speed,bearing=command.airspeed,command.bearing_deg
                        v.plane_reference(command_speed,bearing,command.lateral_accel_limit,
                                          lateral_mode='course',altitude=command.altitude)
                        last_transmissions[i]=('plane',command)
                        command_streamer.update(
                            i,last_transmissions[i],v.command_stream_context())
                        last_command_wall[i]=time.monotonic()
                        write_json_record(guidance_stream,dict(member=i,t=now,phase=phase,reference=reference,
                            measured=dict(x=row['east'],y=row['north'],airspeed=row['airspeed'],groundspeed=row['groundspeed'],
                                          roll=row['roll'],altitude=row['alt'],velocity_t=row['velocity_sim_s']),
                            command=asdict(command),acks=v.command_acks))
                    else:
                        command_speed,bearing,bank_command=plane_command(m,(x,y,psi,speed,ki),
                            (row['east'],row['north'],row['vn'],row['ve'],row['groundspeed']))
                        previous_speed=plane_speeds.get(i,max(m.v_min,min(m.v_max,row['airspeed'])))
                        command_speed=previous_speed+max(m.a_t_min*dt,min(m.a_t_max*dt,command_speed-previous_speed))
                        plane_speeds[i]=command_speed
                        v.plane_reference(command_speed,bearing,3.,bank=bank_command,lateral_mode='external_roll')
                else:
                    if phase=='join':
                        command_speed=speed
                        dx,dy=x-row['east'],y-row['north']; scale=min(1.,5./max(math.hypot(dx,dy),1e-6))
                        v.copter_reference(row['east']+scale*dx,row['north']+scale*dy,speed,0.,accel=(0.,0.))
                    else:
                        if args.copter_reference_only:
                            command_target,command_velocity,acceleration,yaw_rate_ned=reference_copter_setpoint(ref)
                            command_speed=speed
                            command_yaw_rate=-yaw_rate_ned
                            if not args.copter_yaw_rate:
                                yaw_rate_ned=None
                            command_yaw_rate=send_copter_setpoint(v,command_target,command_velocity,
                                acceleration,yaw_rate_ned)
                        else:
                            if i not in states:
                                mpc=DroneMPC(i,m.v_max,analytic_gradient=args.copter_analytic_gradient,
                                             held_control=args.copter_held_control); states[i]=mpc
                                solve_at[i]=-100.
                            mpc=states[i]
                            source_now=max(row['sim_s'],row['velocity_sim_s'])
                            # Align reference and measurement using each simulator's
                            # initial clock offset, retaining residual drift in the log.
                            alignment_delta=(source_now-starts[i])-(now-starts[0])
                            aligned_time=max(0.,now-tracking_start+alignment_delta)
                            aligned_s=profile.advance(0.,aligned_time)
                            aligned_terminal=(max(0.,aligned_time-float(profile.time[-1]))
                                if cfg['endpoint']=='flythrough' else 0.)
                            try:
                                measured=v.feedback.at(source_now,source_now,time.monotonic())
                            except FeedbackError as exc:
                                write_json_record(feedback_stream,dict(member=i,event='FEEDBACK_INVALID',
                                    reason=str(exc),fleet_time=now,source_time=source_now))
                                v.mode('LOITER')
                                raise RuntimeError('member %d feedback invalid: %s'%(i,exc))
                            if source_now-solve_at[i]>=control_period-1e-6:
                                rr=np.zeros((3,N+1)); vv=np.zeros(N)
                                for j in range(N+1):
                                    (xx,yy,pp),future_speed=preview_member(route,profile,aligned_s,m.r_y,j*DT+aligned_terminal,cfg['endpoint']=='flythrough')
                                    rr[:,j]=(origin[0]+xx,origin[1]+yy,pp)
                                    if j<N:
                                        vv[j]=future_speed
                                controls[i]=mpc.solve_measured(rr,vv,measured,control_period,0.)
                                solve_at[i]=source_now
                                write_json_record(feedback_stream,dict(member=i,event='SOLVE',phase=phase,
                                    fleet_time=now,aligned_s=aligned_s,clock_delta=alignment_delta,
                                    measured=asdict(measured),x0=mpc.state.tolist(),reference0=rr[:,0].tolist(),
                                    control=list(controls[i]),optimizer_sequence=mpc.u_prev.tolist() if mpc.u_prev is not None else None,
                                    status=mpc.last_status,detail=mpc.last_detail,
                                    failures=mpc.fail_count,solve_wall_s=time.monotonic()-began))
                                if mpc.last_status!='SOLVED':
                                    print('NMPC',i,mpc.last_status,mpc.last_detail,flush=True)
                            command_speed,w=controls[i]
                            command_yaw_rate=w if args.copter_yaw_rate else None
                            hold=min(2*control_period,max(0.,source_now-solve_at[i])+control_period)
                            command_target=command_endpoint(mpc.last_feedback.state,command_speed,w,hold)
                            heading=command_target[2]
                            yaw_rate_ned=None
                            if args.copter_yaw_rate:
                                heading=command_endpoint(mpc.last_feedback.state,command_speed,w,
                                    max(0.,source_now-solve_at[i]))[2]
                                yaw_rate_ned=-w
                                command_target=(command_target[0],command_target[1],heading)
                            if args.copter_synchronous:
                                command_target,_=synchronous_copter_target(mpc.last_feedback.state,
                                    command_speed,w,max(0.,source_now-solve_at[i]))
                                heading=command_target[2]
                            command_velocity=(command_speed*math.cos(heading),command_speed*math.sin(heading))
                            ae=at*math.cos(psi)-speed*speed*ki*math.sin(psi)
                            an=at*math.sin(psi)+speed*speed*ki*math.cos(psi)
                            if args.copter_command_governor:
                                if i not in governors:
                                    governors[i]=CopterExecutionGovernor(m,measured.state,math.hypot(row['ve'],row['vn']))
                                    governor_times[i]=source_now
                                elapsed_command=source_now-governor_times[i]
                                if elapsed_command>0:
                                    executed=governors[i].update(command_speed,w,elapsed_command)
                                    governor_times[i]=source_now
                                    command_target=executed['state'];command_velocity=executed['velocity']
                                    ae,an=executed['acceleration'];heading=command_target[2]
                                    command_speed=executed['speed'];command_yaw_rate=executed['yaw_rate']
                                    yaw_rate_ned=-command_yaw_rate
                                else:
                                    command_target=governors[i].state;heading=command_target[2]
                                    command_speed=governors[i].speed;command_yaw_rate=governors[i].rate
                                    command_velocity=(command_speed*math.cos(heading),command_speed*math.sin(heading))
                                    ae,an=0.,0.;yaw_rate_ned=-command_yaw_rate
                            if args.copter_tangent_yaw:
                                heading=psi;command_yaw_rate=speed*ki;yaw_rate_ned=-command_yaw_rate
                                command_target=(command_target[0],command_target[1],heading)
                            if args.copter_velocity_filter is not None:
                                if i not in velocity_filters:
                                    velocity_filters[i]=CopterVelocityFilter(m,(row['ve'],row['vn']),args.copter_velocity_filter)
                                    filter_times[i]=source_now
                                elapsed_filter=source_now-filter_times[i]
                                if elapsed_filter>0:
                                    command_velocity,(ae,an)=velocity_filters[i].update(command_velocity,elapsed_filter)
                                    filter_times[i]=source_now
                                else:
                                    command_velocity=velocity_filters[i].velocity;ae,an=0.,0.
                                command_speed=math.hypot(*command_velocity)
                            if args.copter_consistent_correction:
                                if i not in corrections:
                                    corrections[i]=CopterReferenceCorrection()
                                command_target,command_velocity,(ae,an),yaw_rate_ned=corrections[i].update(
                                    ref,command_target[:2],dt)
                                command_speed=math.hypot(*command_velocity)
                                command_yaw_rate=-yaw_rate_ned
                            command_yaw_rate=send_copter_setpoint(v,command_target,command_velocity,
                                (ae,an),yaw_rate_ned,yaw_angle_only=args.copter_yaw_angle_only)
                        if phase == 'track':
                            last_transmissions[i]=('copter',(command_target,command_velocity,(ae,an),
                                                            yaw_rate_ned,args.copter_yaw_angle_only))
                            command_streamer.update(
                                i,last_transmissions[i],v.command_stream_context())
                            last_command_wall[i]=time.monotonic()
                v.phase=phase
                drift=(row['sim_s']-starts[i])-(now-starts[0])
                if phase=='track' and abs(drift)>1.:
                    raise RuntimeError('SITL clocks diverged by more than 1 s')
                if phase=='track' and (error>30 or abs(math.degrees(row['roll']))>60):
                    raise RuntimeError('tracking protection: member %d error %.2f'%(i,error))
                actual_speed=row['airspeed'] if m.platform=='fixedwing' else math.hypot(row['ve'],row['vn'])
                if cfg.get('frozen_contract'):
                    outside=(abs(actual_speed-speed)>1.0 or
                             (m.platform=='fixedwing' and abs(row['roll'])>math.radians(27.0)))
                else:
                    outside=(actual_speed<m.v_min-.5 or actual_speed>m.v_max+.5 or
                             (m.bank_max is not None and abs(row['roll'])>m.bank_max+math.radians(3)))
                if phase=='track' and outside:
                    violations_since.setdefault(i,now)
                    if now-violations_since[i]>2.:
                        raise RuntimeError('sustained platform envelope excursion: member %d'%i)
                else:
                    violations_since.pop(i,None)
                nu=profile.nu_at(s) if phase=='track' else nominal
                k=route.curvature_at(s) if phase=='track' else 0.
                lower,upper,lower_owner,upper_owner=team_interval(members,k)
                flight_writer.writerow(dict(t=now-(tracking_start if tracking_start is not None else entry_start),phase=phase,
                    attitude_t=row['sim_s'],velocity_t=row['velocity_sim_s'],airspeed_t=row['airspeed_sim_s'],
                    member=i,s=s,nu=nu,nu_dot=.5*profile.slope_at(s) if phase=='track' and terminal_elapsed==0 else 0.,kappa=k,
                    kappa_prime=route.curvature_derivative_at(s) if phase=='track' else 0.,radius=result.selected_radius,
                    ref_x=x,ref_y=y,ref_yaw=psi,ref_speed=speed,ref_member_curvature=ki,lower=lower,upper=upper,
                    lower_owner=lower_owner[0],upper_owner=upper_owner[0],lower_limit=int(lower_owner[1]),upper_limit=int(upper_owner[1]),
                    x=row['east'],y=row['north'],alt=row['alt'],
                    vx=row['ve'],vy=row['vn'],airspeed=row['airspeed'],groundspeed=row['groundspeed'],roll=row['roll'],
                    pitch=row['pitch'],yaw=math.pi/2-row['yaw'],command_speed=command_speed,command_bank=bank_command,
                    mode=row['mode'],armed=row['armed'],mpc_failures=states[i].fail_count if i in states else 0,
                    controller_seconds=time.monotonic()-began,clock_drift=drift,
                    command_x=command_target[0] if command_target else None,
                    command_y=command_target[1] if command_target else None,
                    command_yaw=command_target[2] if command_target else None,
                    command_yaw_rate=command_yaw_rate,
                    feedback_x=states[i].last_feedback.x if i in states else None,
                    feedback_y=states[i].last_feedback.y if i in states else None,
                    feedback_yaw=states[i].last_feedback.yaw if i in states else None,
                    feedback_stamp=states[i].last_feedback.stamp if i in states else None,
                    optimizer_speed=controls[i][0] if i in controls else None,
                    optimizer_yaw_rate=controls[i][1] if i in controls else None))
            if phase=='join':
                speed_errors=[abs((v.row()['airspeed'] if v.vehicle=='plane' else v.row()['groundspeed'])-nominal) for v in vehicles]
                settled=all(abs(v.row()['alt']-cfg['altitude'])<2.
                    and abs(math.atan2(v.row()['vn'],v.row()['ve']))<math.radians(5)
                    for v in vehicles)
                if max(errors)<3. and max(speed_errors)<.5 and settled:
                    if joined_since is None:
                        joined_since=now
                    if now-joined_since>5:
                        phase='track'; tracking_start=now
                        write_json(directory/'world_origin.json',dict(origin=origin,join_seconds=elapsed,initial_clock_offsets=starts))
                        print('TRACKING',origin,flush=True)
                else:
                    joined_since=None
                if elapsed>450:
                    raise RuntimeError('formation join timeout')
            elif s>=route.total_length()-1e-5:
                status.update(status='COMPLETED',completed=True,duration=now-tracking_start,full_route_completed=True)
                break
            elif args.track_seconds is not None and now-tracking_start>=args.track_seconds:
                status.update(status='SHORT_TEST_COMPLETED',completed=True,duration=now-tracking_start,
                              full_route_completed=False)
                break
            if elapsed-last_print>10:
                print(phase,'s',round(s,1),'errors',[round(e,2) for e in errors],flush=True); last_print=elapsed
            if time.monotonic()>wall_deadline:
                raise TimeoutError('fleet deadline')
    except (Exception, KeyboardInterrupt) as exc:
        status['error']=repr(exc); print('FAILED',repr(exc),flush=True)
        write_json_record(feedback_stream,dict(event='RUN_ABORT',reason=repr(exc),wall_time=time.time()))
    finally:
        if command_streamer is not None:
            try:
                command_streamer.stop()
            except Exception as exc:
                status.setdefault('exit_errors',[]).append(repr(exc))
        # Set every vehicle into its exit mode before waiting for any one of them.
        for v in vehicles:
            try:
                if v.link and 'HEARTBEAT' in v.messages:
                    # End the task phase before any vehicle waits through another
                    # member's landing. Otherwise source-clock telemetry from
                    # cleanup can be mislabeled as track data.
                    v.set_phase('exit')
                    v.mode('LOITER' if v.vehicle=='plane' else 'LAND')
            except Exception as exc:
                status.setdefault('exit_errors',[]).append(repr(exc))
        for v in vehicles:
            try:
                v.finish()
            except Exception as exc:
                status.setdefault('exit_errors',[]).append(repr(exc))
            finally:
                v.close()
        try:
            for stream in (flight_stream,feedback_stream,guidance_stream):
                stream.flush(); stream.close()
        except Exception as exc:
            status.setdefault('exit_errors',[]).append('evidence serialization: '+repr(exc))
        status['mpc_failures']={i:m.fail_count for i,m in states.items()}
        status['task_completed']=status['completed']
        status['exit_completed']=not bool(status.get('exit_errors'))
        if not status['exit_completed']:
            status.update(status='EXIT_FAILED',completed=False)
        write_json(directory/'flight_status.json',status)
        manifest=json.loads((directory/'run_manifest.json').read_text())
        manifest['result']=status
        write_json(directory/'run_manifest.json',manifest)
        raw=directory/'raw'; raw.mkdir(exist_ok=True)
        write_json(raw/'index.json',dict(vehicles=[str(v.directory.relative_to(directory)) for v in vehicles],
            files='each vehicle contains raw.tlog, logs/*.BIN, telemetry.csv, events.jsonl, sitl.log and defaults.parm'))
    return 0 if status['completed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
