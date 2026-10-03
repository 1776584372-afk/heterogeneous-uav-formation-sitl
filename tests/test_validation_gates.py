import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

from scripts.paper_batch import prepare
from scripts.compare_reference import integrate_individual_progress
from uav_formation.icc_traj import IccRoute
from uav_formation.capability_model import MemberSpec
from scripts.run_paper_sitl import (rotor_development_gate,
    rotor_precheck_planner_compatible, update_join_origin,
                                    CopterCommandStreamer)
from scripts.audit_measured_feedback import audit


class ValidationGates(unittest.TestCase):
    def test_command_streamer_repeats_only_cached_copter_commands(self):
        class Vehicle:
            def __init__(self): self.calls=[]
            def copter_reference(self,*args,**kwargs): self.calls.append((args,kwargs))
        vehicle=Vehicle()
        transmission=('copter',((1.,2.,0.),(3.,4.),(.1,.2),-.3,False))
        streamer=CopterCommandStreamer([vehicle],{0:transmission},interval=.045)
        streamer.next_send=1.

        assert streamer.step(.99)==0
        assert streamer.step(1.)==1
        assert len(vehicle.calls)==1
        assert streamer.next_send==1.045

    def test_command_streamer_retransmits_cached_plane_commands(self):
        class Vehicle:
            def __init__(self): self.calls=[]
            def plane_reference(self,*args,**kwargs): self.calls.append((args,kwargs))
        vehicle=Vehicle()
        command=type('Command', (), {
            'airspeed': 14., 'bearing_deg': 90.,
            'lateral_accel_limit': 2., 'altitude': 60.})()
        streamer=CopterCommandStreamer([vehicle],{0:('plane',command)},interval=.045)
        streamer.next_send=1.
        assert streamer.step(.99)==0
        assert streamer.step(1.)==1
        assert len(vehicle.calls)==1

    def test_mixed_join_origin_advances_without_following_vehicle_centroid(self):
        rotor=MemberSpec(0,'multirotor',-10,14,15,.8579)
        plane=MemberSpec(1,'fixedwing',10,14,15,.8579,bank_max=.1)
        origin=[100.,5.]
        update_join_origin(origin,[{'east':0.},{'east':600.}],[rotor,plane],.2,14.5)
        self.assertEqual(origin,[102.9,5.])

        update_join_origin(origin,[{'east':40.},{'east':60.}],[rotor,rotor],.2,14.5)
        self.assertEqual(origin,[50.,5.])

    def test_nmpc_audit_does_not_require_fixedwing_solves(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)
            (path/'config.json').write_text(json.dumps(dict(members=[
                dict(member_id=0,platform='multirotor',v_max=20),
                dict(member_id=1,platform='fixedwing',v_max=24)])))
            rows=[dict(event='SOLVE',member=0,x0=[1,2,0],control=[12,0],
                measured=dict(x=1,y=2,yaw=0,position_age=0,yaw_age=0,
                              extrapolation=0,stamp=t),solve_wall_s=.01,status='OK')
                for t in (0.,.35,.7)]
            (path/'feedback_solves.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
            result=audit(path)
            self.assertTrue(result['feedback_check_passed'])
            self.assertEqual(result['non_nmpc_members'],[1])
            (path/'feedback_solves.jsonl').write_text('')
            self.assertFalse(audit(path)['feedback_check_passed'])

    def test_mixed_fleet_requires_matching_straight_tracking_evidence(self):
        member=MemberSpec(0,'fixedwing',0,12,24,2.5,bank_max=.4)
        cfg={'planner':{'nominal_speed':14},'required_straight_margin':1}
        with self.assertRaises(ValueError): rotor_development_gate([],cfg,[member],True)
        with tempfile.TemporaryDirectory() as folder:
            paths=[]
            for speed in (12,14):
                path=Path(folder)/str(speed);path.mkdir();paths.append(path)
                (path/'acceptance.json').write_text(json.dumps(dict(tracking_qualified=True,full_route_completed=True,measured_constraints_passed=False)))
                (path/'config.json').write_text(json.dumps(dict(members=[{'platform':'multirotor'}],
                    waypoints=[[0,0],[900,0]],planner={'nominal_speed':speed},copter_yaw_rate_feedforward=True)))
            # Legacy gate inputs omit the new optional fields and therefore
            # represent the default yaw-rate backend.
            for path in paths:
                data=json.loads((path/'config.json').read_text())
                data['copter_synchronous_setpoint']=False
                data['copter_tangent_velocity']=False
                data['copter_yaw_cap']=None
                (path/'config.json').write_text(json.dumps(data))
            self.assertEqual(rotor_development_gate(paths,cfg,[member],True)['sampled_speeds'],[12,14])
            candidate_cfg=dict(cfg,copter_platform_candidate={'model':'x','defaults_sha256':{'candidate.parm':'abc'}})
            with self.assertRaisesRegex(ValueError,'mismatch'):
                rotor_development_gate(paths,candidate_cfg,[member],True)
            with self.assertRaisesRegex(ValueError,'mismatch'): rotor_development_gate(paths,cfg,[member],False)
            with self.assertRaisesRegex(ValueError,'mismatch'): rotor_development_gate(paths,cfg,[member],True,True)
            source=json.loads((paths[0]/'config.json').read_text())
            source['copter_predicted_sequence']=True
            (paths[0]/'config.json').write_text(json.dumps(source))
            with self.assertRaisesRegex(ValueError,'mismatch'): rotor_development_gate(paths,cfg,[member],True)
            source['copter_predicted_sequence']=False
            source['copter_yaw_cap']=.25
            (paths[0]/'config.json').write_text(json.dumps(source))
            with self.assertRaisesRegex(ValueError,'mismatch'): rotor_development_gate(paths,cfg,[member],True)

    def test_v3_gate_accepts_matching_system_precheck(self):
        rotor=MemberSpec(0,'multirotor',-10,14,15,.8579,a_t_min=-.3,a_t_max=.3)
        plane=MemberSpec(1,'fixedwing',0,14,15,.8579,bank_max=.1)
        planner={'nominal_speed':14.5,'radii':[240,320]}
        execution={'mode':'measured_feedback_nmpc_consistent_correction'}
        contract={'capability':{'sha256':'frozen'}}
        cfg={'planner':planner,'waypoints':[[0,0],[100,0],[100,100]],
             'members':[], 'frozen_contract':contract,'multicopter_execution':execution}
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)
            (path/'config.json').write_text(json.dumps(dict(
                members=[{'member_id':0,'platform':'multirotor','r_y':-10},
                         {'member_id':2,'platform':'multirotor','r_y':10}],
                planner=planner,waypoints=cfg['waypoints'],frozen_contract=contract,
                multicopter_execution=execution,copter_yaw_rate_feedforward=True,
                copter_synchronous_setpoint=True,copter_analytic_gradient=True,
                copter_tangent_yaw=True,copter_consistent_correction=True)))
            (path/'v3_formal_acceptance.json').write_text(json.dumps(dict(
                classification='SYSTEM_PRECHECK_QUALIFIED',system_precheck_qualified=True,
                invalid_reasons=[],acceptance_failures=[],
                lifecycle={'completed':True,'exit_completed':True,'duration':90.})))
            result=rotor_development_gate([path],cfg,[rotor,plane],True,
                synchronous=True,analytic_gradient=True,tangent_yaw=True,
                consistent_correction=True)
            self.assertEqual(result['status'],'V3_SYSTEM_PRECHECK_GATE')
            self.assertEqual(result['evidence'][0]['members'],2)

            conservative=dict(cfg)
            conservative['planner']=dict(planner,radii=[320],nominal_radius=320)
            result=rotor_development_gate([path],conservative,[rotor,plane],True,
                synchronous=True,analytic_gradient=True,tangent_yaw=True,
                consistent_correction=True)
            self.assertEqual(result['status'],'V3_SYSTEM_PRECHECK_GATE')

            bad=json.loads((path/'v3_formal_acceptance.json').read_text())
            bad['invalid_reasons']=['member_0_command_timing']
            (path/'v3_formal_acceptance.json').write_text(json.dumps(bad))
            with self.assertRaisesRegex(ValueError,'did not qualify'):
                rotor_development_gate([path],cfg,[rotor,plane],True,
                    synchronous=True,analytic_gradient=True,tangent_yaw=True,
                    consistent_correction=True)

    def test_rotor_precheck_allows_radius_policy_but_not_speed_change(self):
        source={'nominal_speed':14.5,'start_speed':14.5,'end_speed':14.5,
                'ds':4.,'trans_len':320.,'radii':[240,280]}
        target=dict(source,radii=[320],nominal_radius=320.)
        self.assertTrue(rotor_precheck_planner_compatible(source,target))
        target['nominal_speed']=15.
        self.assertFalse(rotor_precheck_planner_compatible(source,target))

    def test_short_or_constraint_failed_precheck_cannot_start_formal_batch(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);caps=root/'caps';caps.mkdir();run=root/'run';run.mkdir()
            for scene in ('s1','s2','s3'):
                (caps/(scene+'.yaml')).write_text('{}')
            checks={scene:run for scene in ('s1','s2','s3')}
            for full,qualified,message in ((False,True,'full-route'),(True,False,'acceptance')):
                (run/'flight_status.json').write_text(json.dumps(dict(completed=True,exit_completed=True,full_route_completed=full)))
                (run/'acceptance.json').write_text(json.dumps(dict(formal_qualified=qualified)))
                with self.assertRaisesRegex(ValueError,message): prepare(root/'batch',caps,checks,1.)
                self.assertFalse((root/'batch').exists())

    def test_a1_integrates_distinct_progress_and_continues_past_endpoint(self):
        members=[MemberSpec(i,'multirotor',float(2*i),0,5,3) for i in range(2)]
        route=IccRoute([(0.,0.),(10.,0.)],10.,1.,strict_geometry=True)
        with tempfile.TemporaryDirectory() as folder:
            stats=integrate_individual_progress(Path(folder),route,members,np.array([0.,10.]),
                                               [np.array([2.,2.]),np.array([1.,1.])],dt=.05)
        self.assertAlmostEqual(stats['max_progress_spread_m'],10.,places=6)
        self.assertAlmostEqual(stats['max_reference_shape_error_m'],np.hypot(10.,2.)-2.,places=6)
