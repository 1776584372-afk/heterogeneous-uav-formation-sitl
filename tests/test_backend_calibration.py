import contextlib
import io
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from scripts.analyze_backend_dynamics import distribution, telemetry_metrics
from scripts.calibrate_sitl import main as calibrate_main, quintic_speed
from scripts.calibrate_turns import bank_tracking_metrics, turn_phase_name
from scripts.calibrate_coupled_turns import (
    advance_fixed_curvature, retransmit_reference, scheduled_speed, segment_schedule,
)
from scripts.diagnose_calibration_truth import acceleration_stats


class CalibrationEvidenceTests(unittest.TestCase):
    def test_duplicate_source_timestamp_retransmits_last_reference(self):
        class Vehicle:
            def __init__(self):
                self.plane=[]; self.copter=[]; self.checks=0
            def plane_reference(self,*args,**kwargs): self.plane.append((args,kwargs))
            def check_plane_commands(self): self.checks+=1
            def copter_reference(self,*args,**kwargs): self.copter.append((args,kwargs))
        vehicle=Vehicle()
        command=SimpleNamespace(airspeed=14.,bearing_deg=80.,
            lateral_accel_limit=2.5,altitude=60.)
        self.assertTrue(retransmit_reference(vehicle,('plane',command)))
        self.assertEqual(vehicle.checks,1)
        self.assertEqual(vehicle.plane[0][1],{'lateral_mode':'course','altitude':60.})
        values=(1.,2.,3.,4.,.5,(.1,.2),-.03)
        self.assertTrue(retransmit_reference(vehicle,('copter',values)))
        self.assertEqual(vehicle.copter[0][0],values[:6])
        self.assertEqual(vehicle.copter[0][1],{'yaw_rate':-.03})
        self.assertFalse(retransmit_reference(vehicle,None))

    def test_continuous_position_preserves_time_at_each_speed_step(self):
        class FakeVehicle:
            def __init__(self, platform, model, output, *args, **kwargs):
                self.directory=Path(output)
                self.t=0.
                self.phase='takeoff'
                self.east=self.north=self.ve=self.vn=0.
                self.commands=[]
                self.samples=[]
                self.link=True

            def start(self): pass
            def takeoff(self): pass
            def finish(self): pass
            def close(self): pass

            def row(self):
                return dict(sim_s=self.t,east=self.east,north=self.north,airspeed=0.,
                    alt=60.,mode='GUIDED',armed=True,roll=0.,throttle=20.,
                    attitude_age=0.,velocity_age=0.,airspeed_age=0.,
                    heading_forward=self.ve,heading_left=self.vn,
                    ref_east=self.east,ref_north=self.north)

            def pump(self, duration):
                self.t+=.05
                self.samples.append(self.row())

            def copter_reference(self, east, north, ve, vn, **kwargs):
                self.commands.append((self.phase,self.t,east,north,ve,vn))
                self.east,self.north,self.ve,self.vn=east,north,ve,vn

        for direction in ('forward','backward','left','right'):
            with self.subTest(direction=direction), tempfile.TemporaryDirectory() as tmp:
                vehicle=FakeVehicle('copter','unused',tmp)
                argv=['calibrate_sitl.py','--vehicle','copter','--output',tmp,
                    '--speeds','2,3,2','--ramp-rate','1','--ramp-shape','quintic',
                    '--approach-rate','2','--approach-settle','1',
                    '--ppva','--continuous-position','--direction',direction]
                with patch('sys.argv',argv), patch('scripts.calibrate_sitl.SitlVehicle',return_value=vehicle), contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(calibrate_main(),0)
                transitions=0
                for prev,now in zip(vehicle.commands,vehicle.commands[1:]):
                    dt=now[1]-prev[1]
                    self.assertAlmostEqual(now[2]-prev[2],now[4]*dt,places=9)
                    self.assertAlmostEqual(now[3]-prev[3],now[5]*dt,places=9)
                    if now[0]!=prev[0]:
                        transitions+=1
                        self.assertGreater(np.hypot(now[2]-prev[2],now[3]-prev[3]),0.)
                # v2 records ramp, settle, and steady phases independently so
                # capability statistics cannot mix transition and steady data.
                self.assertEqual(transitions,9)
                formal=[command for command in vehicle.commands
                        if not command[0].startswith('approach_')]
                self.assertAlmostEqual(math.hypot(formal[0][4],formal[0][5]),2.)

    def test_path_guidance_checks_actual_bank_not_an_unsent_bank_target(self):
        errors,diagnostics=bank_tracking_metrics([9.,10.1],7.,course_control=True)
        self.assertEqual(errors,{'bank_abs_deg':10.1})
        self.assertAlmostEqual(diagnostics['equivalent_bank_deviation_deg'],3.1)
        errors,_=bank_tracking_metrics([-29.,-28.],-7.,course_control=True)
        self.assertGreater(errors['bank_abs_deg'],27.)
        errors,_=bank_tracking_metrics([9.,10.1],7.,course_control=False)
        self.assertAlmostEqual(errors['bank_error_deg'],3.1)

    def test_turn_phase_names_separate_settling_and_steady_windows(self):
        self.assertEqual(turn_phase_name('settle',12,5,1),'turn_settle_12_5_1')
        self.assertEqual(turn_phase_name('steady',18,10,-1),'turn_steady_18_10_-1')
        with self.assertRaises(ValueError):
            turn_phase_name('all',12,5,1)

    def test_compass_turn_sign_convention(self):
        east,north,course=advance_fixed_curvature(0.,0.,0.,12.,-.01,.1)
        self.assertLess(course,0.)
        self.assertLess(east,0.)
        self.assertGreater(north,0.)

    def test_fixed_curvature_advance_and_coupled_schedule(self):
        east,north,course=advance_fixed_curvature(0.,0.,0.,12.,.01,.1)
        self.assertAlmostEqual(course,.012)
        self.assertAlmostEqual(math.hypot(east,north),2*math.sin(.006)/.01)
        segments=segment_schedule([12.,13.,12.],.3,settle_s=2.,hold_s=1.)
        self.assertEqual(scheduled_speed(segments,0.,.3),('settle',12.,0.))
        name,speed,accel=scheduled_speed(segments,3.,.3)
        self.assertEqual(name,'accel')
        self.assertGreater(speed,12.)
        self.assertGreater(accel,0.)

    def test_quintic_ramp_has_continuous_acceleration_and_correct_peak(self):
        for start,end in [(14.,18.),(18.,14.)]:
            duration=1.875*abs(end-start)/.15
            self.assertEqual(quintic_speed(start,end,0.,.15),(start,0.))
            self.assertEqual(quintic_speed(start,end,duration,.15),(end,0.))
            for t in np.linspace(.01,duration-.01,40):
                speed,accel=quintic_speed(start,end,t,.15)
                self.assertLessEqual(abs(accel),.15+1e-12)
                self.assertGreaterEqual(speed,min(start,end))
                self.assertLessEqual(speed,max(start,end))
                numeric=(quintic_speed(start,end,t+1e-4,.15)[0]-quintic_speed(start,end,t-1e-4,.15)[0])/2e-4
                self.assertAlmostEqual(numeric,accel,places=7)
        with self.assertRaises(ValueError):quintic_speed(14.,18.,1.,0.)

    def test_duplicate_samples_do_not_create_acceleration_spikes(self):
        rows=[]
        for t in np.repeat(np.arange(10)*.1,2):
            r=dict(sim_s=t,velocity_sim_s=t,ve=12.+.3*t,vn=0.,yaw=1.57,
                   roll=0.,pitch=0.,p=0.,throttle=50.,alt=60.,
                   heading_forward=12.+.3*t,heading_left=0.,forward=12.,left=0.)
            rows.append(r)
        metrics=telemetry_metrics(rows,'copter')
        self.assertAlmostEqual(metrics['tangential_accel']['maximum'],.3)
        self.assertEqual(metrics['tangential_accel']['upper_violation_fraction'],0.)
        self.assertIsNone(metrics['bank_rate_capability_bound'])

    def test_violations_are_duration_weighted_and_never_smoothed(self):
        result=distribution([0.,1.,0.],[0.,.2,1.],upper=.4)
        self.assertAlmostEqual(result['upper_violation_fraction'],.8)
        self.assertEqual(result['maximum'],1.)
        with self.assertRaises(ValueError): distribution([0.,1.,2.],[0.,2.,1.])

    def test_endpoint_violation_and_nonfinite_timestamps_are_not_lost(self):
        result=distribution([0.,0.,1.],[0.,.2,1.],upper=.4)
        self.assertEqual(result['upper_violation_fraction'],0.)
        self.assertEqual(result['upper_violation_samples'],1)
        for times in ([0.,1.,np.nan], [0.,1.]):
            with self.assertRaises(ValueError): distribution([0.,0.,0.],times)

    def test_copter_actual_speed_is_checked_not_just_acceleration(self):
        rows=[dict(sim_s=t,velocity_sim_s=t,ve=21.,vn=0.,yaw=0.,roll=0.,pitch=0.,p=0.,
            throttle=50.,alt=60.,heading_forward=21.,heading_left=0.,forward=21.,left=0.)
            for t in (0.,.1,.2)]
        metric=telemetry_metrics(rows,'copter')['groundspeed']
        self.assertEqual(metric['upper_bound'],20.)
        self.assertEqual(metric['upper_violation_fraction'],1.)

    def test_truth_diagnostics_preserve_unknown_data_and_peak_timestamps(self):
        self.assertIn('unavailable',acceleration_stats([],.4))
        metric=acceleration_stats([[0.,12.,0.],[1.,12.1,0.],[2.,12.6,0.]],.4)
        self.assertAlmostEqual(metric['maximum'],.5)
        self.assertEqual(metric['maximum_time_s'],2.)
        self.assertEqual(metric['upper_violation_samples'],1)


if __name__=='__main__':
    unittest.main()
