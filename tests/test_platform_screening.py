import json
from pathlib import Path
import tempfile
import unittest

from scripts.screen_platform_candidates import candidates, jobs
from scripts.summarize_platform_screening import bounds_pass, summarize, expected_points, REQUIRED_BOUNDS
from scripts.analyze_backend_dynamics import distribution


class ScreeningTests(unittest.TestCase):
    def test_candidate_parameters_are_matched_and_failure_is_not_a_pass(self):
        for model,defaults in candidates().values():
            self.assertTrue(model)
            self.assertTrue(defaults)
        self.assertNotIn('freestyle',candidates()['standard_x'][1])
        self.assertIn('Callisto.parm',candidates()['callisto'][1])
        self.assertFalse(bounds_pass({}))
        self.assertFalse(bounds_pass({'unavailable':'no data'}))
        self.assertFalse(bounds_pass({'yaw':{'upper_violation_fraction':.001}}))
        self.assertFalse(bounds_pass({'yaw':{'upper_violation_fraction':0.,'lower_violation_fraction':0.}}))
        self.assertTrue(bounds_pass({'yaw':distribution([0.,0.,0.],[0.,.1,.2],-.25,.25)}))

    def test_missing_metrics_and_endpoint_peaks_fail_closed(self):
        self.assertFalse(bounds_pass({'yaw':distribution([0.,0.,.3],[0.,.1,.2],-.25,.25)}))
        metrics={name:distribution([max(0.,lower)]*3,[0.,.1,.2],lower,upper)
                 for name,(lower,upper) in REQUIRED_BOUNDS['copter'].items()}
        self.assertTrue(bounds_pass(metrics,'copter'))
        del metrics['groundspeed']
        self.assertFalse(bounds_pass(metrics,'copter'))
        self.assertEqual(expected_points({'speeds':[12.,14.]}),2)
        self.assertEqual(expected_points({'speeds':[12.], 'banks':[7.,12.], 'turn_signs':[1,-1]}),4)

    def test_heading_comparison_changes_only_schedule(self):
        variants=jobs('plane_heading')
        self.assertEqual(variants[0][1][:-1],variants[1][1][:-1])
        self.assertEqual([v[1][-1] for v in variants],['0.0','0.2'])

    def test_handover_comparison_preserves_model_guidance_and_bounds(self):
        original=jobs('plane_preview')[0][1]
        handover=jobs('plane_handover')[0][1]
        self.assertEqual(handover[:-1],original)
        self.assertEqual(handover[-1],'--handover-command-speed')

    def test_followups_preserve_model_noise_and_bounds(self):
        name,command=jobs('rotor_reference')[0]
        self.assertEqual(name,'freestyle_quintic')
        self.assertEqual(command[command.index('--model')+1],candidates()['freestyle'][0])
        self.assertEqual(command[command.index('--ramp-shape')+1],'quintic')
        self.assertEqual(command[command.index('--speeds')+1],'12,14,12')
        combined=dict(jobs('plane_speed_config'))['speed_combined']
        path=Path(combined[combined.index('--defaults')+1].split(',')[-1])
        params={parts[0]:float(parts[1]) for line in path.read_text().splitlines()
                if (parts:=line.split()) and not line.startswith('#')}
        self.assertEqual(params,{'TECS_SPDWEIGHT':0.,'TECS_SPD_OMEGA':.5})
        with self.assertRaises(ValueError): jobs('not_a_stage')

    def test_low_rate_followups_are_explicit_and_do_not_change_frozen_bounds(self):
        rotor=dict(jobs('rotor_low_rate'))['freestyle_quintic_low_rate']
        plane=dict(jobs('plane_low_rate'))['plane_low_rate']
        self.assertEqual(rotor[rotor.index('--ramp-rate')+1],'.05')
        self.assertEqual(rotor[rotor.index('--ramp-shape')+1],'quintic')
        self.assertEqual(plane[plane.index('--ramp-rate')+1],'.02')
        self.assertNotIn('--defaults',plane)

    def test_failed_attempt_and_unknown_dynamics_are_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);batch=root/'batch';batch.mkdir()
            data=dict(stage='rotor_straight',frozen_unchanged=True,jobs=[dict(
                name='bad',returncode=1,result=dict(steps=[dict(status='FAIL')],exit_completed=True))])
            (batch/'batch.json').write_text(json.dumps(data))
            report=summarize(root)
            self.assertEqual(len(report['attempts']),1)
            self.assertFalse(report['attempts'][0]['screening_pass'])
            self.assertFalse(report['envelope_certified'])
            self.assertTrue(report['old_frozen_limits_unchanged'])
            data['frozen_unchanged']=False
            (batch/'batch.json').write_text(json.dumps(data))
            self.assertFalse(summarize(root)['old_frozen_limits_unchanged'])


if __name__=='__main__':unittest.main()
