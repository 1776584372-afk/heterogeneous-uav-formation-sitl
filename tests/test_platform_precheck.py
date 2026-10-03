import csv
import json
import math
from pathlib import Path
import tempfile
import unittest

from scripts.platform_precheck import analyze


class PreviewTests(unittest.TestCase):
    def evaluate(self, change=None):
        protocol = dict(nominal_tolerance_m_s=.02, max_gap_s=.3, settle_s=1., window_s=2.,
                        min_window_s=1.8, altitude_m=10., speed_uncertainty_m_s=.3,
                        max_speed_error_m_s=.5, max_cross_track_m=1.,
                        max_planar_tracking_error_m=8., max_altitude_error_m=1.,
                        max_yaw_error_deg=5., max_abs_roll_deg=15.)
        rows = [dict(t=j*.1, phase='track', ref_speed=2., vx=0., vy=2., yaw=math.pi/2,
                     ref_yaw=math.pi/2, x=0., y=j*.2, ref_x=0., ref_y=j*.2,
                     z=10., roll=0., mode='GUIDED', armed='True', pose_age=.1,
                     velocity_age=.1, state_age=.1) for j in range(40)]
        if change:
            change(rows)
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run/'flight_status.json').write_text(json.dumps(dict(completed=True, landed=True)))
            with (run/'telemetry.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            return analyze(run, 2., protocol)

    def test_forward_speed_is_in_body_frame(self):
        result = self.evaluate()
        self.assertEqual(result['status'], 'PASS')
        self.assertAlmostEqual(result['mean_forward_m_s'], 2.)
        self.assertAlmostEqual(result['max_abs_left_m_s'], 0.)

    def test_disconnected_windows_do_not_pass(self):
        def change(rows):
            for j in (9, 18, 27, 36):
                rows[j]['ref_speed'] = 0.
        self.assertEqual(self.evaluate(change)['status'], 'FAIL')

    def test_tracking_failure_cannot_be_reported_as_speed_success(self):
        def change(rows):
            for row in rows:
                row['x'] = 9.
        self.assertEqual(self.evaluate(change)['status'], 'FAIL')

    def test_nonfinite_velocity_rejected(self):
        self.assertEqual(self.evaluate(lambda rows: rows[20].update(vx=float('nan')))['status'], 'FAIL')
