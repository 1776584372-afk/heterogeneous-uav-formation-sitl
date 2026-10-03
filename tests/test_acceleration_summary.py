import json
from pathlib import Path
import tempfile
import unittest

from scripts.summarize_acceleration_calibration import summarize_run


class AccelerationSummaryTests(unittest.TestCase):
    def test_uses_manifest_platform_and_does_not_certify(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'result.json').write_text(json.dumps({
                'protocol': {'controller': 'renamed backend', 'reference_ramp_m_s2': .02},
                'status': 'COMPLETED', 'all_tested_points_passed': True}))
            (root / 'manifest.json').write_text(json.dumps({'vehicle': 'plane', 'model': 'plane'}))
            (root / 'truth_phases.json').write_text(json.dumps({'phases': {'step_00_14': {
                'window': [0., 1.], 'simulator_truth': {'minimum': -.1, 'maximum': .1},
                'telemetry': {'minimum': -.09, 'maximum': .09}}}}))
            report = summarize_run(root)
            self.assertEqual(report['platform'], 'plane')
            self.assertEqual(report['frozen_comparison_limit_m_s2'], .2)
            self.assertEqual(report['status'], 'OBSERVATION_ONLY')
            self.assertEqual(report['phases'][0]['truth_max'], .1)
            (root / 'manifest.json').write_text(json.dumps({'vehicle': 'unknown', 'model': 'plane'}))
            with self.assertRaisesRegex(ValueError, 'unsupported vehicle'):
                summarize_run(root)

    def test_missing_truth_is_not_a_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'result.json').write_text(json.dumps({'protocol': {}}))
            (root / 'manifest.json').write_text(json.dumps({'vehicle': 'copter', 'model': 'x'}))
            report = summarize_run(root)
            self.assertEqual(report['status'], 'INCOMPLETE')


if __name__ == '__main__':
    unittest.main()
