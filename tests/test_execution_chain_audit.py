import unittest

import numpy as np

from scripts.diagnose_execution_chain import (
    command_heading_derivative, geometry_phase_masks, measured_airspeed_metrics,
)


class ExecutionChainAuditTests(unittest.TestCase):
    def test_command_heading_uses_command_sample_clock_and_unwraps(self):
        times = np.arange(6)*.1
        headings = (np.pi-.02+.3*times+np.pi)%(2*np.pi)-np.pi
        rows = [dict(t=t, attitude_t=100.+.1*(i//2), command_yaw=yaw)
                for i, (t, yaw) in enumerate(zip(times, headings))]
        np.testing.assert_allclose(command_heading_derivative(rows), .3, atol=1e-12)

    def test_geometry_phases_are_exclusive_and_cover_boundary_samples(self):
        masks = geometry_phase_masks([0., 1e-10, .01, .01, 0.],
                                     [0., .001, .002, 0., 1e-8])
        np.testing.assert_array_equal(sum(mask.astype(int) for mask in masks.values()), 1)
        np.testing.assert_array_equal(masks['transition'], [False, True, True, False, False])

    def test_airspeed_derivative_does_not_use_groundspeed_or_fleet_clock(self):
        rows = [dict(t=i*.1, airspeed_t=100.+i, airspeed=14.+.1*i,
                     vx=25.+10*i, vy=0.) for i in range(5)]
        data, result = measured_airspeed_metrics(rows, 12., 24.)
        np.testing.assert_allclose(data['actual_airspeed_derivative'], .1, atol=1e-12)
        self.assertAlmostEqual(result['speed']['maximum'], 14.4)
        self.assertNotIn('upper_bound', result['derivative'])
        self.assertIn('no source time', result['clock'])

    def test_missing_airspeed_clock_is_unknown_not_fleet_time_fallback(self):
        rows = [dict(t=i, airspeed=14.) for i in range(3)]
        data, result = measured_airspeed_metrics(rows, 12., 24.)
        self.assertEqual(data, {})
        self.assertIn('unavailable', result)

    def test_held_airspeed_samples_do_not_generate_fake_spikes(self):
        rows = [dict(airspeed_t=t, airspeed=14.+.1*t)
                for t in [100., 100., 101., 101., 102., 102.]]
        data, result = measured_airspeed_metrics(rows, 12., 24.)
        np.testing.assert_allclose(data['actual_airspeed_derivative'], .1, atol=1e-12)
        self.assertAlmostEqual(result['derivative']['maximum'], .1)


if __name__ == '__main__':
    unittest.main()
