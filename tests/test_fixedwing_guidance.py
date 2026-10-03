#!/usr/bin/env python3
import math
import unittest

from uav_formation.fixedwing_guidance import FixedWingGuidance, FixedWingState
from uav_formation.icc_traj import IccRoute
from uav_formation.platform_constraints import fixedwing_capability, slot_reference


class FixedWingGuidanceTests(unittest.TestCase):
    def test_cross_track_error_commands_back_towards_path(self):
        route = IccRoute([(0.0, 0.0), (100.0, 0.0)], 20.0, 15.0)
        reference = slot_reference(route, 20.0, (0.0, 0.0, 0.0))
        guidance = FixedWingGuidance(
            fixedwing_capability(10.0, 25.0, max_bank_deg=35.0))
        state = FixedWingState(20.0, 10.0, 50.0, 0.0, 15.0)
        command = guidance.update(reference, state, target_altitude=60.0)
        self.assertGreater(command.cross_track_error, 0.0)
        self.assertLess(command.target_bank, 0.0)
        self.assertAlmostEqual(command.target_altitude, 60.0)

    def test_command_respects_airspeed_and_bank_limits(self):
        route = IccRoute([(0.0, 0.0), (30.0, 0.0), (30.0, 30.0)],
                         r_center=5.0, v_cruise=30.0, trans_len=0.0)
        reference = slot_reference(route, 35.0, (0.0, 0.0, 0.0))
        cap = fixedwing_capability(12.0, 20.0, max_bank_deg=30.0)
        command = FixedWingGuidance(cap).update(
            reference, FixedWingState(reference.x, reference.y, 40.0,
                                      reference.course, 15.0))
        self.assertGreaterEqual(command.target_airspeed, 12.0)
        self.assertLessEqual(command.target_airspeed, 20.0)
        self.assertLessEqual(abs(command.target_bank), math.radians(30.0) + 1e-9)


if __name__ == "__main__":
    unittest.main()
