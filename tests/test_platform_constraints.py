#!/usr/bin/env python3
import math
import unittest

from uav_formation.formation_reference import build_feasible_route
from uav_formation.icc_traj import IccRoute
from uav_formation.platform_constraints import (
    check_formation_feasibility,
    check_public_progress_feasibility,
    fixedwing_capability,
    multirotor_capability,
    slot_reference,
)


class PlatformConstraintTests(unittest.TestCase):
    def test_straight_lateral_slot_preserves_speed_and_course(self):
        route = IccRoute([(0.0, 0.0), (100.0, 0.0)], 20.0, 10.0)
        ref = slot_reference(route, 20.0, (0.0, 5.0, 0.0))
        self.assertAlmostEqual(ref.speed, 10.0, places=6)
        self.assertAlmostEqual(ref.course, 0.0, places=6)
        self.assertAlmostEqual(ref.curvature, 0.0, places=6)

    def test_fixedwing_bank_limit_rejects_tight_turn(self):
        route = IccRoute([(0.0, 0.0), (30.0, 0.0), (30.0, 30.0)],
                         r_center=5.0, v_cruise=20.0, trans_len=0.0)
        cap = fixedwing_capability(10.0, 25.0, max_bank_deg=35.0)
        report = check_formation_feasibility(route, [(0.0, 0.0, 0.0)], [cap])
        self.assertFalse(report.feasible)
        self.assertTrue(any(v.constraint == "max_bank" for v in report.violations))

    def test_route_planner_increases_radius_for_fixedwing(self):
        cap = fixedwing_capability(10.0, 25.0, max_bank_deg=35.0)
        plan = build_feasible_route(
            [(0.0, 0.0), (80.0, 0.0), (80.0, 80.0)],
            [(0.0, 0.0, 0.0)], [cap],
            center_speed=20.0, center_radius=5.0,
            turn_margin=1.0, trans_len=0.0,
        )
        self.assertTrue(plan.report.feasible)
        self.assertGreater(plan.center_radius, 5.0)
        self.assertLessEqual(plan.report.max_bank_deg, 35.0 + 1e-6)

    def test_public_progress_intersection_exposes_fixedwing_bottleneck(self):
        route = IccRoute([(0.0, 0.0), (30.0, 0.0), (30.0, 30.0)],
                         r_center=5.0, v_cruise=1.0, trans_len=0.0)
        caps = [fixedwing_capability(12.0, 25.0, max_bank_deg=35.0)]
        report = check_public_progress_feasibility(
            route, [(0.0, 0.0, 0.0)], caps)
        self.assertFalse(report.feasible)
        bad = next(x for x in report.slices if not x.feasible)
        self.assertGreater(bad.lower, bad.upper)

        wide = IccRoute([(0.0, 0.0), (30.0, 0.0), (30.0, 30.0)],
                        r_center=80.0, v_cruise=1.0, trans_len=0.0)
        report = check_public_progress_feasibility(
            wide, [(0.0, 0.0, 0.0)], caps)
        self.assertTrue(report.feasible)
        self.assertTrue(all(x.lower <= x.upper for x in report.slices))

    def test_multirotor_profile_accepts_existing_scale(self):
        route = IccRoute([(0.0, 0.0), (50.0, 0.0), (50.0, 50.0)],
                         r_center=15.0, v_cruise=1.5)
        caps = [multirotor_capability(30.0, 5.0)] * 3
        slots = [(0.0, -10.0, 0.0), (0.0, 0.0, 0.0), (0.0, 10.0, 0.0)]
        report = check_formation_feasibility(route, slots, caps)
        self.assertTrue(report.feasible, report.summary)
        self.assertGreaterEqual(report.min_speed, 0.0)


if __name__ == "__main__":
    unittest.main()
