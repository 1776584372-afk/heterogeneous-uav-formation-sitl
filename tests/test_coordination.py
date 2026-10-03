import math
import unittest
from dataclasses import replace
import numpy as np
from uav_formation.capability_model import (MemberSpec, Limit, slot_scale, static_progress_interval,
                                           team_interval, member_demands, dynamic_margin)
from uav_formation.icc_traj import IccRoute
from uav_formation.progress_profile import plan_progress, validate_profile, PlanningError, ProgressProfile
from uav_formation.reference_coordinator import ReferenceCoordinator


def rotor(i=0, ry=0):
    return MemberSpec(i, 'multirotor', ry, 0., 10., 3., yaw_rate_max=.8, a_t_min=-1., a_t_max=1.)


class CoordinationTests(unittest.TestCase):
    def test_straight_and_identity(self):
        for k in (0., .05, -.05):
            for ry in (-5., 0., 5.):
                m = rotor(0, ry)
                v, ki, omega, _ = member_demands(m, k, 3.)
                self.assertAlmostEqual(v*ki, 3*k)
                self.assertAlmostEqual(omega, 3*k)
                if k == 0: self.assertEqual(v, 3.)

    def test_turn_slot_symmetry_and_active_owner(self):
        members = [rotor(0,-5), rotor(1,5)]
        self.assertEqual(team_interval(members,.1)[3][0], 0)
        self.assertEqual(team_interval(members,-.1)[3][0], 1)
        self.assertEqual(team_interval(members,.1)[:2], team_interval(members,-.1)[:2])

    def test_reject_geometry_and_empty_intersection(self):
        with self.assertRaises(ValueError): slot_scale(10, .1)
        with self.assertRaises(ValueError): slot_scale(11, .1)
        wing = MemberSpec(1,'fixedwing',5,12,25,6,bank_max=.6)
        lower, upper, lo, hi = team_interval([rotor(), wing],.05)
        self.assertGreater(lower,upper)
        self.assertEqual(lo[0],1)
        self.assertEqual(hi[0],0)

    def test_yaw_and_correct_outer_acceleration_bound(self):
        m = replace(rotor(0,-10), a_lat_max=3., yaw_rate_max=None)
        _, upper, active = static_progress_interval(m,.1)
        self.assertAlmostEqual(upper,math.sqrt(3/.2))
        self.assertEqual(active[1],Limit.LATERAL_ACCEL)
        _, upper, active = static_progress_interval(replace(m,yaw_rate_max=.1),.1)
        self.assertAlmostEqual(upper,1.)
        self.assertEqual(active[1],Limit.HEADING_RATE)

    def test_derivative_and_geometric_rejections(self):
        r = IccRoute([(0,0),(80,0),(80,80)],20,1,trans_len=8,strict_geometry=True)
        for s in np.linspace(0,r.total_length(),100):
            if 1e-4 < s < r.total_length()-1e-4:
                numeric=(r.curvature_at(s+1e-5)-r.curvature_at(s-1e-5))/2e-5
                self.assertAlmostEqual(numeric,r.curvature_derivative_at(s),places=6)
        with self.assertRaises(ValueError): IccRoute([(0,0),(20,0),(20,20)],50,1,strict_geometry=True)
        with self.assertRaises(ValueError): IccRoute([(0,0),(80,0),(80,80)],10,1,trans_len=20,strict_geometry=True)

    def test_dynamic_profile_accelerates_from_rest_and_stops(self):
        r = IccRoute([(0,0),(30,0)],10,1,strict_geometry=True)
        p = plan_progress(r,[rotor()],8.,ds=1.,start_speed=0,end_speed=0)
        self.assertGreater(p.advance(0,.1),0)
        self.assertEqual(p.nu_at(0),0)
        self.assertEqual(p.nu_at(r.total_length()),0)
        self.assertLessEqual(max(p.nu),math.sqrt(30)+1e-6)
        self.assertAlmostEqual(p.advance(0,p.time[-1]),30)
        validate_profile(r,[rotor()],p)

    def test_pointwise_feasible_but_dynamically_invalid(self):
        r = IccRoute([(0,0),(10,0)],10,1,strict_geometry=True)
        ss=np.array([0.,1.,10.])
        bad=ProgressProfile(ss,np.array([0.,64.,64.]),np.zeros(3),np.ones(3)*10,
                            np.zeros((3,2)),np.zeros((3,2)),{})
        with self.assertRaises(PlanningError): validate_profile(r,[rotor()],bad)
        p=plan_progress(r,[rotor()],8,ds=1,start_speed=0)
        self.assertLess(p.nu_at(1),2.)

    def test_full_search_deterministic_and_nonmonotonic(self):
        wing=MemberSpec(0,'fixedwing',0,12,20,6,bank_max=.6,bank_rate_max=.5)
        c=ReferenceCoordinator([wing],[10,40,100],10,14,ds=2,trans_len=40)
        a=c.plan([(0,0),(90,0),(90,90)])
        b=c.plan([(0,0),(90,0),(90,90)])
        self.assertTrue(a.feasible, a.candidates)
        self.assertEqual(len(a.candidates),3)
        self.assertNotEqual(a.candidates[0].status,'FEASIBLE')
        self.assertEqual(a.candidates[1].status,'FEASIBLE')
        self.assertEqual(a.candidates[2].status,'GEOMETRY_REJECTED')
        self.assertEqual(a.selected_radius,b.selected_radius)
        np.testing.assert_array_equal(a.profile.z,b.profile.z)

    def test_fixedwing_cannot_stop(self):
        wing=MemberSpec(0,'fixedwing',0,12,20,6,bank_max=.6)
        r=IccRoute([(0,0),(100,0)],40,1,strict_geometry=True)
        with self.assertRaises(PlanningError): plan_progress(r,[wing],14,end_speed=0)

    def test_roll_derivative_matches_time_difference(self):
        wing=MemberSpec(0,'fixedwing',5,12,25,6,bank_max=.6,bank_rate_max=.5)
        k,kp,z,zp=.01,.002,225.,2.
        margin=dynamic_margin(wing,k,kp,z,zp)[Limit.BANK_RATE]
        h=1e-5
        phis=[member_demands(wing,k+kp*d,math.sqrt(z+zp*d))[3] for d in (-h,h)]
        self.assertAlmostEqual(.5-margin,abs((phis[1]-phis[0])/(2*h)*math.sqrt(z)),places=7)


if __name__ == '__main__': unittest.main()
