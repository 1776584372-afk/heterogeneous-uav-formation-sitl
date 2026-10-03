import math
import unittest

from uav_formation.capability_model import MemberSpec
from uav_formation.paper_tracking import plane_command, advance_unicycle, preview_member, member_reference
from uav_formation.icc_traj import IccRoute
from uav_formation.progress_profile import plan_progress


class PlaneTrackingTests(unittest.TestCase):
    def test_flythrough_prediction_continues_past_endpoint(self):
        route=IccRoute([(0.,0.),(100.,0.)],20.,1.,strict_geometry=True)
        m=MemberSpec(0,'multirotor',0.,0.,20.,3.,a_t_min=-.4,a_t_max=.4)
        p=plan_progress(route,[m],14.,ds=4.,start_speed=14.,end_speed=14.)
        point,speed=preview_member(route,p,99.,5.,.5,True)
        self.assertAlmostEqual(point[0],106.,places=6)
        self.assertAlmostEqual(point[1],5.)
        self.assertAlmostEqual(speed,14.)
        stopped,_=preview_member(route,p,99.,5.,.5,False)
        self.assertAlmostEqual(stopped[0],100.)

    def test_flythrough_send_matches_preview_across_last_tick(self):
        route=IccRoute([(0.,0.),(100.,0.)],20.,1.,strict_geometry=True)
        m=MemberSpec(0,'multirotor',5.,0.,20.,3.,a_t_min=-.4,a_t_max=.4)
        p=plan_progress(route,[m],14.,ds=4.,start_speed=14.,end_speed=14.)
        for dt in (.05,.1,.3):
            overshoot=p.time_at(99.9)+dt-p.time[-1]
            actual=member_reference(route,p,100.,5.,overshoot)
            predicted,speed=preview_member(route,p,99.9,5.,dt,True)
            for value,expected in zip(actual[:3],predicted):
                self.assertAlmostEqual(value,expected,places=7)
            self.assertAlmostEqual(actual[0]-99.9,14.*dt,places=6)
            self.assertAlmostEqual(actual[3],speed)
            self.assertEqual(actual[4:],(0.,0.))
        with self.assertRaises(ValueError):
            member_reference(route,p,99.,5.,.1)
        with self.assertRaises(ValueError):
            member_reference(route,p,100.,5.,float('nan'))

    def test_held_input_propagation_matches_elapsed_time(self):
        state=(0.,0.,0.)
        for dt in (.05,.1,.05,.1,.05):
            state=advance_unicycle(state,14.,0.,dt)
        self.assertAlmostEqual(state[0],14.*.35)
        self.assertEqual(state[1],0.)
        circle=advance_unicycle((0.,0.,0.),10.,.2,math.pi/.2)
        self.assertAlmostEqual(circle[0],0.)
        self.assertAlmostEqual(circle[1],100.)

    def setUp(self):
        self.member=MemberSpec(0,'fixedwing',0,12,24,3,bank_max=math.radians(30))

    def test_left_curve_commands_negative_ned_roll(self):
        speed,course,bank=plane_command(self.member,(0,0,0,14,.01),(0,0,0,14,14))
        self.assertLess(bank,0)
        self.assertEqual(course,90)
        self.assertEqual(speed,14)

    def test_north_error_on_east_path_commands_right_bank(self):
        _,_,bank=plane_command(self.member,(0,0,0,14,0),(0,10,0,14,14))
        self.assertGreater(bank,0)

    def test_along_track_feedback_is_command_only_and_bounded(self):
        ref=(0,0,0,14,0)
        speed,_,_=plane_command(self.member,ref,(-100,0,0,14,14))
        self.assertEqual(speed,24)
        self.assertEqual(ref[3],14)

    def test_invalid_telemetry_rejected(self):
        with self.assertRaises(ValueError):
            plane_command(self.member,(0,0,0,14,0),(0,0,0,float('nan'),14))
