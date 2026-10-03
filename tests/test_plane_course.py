import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from uav_formation.capability_model import MemberSpec
from uav_formation.paper_tracking import course_command, PlaneCourseTracker
from uav_formation.fixedwing_adapter import encode_course_command
from uav_formation.sitl_vehicle import SitlVehicle


class PlaneCourseTests(unittest.TestCase):
    def setUp(self):
        self.member=MemberSpec(0,'fixedwing',0,12,24,2.5,bank_max=math.radians(27),a_t_min=-.2,a_t_max=.2)

    def test_course_left_right_and_cross_track_sign(self):
        for sign in (-1,1):
            c=course_command(self.member,(0,0,0,14,sign/160,60),(0,0))
            self.assertLess(sign*(c.bearing_deg-90),0)
            # Chord direction bisects the constant-curvature preview arc.
            self.assertAlmostEqual(c.bearing_deg,90-math.degrees(sign*40/320))
        c=course_command(self.member,(0,0,0,14,0,60),(0,10))
        self.assertGreater(c.bearing_deg,90)
        c=course_command(self.member,(0,0,math.pi/2,14,0,60),(0,0))
        self.assertAlmostEqual(c.bearing_deg,0)

    def test_along_track_correction_slew_and_reference_unchanged(self):
        tracker=PlaneCourseTracker(self.member)
        ref=(0,0,0,14,0,60)
        c=tracker.update(ref,(-100,0),14,.5)
        self.assertAlmostEqual(c.airspeed,14.1)
        self.assertTrue(c.speed_saturated)
        c=tracker.update(ref,(100,0),14,.5)
        self.assertAlmostEqual(c.airspeed,14)
        self.assertEqual(ref,(0,0,0,14,0,60))
        self.assertEqual(c.bearing_deg,90)  # overshoot cannot cause a U-turn

    def test_command_accel_scale_only_changes_external_slew(self):
        tracker=PlaneCourseTracker(self.member,command_accel_scale=.25)
        command=tracker.update((0,0,0,16,0,60),(-100,0),14,.5)
        self.assertAlmostEqual(command.airspeed,14.025)
        self.assertAlmostEqual(command.requested_airspeed,16+30.)

    def test_handover_preserves_sent_speed_and_default_keeps_measured_init(self):
        reference=(0,0,0,14,0,60)
        legacy=PlaneCourseTracker(self.member)
        self.assertAlmostEqual(legacy.update(reference,(0,0),13.8,.05).airspeed,13.81)
        tracker=PlaneCourseTracker(self.member,initial_command_airspeed=14.)
        self.assertEqual(tracker.update(reference,(0,0),13.8,.05).airspeed,14.)
        for target,measured in [(16.,12.),(12.,24.),(16.,14.)]:
            previous=tracker.previous_speed
            command=tracker.update((0,0,0,target,0,60),(0,0),measured,.05)
            self.assertLessEqual(abs(command.airspeed-previous),.2*.05+1e-12)
        for value in (0.,11.9,24.1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                PlaneCourseTracker(self.member,initial_command_airspeed=value)

    def test_real_path_preview_and_protocol_fields(self):
        c=course_command(self.member,(0,0,0,14,0,60,40,10),(0,0))
        commands=encode_course_command(c)
        self.assertLess(c.bearing_deg,90)
        self.assertEqual([x.command for x in commands],[43000,186,43002])
        self.assertEqual(commands[0].param1,0)
        self.assertEqual(commands[0].param3,0)
        self.assertEqual(commands[1].param1,60)
        self.assertEqual(commands[1].param2,3)
        self.assertEqual(commands[2].param3,2.5)

    def test_frenet_guidance_has_no_chord_bias_and_corrects_cross_track(self):
        reference=(0,0,0,14,1/160,60,40,10)
        on_path=course_command(self.member,reference,(0,0),guidance_mode='frenet')
        self.assertAlmostEqual(on_path.bearing_deg,90.)
        left=course_command(self.member,reference,(0,10),guidance_mode='frenet')
        right=course_command(self.member,reference,(0,-10),guidance_mode='frenet')
        self.assertGreater(left.bearing_deg,90.)
        self.assertLess(right.bearing_deg,90.)
        stronger=course_command(self.member,reference,(0,10),guidance_mode='frenet',
                                cross_track_gain=2.)
        self.assertGreater(stronger.bearing_deg,left.bearing_deg)

    def test_curvature_feedforward_maps_required_bank_to_course_lead(self):
        curvature=1/160
        reference=(0,0,0,14,curvature,60)
        gain=math.radians(50.)
        command=course_command(self.member,reference,(0,0),guidance_mode='curvature_ff',
                               course_to_bank_gain=gain)
        expected_lead=math.atan2(14**2*curvature,9.80665)/gain
        self.assertAlmostEqual(command.bearing_deg,90-math.degrees(expected_lead))
        opposite=course_command(self.member,(0,0,0,14,-curvature,60),(0,0),
                                guidance_mode='curvature_ff',course_to_bank_gain=gain)
        self.assertAlmostEqual(opposite.bearing_deg,90+math.degrees(expected_lead))

    def test_guidance_configuration_is_validated(self):
        for mode in ('bad','pure_pursuit'):
            with self.assertRaises(ValueError):
                PlaneCourseTracker(self.member,guidance_mode=mode)
        for gain in (0.,-1.,float('nan')):
            with self.assertRaises(ValueError):
                PlaneCourseTracker(self.member,cross_track_gain=gain)
            with self.assertRaises(ValueError):
                PlaneCourseTracker(self.member,course_to_bank_gain=gain)

    def test_invalid_reference_and_timing_rejected(self):
        tracker=PlaneCourseTracker(self.member)
        for ref in [(0,0,0,10,0,60),(0,0,0,14,.1,60),(0,0,float('nan'),14,0,60)]:
            with self.assertRaises(ValueError): tracker.update(ref,(0,0),14,.1)
        for dt in (0,-.1,2,float('nan')):
            with self.assertRaises(ValueError): tracker.update((0,0,0,14,0,60),(0,0),14,dt)

    def test_course_backend_does_not_send_roll_and_forbids_hot_switch(self):
        v=SitlVehicle.__new__(SitlVehicle)
        v.link=SimpleNamespace(mav=Mock()); v.target={};v.altitude=60
        v.plane_lateral_mode=None;v.command_first_sent={};v.command_acks={}
        v.plane_reference(14,90,2.5,lateral_mode='course')
        self.assertEqual(v.link.mav.command_int_send.call_count,3)
        v.link.mav.set_attitude_target_send.assert_not_called()
        with self.assertRaises(ValueError): v.plane_reference(14,90,bank=.1,lateral_mode='external_roll')
        self.assertEqual(v.link.mav.command_int_send.call_count,3)
        v.command_acks[43000]=(v.command_first_sent[43000]+.01,2)
        with self.assertRaisesRegex(RuntimeError,'rejected'): v.check_plane_commands()
        v.command_acks={};v.command_first_sent={43000:0.}
        with self.assertRaisesRegex(RuntimeError,'timeout'): v.check_plane_commands()

    def test_optional_copter_yaw_rate_keeps_default_interface(self):
        v=SitlVehicle.__new__(SitlVehicle)
        v.link=SimpleNamespace(mav=Mock());v.target={};v.altitude=60;v.home_offset=(0,0)
        v.copter_reference(1,2,12,0,accel=(0,0))
        self.assertEqual(v.link.mav.set_position_target_local_ned_send.call_args.args[4],2048)
        v.copter_reference(1,2,12,0,accel=(0,0),yaw_rate=-.2)
        values=v.link.mav.set_position_target_local_ned_send.call_args.args
        self.assertEqual(values[4],0)
        self.assertEqual(values[-1],-.2)
