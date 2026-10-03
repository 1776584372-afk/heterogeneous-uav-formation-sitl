import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from uav_formation.command_schedule import HeadingSchedule
from uav_formation.sitl_vehicle import SitlVehicle


class HeadingScheduleTests(unittest.TestCase):
    def test_unchanged_target_persists_and_latest_changed_target_is_sent(self):
        s=HeadingSchedule(.2)
        self.assertTrue(s.due(0.,359.,2.))
        self.assertFalse(s.due(.1,1.,2.))
        self.assertTrue(s.due(.2,2.,2.))
        self.assertFalse(s.due(100.,362.,2.))
        self.assertTrue(s.due(100.,2.,2.5))

    def test_invalid_values_and_clock_reversal(self):
        for value in (0.,-1.,math.nan):
            with self.assertRaises(ValueError): HeadingSchedule(value)
        s=HeadingSchedule(.2)
        s.due(2.,0.,2.)
        with self.assertRaises(ValueError): s.due(1.,0.,2.)
        with self.assertRaises(ValueError): s.due(3.,math.nan,2.)
        s.due(10.,0.,2.)
        with self.assertRaises(ValueError): s.due(9.,0.,2.)

    def test_scheduling_does_not_starve_speed_altitude_or_ignore_missing_ack(self):
        v=SitlVehicle.__new__(SitlVehicle)
        v.link=SimpleNamespace(mav=Mock());v.target={};v.altitude=60.
        v.plane_lateral_mode=None;v.command_first_sent={};v.command_acks={}
        v.heading_schedule=HeadingSchedule(.2);v.heading_pending_since=None
        v.messages={'ATTITUDE':SimpleNamespace(time_boot_ms=1000)}
        v.plane_reference(14.,90.,lateral_mode='course')
        self.assertEqual(v.link.mav.command_int_send.call_count,3)
        v.messages['ATTITUDE'].time_boot_ms=1050
        v.plane_reference(14.01,91.,lateral_mode='course')
        self.assertEqual(v.link.mav.command_int_send.call_count,5)
        v.heading_pending_since=time.monotonic()-3.
        with self.assertRaisesRegex(RuntimeError,'43002'): v.check_plane_commands()
        v.heading_pending_since=None
        v.command_acks[43002]=(v.command_first_sent[43002]+.01,0)
        v.check_plane_commands()
        v.command_acks[43002]=(time.monotonic(),2)
        with self.assertRaisesRegex(RuntimeError,'rejected'): v.check_plane_commands()


if __name__=='__main__': unittest.main()
