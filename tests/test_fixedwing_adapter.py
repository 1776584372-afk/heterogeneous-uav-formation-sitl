import math
import struct
import unittest
from types import SimpleNamespace
from uav_formation.capability_model import MemberSpec
from uav_formation.fixedwing_adapter import encode_reference,decode_command_ack


class AdapterTests(unittest.TestCase):
    def setUp(self): self.member=MemberSpec(0,'fixedwing',0,12,25,6,bank_max=.6)

    def test_ned_bearing_conversion_and_airspeed_type(self):
        commands=encode_reference(self.member,(0,0,0,15,0,60),(0,0))
        self.assertEqual(commands[0].command,43000)
        self.assertEqual(commands[0].param1,0)
        self.assertEqual(commands[0].param2,15)
        self.assertEqual(commands[2].param2,90)
        self.assertEqual(commands[1].command,186)
        self.assertEqual(commands[1].param1,60)
        self.assertEqual(commands[1].param2,3)
        north=encode_reference(self.member,(0,0,math.pi/2,15,0,60),(0,0))
        self.assertAlmostEqual(north[2].param2,0)

    def test_cross_track_and_curvature_change_heading(self):
        left=encode_reference(self.member,(0,0,0,15,0,60),(0,10))[-1]
        self.assertGreater(left.param2,90)
        turn=encode_reference(self.member,(0,0,0,15,.01,60),(0,0))[-1]
        self.assertLess(turn.param2,90)
        with self.assertRaises(ValueError): encode_reference(self.member,(0,0,0,2,0,60),(0,0))

    def test_ack_payload(self):
        raw=struct.pack('<HB',43000,0).ljust(8,b'\0')
        msg=SimpleNamespace(msgid=77,payload64=[struct.unpack('<Q',raw)[0]])
        self.assertEqual(decode_command_ack(msg),(43000,0))
