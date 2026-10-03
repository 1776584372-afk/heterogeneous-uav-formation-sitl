import math
import unittest

from uav_formation.paper_tracking import synchronous_copter_target, CopterExecutionGovernor, CopterVelocityFilter
from uav_formation.capability_model import MemberSpec


class ExecutionTimingTests(unittest.TestCase):
    def test_velocity_filter_limits_speed_change_and_turn_with_consistent_acceleration(self):
        member=MemberSpec(0,'multirotor',0.,0.,20.,3.,yaw_rate_max=.25,a_t_min=-.4,a_t_max=.4)
        filt=CopterVelocityFilter(member,(14.,0.),.35)
        previous=(14.,0.)
        for _ in range(100):
            velocity,accel=filt.update((0.,14.),.05)
            self.assertLessEqual(abs(math.hypot(*velocity)-math.hypot(*previous))/.05,.2+1e-10)
            da=math.atan2(velocity[1],velocity[0])-math.atan2(previous[1],previous[0])
            self.assertLessEqual(abs(da)/.05*max(math.hypot(*previous),math.hypot(*velocity)),2.4+1e-10)
            for j in range(2):self.assertAlmostEqual((velocity[j]-previous[j])/.05,accel[j])
            previous=velocity
        with self.assertRaises(ValueError):filt.update((1.,2.),1.)

    def test_command_governor_bounds_and_continuity_without_changing_measurement(self):
        member=MemberSpec(0,'multirotor',0.,0.,20.,3.,yaw_rate_max=.25,a_t_min=-.4,a_t_max=.4)
        initial=(1.,2.,math.pi-.01)
        governor=CopterExecutionGovernor(member,initial,14.)
        old_speed=14.;old_yaw=initial[2]
        for index in range(200):
            output=governor.update(18. if index<100 else 10.,1.2 if index<100 else -1.2,.05)
            self.assertLessEqual(abs(output['speed']-old_speed)/.05,.2+1e-10)
            self.assertLessEqual(abs(output['yaw_rate']),.125+1e-10)
            self.assertLessEqual(abs(output['yaw_rate']*output['speed']),1.5+1e-10)
            self.assertLessEqual(abs(output['state'][2]-old_yaw),.125*.05+1e-10)
            old_speed=output['speed'];old_yaw=output['state'][2]
        self.assertEqual(initial,(1.,2.,math.pi-.01))
        for dt in (0.,.6,float('nan')):
            with self.assertRaises(ValueError):governor.update(14.,0.,dt)

    def test_initial_position_is_measurement_not_future_preview(self):
        state=(1.,2.,math.pi-.02)
        target,velocity=synchronous_copter_target(state,14.,.2,0.)
        self.assertEqual(target,state)
        self.assertAlmostEqual(math.hypot(*velocity),14.)

    def test_position_derivative_matches_velocity_across_yaw_boundary(self):
        state=(1.,2.,math.pi-.02)
        target,velocity=synchronous_copter_target(state,14.,.2,.3)
        following,_=synchronous_copter_target(state,14.,.2,.300001)
        for axis in (0,1):
            self.assertAlmostEqual((following[axis]-target[axis])/1e-6,velocity[axis],places=5)
        self.assertGreater(target[2],math.pi)
        for elapsed in (-1.,1.01,float('nan')):
            with self.assertRaises(ValueError): synchronous_copter_target(state,14.,.2,elapsed)
