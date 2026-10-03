import unittest

from uav_formation.capability_model import MemberSpec
from uav_formation.icc_traj import IccRoute
from uav_formation.progress_profile import plan_progress


class QuadraticObjectiveTests(unittest.TestCase):
    def test_three_node_analytic_optimum(self):
        route=IccRoute([(0.,0.),(2.,0.)],20.,1.,strict_geometry=True)
        member=MemberSpec(0,'multirotor',0.,0.,6.,100.,a_t_min=-100.,a_t_max=100.)
        profile=plan_progress(route,[member],2.,ds=1.,start_speed=0.,end_speed=0.,smooth_weight=1.)
        self.assertEqual(len(profile.z),3)
        # (z-4)^2 + (z-0)^2 + (0-z)^2 has minimum z=4/3.
        self.assertAlmostEqual(profile.z[1],4./3.,places=6)

    def test_long_route_acceleration_limited_qp_converges(self):
        route=IccRoute([(0.,0.),(800.,0.),(800.,800.),(1600.,800.)],80.,1.,trans_len=80.,strict_geometry=True)
        members=[MemberSpec(i,'multirotor',float((i-1)*10),0.,20.,3.,
                           yaw_rate_max=.25,a_t_min=-.4,a_t_max=.4) for i in range(3)]
        profile=plan_progress(route,members,14.,ds=4.,start_speed=14.,end_speed=14.)
        self.assertLess(min(profile.nu),13.)
        self.assertTrue(all(min(margins.values())>=-2e-6 for margins in profile.margins.values()))


if __name__=='__main__':
    unittest.main()
