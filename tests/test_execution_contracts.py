"""Run with scripts/experiment_env.sh to include real ROS message libraries."""
import os
os.environ.setdefault('ROS_DOMAIN_ID','86')
os.environ.setdefault('ROS_LOCALHOST_ONLY','1')
import math
from pathlib import Path
import unittest
import numpy as np
try:
    import rclpy
    from uav_formation.egompc_formation import EgoMpcFormation, DroneMPC, N
    from uav_formation.icc_traj import IccRoute, adaptive_turn_radius, corner_factor_by_spread
    import quadrotor_msgs.msg
    ROS_AVAILABLE=True
except ImportError:
    ROS_AVAILABLE=False


@unittest.skipUnless(ROS_AVAILABLE,'source scripts/experiment_env.sh for ROS execution tests')
class ExecutionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def test_default_compatibility_matches_original_geometry_and_speed(self):
        node=EgoMpcFormation(3,spacing=10,route_str='0,0 80,0 80,80',speed=8.)
        try:
            self.assertFalse(node.capability_coordination)
            radius=adaptive_turn_radius(10,8,5,5,0,3.5)
            factor=min(1.,math.sqrt(5*radius)/8,corner_factor_by_spread(radius,10,8,3.5))
            original=IccRoute([(0,0),(80,0),(80,80)],radius,8,corner_slow=lambda th:factor)
            np.testing.assert_array_equal(original._S,node.route._S)
            np.testing.assert_array_equal(original._V,node.route._V)
        finally: node.destroy_node()

    def test_future_speed_and_failed_plan_invalidation(self):
        config=str(Path(__file__).resolve().parents[1]/'config/experiments/s1.yaml')
        node=EgoMpcFormation(3,route_str='0,0 80,0 80,80',speed=4,
                             capability_coordination=True,experiment_config=config)
        try:
            node.route_active=True; node.s=.5
            ref,speeds=node._make_ref(0)
            self.assertEqual(speeds.shape,(N,))
            self.assertGreater(speeds[-1],speeds[0])
            self.assertEqual(ref.shape,(3,N+1))
            with self.assertRaises(ValueError): node._build_route([(0,0),(1,0),(1,1)])
            self.assertIsNone(node.route)
            self.assertIsNone(node.progress_profile)
            self.assertTrue(node.planning_fault)
            self.assertFalse(node.route_active)
        finally: node.destroy_node()

    def test_scalar_and_constant_sequence_mpc_equivalence(self):
        a,b=DroneMPC(0),DroneMPC(1)
        a.state=b.state=np.array([0.,0.,0.])
        ref=np.zeros((3,N+1)); ref[0]=np.arange(N+1)*.05*2
        np.testing.assert_allclose(a.solve(ref,2.,v_min=0),b.solve(ref,np.full(N,2.),v_min=0),atol=1e-10)
        self.assertEqual(a.fail_count+b.fail_count,0)
