"""Feedback provenance, timing, frame transforms, solver and bridge contracts."""
import math
import os
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

os.environ.setdefault('ROS_DOMAIN_ID', '86')
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
from uav_formation.measured_feedback import (MeasuredFeedback, FeedbackError, WorldTransform,
    quaternion_yaw, command_endpoint, wrap_angle)
from uav_formation.egompc_formation import DroneMPC, EgoMpcFormation, N, DT, RATE, W_MAX


class MeasurementTests(unittest.TestCase):
    def test_rotation_translation_inverse_and_yaw(self):
        transform=WorldTransform(10.,-5.,math.pi/2)
        np.testing.assert_allclose(transform.position(2,3),(7,-3),atol=1e-12)
        np.testing.assert_allclose(transform.local_position(7,-3),(2,3),atol=1e-12)
        self.assertAlmostEqual(transform.heading(math.pi),-math.pi/2)
        self.assertAlmostEqual(quaternion_yaw(0,0,2,2),math.pi/2)
        with self.assertRaises(FeedbackError): quaternion_yaw(0,0,0,0)

    def test_stream_alignment_uses_measurements_and_shortest_yaw_arc(self):
        b=MeasuredFeedback(WorldTransform(10,20))
        b.push_position(1.,0.,0.,1.)
        b.push_position(1.1,1.,0.,1.1)
        b.push_yaw(1.02,math.radians(179),1.02)
        b.push_yaw(1.12,math.radians(-179),1.12)
        value=b.at(1.12,1.12,1.12)
        self.assertAlmostEqual(value.x,11.2)
        self.assertAlmostEqual(value.y,20)
        self.assertAlmostEqual(value.yaw,math.radians(-179))
        self.assertAlmostEqual(b.at(1.07,1.12,1.12).yaw,math.pi)
        self.assertAlmostEqual(value.extrapolation,.02)

    def test_missing_stale_duplicate_future_clock_reset_and_long_gap(self):
        b=MeasuredFeedback()
        with self.assertRaises(FeedbackError): b.at(1,1,1)
        b.push_pose(1,0,0,0,1)
        b.push_pose(1.1,1,0,0,1.1)
        with self.assertRaises(FeedbackError): b.at(1.7,1.7,1.7)
        b.push_pose(1.1,999,0,0,2.)
        with self.assertRaises(FeedbackError): b.at(1.1,1.1,2.)
        with self.assertRaises(FeedbackError): b.at(1.1,1.,1.1)
        with self.assertRaises(FeedbackError): b.at(1.4,1.4,1.4)
        with self.assertRaises(FeedbackError): b.push_pose(.8,0,0,0,2.)
        self.assertEqual(len(b.positions),0)
        b.push_pose(3.,0,0,0,3.)
        b.push_pose(3.4,0,0,0,3.4)
        with self.assertRaises(FeedbackError): b.at(3.2,3.4,3.4)
        with self.assertRaises(FeedbackError): b.push_yaw(3.5,float('inf'),3.5)
        self.assertEqual(len(b.headings),0)


class SolverFeedbackTests(unittest.TestCase):
    def reference(self, speed=4.):
        ref=np.zeros((3,N+1)); ref[0]=np.arange(N+1)*DT*speed
        return ref

    def measured(self,x,y,yaw=0,stamp=1.):
        b=MeasuredFeedback(); b.push_pose(stamp,x,y,yaw,stamp)
        return b.at(stamp,stamp,stamp)

    def test_new_actual_pose_changes_next_optimization_not_command_state(self):
        solver=DroneMPC(0,20)
        reference=self.reference()
        first=solver.solve_measured(reference,4.,self.measured(0,0),RATE)
        self.assertEqual(solver.last_status,'SOLVED')
        solver.state=np.array([999.,999.,2.])
        second=solver.solve_measured(reference,4.,self.measured(-2,1,stamp=2),RATE)
        np.testing.assert_allclose(solver.state,(-2,1,0),atol=1e-12)
        self.assertEqual(solver.last_status,'SOLVED')
        self.assertGreater(np.linalg.norm(np.subtract(first,second)),.1)
        self.assertLess(second[1],0.)
        self.assertNotEqual(command_endpoint(solver.state,*second,RATE),tuple(solver.state))

    def test_yaw_wrap_and_bounded_controls(self):
        reference=self.reference(); reference[0]*=-1; reference[2]=math.radians(179)
        s=DroneMPC(0,6)
        v,w=s.solve_measured(reference,4.,self.measured(0,0,math.radians(-179)),RATE)
        self.assertLess(abs(s.state[2]-reference[2,0]),math.radians(3))
        self.assertTrue(0<=v<=6 and abs(w)<=W_MAX)
        for y in [-50,50]:
            v,w=s.solve_measured(self.reference(30),30.,self.measured(-20,y),RATE)
            self.assertTrue(0<=v<=6 and abs(w)<=W_MAX)

    def test_solver_failure_exception_and_nan_result_have_bounded_fallback(self):
        ref=self.reference(); ref[1]=20
        s=DroneMPC(0,6)
        for fake in [SimpleNamespace(success=False,message='iteration limit',x=np.zeros(2*N)),
                     SimpleNamespace(success=True,message='bad result',x=np.full(2*N,np.nan))]:
            with patch('uav_formation.egompc_formation.scipy_minimize',return_value=fake):
                v,w=s.solve_measured(ref,4.,self.measured(0,0),RATE)
            self.assertTrue(0<=v<=6 and abs(w)<=W_MAX)
            self.assertEqual(s.last_status,'FALLBACK')
        with patch('uav_formation.egompc_formation.scipy_minimize',side_effect=RuntimeError('failure')):
            v,w=s.solve_measured(ref,4.,self.measured(0,0),RATE)
        self.assertTrue(0<=v<=6 and abs(w)<=W_MAX)
        self.assertEqual(s.fail_count,3)
        s.state=np.array([np.nan,0,0])
        self.assertEqual(s.solve(ref,4), (0.,0.))
        self.assertEqual(s.solve_measured(np.zeros((2,3)),4,self.measured(0,0)), (0.,0.))


class RosFeedbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        import rclpy
        rclpy.shutdown()

    def test_ros_tick_uses_each_odom_and_stale_member_freezes_progress(self):
        node=EgoMpcFormation(2,home_offsets=[(0,0),(10,-4)],route_str='0,0 100,0',speed=4)
        node.center_live=(0,0); node.route_active=True
        from nav_msgs.msg import Odometry
        now=node.get_clock().now().nanoseconds*1e-9
        for i in range(2):
            for stamp in (now-.05,now):
                msg=Odometry(); msg.header.stamp.sec=int(stamp)
                msg.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
                msg.pose.pose.position.x=2.+i; msg.pose.pose.orientation.w=1.
                node._feedback_cb(i,msg)
        try:
            node.tick()
            self.assertAlmostEqual(node.mpc[0].state[0],2.)
            self.assertAlmostEqual(node.mpc[1].state[0],13.)
            self.assertAlmostEqual(node.mpc[1].state[1],-4.)
            progress=node.s
            node.feedback[1].invalidate('test dropout')
            node.tick()
            self.assertEqual(node.s,progress)
            self.assertFalse(node.feedback_valid[1])
        finally: node.destroy_node()

    def test_bridge_position_yaw_only_and_watchdog_stops_forwarding(self):
        from uav_formation.poscmd_bridge import PosCmdBridge
        from quadrotor_msgs.msg import PositionCommand
        node=PosCmdBridge(1,[(10,20,math.pi/2)])
        class Recorder:
            def __init__(self): self.sent=[]
            def publish(self,msg): self.sent.append(msg)
        output=Recorder(); node.pubs=[output]
        node.states[0]=(True,'GUIDED'); node.alts[0]=60.
        node.feedback_ok[0]=True
        now=time.monotonic()
        node.last_state[0]=node.last_pose[0]=node.last_feedback[0]=now
        msg=PositionCommand(); msg.header.stamp=node.get_clock().now().to_msg()
        msg.position.x=7.;msg.position.y=22.;msg.position.z=60.;msg.yaw=math.pi
        msg.velocity.x=99.;msg.yaw_dot=9.
        try:
            node._mk_cb(0)(msg)
            self.assertEqual(len(output.sent),1)
            sent=output.sent[0]
            self.assertAlmostEqual(sent.pose.position.x,2)
            self.assertAlmostEqual(sent.pose.position.y,3)
            q=sent.pose.orientation
            self.assertAlmostEqual(quaternion_yaw(q.x,q.y,q.z,q.w),math.pi/2)
            self.assertFalse(hasattr(sent,'velocity'))
            node.last_feedback[0]=now-2
            node._watchdog()
            self.assertIn(0,node.faulted)
            node._mk_cb(0)(msg)
            self.assertEqual(len(output.sent),1)
        finally: node.destroy_node()

    def test_ros_bunched_timestamps_are_not_extrapolated_to_wall_time(self):
        node=EgoMpcFormation(2,home_offsets=[(0,0),(0,0)])
        node.center_live=(0,0)
        now=node.get_clock().now().nanoseconds*1e-9
        receipt=time.monotonic()
        # Two samples delivered in a burst must not invent a huge velocity.
        for buffer in node.feedback:
            buffer.push_pose(now-.1001,0,0,0,receipt)
            buffer.push_pose(now-.1,1,0,0,receipt)
        try:
            node.tick()
            for mpc in node.mpc:
                self.assertAlmostEqual(mpc.state[0],1.)
                self.assertEqual(mpc.last_feedback.extrapolation,0.)
                self.assertAlmostEqual(mpc.last_feedback.stamp,now-.1)
            node.tick()
            self.assertAlmostEqual(node._last_feedback_time,now-.1)
        finally: node.destroy_node()

    def test_bridge_stale_or_malformed_health_cannot_enable_output(self):
        import json
        from uav_formation.poscmd_bridge import PosCmdBridge
        from std_msgs.msg import String
        node=PosCmdBridge(1,[(0,0)])
        try:
            msg=String()
            now=node.get_clock().now().nanoseconds*1e-9
            msg.data=json.dumps({'valid':True,'stamp':now-2})
            node._feedback_cb(0,msg)
            self.assertFalse(node.feedback_ok[0])
            msg.data='null'; node._feedback_cb(0,msg)
            self.assertFalse(node.feedback_ok[0])
            msg.data=json.dumps({'valid':'false','stamp':now})
            node._feedback_cb(0,msg)
            self.assertFalse(node.feedback_ok[0])
        finally: node.destroy_node()
