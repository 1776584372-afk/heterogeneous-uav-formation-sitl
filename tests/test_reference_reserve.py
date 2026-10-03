import csv
import json
from pathlib import Path
import tempfile
import unittest
from uav_formation.capability_model import MemberSpec
from uav_formation.experiment_config import planning_members, coordinator_from_config
from uav_formation.experiment_artifacts import export_plan
from uav_formation.progress_profile import validate_profile


class ReferenceReserveTests(unittest.TestCase):
    def test_planning_reserve_keeps_physical_limits_immutable(self):
        member=MemberSpec(0,'multirotor',0,0,20,3,a_t_min=-.4,a_t_max=.4)
        reserved=planning_members({'reference_reserve':{'tangential_fraction':.5}},[member])[0]
        self.assertEqual(member.a_t_max,.4)
        self.assertEqual(reserved.a_t_max,.2)
        self.assertEqual(reserved.v_max,member.v_max)
        self.assertEqual(planning_members({},[member])[0],member)
        for factor in (0,2,float('nan'),True,False,'0.5'):
            with self.assertRaises(ValueError): planning_members({'reference_reserve':{'lateral_fraction':factor}},[member])

    def test_reserved_plan_passes_physical_audit_and_export_labels(self):
        members=[MemberSpec(0,'multirotor',0,0,20,3,a_t_min=-.4,a_t_max=.4)]
        cfg=dict(endpoint='hover',reference_reserve={'tangential_fraction':.5},
                 planner=dict(nominal_radius=10,radii=[10],nominal_speed=4,ds=2))
        result=coordinator_from_config(cfg,members).plan([(0,0),(100,0)])
        self.assertTrue(result.feasible)
        physical=validate_profile(result.route,members,result.profile)
        self.assertGreaterEqual(physical[0][7],.199)
        self.assertGreaterEqual(physical[0][8],.199)
        with tempfile.TemporaryDirectory() as folder:
            export_plan(folder,result,members)
            metadata=json.loads((Path(folder)/'constraint_semantics.json').read_text())
            self.assertTrue(metadata['reserve_is_not_capability_change'])
            with (Path(folder)/'reference.csv').open() as stream:
                rows=list(csv.DictReader(stream))
            self.assertIn('lower',rows[0])
            self.assertIn('upper',rows[0])
            self.assertAlmostEqual(float(rows[0]['nu']),0)
            self.assertAlmostEqual(float(rows[-1]['nu']),0)

    def test_reserve_can_reject_previously_feasible_task(self):
        members=[MemberSpec(0,'multirotor',0,0,20,3,a_t_min=-.4,a_t_max=.4)]
        cfg=dict(endpoint='hover',planner=dict(nominal_radius=10,radii=[10],
                 nominal_speed=4,start_speed=0,end_speed=4,ds=2))
        self.assertTrue(coordinator_from_config(cfg,members).plan([(0,0),(30,0)]).feasible)
        cfg['reference_reserve']={'tangential_fraction':.5}
        self.assertFalse(coordinator_from_config(cfg,members).plan([(0,0),(30,0)]).feasible)
