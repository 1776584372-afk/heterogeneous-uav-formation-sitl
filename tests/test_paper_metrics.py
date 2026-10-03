import csv
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

from uav_formation.paper_metrics import compute, signal_derivative


class FleetMetricTests(unittest.TestCase):
    def test_held_telemetry_does_not_create_acceleration_spikes(self):
        times=np.array([0.,0.,.1,.1,.2,.2,.3,.3])
        np.testing.assert_allclose(signal_derivative(12.+.2*times,times),.2,atol=1e-12)

    def test_weighted_partial_failure_and_shape(self):
        cfg=dict(schema_version=1,name='synthetic',wind_m_s=0,altitude=60,endpoint='flythrough',
                 waypoints=[[0,0],[10,0]],members=[dict(member_id=i,platform='multirotor',r_y=i*10.,
                 v_min=0.,v_max=6.,a_lat_max=3.,a_t_min=-1.,a_t_max=1.) for i in range(2)])
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            (root/'config.json').write_text(json.dumps(cfg))
            (root/'flight_status.json').write_text(json.dumps(dict(completed=False,error='protected stop')))
            rows=[]
            for t in (0.,.25,.75,1.):
                for i in range(2):
                    rows.append(dict(t=t,phase='track',member=i,x=t,y=10*i,ref_x=t,ref_y=10*i,
                                     vx=7. if t==.25 else 2.,vy=0.,airspeed=2.,yaw=0.,roll=0.,
                                     controller_seconds=.001,mpc_failures=0,clock_drift=0.,radius=80.,lower=0.,upper=6.))
            with (root/'flight.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
                writer.writeheader(); writer.writerows(rows)
            result=compute(root)
            self.assertFalse(result['completed'])
            self.assertEqual(result['failure'],'protected stop')
            self.assertEqual(result['shape_rmse'],0.)
            self.assertEqual(result['minimum_separation'],10.)
            self.assertTrue(result['valid_telemetry'])
            for member in result['members'].values():
                self.assertEqual(member['tracking_rmse'],0.)
                self.assertEqual(member['violation_time_fraction']['speed_upper'],.5)

    def test_v3_capability_without_legacy_direction_limits_is_supported(self):
        cfg=dict(schema_version=1,name='v3',wind_m_s=0,altitude=60,endpoint='flythrough',
                 waypoints=[[0,0],[10,0]],members=[dict(member_id=0,platform='multirotor',r_y=0.,
                 v_min=0.,v_max=6.,a_lat_max=3.,a_t_min=-1.,a_t_max=1.)])
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            capability=root/'capability_v3.yaml'
            capability.write_text('multicopter:\n  planning_limit:\n    turn_speed_m_s: [0, 6]\n')
            cfg['capability_source']=str(capability)
            (root/'config.json').write_text(json.dumps(cfg))
            (root/'flight_status.json').write_text(json.dumps(dict(completed=True)))
            rows=[dict(t=t,phase='track',member=0,x=t,y=0.,ref_x=t,ref_y=0.,vx=2.,vy=0.,
                       airspeed=2.,yaw=0.,roll=0.,controller_seconds=.001,mpc_failures=0,
                       clock_drift=0.,radius=80.,lower=0.,upper=6.) for t in (0.,.5,1.)]
            with (root/'flight.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
                writer.writeheader();writer.writerows(rows)

            result=compute(root)

            self.assertEqual(result['members'][0]['horizontal_heading_direction_max_m_s'],{})
