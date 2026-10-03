import csv
import json
from pathlib import Path
import tempfile
import unittest
from uav_formation.experiment_config import load_config
from uav_formation.experiment_metrics import compute_metrics


class MetricTests(unittest.TestCase):
    def test_perfect_measured_formation_and_time_weighted_speed_violation(self):
        cfg,_=load_config(Path(__file__).resolve().parents[1]/'config/experiments/s1.yaml')
        with tempfile.TemporaryDirectory() as folder:
            directory=Path(folder)
            (directory/'config.json').write_text(json.dumps(cfg))
            (directory/'flight_status.json').write_text('{"completed":true}')
            rows=[]
            for t in (0.,1.,3.,4.):
                for m in cfg['members']:
                    row=dict(t=t,phase='track',member=m['member_id'],x=2*t,y=m['r_y'],ref_x=2*t,ref_y=m['r_y'],
                             vx=7. if t==1 else 2.,vy=0.,yaw=0.,roll=0.,pose_age=.01,velocity_age=.01,state_age=.01,
                             radius=20,mpc_fail_count=0)
                    rows.append(row)
            with (directory/'telemetry.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
            metrics=compute_metrics(directory)
            self.assertEqual(metrics['shape_distance_rmse'],0.)
            self.assertEqual(metrics['minimum_separation'],5.)
            for result in metrics['members'].values():
                self.assertEqual(result['planar_tracking_rmse'],0.)
                self.assertEqual(result['violations_time_fraction']['speed_max'],.5)
