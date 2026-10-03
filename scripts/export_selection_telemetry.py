#!/usr/bin/env python3
"""Export measured body velocity, attitude and position commands from ROS bags.

Run through experiment_env.sh. CSV timestamps are original bag receive times;
channels are not silently interpolated or synchronized.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def export(run):
    target = run / 'measured_channels'
    target.mkdir(exist_ok=False)
    topics = {
        '/uav0/mavros/local_position/velocity_body': ('body_velocity', ['forward_m_s','left_m_s','up_m_s']),
        '/uav0/mavros/local_position/velocity_local': ('enu_velocity', ['east_m_s','north_m_s','up_m_s']),
        '/uav0/mavros/imu/data': ('attitude', ['roll_rad','pitch_rad','yaw_rad','p_rad_s','q_rad_s','r_rad_s']),
        '/uav0/mavros/setpoint_position/local': ('position_command', ['east_m','north_m','up_m','yaw_rad']),
    }
    counts = {name: 0 for name in topics}
    def angles(q):
        return [math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
                math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))),
                math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
    streams = {}
    writers = {}
    try:
        for topic, (name, fields) in topics.items():
            streams[topic] = (target/(name+'.csv')).open('w', newline='')
            writers[topic] = csv.writer(streams[topic])
            writers[topic].writerow(['bag_time_ns','message_time_ns','frame_id']+fields)
        for db in sorted((run/'bag').glob('*.db3')):
            with sqlite3.connect('file:'+str(db)+'?mode=ro', uri=True) as conn:
                mapping = {i: (n, get_message(t)) for i,n,t in conn.execute('select id,name,type from topics') if n in topics}
                ids = ','.join(str(i) for i in mapping)
                if not ids:
                    continue
                for tid, timestamp, data in conn.execute('select topic_id,timestamp,data from messages where topic_id in ('+ids+') order by timestamp'):
                    topic, cls = mapping[tid]
                    msg = deserialize_message(data, cls)
                    stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                    if 'velocity' in topic:
                        v = msg.twist.linear
                        values = [v.x,v.y,v.z]
                    elif topic.endswith('/data'):
                        w = msg.angular_velocity
                        values = angles(msg.orientation)+[w.x,w.y,w.z]
                    else:
                        p = msg.pose.position
                        values = [p.x,p.y,p.z,angles(msg.pose.orientation)[2]]
                    writers[topic].writerow([timestamp,stamp,msg.header.frame_id]+values)
                    counts[topic] += 1
    finally:
        for stream in streams.values():
            stream.close()
    (target/'manifest.json').write_text(json.dumps(dict(counts=counts,
        body_frame='MAVROS base_link FLU, full quaternion rotation in local_position plugin',
        evaluator_frame='precheck_result uses yaw-aligned horizontal projection; this export also retains full body velocity',
        interpolation='none', source=str(run/'bag'),
        extraction_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
    print(run, counts)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+')
    args=parser.parse_args()
    for path in args.runs:
        export(Path(path).resolve())
