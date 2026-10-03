#!/usr/bin/env python3
"""Own local SITL processes, run S1, record a bag, and produce replayable metrics."""
import argparse
from datetime import datetime
import fcntl
import math
import os
import random
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

from uav_formation.experiment_config import load_config
from uav_formation.experiment_artifacts import freeze_run, write_json, ROOT


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(ROOT/'config/experiments/s1.yaml'))
    parser.add_argument('--output')
    parser.add_argument('--method',choices=['F','legacy'],default='F')
    parser.add_argument('--domain',type=int,default=72)
    parser.add_argument('--base-port',type=int,default=16660)
    parser.add_argument('--timeout',type=float,default=300.)
    parser.add_argument('--ardupilot',default=str(Path.home()/'ardupilot'))
    parser.add_argument('--no-bag',action='store_true')
    parser.add_argument('--smoke',action='store_true',help='short development route, never a formal result')
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--position-jitter',type=float,default=0.,help='uniform initial ENU perturbation bound (m)')
    parser.add_argument('--heading-jitter-deg',type=float,default=0.,help='uniform initial heading perturbation bound (deg)')
    args=parser.parse_args()
    cfg,members=load_config(args.config)
    if any(m.platform!='multirotor' for m in members):
        parser.error('S2/S3 flight lifecycle is not validated; use plan_experiment.py for reference checks')
    if args.smoke:
        cfg['waypoints']=[[0.,0.],[45.,0.],[45.,45.],[90.,45.]]
        cfg['planner'].update(nominal_radius=16.,radii=[12.,16.,20.,24.],nominal_speed=3.,ds=1.)
    if not 0<=args.domain<=232: parser.error('invalid ROS domain')
    if not 0<=args.position_jitter<=1 or not 0<=args.heading_jitter_deg<=10:
        parser.error('initial perturbations limited to 1 m and 10 degrees')
    rng=random.Random(args.seed)
    psi=math.atan2(cfg['waypoints'][1][1]-cfg['waypoints'][0][1],cfg['waypoints'][1][0]-cfg['waypoints'][0][0])
    offsets=[(cfg['waypoints'][0][0]-m.r_y*math.sin(psi)+rng.uniform(-args.position_jitter,args.position_jitter),
              cfg['waypoints'][0][1]+m.r_y*math.cos(psi)+rng.uniform(-args.position_jitter,args.position_jitter)) for m in members]
    headings=[(90-math.degrees(psi)+rng.uniform(-args.heading_jitter_deg,args.heading_jitter_deg))%360 for m in members]
    cfg['execution']=dict(seed=args.seed,home_offsets=offsets,home_headings_deg=headings,
                          position_jitter=args.position_jitter,heading_jitter_deg=args.heading_jitter_deg,
                          method=args.method)
    lock=open('/tmp/nvidia-port-experiment-%d.lock'%args.domain,'w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    for i in range(len(members)):
        for delta in (0,2,3):
            with socket.socket() as probe: probe.bind(('127.0.0.1',args.base_port+10*i+delta))
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY']='1'
    binary=Path(args.ardupilot)/'build/sitl/bin/arducopter'
    if not binary.is_file(): parser.error('missing '+str(binary))
    directory=freeze_run(args.output or ROOT/'runs'/('sitl-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f')),cfg,
                         'development_smoke' if args.smoke else 'S1_SITL')
    config_path=directory/'config.json'
    os.environ['ROS_LOG_DIR']=str(directory/'ros_logs')
    print('RUN DIRECTORY:',directory.resolve(),flush=True)
    children=[]; streams=[]; commands=[]

    def launch(command,name,cwd=None):
        stream=(directory/(name+'.log')).open('w'); streams.append(stream)
        proc=subprocess.Popen([str(x) for x in command],stdout=stream,stderr=subprocess.STDOUT,
                              cwd=cwd or ROOT,start_new_session=True)
        children.append(proc); commands.append(dict(name=name,pid=proc.pid,command=[str(x) for x in command]))
        write_json(directory/'processes.json',commands)
        return proc

    try:
        psi=math.atan2(cfg['waypoints'][1][1]-cfg['waypoints'][0][1],cfg['waypoints'][1][0]-cfg['waypoints'][0][0])
        for m in members:
            i=m.member_id
            folder=directory/('sitl%d'%i); folder.mkdir()
            ox,oy=offsets[i]
            lat=30.893+math.degrees(oy/6378137.)
            lon=120.087+math.degrees(ox/(6378137.*math.cos(math.radians(30.893))))
            defaults=(ROOT/'config/copter_battery.parm').read_text()
            defaults+='\nSIM_WIND_SPD 0\nSIM_WIND_TURB 0\nSIM_SPEEDUP 1\nWP_SPD %g\nWP_ACC 5\n'%m.v_max
            param=folder/'defaults.parm'; param.write_text(defaults)
            port=args.base_port+10*i
            launch([binary,'--model','+','--speedup','1','--base-port',str(args.base_port),'-I',str(i),
                    '--sysid',str(i+1),'--home','%.8f,%.8f,10,%.3f'%(lat,lon,headings[i]),
                    '--defaults',param.resolve(),'--serial0','tcp:%d'%port,'--serial1','tcp:%d'%(port+2),
                    '--serial2','tcp:%d'%(port+3)],'sitl%d'%i,folder)
        # Readiness based on open sockets, not a fixed startup delay.
        for i in range(len(members)):
            deadline=time.monotonic()+20
            while True:
                try:
                    with socket.create_connection(('127.0.0.1',args.base_port+10*i),timeout=.2): break
                except OSError:
                    if time.monotonic()>deadline: raise TimeoutError('SITL did not start')
                    time.sleep(.2)
            # The installed workspace MAVROS adds its own /mavros prefix.
            launch(['ros2','run','mavros','mavros_node','--ros-args','-r','__ns:=/uav%d'%i,
                    '-p','fcu_url:=tcp://127.0.0.1:%d'%(args.base_port+10*i+2),
                    '-p','tgt_system:=%d'%(i+1),'-p','system_id:=255',
                    '--params-file',ROOT/'config/experiments/mavros_plugins.yaml',
                    '--params-file','/opt/ros/humble/share/mavros/launch/apm_config.yaml',
                    '--params-file',ROOT/'config/mavros_formation_override.yaml'],'mavros%d'%i)
        if not args.no_bag:
            launch(['ros2','bag','record','-a','-o',directory/'bag'],'rosbag')
        from uav_formation.experiment_runtime import execute_s1
        execute_s1(cfg,members,str(config_path),directory,args.method,args.timeout)
        from uav_formation.experiment_metrics import compute_metrics
        metrics=compute_metrics(directory)
        print('Completed. Metrics:',directory/'metrics.json',flush=True)
        print(metrics,flush=True)
        return 0
    except Exception as exc:
        write_json(directory/'error.json',dict(type=type(exc).__name__,message=str(exc)))
        raise
    finally:
        for proc in reversed(children):
            if proc.poll() is None: os.killpg(proc.pid,signal.SIGINT)
        for proc in reversed(children):
            try: proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try: proc.wait(timeout=3)
                except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL); proc.wait()
        for stream in streams: stream.close()
        lock.close()


if __name__=='__main__': raise SystemExit(main())
