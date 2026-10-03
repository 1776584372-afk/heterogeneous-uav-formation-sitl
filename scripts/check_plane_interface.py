#!/usr/bin/env python3
"""Ground-only local Plane protocol test: GUIDED, COMMAND_INT, and real ACKs."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
from uav_formation.experiment_artifacts import ROOT,write_json
from uav_formation.fixedwing_adapter import decode_command_ack


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output'); parser.add_argument('--domain',type=int,default=73)
    parser.add_argument('--port',type=int,default=17660)
    args=parser.parse_args()
    for port in (args.port,args.port+2):
        with socket.socket() as probe: probe.bind(('127.0.0.1',port))
    directory=Path(args.output or ROOT/'runs'/('plane-interface-'+datetime.now().strftime('%Y%m%d-%H%M%S'))).resolve()
    directory.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(directory/'ros_logs'))
    procs=[]; streams=[]
    def launch(command,name):
        stream=(directory/(name+'.log')).open('w'); streams.append(stream)
        proc=subprocess.Popen([str(x) for x in command],cwd=directory,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        procs.append(proc)
    try:
        launch([Path.home()/'ardupilot/build/sitl/bin/arduplane','--model','plane','--speedup','1',
                '--base-port',str(args.port),'-I','0','--sysid','1','--home','30.893,120.087,10,90',
                '--serial0','tcp:%d'%args.port,'--serial1','tcp:%d'%(args.port+2)],'plane')
        deadline=time.monotonic()+15
        while True:
            try:
                with socket.create_connection(('127.0.0.1',args.port),timeout=.2): break
            except OSError:
                if time.monotonic()>deadline: raise TimeoutError('Plane startup')
                time.sleep(.2)
        launch(['ros2','run','mavros','mavros_node','--ros-args','-r','__ns:=/uav0',
                '-p','fcu_url:=tcp://127.0.0.1:%d'%(args.port+2),'-p','tgt_system:=1',
                '--params-file',ROOT/'config/experiments/mavros_plugins.yaml',
                '--params-file','/opt/ros/humble/share/mavros/launch/apm_config.yaml'],'mavros')
        import rclpy
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
        from mavros_msgs.msg import State,Mavlink
        from mavros_msgs.srv import SetMode,CommandInt
        rclpy.init(); node=rclpy.create_node('plane_interface_check')
        states=[]; acks={}
        node.create_subscription(State,'/uav0/mavros/state',lambda m:states.append(m),
            QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def ack(msg):
            decoded=decode_command_ack(msg)
            if decoded is not None and msg.sysid==1: acks[decoded[0]]=decoded[1]
        node.create_subscription(Mavlink,'/uas1/mavlink_source',ack,qos_profile_sensor_data)
        mode=node.create_client(SetMode,'/uav0/mavros/set_mode')
        command=node.create_client(CommandInt,'/uav0/mavros/cmd/command_int')
        def until(predicate,timeout=30):
            deadline=time.monotonic()+timeout
            while not predicate():
                rclpy.spin_once(node,timeout_sec=.05)
                if time.monotonic()>deadline: raise TimeoutError('Plane service/state/ACK timeout')
        until(lambda: states and states[-1].connected and mode.service_is_ready() and command.service_is_ready())
        req=SetMode.Request(); req.custom_mode='GUIDED'; future=mode.call_async(req); until(future.done)
        until(lambda:states[-1].mode=='GUIDED')
        for code,p1,p2,p3,z in ((43000,0.,17.,1.,0.),(186,60.,3.,0.,0.),(43002,0.,90.,4.,0.)):
            req=CommandInt.Request(); req.command=code; req.frame=3; req.param1=p1; req.param2=p2; req.param3=p3; req.z=z
            future=command.call_async(req); until(future.done)
            until(lambda:code in acks)
            if acks[code]!=0: raise RuntimeError('command %d rejected: %d'%(code,acks[code]))
        report=dict(test='ground protocol acceptance only',armed=states[-1].armed,mode=states[-1].mode,
                    command_ack=acks,flight_validated=False)
        write_json(directory/'result.json',report); print(report,flush=True)
        node.destroy_node(); rclpy.shutdown()
    finally:
        for proc in reversed(procs):
            if proc.poll() is None: os.killpg(proc.pid,signal.SIGINT)
        for proc in reversed(procs):
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL); proc.wait()
        for stream in streams: stream.close()


if __name__=='__main__': main()
