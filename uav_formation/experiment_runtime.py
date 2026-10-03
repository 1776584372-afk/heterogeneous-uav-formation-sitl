"""SITL-only S1 orchestration, telemetry capture, and bounded flight lifecycle."""
import csv
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from mavros_msgs.msg import State, VfrHud
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, MessageInterval
from .egompc_formation import EgoMpcFormation
from .poscmd_bridge import PosCmdBridge
from .experiment_artifacts import export_plan, write_json


def angles(q):
    roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return roll,yaw


class FlightMonitor(Node):
    def __init__(self, members, offsets, directory):
        super().__init__('experiment_monitor')
        self.members,self.offsets=members,offsets
        self.states={}; self.poses={}; self.velocities={}; self.air={}; self.roll_rates={}
        self.service_clients={}
        self.feedback_stream=(Path(directory)/'feedback_status.jsonl').open('w')
        state_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for m in members:
            i=m.member_id; ns='/uav%d/mavros'%i
            self.create_subscription(State,ns+'/state',lambda msg,i=i:self.states.update({i:(time.monotonic(),msg)}),state_qos)
            self.create_subscription(PoseStamped,ns+'/local_position/pose',lambda msg,i=i:self.poses.update({i:(time.monotonic(),msg)}),qos_profile_sensor_data)
            self.create_subscription(TwistStamped,ns+'/local_position/velocity_local',lambda msg,i=i:self.velocities.update({i:(time.monotonic(),msg)}),qos_profile_sensor_data)
            self.create_subscription(VfrHud,ns+'/vfr_hud',lambda msg,i=i:self.air.update({i:(time.monotonic(),msg.airspeed)}),qos_profile_sensor_data)
            self.create_subscription(Imu,ns+'/imu/data',lambda msg,i=i:self.roll_rates.update({i:(time.monotonic(),msg.angular_velocity.x)}),qos_profile_sensor_data)
            self.create_subscription(String,'/drone_%d/feedback_status'%i,self.record_feedback,10)
            for name,typ,topic in [('mode',SetMode,'set_mode'),('arm',CommandBool,'cmd/arming'),
                                   ('takeoff',CommandTOL,'cmd/takeoff'),('interval',MessageInterval,'set_message_interval')]:
                self.service_clients[i,name]=self.create_client(typ,ns+'/'+topic)
        self.stream=(Path(directory)/'telemetry.csv').open('w',newline='')
        self.writer=csv.writer(self.stream)
        self.writer.writerow(['t','phase','member','s','nu','x','y','z','vx','vy','speed','airspeed','roll',
                              'body_roll_rate','yaw','armed','mode','pose_age','velocity_age','state_age','airspeed_age',
                              'ref_x','ref_y','ref_yaw','ref_speed','kappa','kappa_prime','radius','mpc_fail_count'])
        self.began=time.monotonic()

    def record_feedback(self,msg):
        self.feedback_stream.write(msg.data+'\n')
        self.feedback_stream.flush()

    def position(self,i):
        p=self.poses[i][1].pose.position
        ox,oy=self.offsets[i]
        return p.x+ox,p.y+oy,p.z

    def ready(self):
        now=time.monotonic()
        return all(i in self.states and self.states[i][1].connected and now-self.states[i][0]<3
                   and i in self.poses and now-self.poses[i][0]<2 and i in self.velocities
                   and now-self.velocities[i][0]<2 for i in range(len(self.members)))

    def record(self,node,phase):
        now=time.monotonic()
        for m in self.members:
            i=m.member_id
            if i not in self.poses or i not in self.velocities or i not in self.states: continue
            x,y,z=self.position(i)
            roll,yaw=angles(self.poses[i][1].pose.orientation)
            vel=self.velocities[i][1].twist.linear
            state=self.states[i][1]
            ref,speeds=node._make_ref(i)
            nu=node.progress_profile.nu_at(node.s) if node.progress_profile is not None and node.route_active else 0.
            if node.route_active and node.progress_profile is None:
                nu=node.route.speed_at(node.s)
            k=node.route.curvature_at(node.s) if node.route else 0.
            kp=node.route.curvature_derivative_at(node.s) if node.route else 0.
            self.writer.writerow([now-self.began,phase,i,node.s,nu,x,y,z,vel.x,vel.y,math.hypot(vel.x,vel.y),
                self.air.get(i,(0,''))[1],roll,self.roll_rates.get(i,(0,''))[1],yaw,state.armed,state.mode,
                now-self.poses[i][0],now-self.velocities[i][0],now-self.states[i][0],
                now-self.air[i][0] if i in self.air else '',ref[0,0],ref[1,0],ref[2,0],float(np.asarray(speeds).flat[0]),
                k,kp,node.selected_radius,node.mpc[i].fail_count])
        self.stream.flush()


def execute_s1(cfg,members,config_path,directory,method='F',timeout=300.):
    """Only called by the launcher that owns isolated local SITL instances."""
    if any(m.platform!='multirotor' for m in members):
        raise ValueError('S1 runner accepts multirotors only; S2/S3 require the fixedwing lifecycle')
    rclpy.init()
    route_string=' '.join('%g,%g'%tuple(p) for p in cfg['waypoints'])
    psi0=math.atan2(cfg['waypoints'][1][1]-cfg['waypoints'][0][1],cfg['waypoints'][1][0]-cfg['waypoints'][0][0])
    cx0,cy0=cfg['waypoints'][0]
    offsets=cfg.get('execution',{}).get('home_offsets',
        [(cx0-m.r_y*math.sin(psi0),cy0+m.r_y*math.cos(psi0)) for m in members])
    coordinator=EgoMpcFormation(len(members),spacing=members[1].r_y-members[0].r_y if len(members)>1 else 5.,
        fly_z=cfg['altitude'],route_str=route_string,turn_radius=cfg['planner']['nominal_radius'],
        speed=cfg['planner']['nominal_speed'],v_max=max(m.v_max for m in members),
        a_n_max=min(m.a_lat_max for m in members),trans_len=cfg['planner'].get('trans_len'),
        capability_coordination=method=='F',experiment_config=config_path,home_offsets=offsets)
    coordinator.slots_base=[(0.,m.r_y,0.) for m in members]
    coordinator.slots=list(coordinator.slots_base)
    # Spawn locations use the initial ENU slot positions, avoiding initial crossings.
    psi=coordinator.route.initial_heading()
    cx,cy=cfg['waypoints'][0]
    initial_slots=[(cx-m.r_y*math.sin(psi),cy+m.r_y*math.cos(psi)) for m in members]
    offsets=cfg.get('execution',{}).get('home_offsets',initial_slots)
    bridge=PosCmdBridge(len(members),offsets,gate_alt=.9*cfg['altitude'])
    monitor=FlightMonitor(members,offsets,directory)
    executor=SingleThreadedExecutor()
    for node in (coordinator,bridge,monitor): executor.add_node(node)
    if coordinator.coordination_result is not None:
        export_plan(directory,coordinator.coordination_result,members)
    deadline=time.monotonic()+timeout
    phase='connect'; last_record=0.

    def spin():
        nonlocal last_record
        executor.spin_once(timeout_sec=.02)
        now=time.monotonic()
        if now>deadline: raise TimeoutError('flight timeout during '+phase)
        if now-last_record>.1:
            monitor.record(coordinator,phase); last_record=now

    def until(predicate,seconds):
        limit=min(deadline,time.monotonic()+seconds)
        while not predicate():
            spin()
            if time.monotonic()>limit: raise TimeoutError('phase timeout: '+phase)

    def call(i,name,request):
        client=monitor.service_clients[i,name]
        until(client.service_is_ready,15.)
        future=client.call_async(request)
        until(future.done,15.)
        response=future.result()
        if response is None or not getattr(response,'success',getattr(response,'mode_sent',False)):
            raise RuntimeError('vehicle %d rejected %s: %s'%(i,name,response))

    success=False
    try:
        # Request streams as soon as FCU heartbeats arrive (this MAVROS does not request them).
        until(lambda: all(i in monitor.states and monitor.states[i][1].connected for i in range(len(members))),45.)
        for m in members:
            for msg_id,rate in ((32,20.),(30,20.),(74,10.),(33,5.)):
                req=MessageInterval.Request(); req.message_id=msg_id; req.message_rate=rate
                call(m.member_id,'interval',req)
        until(monitor.ready,40.)
        phase='takeoff'
        # Allow EKF origin and prearm checks to settle; never force-arm.
        settle=time.monotonic()+5
        while time.monotonic()<settle: spin()
        for m in members:
            req=SetMode.Request(); req.custom_mode='GUIDED'; call(m.member_id,'mode',req)
        until(lambda: all(monitor.states[m.member_id][1].mode=='GUIDED' for m in members),10.)
        for m in members:
            req=CommandBool.Request(); req.value=True; call(m.member_id,'arm',req)
            req=CommandTOL.Request(); req.altitude=float(cfg['altitude']); call(m.member_id,'takeoff',req)
        until(lambda: all(monitor.position(m.member_id)[2]>.92*cfg['altitude'] for m in members),55.)
        phase='formation'
        coordinator.center_live=(cx,cy)
        coordinator.psi_leg=psi; coordinator.psi_rot=psi
        until(lambda: all(math.hypot(monitor.position(m.member_id)[0]-initial_slots[m.member_id][0],
                                    monitor.position(m.member_id)[1]-initial_slots[m.member_id][1])<2.
                          for m in members),30.)
        phase='track'
        coordinator.route_active=True
        print('TRACKING',flush=True)
        while coordinator.s < coordinator.route.total_length()-1e-5:
            spin()
            if not monitor.ready(): raise RuntimeError('stale flight telemetry')
            if coordinator.planning_fault: raise RuntimeError('coordination execution fault')
            if any(not monitor.states[m.member_id][1].armed or monitor.states[m.member_id][1].mode!='GUIDED' for m in members):
                raise RuntimeError('vehicle left armed GUIDED during tracking')
        phase='settle'
        until(lambda: all(math.hypot(monitor.position(m.member_id)[0]-coordinator.route.eval_drone(coordinator.s,0,m.r_y)[0],
                                    monitor.position(m.member_id)[1]-coordinator.route.eval_drone(coordinator.s,0,m.r_y)[1])<2.
                          for m in members),25.)
        success=True
    finally:
        phase='land'
        coordinator.timer.cancel()
        coordinator.route_active=False
        for m in members:
            if m.member_id in monitor.states and monitor.states[m.member_id][1].armed:
                req=SetMode.Request(); req.custom_mode='LAND'
                if monitor.service_clients[m.member_id,'mode'].service_is_ready():
                    monitor.service_clients[m.member_id,'mode'].call_async(req)
        land_deadline=time.monotonic()+40.
        while time.monotonic()<land_deadline and any(st[1].armed for st in monitor.states.values()):
            executor.spin_once(timeout_sec=.05)
        write_json(Path(directory)/'flight_status.json',dict(completed=success,method=method,
            landed=all(not st[1].armed for st in monitor.states.values()),
            mpc_failures=[m.fail_count for m in coordinator.mpc],final_progress=coordinator.s))
        monitor.stream.close()
        monitor.feedback_stream.close()
        for node in (coordinator,bridge,monitor): executor.remove_node(node); node.destroy_node()
        executor.shutdown(); rclpy.shutdown()
    return success
