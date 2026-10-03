"""ArduPlane GUIDED command adapter for the locally inspected firmware.

COMMAND_INT 43000/186/43002, NOT Copter position setpoints. This adapter
requires an already airborne vehicle and a separate takeoff/exit lifecycle.
No S2/S3 closed-loop validation is implied by command acceptance.
"""
from dataclasses import dataclass
import math
import struct
from .capability_model import G
from .paper_tracking import course_command, PlaneCourseTracker


@dataclass(frozen=True)
class GuidedCommand:
    command: int
    frame: int = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
    param1: float = 0.
    param2: float = 0.
    param3: float = 0.
    z: float = 0.


def encode_reference(member, reference, position, lookahead=40.):
    """Stateless protocol helper; flight callers also apply command slew."""
    return encode_course_command(course_command(member,reference,position,lookahead))


def encode_course_command(command):
    return (GuidedCommand(43000,param1=0.,param2=command.airspeed,param3=0.),
            GuidedCommand(186,param1=command.altitude,param2=3.),
            GuidedCommand(43002,param1=0.,param2=command.bearing_deg,param3=command.lateral_accel_limit))


def decode_command_ack(msg):
    if msg.msgid!=77: return None
    payload=b''.join(struct.pack('<Q',int(value)) for value in msg.payload64)
    if len(payload)<3: return None
    return struct.unpack_from('<HB',payload)


def main():
    import argparse
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float64MultiArray, Int32MultiArray
    from mavros_msgs.msg import State, VfrHud, Mavlink
    from mavros_msgs.srv import CommandInt, SetMode
    from .experiment_config import load_config
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True); parser.add_argument('--member',required=True,type=int)
    parser.add_argument('--home-east',type=float,default=0.); parser.add_argument('--home-north',type=float,default=0.)
    parser.add_argument('--lookahead',type=float)
    parser.add_argument('--guidance-mode',choices=['preview','frenet','curvature_ff'])
    parser.add_argument('--cross-track-gain',type=float)
    parser.add_argument('--course-to-bank-gain',type=float)
    args=parser.parse_args(); cfg,members=load_config(args.config); member=members[args.member]
    if member.platform!='fixedwing': parser.error('selected member is not fixedwing')
    guidance=dict(cfg.get('fixedwing_guidance',{}))
    for key,value in (('lookahead',args.lookahead),('guidance_mode',args.guidance_mode),
                      ('cross_track_gain',args.cross_track_gain),
                      ('course_to_bank_gain',args.course_to_bank_gain)):
        if value is not None: guidance[key]=value

    class Adapter(Node):
        def __init__(self):
            super().__init__('fixedwing_adapter_%d'%args.member)
            self.reference=None; self.pose=None; self.airspeed=None; self.state=None
            self.pending={}; self.acks={}; self.sent={}; self.fault=False
            self.tracker=PlaneCourseTracker(member,**guidance)
            self.last_update=None
            ns='/uav%d/mavros'%args.member
            sq=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(State,ns+'/state',lambda m:self.save('state',m),sq)
            self.create_subscription(PoseStamped,ns+'/local_position/pose',lambda m:self.save('pose',m),qos_profile_sensor_data)
            self.create_subscription(VfrHud,ns+'/vfr_hud',lambda m:self.save('airspeed',m.airspeed),qos_profile_sensor_data)
            self.create_subscription(Float64MultiArray,'/drone_%d/fixedwing_reference'%args.member,
                                     lambda m:self.save('reference',list(m.data)),10)
            self.create_subscription(Mavlink,'/uas%d/mavlink_source'%(args.member+1),self.ack,qos_profile_sensor_data)
            self.client=self.create_client(CommandInt,ns+'/cmd/command_int')
            self.mode=self.create_client(SetMode,ns+'/set_mode')
            self.diagnostic=self.create_publisher(Int32MultiArray,'/drone_%d/fixedwing_ack'%args.member,10)
            self.create_timer(.2,self.tick)

        def save(self,name,value): setattr(self,name,(time.monotonic(),value))

        def ack(self,msg):
            if msg.sysid!=args.member+1: return
            decoded=decode_command_ack(msg)
            if decoded is not None and decoded[0] in (43000,186,43002):
                self.acks[decoded[0]]=(time.monotonic(),decoded[1])
                report=Int32MultiArray(); report.data=list(decoded); self.diagnostic.publish(report)
                if decoded[1]!=0: self.stop('ArduPlane rejected command %s'%str(decoded))

        def stop(self,reason):
            if self.fault: return
            self.fault=True; self.get_logger().error(reason)
            if self.mode.service_is_ready():
                req=SetMode.Request(); req.custom_mode='LOITER'; self.mode.call_async(req)

        def tick(self):
            if self.fault or self.state is None: return
            now=time.monotonic(); state=self.state[1]
            if not (state.connected and state.armed and state.mode=='GUIDED'): return
            if self.reference is None: return
            if now-self.state[0]>3 or any(v is None or now-v[0]>.8 for v in (self.reference,self.pose,self.airspeed)):
                self.stop('stale reference or telemetry'); return
            if self.airspeed[1]<member.v_min:
                self.stop('measured airspeed below configured minimum'); return
            for command,t0 in self.sent.items():
                ack=self.acks.get(command)
                if now-t0>2 and (ack is None or now-ack[0]>2):
                    self.stop('COMMAND_ACK timeout'); return
            if not self.client.service_is_ready(): return
            p=self.pose[1].pose.position
            try:
                dt=.2 if self.last_update is None else now-self.last_update
                guidance=self.tracker.update(self.reference[1],(p.x+args.home_east,p.y+args.home_north),self.airspeed[1],dt)
                commands=encode_course_command(guidance)
                self.last_update=now
            except ValueError as exc:
                self.stop(str(exc)); return
            for c in commands:
                if c.command in self.pending:
                    future=self.pending[c.command]
                    if not future.done(): continue
                    try:
                        if not future.result().success: self.stop('COMMAND_INT transport failed'); return
                    except Exception as exc: self.stop(str(exc)); return
                req=CommandInt.Request()
                for name in ('command','frame','param1','param2','param3','z'): setattr(req,name,getattr(c,name))
                self.pending[c.command]=self.client.call_async(req)
                self.sent.setdefault(c.command,now)

    rclpy.init(); node=Adapter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
