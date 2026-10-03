"""Owned localhost SITL lifecycle and raw MAVLink evidence for calibration.

This module cannot attach to an external vehicle. It starts its own process,
uses a fresh EEPROM directory, and never force-arms or alters speed limits.
"""
import csv
from contextlib import nullcontext
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import socket
import subprocess
import threading
import time

from pymavlink import mavutil
from .measured_feedback import MeasuredFeedback, WorldTransform, FeedbackError
from .command_schedule import HeadingSchedule
from .send_journal import JournaledSocket
from .telemetry_monitor import CONFIG as MONITOR_CONFIG, monitor_settings

ROOT = Path(__file__).resolve().parents[1]
AP = Path.home()/'ardupilot'
EVIDENCE_BUFFER_BYTES = 16 * 1024 * 1024


def body_velocity(vn, ve, vd, roll, pitch, yaw):
    cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
    return (cp*cy*vn+cp*sy*ve-sp*vd,
            -((sr*sp*cy-cr*sy)*vn+(sr*sp*sy+cr*cy)*ve+sr*cp*vd),
            -((cr*sp*cy+sr*sy)*vn+(cr*sp*sy-sr*cy)*ve+cr*cp*vd))


class SitlVehicle:
    def __init__(self, vehicle, model, directory, port, altitude=60., speedup=2., defaults=None, home_offset=(0.,0.), *, plane_heading_period=0., gcs_system_id=1):
        if vehicle not in ('copter','plane'):
            raise ValueError('unsupported vehicle')
        if not math.isfinite(plane_heading_period) or plane_heading_period < 0 or (vehicle != 'plane' and plane_heading_period):
            raise ValueError('heading scheduling is a nonnegative Plane-only option')
        self.heading_schedule = HeadingSchedule(plane_heading_period) if plane_heading_period else None
        self.gcs_settings = monitor_settings(gcs_system_id)
        self.gcs_process = None
        self.gcs_log = None
        self.heading_pending_since = None
        self.vehicle,self.model,self.directory,self.port=vehicle,model,Path(directory).resolve(),port
        self.altitude,self.speedup=altitude,speedup
        self.home_offset=tuple(home_offset)
        self.feedback=MeasuredFeedback(WorldTransform(*self.home_offset))
        self.directory.mkdir(parents=True,exist_ok=False)
        self.messages={}; self.received={}; self.message_clock={}; self.phase='connect'; self.target={}
        self.process=None; self.link=None; self.log=None; self.csv=None; self.events=None
        self.command_lock=threading.RLock()
        self.telemetry_lock=threading.RLock(); self.evidence_lock=threading.RLock()
        self.telemetry_stop=threading.Event(); self.telemetry_thread=None; self.telemetry_error=None
        self.send_journal=None; self.send_log=None
        self.plane_lateral_mode=None; self.command_acks={}; self.command_first_sent={}
        self.last_record=0.; self.last_heartbeat=0.; self.samples=[]
        self.start_wall=time.monotonic()
        sitl_cpu_text=os.environ.get('UAV_SITL_CPU')
        self.sitl_cpu=(int(sitl_cpu_text) if sitl_cpu_text is not None else None)
        aux_cpu_text=os.environ.get('UAV_AUX_CPU')
        self.aux_cpu=(int(aux_cpu_text) if aux_cpu_text is not None else None)
        if (self.sitl_cpu is not None
                and not 0 <= self.sitl_cpu < (os.cpu_count() or 1)):
            raise ValueError('UAV_SITL_CPU is outside the host CPU range')
        if (self.aux_cpu is not None
                and not 0 <= self.aux_cpu < (os.cpu_count() or 1)):
            raise ValueError('UAV_AUX_CPU is outside the host CPU range')
        binary=AP/'build/sitl/bin'/('ardu'+vehicle)
        if defaults is None:
            defaults=AP/'Tools/autotest/models/plane.parm' if vehicle=='plane' else ROOT/'config/copter_battery.parm'
        default_paths=[Path(p).expanduser().resolve() for p in str(defaults).split(',')]
        param_text='\n'.join(p.read_text() for p in default_paths)
        # Logging and no-wind experiment conditions; capability parameters remain as supplied.
        param_text+='\nSIM_WIND_SPD 0\nSIM_WIND_TURB 0\nLOG_DISARMED 1\nLOG_BITMASK 65535\n'
        if vehicle=='plane':
            param_text+='TKOFF_ALT %g\n'%altitude
        else:
            # Same simulated accelerometer initialization used by upstream plane.parm.
            for prefix in ('INS_ACC','INS_ACC2'):
                for axis in 'XYZ':
                    param_text+='%sOFFS_%s 0.001\n%sSCAL_%s 1.001\n'%(prefix,axis,prefix,axis)
            param_text+='INS_GYR_CAL 0\n'
        params=self.directory/'defaults.parm'
        params.write_text(param_text)
        for p in (port,port+2,port+3):
            with socket.socket() as sock:
                sock.bind(('127.0.0.1',p))
        lat=30.893+math.degrees(home_offset[1]/6378137.)
        lon=120.087+math.degrees(home_offset[0]/(6378137.*math.cos(math.radians(30.893))))
        self.command=[str(binary),'--model',model,'--speedup',str(speedup),'--base-port',str(port),
                      '-I','0','--sysid','1','--home','%.8f,%.8f,10,90'%(lat,lon),'--defaults',str(params),
                      '--serial0','tcp:%d'%port,'--serial1','tcp:%d'%(port+2),'--serial2','tcp:%d'%(port+3)]
        manifest=dict(command=self.command, model=model,vehicle=vehicle,altitude=altitude,speedup=speedup,home_offset=home_offset,
                      git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                      git_status=subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True),
                      ardupilot_commit=subprocess.check_output(['git','-C',str(AP),'rev-parse','HEAD'],text=True).strip(),
                      binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                      defaults_source=str(defaults),defaults_sources=[str(p) for p in default_paths],
                      plane_heading_period_sim_s=plane_heading_period,source_sha256={},
                      telemetry_monitor=self.gcs_settings,
                      host_scheduling=dict(control_affinity=sorted(os.sched_getaffinity(0)),
                                           sitl_cpu=self.sitl_cpu,aux_cpu=self.aux_cpu),
                      outgoing_evidence=dict(file='outgoing_packets.jsonl',schema_version=1,
                          scope='socket.send bytes/count/errors; local acceptance is not aircraft receipt',
                          boot_clock='latest received ATTITUDE/LOCAL_POSITION_NED; not exact send boot time'))
        snapshot=self.directory/'source_snapshot'; snapshot.mkdir()
        model_sources=[]
        if ':' in model and model.endswith('.json'):
            model_path=Path(model.split(':',1)[1])
            if not model_path.is_file():
                model_path=AP/'Tools/autotest/models'/model_path
            model_sources.append(model_path)
        for source in [*ROOT.glob('uav_formation/*.py'),*ROOT.glob('scripts/calibrate*.py'),
                       ROOT/'scripts/run_paper_sitl.py',MONITOR_CONFIG,*default_paths,*model_sources]:
            if source.exists():
                shutil.copy2(source,snapshot/source.name)
                manifest['source_sha256'][str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
        (self.directory/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

    def start(self):
        self.log=(self.directory/'sitl.log').open('w')
        preexec = None
        if self.sitl_cpu is not None:
            preexec = lambda: os.sched_setaffinity(0, {self.sitl_cpu})
        self.process=subprocess.Popen(self.command,cwd=self.directory,stdout=self.log,
                                      stderr=subprocess.STDOUT,preexec_fn=preexec)
        deadline=time.monotonic()+20
        while self.link is None:
            if self.process.poll() is not None:
                raise RuntimeError('SITL exited during startup')
            try:
                self.link=mavutil.mavlink_connection('tcp:127.0.0.1:%d'%self.port,source_system=255)
            except OSError:
                if time.monotonic()>deadline:
                    raise TimeoutError('SITL connection')
                time.sleep(.1)
        if self.link.autoreconnect:
            raise RuntimeError('owned SITL packet journal requires non-reconnecting TCP')
        self.send_log=(self.directory/'outgoing_packets.jsonl').open(
            'x', buffering=EVIDENCE_BUFFER_BYTES)
        self.send_journal=JournaledSocket(self.link.port,self.send_log,self._send_context,origin=self.start_wall)
        self.link.port=self.send_journal
        self.link.logfile=(self.directory/'raw.tlog').open(
            'wb', buffering=EVIDENCE_BUFFER_BYTES)
        self.events=(self.directory/'events.jsonl').open(
            'w', buffering=EVIDENCE_BUFFER_BYTES)
        self.csv=(self.directory/'telemetry.csv').open(
            'w', newline='', buffering=EVIDENCE_BUFFER_BYTES)
        self.fields=['wall_s','sim_s','velocity_sim_s','airspeed_sim_s','phase','mode','armed','east','north','alt','vn','ve','vd',
                     'forward','left','up','heading_forward','heading_left','airspeed','groundspeed','roll','pitch','yaw','p','q','r',
                     'throttle','servo1','servo2','servo3','servo4','ref_speed','ref_accel','ref_course','ref_east','ref_north',
                     'ref_bank','target_bank','attitude_age','velocity_age','airspeed_age',
                     'sent_ve','sent_vn','sent_ae','sent_an','sent_yaw','sent_yaw_rate','sent_mask','ref_path_course']
        self.writer=csv.DictWriter(self.csv,fieldnames=self.fields); self.writer.writeheader()
        if self.gcs_settings['enabled']:
            self.gcs_log=(self.directory/'gcs_forwarder.log').open('w')
            command=['/bin/bash',str(Path(__file__).resolve().parents[1]/'scripts/experiment_env.sh'),
                'python3','-m','uav_formation.gcs_forward_process',
                '--tcp-port',str(self.port+3),'--host',self.gcs_settings['host'],
                '--udp-port',str(self.gcs_settings['port']),'--display-system-id',
                str(self.gcs_settings['display_system_id']),'--summary',
                str(self.directory/'gcs_forwarding.json')]
            if self.aux_cpu is not None:
                command.extend(['--cpu',str(self.aux_cpu)])
            self.gcs_process=subprocess.Popen(command,cwd=ROOT,stdout=self.gcs_log,
                                               stderr=subprocess.STDOUT)
        print('GCS telemetry', self.directory.name, self.gcs_settings, flush=True)
        self.wait(lambda:'HEARTBEAT' in self.messages,15,'heartbeat')
        with self.command_lock:
            for mid,hz in ((30,20),(32,20),(33,10),(74,10),(36,10),(62,10)):
                self.link.mav.command_long_send(1,1,511,0,mid,1e6/hz,0,0,0,0,0)
            self.link.mav.param_request_list_send(1,1)
        self.wait(lambda:all(n in self.messages for n in ('ATTITUDE','LOCAL_POSITION_NED','GLOBAL_POSITION_INT','VFR_HUD')),45,'telemetry')
        self.start_telemetry_receiver()
        return self

    def _check_runtime(self):
        if self.send_journal is not None:
            self.send_journal.check_logging()
        if self.process.poll() is not None:
            raise RuntimeError('owned SITL exited')
        if self.telemetry_error is not None:
            raise RuntimeError('telemetry receiver failed: '+repr(self.telemetry_error))

    def _receive_for(self, seconds):
        self._check_runtime()
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            msg=self.link.recv_match(blocking=True,timeout=min(.01,max(0,until-time.monotonic())))
            if msg is not None:
                self._record_message(msg)
                # pymavlink parsing and CSV/evidence bookkeeping execute under
                # the CPython GIL. Yield after each frame so the independent
                # cached-command streamer cannot be starved by a burst of
                # buffered telemetry.
                time.sleep(.001)
            now=time.monotonic()
            if now-self.last_heartbeat>1:
                with self.command_lock:
                    self.link.mav.heartbeat_send(6,8,0,0,4)
                self.last_heartbeat=now
            with self.telemetry_lock:
                record_due=(now-self.last_record>.05/self.speedup
                    and all(n in self.messages for n in ('ATTITUDE','LOCAL_POSITION_NED',
                                                         'GLOBAL_POSITION_INT','VFR_HUD','HEARTBEAT')))
            if record_due:
                row=self.row(); self.writer.writerow(row); self.last_record=now

    def _record_message(self, msg):
        name=msg.get_type(); received=time.monotonic()
        with self.telemetry_lock:
            self.messages[name]=msg; self.received[name]=received
            try:
                if name=='LOCAL_POSITION_NED':
                    self.feedback.push_position(msg.time_boot_ms/1000.,msg.y,msg.x,received)
                elif name=='ATTITUDE':
                    self.feedback.push_yaw(msg.time_boot_ms/1000.,math.pi/2-msg.yaw,received)
            except FeedbackError as exc:
                self.feedback.invalidate(str(exc))
            if name=='VFR_HUD' and 'ATTITUDE' in self.messages:
                self.message_clock[name]=self.messages['ATTITUDE'].time_boot_ms/1000.
            if name=='COMMAND_ACK':
                self.command_acks[msg.command]=(received,int(msg.result))
                if msg.command==43002 and self.heading_pending_since is not None:
                    if received>=self.heading_pending_since and int(msg.result)==0:
                        self.heading_pending_since=None
        if name in ('STATUSTEXT','COMMAND_ACK','PARAM_VALUE'):
            with self.evidence_lock:
                self.events.write(json.dumps(dict(wall_s=received-self.start_wall,
                                                  message=msg.to_dict()))+'\n')

    def _telemetry_worker(self):
        try:
            if self.aux_cpu is not None:
                os.sched_setaffinity(threading.get_native_id(), {self.aux_cpu})
            while not self.telemetry_stop.is_set():
                self._receive_for(.01)
        except BaseException as exc:
            self.telemetry_error=exc

    def start_telemetry_receiver(self):
        if self.telemetry_thread is not None:
            raise RuntimeError('telemetry receiver already started')
        self.telemetry_stop.clear(); self.telemetry_error=None
        self.telemetry_thread=threading.Thread(target=self._telemetry_worker,
            name='sitl-telemetry-'+self.directory.name,daemon=True)
        self.telemetry_thread.start()

    def stop_telemetry_receiver(self):
        thread=self.telemetry_thread
        if thread is None:
            return
        self.telemetry_stop.set(); thread.join(timeout=2.)
        if thread.is_alive():
            raise RuntimeError('telemetry receiver did not stop')
        self.telemetry_thread=None
        if self.telemetry_error is not None:
            raise RuntimeError('telemetry receiver failed: '+repr(self.telemetry_error))

    def pump(self, seconds=.02):
        if self.telemetry_thread is None:
            self._receive_for(seconds)
            return
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            self._check_runtime()
            time.sleep(min(.002,max(0.,deadline-time.monotonic())))

    def _send_context(self):
        with self.command_lock:
            target=dict(self.target)
            with self.telemetry_lock:
                clocks={}
                for name in ('ATTITUDE','LOCAL_POSITION_NED'):
                    message=self.messages.get(name)
                    clocks[name]=dict(boot_s=message.time_boot_ms/1000. if message is not None else None,
                                      received_monotonic_s=self.received.get(name))
                phase=self.phase
        return dict(instance=self.directory.name,vehicle=self.vehicle,model=self.model,
                    port=self.port,phase=phase,clocks=clocks,target=target)

    def command_stream_descriptor(self, member):
        """Spawn-safe description of the unused second SITL MAVLink link."""
        return dict(
            member=int(member),
            instance=self.directory.name,
            vehicle=self.vehicle,
            model=self.model,
            directory=str(self.directory),
            tcp_port=self.port+2,
            altitude=float(self.altitude),
            home_offset=[float(value) for value in self.home_offset],
            origin=float(self.start_wall),
            initial_context=self._send_context(),
        )

    def command_stream_context(self):
        """Snapshot source clocks and command semantics for stream evidence."""
        return self._send_context()

    def wait(self,predicate,timeout,label):
        deadline=time.monotonic()+timeout
        while not predicate():
            self.pump()
            if time.monotonic()>deadline:
                raise TimeoutError(label)

    def row(self):
        with getattr(self,'command_lock',nullcontext()):
            target=dict(self.target)
            with getattr(self,'telemetry_lock',nullcontext()):
                a,v,g,h,b=[self.messages[n] for n in ('ATTITUDE','LOCAL_POSITION_NED','GLOBAL_POSITION_INT','VFR_HUD','HEARTBEAT')]
                s=self.messages.get('SERVO_OUTPUT_RAW'); nav=self.messages.get('NAV_CONTROLLER_OUTPUT')
                received=dict(self.received); message_clock=dict(self.message_clock); phase=self.phase
        forward,left,up=body_velocity(v.vx,v.vy,v.vz,a.roll,a.pitch,a.yaw)
        row=dict(wall_s=time.monotonic()-self.start_wall,sim_s=a.time_boot_ms/1000,phase=phase,
                 velocity_sim_s=v.time_boot_ms/1000.,airspeed_sim_s=message_clock.get('VFR_HUD',a.time_boot_ms/1000.),
                 mode=mavutil.mode_string_v10(b),armed=bool(b.base_mode & 128),east=v.y+self.home_offset[0],north=v.x+self.home_offset[1],
                 alt=g.relative_alt/1000,vn=v.vx,ve=v.vy,vd=v.vz,forward=forward,left=left,up=up,
                 heading_forward=v.vx*math.cos(a.yaw)+v.vy*math.sin(a.yaw),
                 heading_left=v.vx*math.sin(a.yaw)-v.vy*math.cos(a.yaw),
                 airspeed=h.airspeed,groundspeed=h.groundspeed,roll=a.roll,pitch=a.pitch,yaw=a.yaw,
                 p=a.rollspeed,q=a.pitchspeed,r=a.yawspeed,throttle=h.throttle,
                 target_bank=nav.nav_roll if nav else None,
                 attitude_age=time.monotonic()-received['ATTITUDE'],
                 velocity_age=time.monotonic()-received['LOCAL_POSITION_NED'],
                 airspeed_age=time.monotonic()-received['VFR_HUD'])
        row.update({'servo%d'%i:getattr(s,'servo%d_raw'%i,None) for i in range(1,5)})
        row.update({name:target.get(name) for name in ('ref_speed','ref_accel','ref_course','ref_east','ref_north','ref_bank',
            'sent_ve','sent_vn','sent_ae','sent_an','sent_yaw','sent_yaw_rate','sent_mask','ref_path_course')})
        return row

    def set_phase(self,name):
        if not isinstance(name,str) or not name:
            raise ValueError('phase must be a nonempty string')
        with getattr(self,'telemetry_lock',nullcontext()):
            if name==self.phase:
                return
            previous=self.phase;self.phase=name
            attitude=getattr(self,'messages',{}).get('ATTITUDE')
        if self.events is not None:
            boot_s=attitude.time_boot_ms/1000. if attitude is not None else None
            with getattr(self,'evidence_lock',nullcontext()):
                self.events.write(json.dumps(dict(wall_s=time.monotonic()-self.start_wall,
                    event='phase_transition',previous=previous,phase=name,boot_s=boot_s))+'\n')

    def mode(self,name):
        mode_id=self.link.mode_mapping()[name]
        with self.command_lock:
            self.link.mav.set_mode_send(1,1,mode_id)
        self.wait(lambda:mavutil.mode_string_v10(self.messages['HEARTBEAT'])==name,10,'mode '+name)

    def arm(self):
        deadline=time.monotonic()+70
        while not (self.messages['HEARTBEAT'].base_mode&128):
            with self.command_lock:
                self.link.mav.command_long_send(1,1,400,0,1,0,0,0,0,0,0)
            end=time.monotonic()+2
            while time.monotonic()<end:
                self.pump()
            if time.monotonic()>deadline:
                raise TimeoutError('normal arming; inspect STATUSTEXT')

    def takeoff(self):
        self.phase='takeoff'
        self.mode('TAKEOFF' if self.vehicle=='plane' else 'GUIDED')
        self.arm()
        if self.vehicle=='copter':
            with self.command_lock:
                self.link.mav.command_long_send(1,1,22,0,0,0,0,0,0,0,self.altitude)
        self.wait(lambda:self.row()['alt']>self.altitude-2.,90,'climb')
        self.mode('GUIDED')
        self.phase='settle'

    def plane_reference(self,speed,course_deg,accel=2.,bank=None,*,lateral_mode=None,altitude=None):
        if not all(math.isfinite(v) for v in (speed,course_deg,accel)) or speed<=0 or accel<=0:
            raise ValueError('invalid Plane command')
        if bank is not None and (not math.isfinite(bank) or abs(bank)>math.radians(60)):
            raise ValueError('invalid external roll')
        if lateral_mode is not None:
            if lateral_mode not in ('course','external_roll') or ((bank is None)!=(lateral_mode=='course')):
                raise ValueError('choose exactly one lateral controller')
            if self.plane_lateral_mode not in (None,lateral_mode):
                raise ValueError('restart flight before switching lateral controller')
            self.plane_lateral_mode=lateral_mode
        height=self.altitude if altitude is None else altitude
        if not math.isfinite(height) or height<=0:
            raise ValueError('invalid target altitude')
        with getattr(self,'command_lock',nullcontext()):
            self.target.update(ref_speed=speed,ref_course=course_deg)
            # The external reference already contains the speed ramp. Zero disables
            # a second firmware slew limiter whose timestamp resets on each packet.
            for code,p1,p2,p3,z in ((43000,0,speed,0.,0),(186,height,3,0,0),(43002,0,course_deg%360,accel,0)):
                schedule=getattr(self,'heading_schedule',None)
                if code==43002 and schedule is not None:
                    with getattr(self,'telemetry_lock',nullcontext()):
                        attitude_boot_s=self.messages['ATTITUDE'].time_boot_ms/1000.
                    if not schedule.due(attitude_boot_s,course_deg,accel):
                        continue
                    if self.heading_pending_since is None:
                        self.heading_pending_since=time.monotonic()
                self.link.mav.command_int_send(1,1,3,code,0,0,p1,p2,p3,0,0,0,z)
                self.command_first_sent.setdefault(code,time.monotonic())
                if code==43002 and getattr(self,'events',None) is not None:
                    with getattr(self,'telemetry_lock',nullcontext()):
                        sim_s=self.messages['ATTITUDE'].time_boot_ms/1000.
                    with getattr(self,'evidence_lock',nullcontext()):
                        self.events.write(json.dumps(dict(event='heading_sent',wall_s=time.monotonic()-self.start_wall,
                            sim_s=sim_s,bearing_deg=p2,accel_limit=p3))+'\n')
            if bank is not None:
                # This ArduPlane revision interprets mask 254 as quaternion roll only.
                # Pitch and throttle remain with TECS; see GCS_MAVLink_Plane.cpp:1043.
                self.target['ref_bank']=bank
                self.link.mav.set_attitude_target_send(0,1,1,254,[math.cos(bank/2),math.sin(bank/2),0,0],0,0,0,0)

    def check_plane_commands(self, timeout=2.):
        now=time.monotonic()
        for code,first in self.command_first_sent.items():
            ack=self.command_acks.get(code)
            if ack is not None and ack[0]>=first and ack[1]!=0:
                raise RuntimeError('Plane rejected command %d: %d'%(code,ack[1]))
            if code==43002 and getattr(self,'heading_schedule',None) is not None:
                # An acknowledged heading persists in this firmware. Lack of a
                # new ACK for an intentionally unsent target is not link loss.
                pending=self.heading_pending_since
                if pending is not None and now-pending>timeout:
                    raise RuntimeError('Plane command ACK timeout: %d'%code)
                continue
            if now-first>timeout and (ack is None or ack[0]<first or now-ack[0]>timeout):
                raise RuntimeError('Plane command ACK timeout: %d'%code)

    def copter_reference(self,east,north,ve,vn,yaw=math.pi/2,accel=None,*,yaw_rate=None):
        if yaw_rate is not None and not math.isfinite(yaw_rate):
            raise ValueError('nonfinite yaw rate')
        with getattr(self,'command_lock',nullcontext()):
            self.target.update(ref_speed=math.hypot(ve,vn),ref_course=math.degrees(yaw),ref_east=east,ref_north=north)
            ae,an=(0.,0.) if accel is None else accel
            mask=64+128+256+2048 if accel is None else 2048
            if yaw_rate is not None:
                mask &= ~2048
            self.target.update(sent_ve=ve,sent_vn=vn,sent_ae=ae if accel is not None else None,
                               sent_an=an if accel is not None else None,sent_yaw=yaw,
                               sent_yaw_rate=yaw_rate,sent_mask=mask)
            self.link.mav.set_position_target_local_ned_send(0,1,1,1,mask,
                north-self.home_offset[1],east-self.home_offset[0],-self.altitude,vn,ve,0,an,ae,0,yaw,
                0 if yaw_rate is None else yaw_rate)

    def finish(self):
        self.set_phase('exit')
        if self.link and 'HEARTBEAT' in self.messages and self.messages['HEARTBEAT'].base_mode&128:
            self.mode('LOITER' if self.vehicle=='plane' else 'LAND')
            if self.vehicle=='copter':
                self.wait(lambda:not self.messages['HEARTBEAT'].base_mode&128,100,'landing')
            else:
                start=self.row()['sim_s']
                self.wait(lambda:self.row()['sim_s']-start>5,15,'loiter exit observation')

    def close(self):
        receiver_error=None
        try:
            self.stop_telemetry_receiver()
        except Exception as exc:
            receiver_error=exc
        if self.link:
            if self.link.logfile:
                self.link.logfile.close()
            self.link.close()
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait()
        for name in ('log','csv','events','send_log'):
            stream=getattr(self,name,None)
            if stream:
                stream.close()
        if self.gcs_process is not None:
            self.gcs_process.terminate()
            try:
                self.gcs_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.gcs_process.kill(); self.gcs_process.wait()
            self.gcs_process=None
        if self.gcs_log is not None:
            self.gcs_log.close(); self.gcs_log=None
        if receiver_error is not None:
            raise receiver_error
