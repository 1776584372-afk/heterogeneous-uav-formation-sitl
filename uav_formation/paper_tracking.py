"""Shared Plane path/course feedback and the separate legacy external-roll branch."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PlaneCourseCommand:
    airspeed: float
    bearing_deg: float
    altitude: float
    lateral_accel_limit: float
    cross_track: float
    along_track: float
    requested_airspeed: float
    speed_saturated: bool
    preview_x: float
    preview_y: float


def course_command(member, reference, position, lookahead=40., progress_gain=.3,
                   guidance_mode='preview', cross_track_gain=1.,
                   course_to_bank_gain=math.radians(50.)):
    """Position feedback to a chord preview; ENU input, compass bearing output.

    Reference is (x,y,tangent,speed,curvature,altitude[,preview_x,preview_y]).
    A full-path preview is preferred; six-field callers use the osculating arc.
    The common reference is never modified by along-track speed correction.
    """
    if member.platform != 'fixedwing':
        raise ValueError('fixedwing member required')
    if (len(reference) not in (6,8) or len(position)!=2
            or not all(math.isfinite(v) for v in (*reference,*position,lookahead,
                                                  progress_gain,cross_track_gain,
                                                  course_to_bank_gain))
            or lookahead<=0 or progress_gain<0 or cross_track_gain<=0
            or course_to_bank_gain<=0
            or guidance_mode not in ('preview','frenet','curvature_ff')):
        raise ValueError('finite reference, position and valid guidance gains required')
    x,y,psi,speed,curvature,alt=reference[:6]
    if not member.v_min<=speed<=member.v_max or alt<=0:
        raise ValueError('reference outside operating speed/altitude bounds')
    limit=min(member.a_lat_max,9.80665*math.tan(member.bank_max))
    if speed*speed*abs(curvature)>limit+1e-6:
        raise ValueError('reference outside lateral/bank envelope')
    cross=-(position[0]-x)*math.sin(psi)+(position[1]-y)*math.cos(psi)
    along=(x-position[0])*math.cos(psi)+(y-position[1])*math.sin(psi)
    if len(reference)==8:
        px,py=reference[6:]
    else:
        half=.5*curvature*lookahead
        chord=lookahead*(math.sin(half)/half if abs(half)>1e-8 else 1.)
        px,py=x+chord*math.cos(psi+half),y+chord*math.sin(psi+half)
    if guidance_mode == 'preview':
        # Legacy chord-preview guidance. Keep it available so controller changes
        # can be compared against the exact historical implementation.
        forward=(px-position[0])*math.cos(psi)+(py-position[1])*math.sin(psi)
        lateral=-(px-position[0])*math.sin(psi)+(py-position[1])*math.cos(psi)
        desired=psi+math.atan2(lateral,max(.25*lookahead,forward))
    else:
        # A path tangent is an equilibrium command on the path. Cross-track
        # feedback rotates that tangent toward the path without the systematic
        # half-chord lead produced by pure pursuit on a circular reference.
        curvature_lead=0.
        if guidance_mode == 'curvature_ff':
            # ArduPlane's GUIDED course loop converts course error to bank. Give
            # it the coordinated-turn bank as an explicit course-error bias;
            # cross-track feedback then removes model and sideslip residuals.
            required_bank=math.atan2(speed*speed*curvature,9.80665)
            curvature_lead=required_bank/course_to_bank_gain
        desired=psi+curvature_lead-math.atan2(cross_track_gain*cross,lookahead)
    requested=speed+progress_gain*along
    airspeed=max(member.v_min,min(member.v_max,requested))
    return PlaneCourseCommand(airspeed,(90.-math.degrees(desired))%360.,alt,limit,
                              cross,along,requested,abs(airspeed-requested)>1e-9,px,py)


class PlaneCourseTracker:
    """Stateful command slew only; actual acceleration remains a measured metric."""
    def __init__(self, member, lookahead=40., progress_gain=.3, command_accel_scale=1.,
                 guidance_mode='preview', cross_track_gain=1.,
                 course_to_bank_gain=math.radians(50.), *,
                 initial_command_airspeed=None):
        self.member=member
        self.lookahead=lookahead
        self.progress_gain=progress_gain
        if guidance_mode not in ('preview','frenet','curvature_ff'):
            raise ValueError('unknown fixed-wing guidance mode')
        if not math.isfinite(cross_track_gain) or cross_track_gain<=0:
            raise ValueError('cross-track gain must be finite and positive')
        if not math.isfinite(course_to_bank_gain) or course_to_bank_gain<=0:
            raise ValueError('course-to-bank gain must be finite and positive')
        self.guidance_mode=guidance_mode
        self.cross_track_gain=cross_track_gain
        self.course_to_bank_gain=course_to_bank_gain
        if not math.isfinite(command_accel_scale) or not 0 < command_accel_scale <= 1:
            raise ValueError('command_accel_scale must be in (0,1]')
        self.command_accel_scale=command_accel_scale
        if initial_command_airspeed is not None and (
                not math.isfinite(initial_command_airspeed)
                or not member.v_min<=initial_command_airspeed<=member.v_max):
            raise ValueError('handover command airspeed must be finite and inside member bounds')
        # A live handover retains the last transmitted target, not a noisy
        # measurement. None preserves the legacy measured-speed initialization.
        self.previous_speed=initial_command_airspeed

    def update(self, reference, position, measured_airspeed, dt):
        from dataclasses import replace
        if not (math.isfinite(dt) and 0<dt<=1. and math.isfinite(measured_airspeed)
                and measured_airspeed>0):
            raise ValueError('invalid guidance timing/airspeed')
        command=course_command(self.member,reference,position,self.lookahead,self.progress_gain,
                               self.guidance_mode,self.cross_track_gain,
                               self.course_to_bank_gain)
        previous=(max(self.member.v_min,min(self.member.v_max,measured_airspeed))
                  if self.previous_speed is None else self.previous_speed)
        lower=self.member.a_t_min*self.command_accel_scale
        upper=self.member.a_t_max*self.command_accel_scale
        speed=previous+max(lower*dt,min(upper*dt,command.airspeed-previous))
        self.previous_speed=speed
        return replace(command,airspeed=speed)


def advance_unicycle(state, speed, yaw_rate, dt):
    """Advance the previous held input to the current reference timestamp."""
    x,y,psi=state
    angle=yaw_rate*dt
    half=angle/2
    factor=math.sin(half)/half if abs(half)>1e-10 else 1.
    distance=speed*dt*factor
    return (x+distance*math.cos(psi+half),y+distance*math.sin(psi+half),psi+angle)


def synchronous_copter_target(measured_state, speed, yaw_rate, elapsed):
    """Propagate one measured solve anchor to the sending time, never beyond it."""
    if (not all(math.isfinite(x) for x in (*measured_state,speed,yaw_rate,elapsed))
            or elapsed < 0 or elapsed > 1.):
        raise ValueError('invalid measured anchor or command age')
    x,y,yaw=advance_unicycle(measured_state,speed,yaw_rate,elapsed)
    return (x,y,yaw), (speed*math.cos(yaw),speed*math.sin(yaw))


class CopterExecutionGovernor:
    """Continuous command trajectory only; never supplies NMPC measured X0.

    This development adapter smooths the optimized input after the solve. Its
    margins are command reserves, not a certificate of actual aircraft motion.
    """
    def __init__(self, member, measured_state, measured_speed, reserve=.5, yaw_accel=.1):
        if (member.platform!='multirotor' or member.yaw_rate_max is None
                or not 0<reserve<=1 or not math.isfinite(yaw_accel) or yaw_accel<=0
                or len(measured_state)!=3 or not all(math.isfinite(x) for x in (*measured_state,measured_speed))):
            raise ValueError('valid multirotor measurement and command reserve required')
        self.member=member;self.state=tuple(measured_state)
        self.speed=max(member.v_min,min(member.v_max,measured_speed));self.rate=0.
        self.reserve=reserve;self.yaw_accel=yaw_accel

    def update(self, speed, yaw_rate, dt):
        if not all(math.isfinite(x) for x in (speed,yaw_rate,dt)) or not 0<dt<=.5:
            raise ValueError('finite input and bounded command interval required')
        m=self.member;old_speed=self.speed;old_rate=self.rate
        requested=max(m.v_min,min(m.v_max,speed))
        self.speed+=max(self.reserve*m.a_t_min*dt,min(self.reserve*m.a_t_max*dt,requested-self.speed))
        rate_limit=self.reserve*min(m.yaw_rate_max,m.a_lat_max/max(self.speed,.1))
        requested_rate=max(-rate_limit,min(rate_limit,yaw_rate))
        self.rate+=max(-self.yaw_accel*dt,min(self.yaw_accel*dt,requested_rate-self.rate))
        # Recheck the rate limit if the speed-dependent lateral bound tightens.
        self.rate=max(-rate_limit,min(rate_limit,self.rate))
        self.state=advance_unicycle(self.state,.5*(old_speed+self.speed),.5*(old_rate+self.rate),dt)
        yaw=self.state[2];c,s=math.cos(yaw),math.sin(yaw)
        tangential=(self.speed-old_speed)/dt
        return dict(state=self.state,speed=self.speed,yaw_rate=self.rate,
                    velocity=(self.speed*c,self.speed*s),
                    acceleration=(tangential*c-self.speed*self.rate*s,tangential*s+self.speed*self.rate*c))


class CopterVelocityFilter:
    """Development-only velocity output smoothing; common reference is unchanged."""
    def __init__(self, member, velocity, time_constant):
        if (member.platform!='multirotor' or not math.isfinite(time_constant) or time_constant<=0
                or len(velocity)!=2 or not all(math.isfinite(v) for v in velocity)):
            raise ValueError('valid multirotor velocity and filter time constant required')
        self.member=member;self.velocity=tuple(velocity);self.time_constant=time_constant

    def update(self, requested, dt):
        if len(requested)!=2 or not all(math.isfinite(v) for v in (*requested,dt)) or not 0<dt<=.5:
            raise ValueError('finite velocity and bounded interval required')
        old=self.velocity;alpha=-math.expm1(-dt/self.time_constant)
        candidate=tuple(v+alpha*(target-v) for v,target in zip(old,requested))
        speed=math.hypot(*old);heading=math.atan2(old[1],old[0])
        m=self.member
        new_speed=max(m.v_min,min(m.v_max,speed+max(.5*m.a_t_min*dt,
                      min(.5*m.a_t_max*dt,math.hypot(*candidate)-speed))))
        requested_heading=math.atan2(candidate[1],candidate[0])
        delta=math.atan2(math.sin(requested_heading-heading),math.cos(requested_heading-heading))
        max_delta=.8*m.a_lat_max/max(speed,new_speed,.1)*dt
        heading+=max(-max_delta,min(max_delta,delta))
        self.velocity=(new_speed*math.cos(heading),new_speed*math.sin(heading))
        return self.velocity,tuple((new-prev)/dt for new,prev in zip(self.velocity,old))


class CopterReferenceCorrection:
    """Bounded C1 position correction driven by NMPC without changing fleet progress.

    The emitted PPVA triplet is the geometric route plus one integrated offset.
    The correction bounds are execution settings, never aircraft capability
    evidence or a guarantee about the realized motion.
    """
    def __init__(self, max_offset=1., max_rate=.2, max_accel=.1, response=.5):
        parameters=(max_offset,max_rate,max_accel,response)
        if not all(math.isfinite(x) and x>0 for x in parameters):
            raise ValueError('positive finite correction settings required')
        self.max_offset=max_offset
        self.max_rate=max_rate
        self.max_accel=max_accel
        self.response=response
        self.offset=(0.,0.)
        self.rate=(0.,0.)

    @staticmethod
    def limit(vector, magnitude):
        length=math.hypot(*vector)
        scale=min(1.,magnitude/length) if length else 1.
        return tuple(float(value*scale) for value in vector)

    def update(self, reference, requested_position, dt):
        """Reference: (x,y,psi,v,kappa,a_t); returned yaw remains path tangent."""
        if (len(reference)!=6 or len(requested_position)!=2
                or not all(math.isfinite(x) for x in (*reference,*requested_position,dt))
                or not 0<dt<=.5 or reference[3]<0):
            raise ValueError('finite route, request and bounded tick required')
        x,y,psi,speed,curvature,tangential=reference
        stopping_distance=self.max_rate*self.max_rate/(2*self.max_accel)
        if stopping_distance >= self.max_offset:
            raise ValueError('correction offset must exceed worst-case stopping distance')
        desired=self.limit((requested_position[0]-x,requested_position[1]-y),
                           self.max_offset-stopping_distance)
        omega=self.response
        a_requested=tuple(omega*omega*(desired[i]-self.offset[i])-2*omega*self.rate[i]
                          for i in range(2))
        acceleration=self.limit(a_requested,self.max_accel)
        previous=self.rate
        self.rate=self.limit(tuple(previous[i]+acceleration[i]*dt for i in range(2)),self.max_rate)
        # Recompute the actual applied acceleration after a possible rate cap.
        acceleration=tuple((self.rate[i]-previous[i])/dt for i in range(2))
        next_offset=tuple(self.offset[i]+.5*(previous[i]+self.rate[i])*dt for i in range(2))
        if math.hypot(*next_offset)>self.max_offset+1e-9:
            raise ValueError('correction offset envelope exceeded')
        self.offset=next_offset
        c,s=math.cos(psi),math.sin(psi)
        position=(x+self.offset[0],y+self.offset[1],psi)
        velocity=(speed*c+self.rate[0],speed*s+self.rate[1])
        accel=(tangential*c-speed*speed*curvature*s+acceleration[0],
               tangential*s+speed*speed*curvature*c+acceleration[1])
        return position,velocity,accel,-speed*curvature


def member_reference(route, profile, s, r_y, terminal_elapsed=0.):
    """Member kinematics, with straight flythrough extension after route end.

    terminal_elapsed is elapsed task time beyond the profile, not a request
    to stop at the final position while retaining nonzero velocity.
    """
    if not math.isfinite(terminal_elapsed) or terminal_elapsed < 0:
        raise ValueError('nonnegative finite terminal elapsed time required')
    x,y,psi,_=route.eval_drone(s,0.,r_y)
    k=route.curvature_at(s); kp=route.curvature_derivative_at(s)
    if terminal_elapsed > 0:
        if abs(s-route.total_length())>1e-6 or abs(k)>1e-8 or abs(kp)>1e-8:
            raise ValueError('flythrough extension requires a straight route endpoint')
        speed=float(profile.nu[-1])
        distance=speed*terminal_elapsed
        return (x+distance*math.cos(psi),y+distance*math.sin(psi),psi,speed,0.,0.)
    nu=profile.nu_at(s); q=1-r_y*k
    if q<=0:
        raise ValueError('invalid slot geometry')
    at=.5*q*profile.slope_at(s)-r_y*kp*nu*nu
    return x,y,psi,nu*q,k/q,at


def preview_member(route, profile, s, r_y, seconds, flythrough):
    future_time=profile.time_at(s)+seconds
    if flythrough and future_time>profile.time[-1]:
        x,y,psi,_=route.eval_drone(route.total_length(),0.,r_y)
        distance=(future_time-profile.time[-1])*profile.nu[-1]
        return (x+distance*math.cos(psi),y+distance*math.sin(psi),psi),float(profile.nu[-1])
    future_s=profile.advance(s,seconds)
    x,y,psi,_=route.eval_drone(future_s,0.,r_y)
    speed=profile.nu_at(future_s)*(1-r_y*route.curvature_at(future_s))
    return (x,y,psi),speed


def plane_command(member, reference, actual, lookahead=40., course_gain=1.6, progress_gain=.3):
    x,y,psi,speed,curvature=reference
    east,north,vn,ve,groundspeed=actual
    if not all(math.isfinite(v) for v in (*reference,*actual)) or lookahead<=0:
        raise ValueError('finite reference/state and positive lookahead required')
    cross=-(east-x)*math.sin(psi)+(north-y)*math.cos(psi)
    along=(x-east)*math.cos(psi)+(y-north)*math.sin(psi)
    course=math.atan2(vn,ve)
    desired=psi-math.atan2(cross,lookahead)
    error=math.atan2(math.sin(desired-course),math.cos(desired-course))
    acceleration=groundspeed**2*curvature+course_gain*groundspeed*error
    bank=max(-member.bank_max,min(member.bank_max,-math.atan2(acceleration,9.80665)))
    airspeed=max(member.v_min,min(member.v_max,speed+progress_gain*along))
    return airspeed,(90-math.degrees(desired))%360,bank
