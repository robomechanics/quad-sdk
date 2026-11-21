"""Go2 geometry, fixed-tempo step references, and contact-event measurements."""
from dataclasses import dataclass
import math
import torch

FOOTS=('FL','FR','RL','RR')
RADIUS=.022
HIP_X=(.1934,.1934,-.1934,-.1934)
HIP_Y=(.0465,-.0465,.0465,-.0465)
SIDE=(1.,-1.,1.,-1.)


def smooth(x):
    x=x.clamp(0,1)
    return x*x*x*(10+x*(-15+6*x))


def timing_valid(error,tolerance):
    """Inclusive contact window with 10 us float32 clock roundoff allowance."""
    return error.abs()<=tolerance+1e-5


@dataclass
class GaitConfig:
    bpm: float=90.
    swing_seconds: float=.30
    clearance: float=.04
    leg_height: float=.30  # hip plane to foot centre; body height includes sole radius
    residual_scale: float=.12
    max_speed: float=.15
    max_yaw: float=.25
    timing_tolerance: float=.10
    kick_lateral: float=.08
    kick_height: float=.05
    kick_duration: float=.25
    kick_hold: float=.75
    bounce_amplitude: float=0.
    mode: str='walk'
    load_compensation: float=0.
    attitude_compensation: float=0.
    velocity_reward_std: float=.25
    yaw_reward_std: float=.4
    training_zero_fraction: float=.3
    training_backward_fraction: float=.5  # among nonzero sampled translations
    training_lateral_scale: float=1.  # training only; 0 for restricted forward demo
    training_turn_fraction: float=0.  # pure yaw among nonzero commands
    balanced_phrases: bool=False
    wrench_compensation: float=0.
    freeze_hold_phase: bool=False
    support_velocity_damping: float=0.
    support_angular_damping: float=2.
    recovery_strategy: str='land_and_rejoin'
    extended_hold_fraction: float=0.
    hold_angular_std: float=0.  # zero preserves the original endpoint-only reward
    support_com_x: float=-.025  # historical calibration; selected settings override
    support_com_z: float=0.
    kick_shift_back: float=0.
    kick_shift_side: float=0.
    integrated_walk_reference: bool=False
    linear_acceleration_limit: float=0.  # zero preserves instantaneous command updates
    yaw_acceleration_limit: float=0.
    exclude_pause_tracking: bool=False  # opt-in, preserve archived experiments
    estimated_feedback: bool=False  # same flat-ground encoder/IMU feedback as deployment
    training_command_period: float=0.  # mean seconds; sampled in [0.75,1.25]*period
    strict_kick_timing_reward: bool=False
    kick_tracking_std: float=.03  # reward only; preserve legacy endpoint shaping

    @property
    def beat(self):return 60/self.bpm
    @property
    def cycle(self):return 2*self.beat
    def __post_init__(self):
        if self.bpm!=90 or not 0<self.swing_seconds<self.beat:
            raise ValueError('CPG uses fixed 90 BPM and nonoverlapping diagonal swings')
        if not 0<self.max_speed<=.5 or not 0<self.clearance<=.10 or not 0<self.residual_scale<=.3:
            raise ValueError('Invalid CPG bounds')
        if not 0<=self.bounce_amplitude<=.0075 or not 0<=self.training_zero_fraction<=1:
            raise ValueError('Bounce/command curriculum exceeds its specification')
        if self.velocity_reward_std<=0 or self.yaw_reward_std<=0 or self.max_yaw<0:
            raise ValueError('Invalid tracking scale or yaw bound')
        if (self.kick_lateral,self.kick_height,self.kick_duration)!=(.08,.05,.25):
            raise ValueError('Kick geometry and extension duration are fixed by the motion specification')
        if not 0<=self.wrench_compensation<=1 or not 0<=self.load_compensation<=1:
            raise ValueError('Invalid load compensation interpolation')
        if self.support_velocity_damping<0 or self.support_angular_damping<0 or self.recovery_strategy not in ('land_and_rejoin','land_and_stop','hold_and_stop'):
            raise ValueError('Invalid support damping or recovery strategy')
        if not 0<=self.extended_hold_fraction<=1 or self.hold_angular_std<0:
            raise ValueError('Invalid extended-hold curriculum fraction')
        if not 0<=self.kick_shift_back<=.06 or not 0<=self.kick_shift_side<=.04:
            raise ValueError('Kick preparation shift exceeds its bounded range')
        if self.linear_acceleration_limit<0 or self.yaw_acceleration_limit<0:
            raise ValueError('Command acceleration bounds cannot be negative')
        if not 0<=self.training_backward_fraction<=1 or not 0<=self.training_turn_fraction<=1:
            raise ValueError('Invalid directional training curriculum fraction')
        if not 0<=self.training_lateral_scale<=1:
            raise ValueError('Invalid lateral curriculum scale')
        if self.training_command_period<0:
            raise ValueError('Command period cannot be negative')
        if not math.isfinite(self.kick_tracking_std) or self.kick_tracking_std<=0:
            raise ValueError('Kick tracking scale must be finite and positive')


def training_direction_bias(velocity,cfg):
    """Training-only distribution; external joystick commands remain unchanged."""
    if cfg.training_backward_fraction!=.5:
        backward=torch.rand(len(velocity),device=velocity.device)<cfg.training_backward_fraction
        velocity[:,0]=velocity[:,0].abs()*torch.where(backward,-1.,1.)
    velocity[:,1]*=cfg.training_lateral_scale
    if cfg.training_turn_fraction:
        turn=torch.rand(len(velocity),device=velocity.device)<cfg.training_turn_fraction
        velocity[turn,:2]=0.
    return velocity


def advance_velocity(current,target,dt,cfg):
    """Bound operator-command acceleration without changing the musical clock."""
    delta=target-current
    xy=delta[:,:2]
    if cfg.linear_acceleration_limit:
        xy=xy*(cfg.linear_acceleration_limit*dt/xy.norm(dim=-1,keepdim=True).clamp_min(1e-9)).clamp_max(1)
    yaw=delta[:,2]
    if cfg.yaw_acceleration_limit:
        yaw=yaw.clamp(-cfg.yaw_acceleration_limit*dt,cfg.yaw_acceleration_limit*dt)
    return current+torch.cat((xy,yaw[:,None]),-1)


def fk(q):
    """URDF forward kinematics, q shape (...,4,3), foot centre in base frame."""
    h,t,k=q.unbind(-1)
    side=q.new_tensor(SIDE)
    x=-.213*(t.sin()+(t+k).sin())
    z=-.213*(t.cos()+(t+k).cos())
    y=.0955*side
    return torch.stack((x+q.new_tensor(HIP_X),h.cos()*y-h.sin()*z+q.new_tensor(HIP_Y),h.sin()*y+h.cos()*z),-1)


def ik(feet):
    """Analytic negative-knee solution with numerical domain protection.

    Callers enforce real articulation soft joint limits and report saturation.
    Reachability error is checked by FK rather than silently called feasible.
    """
    x=feet[...,0]-feet.new_tensor(HIP_X)
    y=feet[...,1]-feet.new_tensor(HIP_Y)
    z=feet[...,2]
    yz=-torch.sqrt((y*y+z*z-.0955**2).clamp_min(1e-8))
    h=torch.atan2(z,y)-torch.atan2(yz,feet.new_tensor(SIDE)*.0955)
    h=torch.atan2(h.sin(),h.cos())
    knee=-torch.acos(((x*x+yz*yz-2*.213**2)/(2*.213**2)).clamp(-1+1e-6,1-1e-6))
    thigh=torch.atan2(-x,-yz)-torch.atan2(.213*knee.sin(),.213*(1+knee.cos()))
    return torch.stack((h,thigh,knee),-1)


def gravity_position_offset(q, force, stiffness):
    """Position bias -J^T F/Kp to compensate stance load through position PD.

    Force is the desired ground reaction on each foot in the base frame. This is
    a compliant-actuator correction after geometric IK, not a different IK map.
    """
    h,t,k=q.unbind(-1)
    y=q.new_tensor(SIDE)*.0955
    z=-.213*(t.cos()+(t+k).cos())
    dz=.213*(t.sin()+(t+k).sin())
    dzk=.213*(t+k).sin()
    jh=torch.stack((torch.zeros_like(h),-h.sin()*y-h.cos()*z,h.cos()*y-h.sin()*z),-1)
    jt=torch.stack((z,-h.sin()*dz,h.cos()*dz),-1)
    jk=torch.stack((-.213*(t+k).cos(),-h.sin()*dzk,h.cos()*dzk),-1)
    return -torch.stack(((jh*force).sum(-1),(jt*force).sum(-1),(jk*force).sum(-1)),-1)/stiffness


def foot_velocity(q,qd):
    """Encoder-derived foot velocity relative to the base, J(q) qdot."""
    h,t,k=q.unbind(-1);hd,td,kd=qd.unbind(-1)
    y=q.new_tensor(SIDE)*.0955
    z=-.213*(t.cos()+(t+k).cos())
    dz=.213*(t.sin()*td+(t+k).sin()*(td+kd))
    dx=-.213*(t.cos()*td+(t+k).cos()*(td+kd))
    dy=(-h.sin()*y-h.cos()*z)*hd-h.sin()*dz
    dz_body=(h.cos()*y-h.sin()*z)*hd+h.cos()*dz
    return torch.stack((dx,dy,dz_body),-1)


def nominal_feet(n,device,cfg):
    feet=torch.zeros(n,4,3,device=device)
    feet[:,:,0]=feet.new_tensor(HIP_X)
    feet[:,:,1]=feet.new_tensor(HIP_Y)+feet.new_tensor(SIDE)*.0955
    feet[:,:,2]=-cfg.leg_height
    return feet


def estimated_foot_feedback(q,qd,gyro,gravity,contact,previous_height):
    """Flat-ground support kinematics, matching deploy/cpg/runtime.py.

    Returns ground-relative sole clearance, body-frame foot velocity, height.
    Median averages the central two samples for four/two support feet, as NumPy.
    No truth root pose, root velocity, or simulator foot state is used.
    """
    feet=fk(q)
    relative=foot_velocity(q,qd)+torch.cross(gyro[:,None,:].expand_as(feet),feet,dim=-1)
    count=contact.sum(-1)
    body_velocity=-(relative*contact[:,:,None]).sum(1)/count.clamp_min(1)[:,None]
    vertical=(feet*(-gravity[:,None,:])).sum(-1)
    candidates=torch.where(contact,.022-vertical,torch.full_like(vertical,float('inf'))).sort(-1).values
    lo=((count-1).clamp_min(0)//2)[:,None];hi=(count//2)[:,None]
    height=.5*(candidates.gather(1,lo).squeeze(1)+candidates.gather(1,hi).squeeze(1))
    height=torch.where(count>0,height,previous_height)
    return vertical+height[:,None]-.022,relative+body_velocity[:,None,:],height


def schedule(time,cfg):
    """Walking scheduler: each beat targets a touchdown, alternating pairs.

    Not a choreography oscillator: scheduler owns admission and can inhibit a
    dispatched swing. Walking is the first configurable curriculum stage.
    """
    beat_index=torch.floor(time/cfg.beat).long()
    within=time-beat_index*cfg.beat
    selected=(torch.arange(4,device=time.device)[None,:]==(beat_index%2)[:,None]) | (torch.arange(4,device=time.device)[None,:]==(3-beat_index%2)[:,None])
    swing=within>=cfg.beat-cfg.swing_seconds
    return beat_index,within,selected,swing


def walking_reference(time,velocity,cfg):
    n=len(time);base=nominal_feet(n,time.device,cfg)
    # Both diagonals share a fixed clock; speed changes only Cartesian displacement.
    offsets=time.new_tensor([cfg.beat,0,0,cfg.beat])
    phase=(time[:,None]+offsets).remainder(cfg.cycle)
    stance=cfg.cycle-cfg.swing_seconds
    u=((phase-stance)/cfg.swing_seconds).clamp(0,1)
    swing=phase>=stance
    stance_path=.5*stance-phase
    swing_path=cfg.cycle*smooth(u)-cfg.swing_seconds*u-.5*stance
    path=torch.where(swing,swing_path,stance_path)
    foot_velocity=velocity[:,None,:2]+velocity[:,None,2:3]*torch.stack((-base[:,:,1],base[:,:,0]),-1)
    base[:,:,:2]+=foot_velocity*path[:,:,None]
    clear=cfg.clearance*torch.sin(math.pi*u).square()*swing
    base[:,:,2]+=clear
    base[:,:,2]-=cfg.bounce_amplitude*torch.sin(2*math.pi*time/cfg.beat)[:,None]
    return base,clear,swing


class WalkingReference:
    """Continuous stance integration with immutable goals for dispatched swings.

    Constant commands reproduce walking_reference. A command change adjusts the
    stance velocity and the next swing goal without teleporting planted targets
    or rewriting an airborne foot's endpoint. The external musical clock is kept.
    """
    def __init__(self,n,device,cfg):
        self.cfg=cfg
        self.feet=nominal_feet(n,device,cfg)
        self.initialized=torch.zeros(n,dtype=torch.bool,device=device)
        self.time=torch.zeros(n,device=device)
        self.swing=torch.zeros(n,4,dtype=torch.bool,device=device)
        self.start=self.feet[:,:,:2].clone()
        self.end=self.start.clone()
        self.swing_velocity=torch.zeros_like(self.start)
        self.start_time=torch.zeros(n,4,device=device)

    def reset(self,ids):
        self.initialized[ids]=False
        self.swing[ids]=False
        self.time[ids]=0.

    def update(self,time,velocity,ready):
        g=self.cfg
        nominal=nominal_feet(len(time),time.device,g)
        analytic,clear,swing=walking_reference(time,velocity,g)
        local_v=velocity[:,None,:2]+velocity[:,None,2:3]*torch.stack((-nominal[:,:,1],nominal[:,:,0]),-1)
        phase=(time[:,None]+time.new_tensor([g.beat,0,0,g.beat])).remainder(g.cycle)
        stance=g.cycle-g.swing_seconds
        age=(phase-stance).clamp_min(0)
        dt=(time-self.time).clamp_min(0)[:,None]
        initialize=ready&~self.initialized
        starting=swing&~self.swing&~initialize[:,None]
        # Split the control interval at the scheduled liftoff, retaining the
        # stance-velocity endpoint derivatives of the original quintic curve.
        before=(dt-age).clamp_min(0)
        start=self.feet[:,:,:2]-local_v*before[:,:,None]
        end=nominal[:,:,:2]+.5*stance*local_v
        self.start=torch.where(starting[:,:,None],start,self.start)
        self.end=torch.where(starting[:,:,None],end,self.end)
        self.swing_velocity=torch.where(starting[:,:,None],local_v,self.swing_velocity)
        self.start_time=torch.where(starting,time[:,None]-age,self.start_time)
        init_swing=initialize[:,None]&swing
        self.start=torch.where(init_swing[:,:,None],nominal[:,:,:2]-.5*stance*local_v,self.start)
        self.end=torch.where(init_swing[:,:,None],end,self.end)
        self.swing_velocity=torch.where(init_swing[:,:,None],local_v,self.swing_velocity)
        self.start_time=torch.where(init_swing,time[:,None]-age,self.start_time)
        elapsed=(time[:,None]-self.start_time).clamp(0,g.swing_seconds)
        u=elapsed/g.swing_seconds
        trajectory=self.start+smooth(u)[:,:,None]*(self.end-self.start+self.swing_velocity*g.swing_seconds)-self.swing_velocity*elapsed[:,:,None]
        landed=~swing&self.swing
        after=(time[:,None]-self.start_time-g.swing_seconds).clamp_min(0)
        after=torch.minimum(after,dt)
        stance_xy=torch.where(landed[:,:,None],self.end-local_v*after[:,:,None],self.feet[:,:,:2]-local_v*dt[:,:,None])
        xy=torch.where(swing[:,:,None],trajectory,stance_xy)
        xy=torch.where(initialize[:,None,None],analytic[:,:,:2],xy)
        self.feet[:,:,:2]=torch.where(ready[:,None,None],xy,self.feet[:,:,:2])
        self.feet[:,:,2]=analytic[:,:,2]
        self.swing=swing&ready[:,None]
        self.time=time.clone()
        self.initialized|=ready
        return self.feet.clone(),clear,swing


def kick_reference(start, nominal_y, ground, leg, elapsed, cfg):
    """World-z/body-y endpoint primitive; hold instead of automatic retraction."""
    target=start.clone();rows=torch.arange(len(start),device=start.device)
    signs=start.new_tensor(SIDE)[leg]
    target[rows,leg,1]=nominal_y[rows,leg]+signs*cfg.kick_lateral
    target[rows,leg,2]=ground+cfg.kick_height
    return start+smooth(elapsed/cfg.kick_duration)[:,None,None]*(target-start)


class ContactEvents:
    def __init__(self,n,device):
        self.contact=torch.zeros(n,4,dtype=torch.bool,device=device)
        self.age=torch.zeros(n,4,device=device)
        self.air=torch.zeros_like(self.age)
        self.confirmed=torch.zeros_like(self.contact)
    def reset(self,ids):
        self.contact[ids]=False;self.age[ids]=0;self.air[ids]=0;self.confirmed[ids]=False
    def update(self,force,time,dt):
        previous=self.contact.clone();confirmed=self.confirmed.clone()
        self.contact=torch.where(previous,force>2.,force>=5.)
        self.age=torch.where(self.contact,self.age+dt,0.)
        self.air=torch.where(~self.contact,self.air+dt,0.)
        self.confirmed=self.contact & (self.age>=.04-1e-6)
        touchdown=self.confirmed & ~confirmed
        liftoff=previous & ~self.contact
        onset=time[:,None]-self.age+dt
        return touchdown,liftoff,onset
