"""Simulator-independent CPG controller shared by Isaac Lab and deployment.

The environment facade supplies encoders, IMU and measured/estimated foot feedback.
No Isaac Lab or MuJoCo import belongs here. Foot truth may be used by simulation
adapters for diagnostics; deployment adapters must explicitly identify estimates.
"""
import math
import torch
from .core import (GaitConfig, ContactEvents, nominal_feet, walking_reference,
                   WalkingReference, advance_velocity, training_direction_bias, schedule, ik, fk,
                   gravity_position_offset, foot_velocity, timing_valid, FOOTS,estimated_foot_feedback)
from .dance import DanceReference
from .sensing import SensorPipeline, MeasuredData
from ..skills.preparation import support_forces


def quat_from_euler_xyz(roll,pitch,yaw):
    cr,sr=torch.cos(roll/2),torch.sin(roll/2)
    cp,sp=torch.cos(pitch/2),torch.sin(pitch/2)
    cy,sy=torch.cos(yaw/2),torch.sin(yaw/2)
    return torch.stack((cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
                        cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy),-1)


def quat_apply_inverse(q,v):
    xyz=q[...,1:]
    return v*(2*q[...,:1].square()-1)-2*q[...,:1]*torch.cross(xyz,v,dim=-1)+2*xyz*(xyz*v).sum(-1,keepdim=True)


class CPGController:
    def __init__(self,cfg,env):
        self.cfg=cfg; self._env=env
        self.robot=env.scene['robot'];self.sensor=env.scene['contact_forces']
        self.foot_ids,_=self.robot.find_bodies([f'{x}_foot' for x in FOOTS],preserve_order=True)
        self.sensor_ids,_=self.sensor.find_bodies([f'{x}_foot' for x in FOOTS],preserve_order=True)
        self.joint_ids,_=self.robot.find_joints([f'{x}_{j}_joint' for x in FOOTS for j in ('hip','thigh','calf')],preserve_order=True)
        n=self.num_envs;d=self.device
        self.time=torch.zeros(n,device=d);self.age=torch.zeros_like(self.time)
        self.ready=torch.zeros(n,dtype=torch.bool,device=d)
        self.stable_age=torch.zeros_like(self.time)
        self.velocity=torch.zeros(n,3,device=d);self.external_velocity=False
        self.requested_velocity=torch.zeros_like(self.velocity)
        self.command_age=torch.zeros(n,device=d)
        self.command_next=torch.zeros(n,device=d)
        self.feedback_height=torch.full((n,),.322,device=d)
        self.feedback_speed=torch.zeros(n,4,device=d)
        self.events=ContactEvents(n,d)
        self.active=torch.zeros_like(self.ready);self.recovery=torch.zeros_like(self.ready)
        self.pair=torch.zeros(n,4,dtype=torch.bool,device=d)
        self.lifted=torch.zeros_like(self.pair);self.landed=torch.zeros_like(self.pair)
        self.swing_latch=torch.zeros_like(self.pair)
        self.timed_landing=torch.ones_like(self.pair)
        self.lift_counts=torch.zeros(n,4,device=d)
        self.support_lost=torch.zeros_like(self.ready)
        self.target_time=torch.zeros_like(self.time)
        self.last_beat=torch.full((n,),-1,device=d,dtype=torch.long)
        self.last_swing=torch.zeros_like(self.ready)
        self.last_tick=-1
        self.q_ref=self.robot.data.default_joint_pos.clone()
        self.feet_ref=nominal_feet(n,d,cfg.gait)
        self.clearance=torch.zeros(n,4,device=d);self.expected=torch.ones_like(self.pair)
        self.reward_event=torch.zeros_like(self.time)
        self.pending_skip=torch.zeros_like(self.ready)
        self.touch_error=torch.zeros(n,4,device=d)
        self.touch_valid=torch.zeros_like(self.pair)
        self.lift_valid=torch.zeros_like(self.pair)
        self.last_records={}
        self.metrics={k:torch.zeros(n,device=d) for k in ('attempts','correct','missed','skipped','FL_RR_correct','FR_RL_correct','limit_clamps')}
        self.ik_error=torch.zeros_like(self.time)
        self.mass=self.robot.root_physx_view.get_masses().sum(-1).to(self.device)
        if cfg.gait.estimated_feedback:self.mass[:]=16.087  # deployment's nominal mass, not randomized truth
        self.walking_reference=WalkingReference(n,d,cfg.gait)
        sensor_settings=getattr(cfg,'sensor_uncertainty',{})
        self.sensor_pipeline=SensorPipeline(n,d,sensor_settings) if sensor_settings else None
        self._measured_data=None

    @property
    def control_data(self):
        return self._measured_data if self._measured_data is not None else self.robot.data

    @property
    def measured_features(self):
        """Body vx/vy from contacting-foot kinematics, then FL/FR/RL/RR contacts."""
        r=self.control_data
        q=r.joint_pos[:,self.joint_ids].reshape(-1,4,3)
        qd=r.joint_vel[:,self.joint_ids].reshape(-1,4,3)
        relative=foot_velocity(q,qd)+torch.cross(r.root_ang_vel_b[:,None,:].expand(-1,4,-1),fk(q),dim=-1)
        weights=self.events.contact.float()
        velocity=-(relative*weights[:,:,None]).sum(1)/weights.sum(-1,keepdim=True).clamp_min(1)
        return torch.cat((velocity[:,:2].clamp(-1,1),weights),-1)

    @property
    def num_envs(self):return self._env.num_envs

    @property
    def device(self):return self._env.device

    @property
    def command(self):
        # Actor inputs are encoders, IMU, operator/reference state only. No truth pose/velocity/contact.
        phase=2*math.pi*self.time/self.cfg.gait.cycle
        phrase=2*math.pi*self.time/(4*self.cfg.gait.beat)
        kick=torch.zeros(self.num_envs,4,device=self.device)
        return torch.cat((self.q_ref,torch.stack((phase.sin(),phase.cos(),phrase.sin(),phrase.cos()),-1),
                          self.expected.float(),self.velocity,self.pair.float(),
                          (self.target_time-self.time).clamp(-1,1)[:,None],kick,
                          (~self.ready|self.recovery).float()[:,None]),-1)

    def set_velocity(self,v):
        self.external_velocity=True
        self.requested_velocity[:]=torch.as_tensor(v,device=self.device)
        speed=self.requested_velocity[:,:2].norm(dim=-1,keepdim=True)
        self.requested_velocity[:,:2]*=(self.cfg.gait.max_speed/speed.clamp_min(1e-9)).clamp_max(1)
        self.requested_velocity[:,2].clamp_(-self.cfg.gait.max_yaw,self.cfg.gait.max_yaw)
        g=self.cfg.gait
        if not (g.linear_acceleration_limit or g.yaw_acceleration_limit):
            self.velocity[:]=self.requested_velocity
        else:
            self.velocity[~self.ready]=self.requested_velocity[~self.ready]

    def _advance_velocity(self):
        g=self.cfg.gait
        if g.training_command_period and not self.external_velocity:
            self.command_age+=self.ready*self._env.step_dt
            due=self.ready&(self.command_age>=self.command_next)
            ids=due.nonzero().flatten()
            if len(ids):
                target=(torch.rand(len(ids),3,device=self.device)*2-1)
                target[:,:2]*=g.max_speed;target[:,2]*=g.max_yaw
                target[:,:2]*=(g.max_speed/target[:,:2].norm(dim=-1,keepdim=True).clamp_min(1e-9)).clamp_max(1)
                training_direction_bias(target,g)
                target[torch.rand(len(ids),device=self.device)<g.training_zero_fraction]=0
                self.requested_velocity[ids]=target
                self.command_age[ids]=0
                self.command_next[ids]=g.training_command_period*(.75+.5*torch.rand(len(ids),device=self.device))
        if g.linear_acceleration_limit or g.yaw_acceleration_limit:
            updated=advance_velocity(self.velocity,self.requested_velocity,self._env.step_dt,g)
            self.velocity[:]=torch.where(self.ready[:,None],updated,self.requested_velocity)

    def reset(self,env_ids=None):
        ids=torch.arange(self.num_envs,device=self.device) if env_ids is None else env_ids
        logs={k:float(v[ids].mean()) for k,v in self.metrics.items()}
        for v in self.metrics.values():v[ids]=0
        for v in (self.time,self.age,self.stable_age,self.ready,self.active,self.recovery,self.pair,self.lifted,
                  self.landed,self.swing_latch,self.lift_counts,self.support_lost,self.last_swing,self.reward_event,self.pending_skip):v[ids]=0
        self.timed_landing[ids]=True
        self.last_beat[ids]=-1;self.events.reset(ids)
        if self.sensor_pipeline is not None:self.sensor_pipeline.reset(ids)
        self.walking_reference.reset(ids)
        self.command_age[ids]=0
        self.command_next[ids]=self.cfg.gait.training_command_period*(.75+.5*torch.rand(len(ids),device=self.device)) if self.cfg.gait.training_command_period else 0
        self.feedback_height[ids]=.322;self.feedback_speed[ids]=0
        self.q_ref[ids]=self.robot.data.default_joint_pos[ids]
        self.expected[ids]=True;self.clearance[ids]=0
        if not self.external_velocity:
            self.velocity[ids]=torch.rand(len(ids),3,device=self.device)*2-1
            self.velocity[ids,:2]*=self.cfg.gait.max_speed
            self.velocity[ids,2]*=self.cfg.gait.max_yaw
            norm=self.velocity[ids,:2].norm(dim=-1,keepdim=True)
            self.velocity[ids,:2]*=(self.cfg.gait.max_speed/norm.clamp_min(1e-8)).clamp_max(1)
            self.velocity[ids]=training_direction_bias(self.velocity[ids],self.cfg.gait)
            self.velocity[ids[torch.rand(len(ids),device=self.device)<self.cfg.gait.training_zero_fraction]]=0
            self.requested_velocity[ids]=self.velocity[ids]
        return logs

    def _resample_command(self,env_ids):pass
    def _update_metrics(self):pass

    def sync(self):
        tick=self._env.common_step_counter
        if tick==self.last_tick:return
        self.last_tick=tick
        dt=self._env.step_dt;g=self.cfg.gait
        truth=self.robot.data
        force=self.sensor.data.net_forces_w[:,self.sensor_ids].norm(dim=-1)
        if self.sensor_pipeline is not None:
            values=self.sensor_pipeline.update(truth.joint_pos,truth.joint_vel,truth.root_ang_vel_b,truth.projected_gravity_b,force)
            self._measured_data=MeasuredData(truth,values[:4]);force=values[4]
        r=self.control_data
        self.age+=dt
        td,lo,onset=self.events.update(force,self.time,dt)
        self.touch_valid=td & self.active[:,None] & self.pair & self.lifted & ~self.landed
        self.touch_error=onset-self.target_time[:,None]
        sole=r.body_pos_w[:,self.foot_ids,2]-.022-self._env.scene.env_origins[:,2,None]
        self.feedback_speed=r.body_lin_vel_w[:,self.foot_ids].norm(dim=-1)
        if g.estimated_feedback:
            sole,foot_speed,self.feedback_height=estimated_foot_feedback(
                r.joint_pos[:,self.joint_ids].reshape(-1,4,3),r.joint_vel[:,self.joint_ids].reshape(-1,4,3),
                r.root_ang_vel_b,r.projected_gravity_b,self.events.contact,self.feedback_height)
            self.feedback_speed=foot_speed.norm(dim=-1)
        verified=(self.events.air>=.04-1e-6)&(sole>.008)
        self.swing_latch &= ~td
        new_lift=verified & ~self.swing_latch & self.pair & self.active[:,None]
        self.swing_latch |= new_lift
        self.lift_valid=new_lift
        self.lift_counts+=new_lift
        self.lifted|=new_lift
        self.landed|=self.touch_valid
        self.support_lost|=((self.events.air>=.04-1e-6)&~self.pair).any(-1)&self.active
        self.reward_event.zero_()
        self.reward_event-=.5*self.pending_skip
        self.reward_event+=((torch.exp(-(self.touch_error/g.timing_tolerance).square())-.5)*self.touch_valid).sum(-1)
        # Close each attempted STEP after the physical contact window, never by reference alone.
        finish=self.active & (self.time>self.target_time+g.timing_tolerance)
        correct=finish & ((self.landed&self.pair).sum(-1)==2) & ~self.support_lost
        # Every selected foot must touch down in the allowed window. Latch errors separately.
        self.timed_landing &= ~(self.touch_valid & ~timing_valid(self.touch_error,g.timing_tolerance))
        correct &= self.timed_landing.all(-1) & ((self.lift_counts*self.pair).sum(-1)==2)
        self.metrics['correct']+=correct;self.metrics['missed']+=finish&~correct
        self.metrics['FL_RR_correct']+=correct&self.pair[:,0]
        self.metrics['FR_RL_correct']+=correct&self.pair[:,1]
        self.reward_event+=correct.float()*.5
        self.reward_event-=.5*(finish&~correct)
        self.active[finish]=False
        self.recovery |= finish & ~self.events.confirmed.all(-1)
        q=r.joint_pos[:,self.joint_ids].reshape(-1,4,3)
        qd=r.joint_vel[:,self.joint_ids].reshape(-1,4,3)
        foot_v=foot_velocity(q,qd)+torch.cross(r.root_ang_vel_b[:,None,:].expand(-1,4,-1),fk(q),dim=-1)
        estimated_body_velocity=-foot_v.mean(1)
        stable=(self.events.confirmed.all(-1)&(r.projected_gravity_b[:,:2].norm(dim=-1)<.15)
                &(estimated_body_velocity.norm(dim=-1)<.08)&(r.root_ang_vel_b.norm(dim=-1)<.3))
        self.stable_age=torch.where(stable,self.stable_age+dt,0.)
        admitted=~self.ready&(self.age>=1.)&(self.stable_age>=.25)
        self.ready|=admitted
        self.recovery&=~(stable&(self.stable_age>=.12))
        self.last_records={'finish':finish.clone(),'correct':correct.clone(),'pair':self.pair.clone(),
                           'touchdown':self.touch_valid.clone(),'touch_error':self.touch_error.clone(),
                           'liftoff':new_lift.clone(),'sole':sole.clone(),'contact':self.events.contact.clone(),
                           'expected':self.expected.clone(),'time':self.time.clone(),'ready':self.ready.clone(),
                           'raw_touchdown':td.clone(),'raw_liftoff':lo.clone(),'target_time':self.target_time.clone()}
        self.last_records['paired_lift']=finish&((self.lifted&self.pair).sum(-1)==2)
        self.last_records['travel_active']=self.travel_active.clone()
        self.last_records['tracking_velocity']=self.velocity.clone()

    @property
    def travel_active(self):return self.ready&~self.recovery

    def _update_command(self):
        self.sync()
        self._advance_velocity()
        g=self.cfg.gait;dt=self._env.step_dt
        self.time+=self.ready*dt
        beat,within,pair,swing=schedule(self.time,g)
        start=swing&~self.last_swing&self.ready
        # Scheduler dispatches at the lead time; bad support skips this opportunity without slowing music.
        admit=start&~self.active&~self.recovery&self.events.confirmed.all(-1)
        self.metrics['skipped']+=start&~admit
        self.pending_skip=start&~admit
        self.recovery|=self.pending_skip
        self.active|=admit;self.pair[admit]=pair[admit]
        self.lifted[admit]=False;self.landed[admit]=False;self.swing_latch[admit]=False;self.support_lost[admit]=False
        self.lift_counts[admit]=0;self.timed_landing[admit]=True
        self.target_time[admit]=(beat[admit]+1)*g.beat
        self.metrics['attempts']+=admit
        self.last_swing=swing
        if g.integrated_walk_reference:
            feet,clear,swingfeet=self.walking_reference.update(self.time,self.velocity,self.ready)
        else:
            feet,clear,swingfeet=walking_reference(self.time,self.velocity,g)
        intent_clear=clear.clone()
        enabled=self.ready&~self.recovery
        # Block unadmitted scheduled swings but retain stance references.
        swing_allowed=self.active[:,None]&self.pair
        feet[:,:,2]=torch.where(swingfeet&~swing_allowed,-g.leg_height,feet[:,:,2])
        clear*=swing_allowed
        nominal=nominal_feet(self.num_envs,self.device,g)
        fallback=self.feet_ref+(nominal-self.feet_ref).clamp(-.005,.005)
        feet=torch.where(enabled[:,None,None],feet,fallback)
        clear*=enabled[:,None]
        self.feet_ref=feet
        # Suppressed swings still have musical goals: standing must not earn a
        # perfect contact score merely because admission rejected the swing.
        self.clearance=intent_clear*self.ready[:,None]
        self.expected=~(self.clearance>.008)
        if g.attitude_compensation:
            gravity=self.control_data.projected_gravity_b
            roll=torch.atan2(-gravity[:,1],-gravity[:,2]).clamp(-.3,.3)*g.attitude_compensation
            pitch=torch.asin(gravity[:,0].clamp(-1,1)).clamp(-.3,.3)*g.attitude_compensation
            tilt=quat_from_euler_xyz(roll,pitch,torch.zeros_like(roll))
            feet=quat_apply_inverse(tilt[:,None,:].expand(-1,4,-1).reshape(-1,4),feet.reshape(-1,3)).reshape(-1,4,3)
        q=ik(feet).flatten(1)
        limits=self.robot.data.soft_joint_pos_limits[:,self.joint_ids]
        clipped=q.clamp(limits[:,:,0],limits[:,:,1])
        self.metrics['limit_clamps']+=(q!=clipped).any(-1)
        self.ik_error=(fk(clipped.reshape(-1,4,3))-feet).norm(dim=-1).amax(-1)
        if g.load_compensation:
            weights=1-(clear/.008).clamp(0,1)
            load=self.mass[:,None]*9.81*weights/weights.sum(-1,keepdim=True).clamp_min(1)
            force=-self.control_data.projected_gravity_b[:,None,:]*load[:,:,None]
            stiffness=self.robot.data.default_joint_stiffness[:,self.joint_ids].reshape(-1,4,3)
            offset=gravity_position_offset(clipped.reshape(-1,4,3),force,stiffness).clamp(-.4,.4)
            clipped=(clipped+g.load_compensation*offset.flatten(1)).clamp(limits[:,:,0],limits[:,:,1])
        self.q_ref[:,self.joint_ids]=clipped


class CPGDanceController(CPGController):
    def __init__(self,cfg,env):
        super().__init__(cfg,env)
        self.dance=DanceReference(self.num_envs,self.device,cfg.gait)
        self.kick_success=torch.zeros_like(self.ready)
        self.kick_failure=torch.zeros_like(self.ready)
        self.kick_active=torch.zeros_like(self.ready)
        self.kick_ever_air=torch.zeros_like(self.ready)
        self.kick_hold_age=torch.zeros_like(self.time)
        self.metrics.update({k:torch.zeros_like(self.time) for k in ('kicks','kick_successes','kick_failures','postkick_successes')})
        self.postkick=torch.zeros_like(self.ready)
        self.kick_deadline_charged=torch.zeros_like(self.ready)

    def reset(self,env_ids=None):
        ids=torch.arange(self.num_envs,device=self.device) if env_ids is None else env_ids
        logs=super().reset(ids)
        if hasattr(self,'dance'):
            self.dance.reset(ids);self.time[ids]=self.dance.time[ids]
            self.target_time[ids]=self.dance.target_time[ids]
            if not self.external_velocity:
                self.velocity[ids[self.dance.extended_hold[ids]]]=0.
                self.requested_velocity[ids]=self.velocity[ids]
            for x in (self.kick_active,self.kick_ever_air,self.kick_success,self.kick_failure,self.postkick,self.kick_hold_age):x[ids]=0
            self.kick_deadline_charged[ids]=False
        return logs

    @property
    def travel_velocity(self):
        pause=(self.dance.kind==self.dance.KICK)|(self.dance.kind==self.dance.HOLD)|(self.dance.kind==self.dance.RECOVERY)
        return self.velocity*(~pause[:,None])

    @property
    def travel_active(self):
        if not hasattr(self,'dance'):return self.ready&~self.recovery
        g=self.cfg.gait
        phrase=torch.floor((self.time+g.swing_seconds)/(4*g.beat)).clamp_min(0)
        local=self.time-phrase*4*g.beat
        return self.ready&(local<2.5*g.beat)&((self.dance.kind==self.dance.SUPPORT)|(self.dance.kind==self.dance.STEP))

    @property
    def command(self):
        features=super().command
        if hasattr(self,'dance'):
            front=torch.where(self.dance.selected.any(-1),self.dance.selected[:,:2].long().argmax(-1),self.dance.initial_front)
            gait_time=(front+1)*self.cfg.gait.beat+self.time-self.dance.target_time
            if self.cfg.gait.freeze_hold_phase:
                gait_time=torch.where(self.dance.kind==self.dance.HOLD,(front+1)*self.cfg.gait.beat,gait_time)
            phase=2*torch.pi*gait_time/self.cfg.gait.cycle
            features[:,12:14]=torch.stack((phase.sin(),phase.cos()),-1)
            # Replace the four reserved kick features without changing policy shape.
            features[:,-5:-1]=torch.stack(((self.dance.kind==2).float(),(self.dance.kind==3).float(),
                                           (self.dance.raised==0).float(),(self.dance.raised==1).float()),-1)
        return features

    def sync(self):
        previous_tick=self.last_tick
        super().sync()
        if previous_tick==self.last_tick or not hasattr(self,'dance'):return
        g=self.cfg.gait;r=self.control_data;dt=self._env.step_dt
        kick=self.dance.kind==self.dance.KICK
        hold=self.dance.kind==self.dance.HOLD
        selected=self.dance.selected
        sole=self.last_records['sole']
        air=((self.events.air>=.04)&selected).any(-1)
        self.kick_ever_air|=kick&air
        body_feet=fk(r.joint_pos[:,self.joint_ids].reshape(-1,4,3))
        # Joint encoders supply lateral endpoint; ground-relative clearance and
        # contact/speed are privileged evaluation measurements, not actor inputs.
        lateral_error=((body_feet[:,:,1]-self.dance.endpoint_body[:,:,1]).abs()*selected).sum(-1)
        planar_error=((body_feet[:,:,:2]-self.dance.endpoint_body[:,:,:2]).norm(dim=-1)*selected).sum(-1)
        height_error=((sole-g.kick_height).abs()*selected).sum(-1)
        speed=(self.feedback_speed*selected).sum(-1)
        support=(self.events.confirmed|selected).all(-1)
        stable=(r.projected_gravity_b[:,:2].norm(dim=-1)<.2)&(r.root_ang_vel_b.norm(dim=-1)<.4)
        endpoint=(planar_error<.02)&(height_error<.015)&(speed<.10)&support&stable&air
        self.kick_success=kick&(self.time>=self.dance.target_time)&endpoint
        self.kick_hold_age=torch.where(hold,self.kick_hold_age+dt,0.)
        # Early touchdown after verified kick lift is failure. Hold must retain air.
        grounded=(self.events.confirmed&selected).any(-1)
        unsupported=((self.events.air>=.04)&~selected).any(-1)
        self.kick_failure=(kick&((self.time>self.dance.target_time+.2)|(self.kick_ever_air&grounded)))|(hold&(grounded|unsupported))
        self.metrics['kick_successes']+=self.kick_success
        self.metrics['kick_failures']+=self.kick_failure
        self.metrics['postkick_successes']+=self.postkick&self.last_records['correct']
        late=kick&(self.time>self.dance.target_time+g.timing_tolerance+1e-5)&~self.kick_deadline_charged
        self.kick_deadline_charged|=late
        timely=self.time<=self.dance.target_time+g.timing_tolerance+1e-5
        credit=self.kick_success & (timely if g.strict_kick_timing_reward else torch.ones_like(timely))
        self.reward_event+=credit.float()*.5-self.kick_failure.float()*.5
        if g.strict_kick_timing_reward:self.reward_event-=.5*late
        self.last_records['kick_deadline_miss']=late.clone()
        self.last_records['kick_success']=self.kick_success.clone()
        self.last_records['kick_failure']=self.kick_failure.clone()
        self.last_records['kick_leg']=selected[:,:2].long().argmax(-1)
        self.last_records['kick_active']=kick.clone()
        self.last_records['hold_active']=hold.clone()
        self.last_records['hold_valid']=hold&endpoint
        self.last_records['kick_lateral_error']=lateral_error.clone()
        self.last_records['kick_planar_error']=planar_error.clone()
        self.last_records['kick_height_error']=height_error.clone()
        self.last_records['kick_speed']=speed.clone()
        self.last_records['kick_endpoint_delay']=self.time-self.dance.target_time
        self.last_records['postkick_success']=self.postkick&self.last_records['correct']

    def _update_command(self):
        self.sync();g=self.cfg.gait
        self._advance_velocity()
        step_success=self.last_records.get('correct',torch.zeros_like(self.ready))
        step_failure=self.last_records.get('finish',torch.zeros_like(self.ready))&~step_success
        previous_missed=self.dance.missed.clone()
        feet,clear=self.dance.update(self._env.step_dt,self.ready,self.events.confirmed,
                                    step_success|self.kick_success,step_failure|self.kick_failure,self.velocity)
        self.time=self.dance.time.clone();self.pair=self.dance.selected.clone()
        self.target_time=self.dance.target_time.clone()
        dispatched=self.dance.dispatched
        step=dispatched&(self.dance.kind==self.dance.STEP)
        kick=dispatched&(self.dance.kind==self.dance.KICK)
        self.active|=step
        self.postkick[step]=self.dance.raised[step]>=0
        self.lifted[step]=False;self.landed[step]=False;self.swing_latch[step]=False
        self.lift_counts[step]=0;self.timed_landing[step]=True;self.support_lost[step]=False
        # An airborne front leg enters the STEP as its first verified swing.
        raised=torch.arange(4,device=self.device)[None,:]==self.dance.raised[:,None]
        entry=step[:,None]&raised
        self.lifted|=entry;self.swing_latch|=entry;self.lift_counts+=entry
        self.kick_ever_air[kick]=False
        self.kick_deadline_charged[kick]=False
        self.metrics['attempts']+=step;self.metrics['kicks']+=kick
        self.pending_skip=self.dance.missed>previous_missed;self.metrics['skipped']+=self.pending_skip
        self.recovery=self.dance.kind==self.dance.RECOVERY
        self.feet_ref=feet;self.clearance=clear;self.expected=clear<.008
        q=ik(feet).flatten(1);limits=self.robot.data.soft_joint_pos_limits[:,self.joint_ids]
        clipped=q.clamp(limits[:,:,0],limits[:,:,1])
        self.metrics['limit_clamps']+=(q!=clipped).any(-1)
        self.ik_error=(fk(clipped.reshape(-1,4,3))-feet).norm(dim=-1).amax(-1)
        if g.load_compensation:
            weights=1-(clear/.008).clamp(0,1)
            load=self.mass[:,None]*9.81*weights/weights.sum(-1,keepdim=True).clamp_min(1)
            force=-self.control_data.projected_gravity_b[:,None,:]*load[:,:,None]
            if g.wrench_compensation:
                gravity=self.control_data.projected_gravity_b
                roll=torch.atan2(-gravity[:,1],-gravity[:,2])
                pitch=torch.asin(gravity[:,0].clamp(-1,1))
                torque=-g.support_angular_damping*self.control_data.root_ang_vel_b.clone()
                torque[:,:2]-=20*torch.stack((roll,pitch),-1)
                com=torch.zeros(self.num_envs,3,device=self.device)
                com[:,0]=g.support_com_x;com[:,2]=g.support_com_z
                desired_force=-gravity*self.mass[:,None]*9.81
                if g.support_velocity_damping:
                    q_measured=self.control_data.joint_pos[:,self.joint_ids].reshape(-1,4,3)
                    qd_measured=self.control_data.joint_vel[:,self.joint_ids].reshape(-1,4,3)
                    rotational=torch.cross(self.control_data.root_ang_vel_b[:,None,:].expand(-1,4,-1),fk(q_measured),dim=-1)
                    support=(self.events.confirmed&~self.dance.selected).float()
                    estimated=-((foot_velocity(q_measured,qd_measured)+rotational)*support[:,:,None]).sum(1)/support.sum(-1,keepdim=True).clamp_min(1)
                    tangent=estimated-gravity*(estimated*gravity).sum(-1,keepdim=True)
                    desired_force-=g.support_velocity_damping*self.mass[:,None]*tangent
                allocated=support_forces(feet,com,desired_force,torque,weights)
                three=(self.dance.kind==self.dance.KICK)|(self.dance.kind==self.dance.HOLD)
                force=torch.where(three[:,None,None],force+g.wrench_compensation*(allocated-force),force)
            stiffness=self.robot.data.default_joint_stiffness[:,self.joint_ids].reshape(-1,4,3)
            offset=gravity_position_offset(clipped.reshape(-1,4,3),force,stiffness).clamp(-.4,.4)
            clipped=(clipped+g.load_compensation*offset.flatten(1)).clamp(limits[:,:,0],limits[:,:,1])
        self.q_ref[:,self.joint_ids]=clipped
