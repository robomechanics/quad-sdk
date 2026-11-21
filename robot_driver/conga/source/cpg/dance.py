"""Batched discrete choreography reference; no continuously running trot oscillator.

This is a later curriculum component. Walking qualification does not validate
kick balance or the airborne post-kick transition.
"""
import torch
from .core import nominal_feet,smooth

class DanceReference:
    SUPPORT=0;STEP=1;KICK=2;HOLD=3;RECOVERY=4
    def __init__(self,n,device,cfg):
        self.cfg=cfg;self.n=n;self.device=device
        self.time=torch.full((n,),-cfg.beat,device=device)
        if cfg.balanced_phrases:self.time+=(torch.arange(n,device=device)%2)*4*cfg.beat
        self.initial_front=(torch.arange(n,device=device)%2)*cfg.balanced_phrases
        self.kind=torch.zeros(n,dtype=torch.long,device=device)
        self.event_id=torch.full_like(self.kind,-1)
        self.feet=nominal_feet(n,device,cfg)
        self.start_feet=self.feet.clone();self.end_feet=self.feet.clone()
        self.selected=torch.zeros(n,4,dtype=torch.bool,device=device)
        self.raised=torch.full_like(self.kind,-1)
        self.target_time=self.time+cfg.beat
        self.start_time=self.target_time-cfg.swing_seconds
        self.missed=torch.zeros_like(self.kind)
        self.phase=torch.zeros_like(self.start_time)
        self.velocity=torch.zeros(n,3,device=device)
        self.dispatched=torch.zeros(n,dtype=torch.bool,device=device)
        # Adjacent environments have opposite kick parity. Assign hold practice
        # in pairs so the curriculum cannot accidentally train only one side.
        self.extended_hold=(torch.arange(n,device=device)//2 % 100)<round(100*cfg.extended_hold_fraction)
        self.support_shift=torch.zeros(n,3,device=device)

    @property
    def endpoint_body(self):
        return self.end_feet-self.support_shift[:,None,:]

    def reset(self,ids):
        self.time[ids]=-self.cfg.beat+(ids%2)*4*self.cfg.beat*self.cfg.balanced_phrases
        self.kind[ids]=0;self.event_id[ids]=-1
        self.feet[ids]=nominal_feet(len(ids),self.device,self.cfg)
        self.selected[ids]=False;self.raised[ids]=-1;self.missed[ids]=0
        self.support_shift[ids]=0.
        self.target_time[ids]=self.time[ids]+self.cfg.beat
        self.start_time[ids]=self.target_time[ids]-self.cfg.swing_seconds
        self.start_feet[ids]=self.feet[ids];self.end_feet[ids]=self.feet[ids]
        self.phase[ids]=0.;self.velocity[ids]=0.;self.dispatched[ids]=False

    def update(self,dt,ready,contacts,success,failure,velocity):
        """Feedback is measured completion from the adapter, not elapsed trajectory time."""
        g=self.cfg;rows=torch.arange(self.n,device=self.device)
        self.time+=ready*dt
        was_step=self.kind==self.STEP;was_kick=self.kind==self.KICK
        self.raised[success&was_step]=-1
        completed_kick=success&was_kick
        self.raised[completed_kick]=self.selected[completed_kick,:2].long().argmax(-1)
        self.kind[success&was_step]=self.SUPPORT
        self.kind[completed_kick]=self.HOLD
        self.kind[failure]=self.RECOVERY
        # A conservative recovery lands slowly; time keeps advancing. No success credit.
        recovering=self.kind==self.RECOVERY
        nominal=nominal_feet(self.n,self.device,g)
        if g.recovery_strategy!='hold_and_stop':
            self.feet[recovering]+=(nominal[recovering]-self.feet[recovering]).clamp(-.003,.003)
        recovered=recovering&contacts.all(-1)&((self.feet-nominal).abs().amax((1,2))<.003)
        if g.recovery_strategy!='land_and_rejoin':recovered[:]=False
        self.kind[recovered]=self.SUPPORT;self.raised[recovered]=-1
        phrase=torch.floor((self.time+g.swing_seconds)/(4*g.beat)).long().clamp_min(0)
        local=self.time-phrase*4*g.beat
        # Planted CoM transfer in the existing half-beat gap before KICK.
        # Keep the bias through the raised entry; restore it only in four-foot
        # support after landing. Internal trajectories remain unbiased, while
        # returned targets and endpoint checks share the same body-frame bias.
        leg=torch.where(self.raised>=0,self.raised,
                        torch.where(self.kind==self.KICK,self.selected[:,:2].long().argmax(-1),(phrase+1)%2))
        full_shift=torch.zeros_like(self.support_shift)
        full_shift[:,0]=-g.kick_shift_back
        full_shift[:,1]=torch.where(leg==1,g.kick_shift_side,-g.kick_shift_side)
        raised=(self.kind==self.KICK)|(self.kind==self.HOLD)|(self.raised>=0)
        preparation=(local>=2*g.beat)&(local<=2.5*g.beat+dt+1e-5)&~recovering
        fraction=smooth((local-2*g.beat)/(.5*g.beat))
        desired=torch.where(raised[:,None],self.support_shift,
                            torch.where(preparation[:,None],fraction[:,None]*full_shift,torch.zeros_like(full_shift)))
        safe_shift=ready&(contacts.all(-1)|raised|recovering)
        self.support_shift+=safe_shift[:,None]*(desired-self.support_shift).clamp(-.004,.004)
        slot=torch.where(local>=2.5*g.beat,3,torch.where(local>=2*g.beat-g.swing_seconds,2,
                         torch.where(local>=g.beat-g.swing_seconds,1,0)))
        event=4*phrase+slot
        event_start=torch.where(slot==3,phrase*4*g.beat+2.5*g.beat,phrase*4*g.beat+slot*g.beat-g.swing_seconds)
        new=ready&(self.time>=event_start)&(event!=self.event_id)
        # A balanced reset may begin during the previous phrase's hold. Never
        # dispatch that already-expired kick at its endpoint in one control tick.
        due=new&(self.time-event_start<=dt+1e-5)
        hold_practice=self.extended_hold|(g.mode=='kick_hold')
        due &= ~(hold_practice&(self.kind==self.HOLD))
        is_kick=slot==3
        front=torch.where(is_kick,(phrase+1)%2,(phrase*3+slot)%2)
        selected=(torch.arange(4,device=self.device)[None,:]==front[:,None])
        selected|=(~is_kick[:,None])&(torch.arange(4,device=self.device)[None,:]==(3-front)[:,None])
        support=(contacts|((torch.arange(4,device=self.device)[None,:]==self.raised[:,None]))).all(-1)
        allowed=(self.kind==self.SUPPORT)|((self.kind==self.HOLD)&~is_kick&(front==self.raised))
        admit=due&support&allowed
        self.missed+=due&~admit;self.event_id[new]=event[new]
        self.dispatched=admit
        self.selected[admit]=selected[admit]
        self.start_feet[admit]=self.feet[admit]
        self.velocity[admit]=velocity[admit]
        self.kind[admit]=torch.where(is_kick[admit],self.KICK,self.STEP)
        self.start_time[admit]=torch.where(is_kick,phrase*4*g.beat+2.5*g.beat,phrase*4*g.beat+slot*g.beat-g.swing_seconds)[admit]
        self.target_time[admit]=torch.where(is_kick,self.start_time+g.kick_duration,phrase*4*g.beat+slot*g.beat)[admit]
        endpoint=nominal.clone()
        stride_time=2*g.beat-g.swing_seconds
        local_velocity=velocity[:,None,:2]+velocity[:,None,2:3]*torch.stack((-nominal[:,:,1],nominal[:,:,0]),-1)
        endpoint[:,:,:2]+=.5*stride_time*local_velocity
        kick_y=nominal[rows,front,1]+torch.where(front==0,g.kick_lateral,-g.kick_lateral)
        endpoint[rows,front,0]=torch.where(is_kick,nominal[rows,front,0],endpoint[rows,front,0])
        endpoint[rows,front,1]=torch.where(is_kick,kick_y,endpoint[rows,front,1])
        endpoint[rows,front,2]=torch.where(is_kick,-g.leg_height+g.kick_height,endpoint[rows,front,2])
        # KICK's endpoint remains the calibrated body-frame nominal +8cm
        # lateral, even while the stance references carry a preparation bias.
        endpoint[rows,front,:2]+=is_kick[:,None]*self.support_shift[:,:2]
        self.end_feet[admit]=endpoint[admit]
        step=self.kind==self.STEP;kick=self.kind==self.KICK
        duration=torch.where(kick,g.kick_duration,g.swing_seconds)
        u=((self.time-self.start_time)/duration).clamp(0,1);self.phase=u
        moving=step|kick
        trajectory=self.start_feet+smooth(u)[:,None,None]*(self.end_feet-self.start_feet)
        start_clear=(self.start_feet[:,:,2]+g.leg_height).clamp_min(0)
        bump=(g.clearance-.5*start_clear).clamp_min(0)
        trajectory[:,:,2]+=step[:,None]*bump*torch.sin(torch.pi*u[:,None]).square()
        self.feet=torch.where((moving[:,None]&self.selected)[:,:,None],trajectory,self.feet)
        # Stance travels only during the three-step portion, never the kick/hold pause.
        travel=ready&(local<2.5*g.beat)&((self.kind==self.SUPPORT)|(self.kind==self.STEP))
        stance=~self.selected|~moving[:,None]
        self.feet[:,:,:2]-=(travel[:,None]&stance)[:,:,None]*local_velocity*dt
        self.feet[:,:,:2]=torch.maximum(torch.minimum(self.feet[:,:,:2],nominal[:,:,:2]+.18),nominal[:,:,:2]-.18)
        output=self.feet-self.support_shift[:,None,:]
        output[:,:,2]-=g.bounce_amplitude*torch.sin(2*torch.pi*self.time/g.beat)[:,None]
        return output,(self.feet[:,:,2]+g.leg_height).clamp_min(0)
