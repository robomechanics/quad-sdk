"""Batched encoder/IMU/contact corruption upstream of control, with reset-safe delay.

Simulation-only uncertainty generator. Deployment uses actual measurements and
never injects this noise. Units are rad, rad/s, unit gravity, and N.
"""
import torch


class SensorPipeline:
    def __init__(self, n, device, settings):
        self.n=n;self.device=device;self.cfg=dict(settings)
        allowed={'q_noise','qd_noise','gyro_noise','gravity_noise','force_noise','q_bias','gyro_bias','gravity_bias','force_bias','force_scale','delay_ticks','packet_hold_probability','contact_dropout_probability'}
        if set(settings)-allowed:raise ValueError('Unknown sensor uncertainty setting')
        for k,v in settings.items():
            if not isinstance(v,(int,float)) or not 0<=v<float('inf'):raise ValueError('Invalid sensor uncertainty '+k)
        self.delay=int(settings.get('delay_ticks',0))
        if self.delay!=settings.get('delay_ticks',0) or self.delay>3:raise ValueError('Sensor delay must be 0..3 control ticks')
        for k in ('packet_hold_probability','contact_dropout_probability','force_scale'):
            if settings.get(k,0)>1:raise ValueError('Invalid probability/scale '+k)
        self.history=torch.zeros(self.delay+1,n,34,device=device);self.cursor=0
        self.valid=torch.zeros(n,dtype=torch.bool,device=device)
        self.previous=torch.zeros(n,34,device=device)
        self.bias=torch.zeros(n,34,device=device);self.force_scale=torch.ones(n,4,device=device)
        self.lag=torch.zeros(n,dtype=torch.long,device=device)
        self.reset(torch.arange(n,device=device))

    def uniform(self,shape,amplitude):return (torch.rand(shape,device=self.device)*2-1)*amplitude

    def reset(self,ids):
        self.valid[ids]=False;self.bias[ids]=0
        for sl,key in [(slice(0,12),'q_bias'),(slice(24,27),'gyro_bias'),(slice(27,30),'gravity_bias'),(slice(30,34),'force_bias')]:
            self.bias[ids,sl]=self.uniform((len(ids),sl.stop-sl.start),self.cfg.get(key,0))
        self.force_scale[ids]=1+self.uniform((len(ids),4),self.cfg.get('force_scale',0))
        self.lag[ids]=torch.randint(self.delay+1,(len(ids),),device=self.device)

    def update(self,q,qd,gyro,gravity,force):
        x=torch.cat((q,qd,gyro,gravity,force),-1).clone()
        x[:,30:34]*=self.force_scale
        x+=self.bias
        for sl,key in [(slice(0,12),'q_noise'),(slice(12,24),'qd_noise'),(slice(24,27),'gyro_noise'),(slice(27,30),'gravity_noise'),(slice(30,34),'force_noise')]:
            if self.cfg.get(key,0):x[:,sl]+=self.uniform((self.n,sl.stop-sl.start),self.cfg[key])
        x[:,27:30]/=x[:,27:30].norm(dim=-1,keepdim=True).clamp_min(1e-8)
        x[:,30:34].clamp_min_(0)
        if self.cfg.get('contact_dropout_probability',0):x[:,30:34]*=(torch.rand(self.n,4,device=self.device)>=self.cfg['contact_dropout_probability'])
        fresh=~self.valid
        self.history[:,fresh]=x[fresh];self.previous[fresh]=x[fresh]
        self.history[self.cursor]=x
        out=self.history[(self.cursor-self.lag)%(self.delay+1),torch.arange(self.n,device=self.device)].clone()
        if self.cfg.get('packet_hold_probability',0):
            hold=self.valid&(torch.rand(self.n,device=self.device)<self.cfg['packet_hold_probability'])
            out[hold]=self.previous[hold]
        self.valid[:]=True;self.previous=out.clone();self.cursor=(self.cursor+1)%(self.delay+1)
        return out[:,:12],out[:,12:24],out[:,24:27],out[:,27:30],out[:,30:34]


class MeasuredData:
    """Override sensor fields only; leave physical state untouched for rewards."""
    def __init__(self,truth,values):
        self.truth=truth
        for name,value in zip(('joint_pos','joint_vel','root_ang_vel_b','projected_gravity_b'),values):setattr(self,name,value)
    def __getattr__(self,name):return getattr(self.truth,name)
