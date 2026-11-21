"""Transport-independent deployment checks; no hardware access or simulator truth."""
import json
import math
import os
import select
import time


class CommandEnvelope:
    def __init__(self, selection):
        limits=selection.get('joystick_limits',[.05,.04,.1])
        self.lower=list(map(float,selection.get('command_min',[-v for v in limits])))
        self.upper=list(map(float,selection.get('command_max',limits)))
        if len(self.lower)!=3 or len(self.upper)!=3 or any(
            not math.isfinite(lo) or not math.isfinite(hi) or not lo<=0<=hi
            for lo,hi in zip(self.lower,self.upper)):
            raise ValueError('Invalid command envelope')
        if any(abs(v)>.5 for v in self.lower[:2]+self.upper[:2]) or max(abs(self.lower[2]),abs(self.upper[2]))>1:
            raise ValueError('Command envelope exceeds deployment bounds')

        self.forward_at_max_yaw=float(selection.get('forward_at_max_yaw',self.upper[0]))
        if not math.isfinite(self.forward_at_max_yaw) or not 0<=self.forward_at_max_yaw<=self.upper[0]:
            raise ValueError('Invalid turning forward-speed cap')

    def apply(self, values):
        if len(values)!=3 or not all(math.isfinite(v) for v in values):
            raise ValueError('Invalid velocity command')
        out=[float(max(lo,min(hi,float(v)))) for v,lo,hi in zip(values,self.lower,self.upper)]
        yaw_limit=max(abs(self.lower[2]),abs(self.upper[2]))
        fraction=abs(out[2])/yaw_limit if yaw_limit else 0.
        out[0]=min(out[0],self.upper[0]-(self.upper[0]-self.forward_at_max_yaw)*fraction)
        return out


class ForceCalibration:
    """Calibration in Quad-SDK FL/RL/FR/RR order, raw counts -> contact evidence.

    Units may be calibrated Newtons or a documented contact-evidence scale.
    The scheduler only uses 5/2 hysteresis; this is not a force/load estimator.
    Each channel needs independent unloaded/loaded measurements.
    """
    def __init__(self, config, require_measured=False):
        self.offset=config['offset'];self.scale=config['scale']
        if config.get('order')!='FL,RL,FR,RR':raise ValueError('Wrong force calibration order')
        if require_measured and config.get('measured') is not True:
            raise ValueError('Hardware needs measured force calibration')
        if len(self.offset)!=4 or len(self.scale)!=4 or not all(math.isfinite(v) for v in self.offset+self.scale) or min(self.scale)<=0:
            raise ValueError('Invalid per-foot calibration')
    def apply(self, raw):
        if len(raw)!=4 or not all(math.isfinite(v) for v in raw):raise ValueError('Invalid raw foot forces')
        return [max(0.,(v-o)*s) for v,o,s in zip(raw,self.offset,self.scale)]


class SensorGate:
    """Require advancing source stamps, fresh reception and synchronized streams."""
    def __init__(self, age=.05, skew=.01):
        self.age=age;self.skew=skew;self.previous={};self.advanced={}
    def check(self, samples, ros_now, wall_now):
        # samples: name -> (source_stamp_seconds, monotonic_receive_seconds)
        stamps=[]
        for name,(stamp,received) in samples.items():
            if not all(math.isfinite(x) for x in (stamp,received,ros_now,wall_now)):
                raise ValueError('Nonfinite sensor clock')
            if not -.01<=ros_now-stamp<=self.age or not 0<=wall_now-received<=self.age:
                raise ValueError('Stale or future '+name)
            if name in self.previous and stamp<self.previous[name]:
                raise ValueError('Regressing '+name+' timestamp')
            if name not in self.previous or stamp>self.previous[name]:
                self.advanced[name]=wall_now;self.previous[name]=stamp
            if wall_now-self.advanced[name]>self.age:raise ValueError('Frozen '+name+' source timestamp')
            stamps.append(stamp)
        if len(stamps)!=3 or max(stamps)-min(stamps)>self.skew:
            raise ValueError('Missing or unsynchronized sensors')


class ControlClockGate:
    """Bound accumulated wall-clock phase error, not only individual tick gaps.

    The fixed-dt scheduler must not silently turn 90 BPM into a slower dance if
    the host repeatedly misses timer periods. Fault instead of skipping states.
    """
    def __init__(self, dt=.02, tolerance=.02):
        self.dt=dt;self.tolerance=tolerance;self.epoch=None
    def check(self, now, tick):
        if not math.isfinite(now):raise ValueError('Nonfinite control clock')
        if self.epoch is None:self.epoch=now-tick*self.dt
        error=(now-self.epoch)-tick*self.dt
        if abs(error)>self.tolerance:
            raise ValueError(f'Accumulated control clock drift {error:.4f}s exceeds {self.tolerance:.3f}s')
        return error


class JsonLineReader:
    """Bound partial worker responses too; select + blocking readline is unsafe."""
    def __init__(self, stream):self.fd=stream.fileno();self.pending=b''
    def read(self, timeout):
        end=time.monotonic()+timeout
        while b'\n' not in self.pending:
            remaining=end-time.monotonic()
            if remaining<=0 or not select.select([self.fd],[],[],remaining)[0]:
                raise TimeoutError('Worker response deadline')
            chunk=os.read(self.fd,4096)
            if not chunk:raise EOFError('Worker exited')
            self.pending+=chunk
            if len(self.pending)>65536:raise ValueError('Oversized worker response')
        line,self.pending=self.pending.split(b'\n',1)
        return json.loads(line)
