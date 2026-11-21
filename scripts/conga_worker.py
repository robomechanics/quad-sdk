"""Bounded JSON-lines process boundary between ROS Python and policy Python.

stdin contains SensorFrame dictionaries plus velocity. stdout contains ready,
position targets/gains, or a latched fault. No sockets or hardware transport.
"""
import argparse
import json
from pathlib import Path
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--conga-root', required=True)
    parser.add_argument('--selection',type=Path,required=True)
    args=parser.parse_args()
    sys.path.insert(0, str(Path(args.conga_root).resolve()))
    import numpy as np
    import torch
    from deploy.cpg.runtime import SensorFrame, load_selection
    from deploy.cpg.safety import CommandEnvelope
    from conga_runtime import ContactFreeRuntime
    torch.set_num_threads(1)
    selection=load_selection(args.selection)
    if selection['onnx_sha256'] != 'bfe0617876fd09abc972161193004974975f402927119023e707ac0037df3051':
        raise ValueError('Conga requires the selected frozen actor')
    envelope=CommandEnvelope(selection)
    runtime=ContactFreeRuntime(selection['deploy_metadata'],selection['onnx_export'])
    kp,kd=np.empty(12),np.empty(12)
    kp[runtime.map],kd[runtime.map]=runtime.kp,runtime.kd
    print(json.dumps({'ready':True,'dt':runtime.dt}),flush=True)
    for line in sys.stdin:
        try:
            if len(line)>65536:raise ValueError('Oversized sensor request')
            packet=json.loads(line)
            if packet.get('request_stand',False):runtime.request_stand()
            sensor=packet['sensor']
            frame=SensorFrame(time=float(sensor['time']),foot_force=np.zeros(4),**{k:np.asarray(sensor[k],dtype=float) for k in ('q','qd','quaternion','gyro')})
            with torch.inference_mode():
                observation=runtime.observe(frame,envelope.apply(packet['velocity']))
                target=runtime.act(observation)
            reply={'target_sdk':target.tolist(),'kp_sdk':kp.tolist(),'kd_sdk':kd.tolist(),
                   'dance_time':runtime.controller.time.item(),'ready':runtime.controller.ready.item(),
                   'standing':runtime.standing,'stand_requested':runtime.stand_requested}
            if packet.get('telemetry'):
                c=runtime.controller
                reply['metrics']={k:v.detach().cpu().tolist() for k,v in c.metrics.items()}
                reply['expected_contact']=c.expected[0].tolist()
                reply['scheduled_support']=c.events.contact[0].tolist()
                reply['records']={k:v.detach().cpu().tolist() for k,v in c.last_records.items()}
                if hasattr(c,'dance'):
                    reply['dance_kind']=c.dance.kind.item()
                    reply['kick_target_time']=c.dance.target_time.item()
                    reply['kick_success']=c.kick_success.item()
                    reply['kick_failure']=c.kick_failure.item()
            print(json.dumps(reply),flush=True)
        except Exception as error:
            print(json.dumps({'fault':str(error)}),flush=True)
            return 1
    return 0


if __name__=='__main__':sys.exit(main())
