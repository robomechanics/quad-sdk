#!/usr/bin/env python3
"""Headless packaged Conga integration check. Launches simulation only."""
import os,subprocess,time,signal,json,pathlib,argparse
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=pathlib.Path,default=pathlib.Path('/tmp/conga-container-check'))
parser.add_argument('--seconds',type=float,default=50.)
args=parser.parse_args()
if not 10 <= args.seconds <= 90:
 raise ValueError('Duration must be 10–90 simulated seconds')
out=args.output
out.mkdir(parents=True,exist_ok=True)
if (out/'trace.jsonl').exists():
 raise RuntimeError('Choose a new output directory or remove the previous test output')
rclpy.init();node=Node('conga_port_audit')
clock=[0.]
node.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),10)
mode=node.create_publisher(UInt8,'/robot_1/control/mode',1)
vel=node.create_publisher(Twist,'/robot_1/cmd_vel',1)
log=(out/'launch.log').open('w')
p=subprocess.Popen(['ros2','launch','quad_utils','quad_conga_gazebo.py','gui:=false','rviz:=false','trace:='+str(out/'trace.jsonl')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
start=time.monotonic();armed=None;done=False
try:
 while time.monotonic()-start<180 and p.poll() is None:
  rclpy.spin_once(node,timeout_sec=.05)
  if clock[0]>4 and mode.get_subscription_count():
   if armed is None:armed=clock[0]
   if clock[0]-armed<1:mode.publish(UInt8(data=1))
   msg=Twist();elapsed=clock[0]-armed
   if elapsed>12:msg.linear.x=.1
   if elapsed>25:msg.linear.x=.05;msg.angular.z=.1
   if elapsed>38:msg.angular.z=-.1
   vel.publish(msg)
   if elapsed>=args.seconds:done=True;break
finally:
 if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=12)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 log.close();node.destroy_node();rclpy.shutdown()
rows=[json.loads(l) for l in (out/'trace.jsonl').read_text().splitlines()] if (out/'trace.jsonl').exists() else []
faults=[r for r in rows if 'fault' in r]
result=dict(completed=done,sim_time=clock[0],frames=len(rows),faults=faults,wall_seconds=time.monotonic()-start,packaged_root=True)
heights=[r['truth_position'][2] for r in rows if r.get('ready') and 'truth_position' in r]
result['min_ready_height']=min(heights) if heights else None
(out/'result.json').write_text(json.dumps(result,indent=2))
passed=bool(done and not faults and len(rows)>args.seconds*35 and heights and min(heights)>.2)
result['passed']=passed
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2),flush=True)
if not passed:
 print('Last launch messages:',flush=True)
 print('\n'.join((out/'launch.log').read_text().splitlines()[-60:]),flush=True)
raise SystemExit(0 if passed else 1)
