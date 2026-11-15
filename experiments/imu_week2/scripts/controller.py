"""Repeated Panda quintic trials driven by actual ROS rate-limited feedback."""
import argparse,csv,json,time
from pathlib import Path
import numpy as np,rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory import reference
p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--out',type=Path,required=True);args=p.parse_args();cfg=json.loads(args.config.read_text())
class Controller(Node):
 def __init__(self):
  super().__init__('week2_frequency_controller');self.anchor=None;self.done=False;self.lastphase=-1
  self.pub=self.create_publisher(JointState,cfg['command_topic'],100)
  self.sub=self.create_subscription(JointState,'/panda_week2/feedback',self.callback,100)
  self.f=(args.out/'tracking.csv').open('w',newline='');self.w=csv.writer(self.f)
  self.w.writerow(['seq','sim_time_s','elapsed_s','phase','requested_hz','phase_time_s','host_receive_ns','command_publish_ns']+[f'q_ref_{i}' for i in range(1,8)]+[f'q_actual_{i}' for i in range(1,8)])
 def callback(self,msg):
  received=time.monotonic_ns();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
  if self.anchor is None:self.anchor=stamp
  elapsed=stamp-self.anchor;phase=int((elapsed+1e-7)/cfg['phase_duration_s'])
  if phase>=len(cfg['feedback_rates_hz']):return
  local=elapsed-phase*cfg['phase_duration_s'];start=np.array(cfg['initial_positions_rad']);end=np.array(cfg['target_positions_rad'])
  if phase%2:start,end=end,start
  q,_,_=reference(local-cfg['hold_before_s'],cfg['trajectory_duration_s'],start,end)
  command=JointState();command.header=msg.header;command.name=cfg['joint_names'][:7];command.position=q[:7].tolist();self.pub.publish(command);sent=time.monotonic_ns()
  actual=np.array([msg.position[msg.name.index(n)] for n in cfg['joint_names'][:7]])
  self.w.writerow([int(msg.header.frame_id.rsplit('_',1)[1]),stamp,elapsed,phase,cfg['feedback_rates_hz'][phase],local,received,sent,*q[:7],*actual])
  if phase!=self.lastphase:print('PHASE',phase,'rate',cfg['feedback_rates_hz'][phase],flush=True);self.lastphase=phase;self.f.flush()
  if phase==len(cfg['feedback_rates_hz'])-1 and local>=cfg['phase_duration_s']-.05:
   self.done=True;self.f.flush();(args.out/'controller_done.json').write_text(json.dumps({'phases':len(cfg['feedback_rates_hz']),'elapsed_sim_s':elapsed}))
rclpy.init();node=Controller();started=time.monotonic()
try:
 while rclpy.ok() and not node.done:
  rclpy.spin_once(node,timeout_sec=.1)
  if time.monotonic()-started>220:raise TimeoutError('Frequency experiment timeout')
finally:node.f.close();node.destroy_node();rclpy.shutdown()
