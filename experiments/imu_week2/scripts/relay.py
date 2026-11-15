"""Rate-limit real Isaac Bridge feedback in simulation time; never addresses hardware."""
import argparse,csv,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--out',type=Path,required=True);args=p.parse_args();cfg=json.loads(args.config.read_text())
class Relay(Node):
 def __init__(self):
  super().__init__('week2_feedback_relay');self.anchor=None;self.last=None;self.seq=0
  self.pub=self.create_publisher(JointState,'/panda_week2/feedback',100)
  self.sub=self.create_subscription(JointState,cfg['state_topic'],self.callback,100)
  self.f=(args.out/'relay.csv').open('w',newline='');self.w=csv.writer(self.f);self.w.writerow(['seq','sim_time_s','phase','requested_hz','raw_receive_ns','relay_publish_ns'])
  self.rawf=(args.out/'raw_feedback.csv').open('w',newline='');self.raw=csv.writer(self.rawf);self.raw.writerow(['sim_time_s','host_receive_ns'])
 def callback(self,msg):
  now=time.monotonic_ns();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;self.raw.writerow([stamp,now])
  if self.pub.get_subscription_count()<1:return
  if self.anchor is None:self.anchor=stamp
  phase=int((stamp-self.anchor+1e-7)/cfg['phase_duration_s'])
  if phase>=len(cfg['feedback_rates_hz']):return
  rate=cfg['feedback_rates_hz'][phase]
  if self.last is not None and stamp-self.last<1/rate-1e-7:return
  msg.header.frame_id=f'week2_sample_{self.seq}';sent=time.monotonic_ns();self.pub.publish(msg)
  self.w.writerow([self.seq,stamp,phase,rate,now,sent]);self.last=stamp;self.seq+=1
 def close(self):self.f.close();self.rawf.close()
rclpy.init();node=Relay()
try:
 while rclpy.ok() and not (args.out/'controller_done.json').exists():rclpy.spin_once(node,timeout_sec=.1)
finally:node.close();node.destroy_node();rclpy.shutdown()
