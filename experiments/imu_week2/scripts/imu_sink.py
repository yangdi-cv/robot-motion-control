"""Independent ROS process recording actual received Imu messages."""
import argparse,csv,time,json
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--seconds',type=float,default=13);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
class Sink(Node):
 def __init__(self):
  super().__init__('week2_imu_sink');self.f=(a.out/'received.csv').open('w',newline='');self.w=csv.writer(self.f);self.w.writerow(['topic','sequence','host_receive_ns','header_ros_ns','ax','ay','az','gx','gy','gz']);self.subs=[];self.counts={'raw':0,'filtered':0}
  for topic in ('raw','filtered'):
   self.subs.append(self.create_subscription(Imu,'/imu_week2/'+topic,lambda m,t=topic:self.callback(t,m),100))
 def callback(self,topic,m):
  self.w.writerow([topic,self.counts[topic],time.monotonic_ns(),m.header.stamp.sec*10**9+m.header.stamp.nanosec,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]);self.counts[topic]+=1
rclpy.init();n=Sink();start=time.monotonic();(a.out/'ready').touch()
try:
 while time.monotonic()-start<a.seconds:rclpy.spin_once(n,timeout_sec=.1)
finally:n.f.close();n.destroy_node();rclpy.shutdown()
