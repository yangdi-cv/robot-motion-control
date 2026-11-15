"""Read-only serial or explicitly marked offline-replay ROS2 six-axis filter node."""
import argparse,csv,json,sys,time,subprocess
from pathlib import Path
import numpy as np,rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from filters import FilterBank
p=argparse.ArgumentParser();p.add_argument('--input',type=Path);p.add_argument('--port');p.add_argument('--out',type=Path,required=True);p.add_argument('--seconds',type=float,default=10);p.add_argument('--filter',choices=['ema','moving_average','butterworth'],default='ema');args=p.parse_args()
if bool(args.input)==bool(args.port):p.error('Exactly one of --input or --port required')
args.out.mkdir(parents=True,exist_ok=False)
class Publisher(Node):
 def __init__(self):
  super().__init__('week2_imu_filter');self.bank=FilterBank();self.count=0;self.start=time.monotonic();self.last=None;self.imu=None
  self.raw=self.create_publisher(Imu,'/imu_week2/raw',100);self.filtered=self.create_publisher(Imu,'/imu_week2/filtered',100)
  self.f=(args.out/'published.csv').open('w',newline='');self.w=csv.writer(self.f);self.w.writerow(['sequence','published_monotonic_ns','published_ros_ns','sample_time_s']+[f'raw_{a}' for a in ('ax','ay','az','gx','gy','gz')]+[f'filtered_{a}' for a in ('ax','ay','az','gx','gy','gz')])
  if args.input:
   self.data=np.genfromtxt(args.input,delimiter=',',names=True);self.timer=self.create_timer(.04,self.replay)
  else:
   if not Path(args.port).exists():raise FileNotFoundError(args.port)
   if subprocess.run(['fuser',args.port],capture_output=True).returncode==0:raise RuntimeError('Serial port already in use; stop existing reader first')
   sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'vendor'))
   from YbImuSerialLib import YbImuSerial
   self.imu=YbImuSerial(args.port);self.imu._dev.timeout=0;self.imu._dev.exclusive=True;original=self.imu._parse_data
   def parse(kind,data):
    original(kind,data)
    if kind==self.imu.FUNC_REPORT_IMU_RAW:self.emit(np.array(self.imu.get_accelerometer_data())*9.80665,np.array(self.imu.get_gyroscope_data()),self.imu.get_imu_quaternion_data(),time.monotonic()-self.start)
   self.imu._parse_data=parse;self.timer=self.create_timer(.005,self.serial_poll)
 def serial_poll(self):
  for byte in self.imu._dev.read(min(4096,self.imu._dev.in_waiting)):self.imu._receive_data(byte)
 def replay(self):
  if self.count>=len(self.data):return
  r=self.data[self.count];values=np.array([r[f'raw_{a}'] for a in ('ax','ay','az','gx','gy','gz')]);self.emit(values[:3],values[3:],None,float(r['sim_time_s']))
 def emit(self,accel,gyro,quaternion,stamp):
  now=time.monotonic_ns();dt=None if self.last is None else stamp-self.last
  if dt is not None and dt<=0:raise ValueError('Non-monotonic sample clock')
  x=np.r_[accel,gyro];values=self.bank.update(x,dt)[args.filter];self.last=stamp
  ros_stamp=self.get_clock().now().to_msg()
  for publisher,v in ((self.raw,x),(self.filtered,values)):
   msg=Imu();msg.header.stamp=ros_stamp;msg.header.frame_id='imu_sensor'
   msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=map(float,v[:3]);msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=map(float,v[3:])
   # Covariance is unknown. No fused attitude or estimated covariance is claimed.
   msg.orientation_covariance[0]=-1.
   if quaternion is not None and abs(np.linalg.norm(quaternion)-1)<.1:
    msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z=map(float,quaternion);msg.orientation_covariance[0]=0.
   publisher.publish(msg)
  self.w.writerow([self.count,now,ros_stamp.sec*10**9+ros_stamp.nanosec,stamp,*x,*values]);self.count+=1
 def close(self):
  self.f.close()
  if self.imu:self.imu._dev.close()
  (args.out/'summary.json').write_text(json.dumps({'source':'offline virtual IMU + explicit synthetic noise' if args.input else 'physical IMU serial RAW packets','samples':self.count,'filter':args.filter,'serial_device_writes':0,'rate_setting_writes':0,'orientation_fused':False,'host_elapsed_s':time.monotonic()-self.start},indent=2))
rclpy.init();node=Publisher()
try:
 while rclpy.ok() and time.monotonic()-node.start<args.seconds:rclpy.spin_once(node,timeout_sec=.1)
finally:node.close();node.destroy_node();rclpy.shutdown()
