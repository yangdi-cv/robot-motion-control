"""External ROS2 Humble node; exclusively targets the isolated simulator topics."""
import argparse
import csv
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory import reference


class Experiment(Node):
    def __init__(self,config,out):
        super().__init__('panda_quintic_experiment')
        self.config=config;self.out=out;self.started_sim=None;self.finished=False;self.count=0
        self.initial=np.array(config['initial_positions_rad'])
        self.target=np.array(config['target_positions_rad'])
        self.names=config['joint_names']
        qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
        self.publisher=self.create_publisher(JointState,config['command_topic'],qos)
        self.subscription=self.create_subscription(JointState,config['state_topic'],self.on_state,qos)
        self.file=(out/'tracking.csv').open('w',newline='')
        self.writer=csv.writer(self.file)
        self.writer.writerow(['sample_index','sim_time_s','trajectory_time_s','host_receive_ns','host_publish_ns']+
                             [f'q_ref_{i}' for i in range(1,8)]+[f'q_actual_{i}' for i in range(1,8)]+
                             [f'qdot_ref_{i}' for i in range(1,8)]+[f'error_{i}' for i in range(1,8)])

    def on_state(self,message):
        received=time.monotonic_ns()
        if self.finished:return
        if not all(name in message.name for name in self.names):return
        sim=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if self.started_sim is None:
            if self.publisher.get_subscription_count()<1:return
            self.started_sim=sim
            self.get_logger().info(f'Trajectory starts at simulation time {sim:.6f}')
        t=sim-self.started_sim
        q,qd,_=reference(t,self.config['trajectory_duration_s'],self.initial,self.target)
        command=JointState();command.header=message.header
        command.name=self.names[:7];command.position=q[:7].tolist()
        self.publisher.publish(command)
        published=time.monotonic_ns()
        actual=np.array([message.position[message.name.index(n)] for n in self.names[:7]])
        if t>self.config['trajectory_duration_s']+1e-6:
            self.finished=True;self.file.flush()
            (self.out/'controller_done.json').write_text(json.dumps({'samples':self.count,'start_sim_time':self.started_sim,'end_sim_time':sim,'duration_s':self.config['trajectory_duration_s']}))
            return
        self.writer.writerow([self.count,sim,t,received,published,*q[:7],*actual,*qd[:7],*(q[:7]-actual)])
        self.count+=1
        if self.count%100==0:self.file.flush();self.get_logger().info(f'Recorded {self.count} samples at t={t:.2f}s')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();config=json.loads(args.config.read_text());args.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Experiment(config,args.out)
    wall_start=time.monotonic()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node,timeout_sec=.2)
            if time.monotonic()-wall_start>180:raise TimeoutError('No completed trajectory within 180 wall seconds')
        if node.count<450:raise RuntimeError(f'Insufficient feedback samples: {node.count}')
    finally:node.file.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
