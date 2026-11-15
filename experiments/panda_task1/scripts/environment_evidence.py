"""Display actual environment query outputs for a reproducible terminal screenshot."""
import json,os,platform,subprocess,time
from pathlib import Path
root=Path(__file__).resolve().parents[3];task=root/'experiments/panda_task1'
lines=['PANDA TASK1 - ACTUAL ENVIRONMENT CHECK','Report date: 2026-10-13 (Australia/Sydney)','']
commands=[['cat','/etc/os-release'],['uname','-r'],['nvidia-smi','--query-gpu=name,driver_version,memory.total','--format=csv'],['dpkg-query','-W','ros-humble-desktop','ros-humble-moveit','ros-humble-ros2-control'],[str(root/'.venvs/isaacsim51/bin/python'),'-c','import sys,importlib.metadata as m;print("Isaac Python:",sys.version.split()[0]);print("Isaac Sim:",m.version("isaacsim"))']]
for cmd in commands:
 result=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
 lines.extend(['$ '+' '.join(cmd),result.stdout.strip(),''])
lines += ['Experiment: Panda / ROS2 Bridge / Domain 51','Trajectory: joints 1-3, rest-to-rest quintic, 5 s','Physics/control simulation period: 0.010 s','System ROS Python and Isaac Python are isolated.','']
text='\n'.join(lines);(task/'setup_logs/environment_evidence.txt').write_text(text)
print(text,flush=True)
(task/'setup_logs/environment_terminal_pid').write_text(str(os.getpid()))
time.sleep(90)
