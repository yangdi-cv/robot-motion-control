"""Reproducible process orchestration, with logs for every command and failure."""
import argparse,json,os,re,signal,subprocess,time,shutil,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];TASK=ROOT/'experiments/panda_task1'
parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True);args=parser.parse_args();out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
config=TASK/'config/experiment.json';(out/'config.json').write_bytes(config.read_bytes())
snapshot=out/'source_snapshot';snapshot.mkdir()
for source in (TASK/'scripts').iterdir():
 if source.is_file():shutil.copy2(source,snapshot/source.name)
(out/'source_sha256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir()},indent=2))
commands=[];processes=[];logs=[]
def spawn(cmd,name):
 log=(out/name).open('w');logs.append(log)
 commands.append({'command':list(map(str,cmd)),'log':name,'started_unix_ns':time.time_ns()});(out/'commands.json').write_text(json.dumps(commands,indent=2))
 process=subprocess.Popen(list(map(str,cmd)),cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(process);return process
sim=spawn([TASK/'scripts/isaac-python',TASK/'scripts/isaac_scene.py','--config',config,'--out',out,'--gui'],'isaac.log')
try:
 started=time.monotonic()
 while not (out/'sim_ready').exists():
  if sim.poll() is not None:raise RuntimeError(f'Isaac exited {sim.returncode}; inspect isaac.log')
  if time.monotonic()-started>150:raise TimeoutError('Isaac scene startup timeout')
  time.sleep(.2)
 tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
 match=re.search(r'(0x[0-9a-f]+) "Isaac Sim Python 5\.1\.0"',tree)
 if not match:raise RuntimeError('Isaac GUI window not found')
 record=spawn([ROOT/'SO101/code/scripts/ubuntu/project-python',TASK/'scripts/record_window.py','--window-id',match.group(1),'--out',out],'recording.log')
 deadline=time.monotonic()+15
 while not (out/'recording_ready').exists():
  if record.poll() is not None:raise RuntimeError('Recording failed; inspect recording.log')
  if time.monotonic()>deadline:raise TimeoutError('Recorder startup timeout')
  time.sleep(.1)
 controller=spawn([TASK/'scripts/ros-python',TASK/'scripts/controller.py','--config',config,'--out',out],'controller.log')
 # Preserve ROS endpoint evidence while the graph and controller are live.
 time.sleep(1)
 diagnostic='source /opt/ros/humble/setup.bash\nexport ROS_DOMAIN_ID=51 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp\nros2 node list\nros2 topic list\nros2 topic info /panda_task1/joint_command -v\nros2 topic info /panda_task1/joint_states -v'
 (out/'ros_diagnostics_command.sh').write_text(diagnostic)
 with (out/'ros_graph.txt').open('w') as f:subprocess.run(['bash',str(out/'ros_diagnostics_command.sh')],stdout=f,stderr=subprocess.STDOUT,timeout=15)
 code=controller.wait(timeout=180)
 if code:raise RuntimeError(f'Controller exited {code}')
 sim.wait(timeout=30);record.wait(timeout=20)
 if sim.returncode or record.returncode:raise RuntimeError('Simulation or video recorder failed')
 analyze=spawn(['/usr/bin/python3',TASK/'scripts/analyze.py',out],'analysis.log')
 if analyze.wait(timeout=30):raise RuntimeError('Analysis failed')
 (out/'RUN_COMPLETE').touch();print(f'Completed: {out}',flush=True)
finally:
 for process in processes:
  if process.poll() is None:
   try:os.killpg(process.pid,signal.SIGINT);process.wait(timeout=10)
   except (ProcessLookupError,subprocess.TimeoutExpired):process.kill()
 for log in logs:log.close()
 for entry,process in zip(commands,processes):entry['exit_code']=process.poll()
 (out/'commands.json').write_text(json.dumps(commands,indent=2))
