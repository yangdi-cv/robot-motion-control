"""Run isolated Isaac sensor and ROS frequency trials, preserving sources/logs."""
import argparse,json,os,signal,subprocess,time,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];TASK=ROOT/'experiments/imu_week2'
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);args=p.parse_args();out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
config=TASK/'config/experiment.json';shutil.copy2(config,out/'config.json');shutil.copytree(TASK/'scripts',out/'source_snapshot',ignore=shutil.ignore_patterns('__pycache__'))
processes=[];logs=[];commands=[]
def spawn(cmd,name):
 f=(out/name).open('w');logs.append(f);commands.append(list(map(str,cmd)));(out/'commands.json').write_text(json.dumps(commands,indent=2));proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,cwd=ROOT,start_new_session=True);processes.append(proc);return proc
sim=spawn([str(TASK/'scripts/isaac-python'),str(TASK/'scripts/isaac_scene.py'),'--config',str(config),'--out',str(out)],'isaac.log')
try:
 deadline=time.monotonic()+150
 while not (out/'sim_ready').exists():
  if sim.poll() is not None:raise RuntimeError('Isaac failed; inspect isaac.log')
  if time.monotonic()>deadline:raise TimeoutError('Isaac startup')
  time.sleep(.2)
 relay=spawn([str(TASK/'scripts/ros-python'),str(TASK/'scripts/relay.py'),'--config',str(config),'--out',str(out)],'relay.log')
 controller=spawn([str(TASK/'scripts/ros-python'),str(TASK/'scripts/controller.py'),'--config',str(config),'--out',str(out)],'controller.log')
 for proc in (controller,relay,sim):
  if proc.wait(timeout=230):raise RuntimeError('Process failed')
 (out/'RUN_COMPLETE').touch();print('COMPLETE',out,flush=True)
finally:
 for proc in processes:
  if proc.poll() is None:
   try:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=10)
   except (ProcessLookupError,subprocess.TimeoutExpired):proc.kill();proc.wait()
 for f in logs:f.close()
 (out/'exit_codes.json').write_text(json.dumps([p.poll() for p in processes]))
