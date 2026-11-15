"""Consistency checks against original messages, timestamps and recorded sensor truth."""
from pathlib import Path
import csv,json,ast,re
import numpy as np
ROOT=Path(__file__).resolve().parents[1];run=ROOT/'runs/run002_balanced';m=json.loads((run/'metrics.json').read_text())
assert len(m['frequency_trials'])==12
for rate in (25,50,100):
 trials=[x for x in m['frequency_trials'] if x['requested_hz']==rate]
 assert len(trials)==4 and [x['motion_direction'] for x in trials].count('forward')==2
 assert all(abs(x['sim_hz']-rate)<.001 for x in trials)
assert m['feedback_missing_sequences']==m['feedback_duplicates']==0
imu=np.genfromtxt(run/'sim_imu.csv',delimiter=',',names=True)
assert np.all(np.diff(imu['sensor_time_s'])>0)
q=np.column_stack([imu[f'quat_{axis}'] for axis in 'wxyz']);assert np.max(abs(np.linalg.norm(q,axis=1)-1))<.01
assert np.max(abs(imu['gyro_z_rad_s']))>.01
checks={'balanced_frequency_trials':12,'imu_physics_frames':len(imu),'quaternion_norm_checked':True,'sensor_clock_monotonic':True,'missing_feedback_sequences':0,'script_parse_count':0,'relative_links_checked':0}
for p in (ROOT/'scripts').glob('*.py'):ast.parse(p.read_text());checks['script_parse_count']+=1
for p in ROOT.glob('*.md'):
 for link in re.findall(r'\]\(([^)]+)\)',p.read_text()):
  if '://' not in link:assert (p.parent/link).exists(),link;checks['relative_links_checked']+=1
(ROOT/'VERIFICATION.json').write_text(json.dumps(checks,indent=2));print(checks)
