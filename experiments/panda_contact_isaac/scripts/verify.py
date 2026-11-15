"""Check recorded timing, trajectory, and paired-condition consistency."""
from pathlib import Path
import json
import numpy as np
root=Path(__file__).resolve().parents[1]
results=json.loads((root/'results.json').read_text())
checks={}
for name in results:
 d=root/'runs'/name
 clock=json.loads((d/'time_audit.json').read_text())
 x=np.genfromtxt(d/'trace.csv',delimiter=',',names=True)
 f=np.genfromtxt(d/'physics_force.csv',delimiter=',',names=True)
 duration=clock['expected_duration']
 assert abs(clock['measured_duration']-duration)<1e-5
 assert len(x)==round(duration*100) and len(f)==round(duration*1000)
 assert all(np.all(np.isfinite(x[k])) for k in x.dtype.names)
 assert np.all(np.isfinite(f['fz_N']))
 assert np.max(np.abs(np.diff(x['t'])-.01))<1e-8
 assert np.max(np.abs(f['fz_N'][f['t']<=5]))<1e-5
 u=x['t'][:500]/5;s=10*u**3-15*u**4+6*u**5
 initial=[0,-.785398,0];delta=[.06,.04,-.04]
 for i in range(3):assert np.max(np.abs(x[f'ref{i}'][:500]-(initial[i]+delta[i]*s)))<1e-12
 checks[name]={'timing_and_rows':'pass','free_space_force':'pass','quintic_reference':'pass'}
a=np.genfromtxt(root/'runs/position_final/trace.csv',delimiter=',',names=True)
b=np.genfromtxt(root/'runs/admittance_final/trace.csv',delimiter=',',names=True)
max_diff=max(float(np.max(np.abs(a[f'q{i}'][:500]-b[f'q{i}'][:500]))) for i in range(7))
assert max_diff<1e-6
checks['paired_first5s_max_difference_rad']=max_diff
if 'admittance_repeat' in results:
 c=np.genfromtxt(root/'runs/admittance_repeat/trace.csv',delimiter=',',names=True)
 checks['repeat_max_force_difference_N']=float(np.max(np.abs(c['fz']-b['fz'])))
a=np.genfromtxt(root/'runs/pid_position/trace.csv',delimiter=',',names=True)
b=np.genfromtxt(root/'runs/pid_admittance/trace.csv',delimiter=',',names=True)
paired=max(float(np.max(np.abs(a[f'q{i}'][:500]-b[f'q{i}'][:500]))) for i in range(7))
assert paired<1e-6
checks['ordinary_pid_paired_first5s_difference_rad']=paired
(root/'verification.json').write_text(json.dumps(checks,indent=2))
print(json.dumps(checks,indent=2))
