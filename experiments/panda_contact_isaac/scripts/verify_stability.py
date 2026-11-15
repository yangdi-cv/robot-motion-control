"""Validate fixed-bin force evaluation and the continuity/abort checks."""
from pathlib import Path
import json
import numpy as np
R=Path(__file__).resolve().parents[1]
S=json.loads((R/'stability_results.json').read_text());checks={}
for name,metrics in S.items():
 d=R/'runs'/name
 raw=np.genfromtxt(d/'physics_force.csv',delimiter=',',names=True)
 x=np.genfromtxt(d/'trace.csv',delimiter=',',names=True)
 meta=json.loads((d/'metadata.json').read_text());clock=json.loads((d/'time_audit.json').read_text())
 dt=meta['control_dt'];duration=clock['expected_duration']
 assert len(raw)==round(duration*1000) and len(x)==round(duration/dt)
 assert abs(clock['measured_duration']-duration)<1e-5
 assert all(np.isfinite(x[k]).all() for k in x.dtype.names)
 force=raw['fz_N'].reshape(-1,10).mean(axis=1);time=raw['t'].reshape(-1,10)[:,-1]
 inside=abs(force-5)<=.5
 valid_windows=[i for i in range(len(inside)-50) if inside[i:i+51].all()]
 assert bool(valid_windows)==(metrics['strict_settle_time_s'] is not None)
 assert abs(np.sqrt(np.mean((force[time>time[-1]-2]-5)**2))-metrics['last2s_rmse_N'])<1e-10
 if meta.get('ik_seed')=='differential':
  ref=np.stack([x[f'ref{i}'] for i in range(7)],axis=1)
  assert np.max(abs(np.diff(ref,axis=0)))<=.003000001
 checks[name]={'rows_and_clock':'pass','finite':'pass','unfiltered_fixed_bin_metric':'pass','has_0_5s_window':bool(valid_windows)}
for name in ['stability_differential_ema','stability_validation_repeat','stability_validation_higher','stability_validation_lower']:
 assert checks[name]['has_0_5s_window']
 # Explicitly retain the fact that the whole final interval is not all in band.
 assert S[name]['last2s_fraction_in_band']<1
G=R/'runs/stability_ik_guard';failure=json.loads((G/'FAILED.json').read_text())
assert failure['stage']=='IK_jump_guard' and failure['max_jump_rad']>failure['limit_rad']
assert not (G/'COMPLETE').exists()
checks['jump_guard']={'status':'pass','abort_time_s':failure['t'],'proposed_jump_deg':float(np.rad2deg(failure['max_jump_rad']))}
(R/'stability_verification.json').write_text(json.dumps(checks,indent=2))
print('PASS: native 5/10/20ms timing; unfiltered 10ms-bin criterion; 4 candidate/validation windows; bounded differential references; guarded abort')
