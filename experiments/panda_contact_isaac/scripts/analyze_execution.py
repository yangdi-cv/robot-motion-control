"""Evaluate synthetic action-stream tests using measured Isaac joint states."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from action_stream import CASES
R=Path(__file__).resolve().parents[1];out={};data={}
for case in CASES:
 d=R/'runs'/f'execution_{case}'
 if not (d/'COMPLETE').exists():raise RuntimeError(f'Incomplete: {case}')
 x=np.genfromtxt(d/'execution.csv',delimiter=',',names=True)
 audit=json.loads((d/'time_audit.json').read_text())
 assert len(x)==500 and abs(audit['measured_duration']-5)<1e-5
 data[case]=x
 actual=np.stack([x[f'actual{i}'] for i in range(7)],axis=1)
 command=np.stack([x[f'command{i}'] for i in range(7)],axis=1)
 oracle=np.stack([x[f'oracle{i}'] for i in range(7)],axis=1)
 velocity=np.diff(command,axis=0)/.01
 acceleration=np.diff(velocity,axis=0)/.01
 raw_force=np.genfromtxt(d/'physics_force.csv',delimiter=',',names=True)
 assert np.max(np.abs(raw_force['fz_N']))<1e-5
 if case in ('jitter_limited','outage_hold'):
  assert np.max(np.abs(velocity))<=.350001
  assert np.max(np.abs(acceleration))<=1.00001
 summary=json.loads((d/'stream_summary.json').read_text())
 out[case]={'actual_oracle_rmse_deg':float(np.rad2deg(np.sqrt(np.mean((actual[:,:3]-oracle[:,:3])**2)))),'command_oracle_rmse_deg':float(np.rad2deg(np.sqrt(np.mean((command[:,:3]-oracle[:,:3])**2)))),'actual_command_rmse_deg':float(np.rad2deg(np.sqrt(np.mean((actual[:,:3]-command[:,:3])**2)))),'command_max_step_deg':float(np.rad2deg(np.max(np.abs(np.diff(command,axis=0))))),'command_max_speed_rad_s':float(np.max(np.abs(velocity))),'command_max_acceleration_rad_s2':float(np.max(np.abs(acceleration))),'peak_effort_Nm':float(max(np.max(np.abs(x[f'effort{i}'])) for i in range(7))),'max_source_age_s':float(np.max(x['age_s'])),'timeout_duration_s':float(np.sum(x['timeout'])*.01),**summary}
(R/'execution_results.json').write_text(json.dumps(out,indent=2))
for name,cases in [('delay',['oracle','delayed_naive','delayed_timestamp']),('jitter',['jitter_raw','jitter_limited']),('outage',['outage_stale','outage_hold'])]:
 fig,axs=plt.subplots(2,1,figsize=(10,6),sharex=True)
 x=data[cases[0]];axs[0].plot(x['t'],np.rad2deg(x['oracle0']),'k--',label='Ideal reference')
 for case in cases:
  x=data[case];axs[0].plot(x['t'],np.rad2deg(x['actual0']),label=case)
  axs[1].plot(x['t'],np.rad2deg(x['command0']),label=case+' command')
 axs[0].set_ylabel('Actual joint 1 (deg)');axs[1].set_ylabel('Command joint 1 (deg)');axs[1].set_xlabel('Simulation time (s)')
 for ax in axs:ax.grid(alpha=.2);ax.legend(fontsize=8)
 if name=='outage':
  for ax in axs:ax.axvspan(2,3,color='orange',alpha=.15)
 fig.tight_layout();fig.savefig(R/'figures'/f'execution_{name}.png',dpi=160);plt.close(fig)
print(json.dumps(out,indent=2))
