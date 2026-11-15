"""Interface-only replay: coverage and alignment, without simulated robot dynamics."""
from pathlib import Path
import itertools,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from action_stream import ActionStream,reference,CASES
R=Path(__file__).resolve().parents[1]
q0=np.array([0,-.785398,0,-2.356194,0,1.570796,.785398,.04,.04])
# Verify that configurable defaults reproduce all commands already executed in Isaac.
verification={}
for case in CASES:
 x=np.genfromtxt(R/'runs'/f'execution_{case}'/'execution.csv',delimiter=',',names=True)
 replay=ActionStream(case,q0);difference=0.
 for row in x:
  measured=q0.astype(np.float32);measured[:7]=[row[f'actual{i}'] for i in range(7)]
  command,_,_,_=replay.update(float(row['t']),measured)
  difference=max(difference,max(abs(command[i]-row[f'command{i}']) for i in range(7)))
 assert difference<1e-10,(case,difference)
 verification[case]=difference
(R/'stream_replay_verification.json').write_text(json.dumps(verification,indent=2))
rows=[]
for delay,period,horizon,method in itertools.product([.04,.12,.24],[.1,.2,.4],[20,40,80],['delayed_naive','delayed_timestamp']):
 stream=ActionStream(method,q0,period=period,delay=delay,horizon_steps=horizon)
 errors=[];expired=[]
 for t in np.arange(0,5,.01):
  ideal=reference(t,q0);command,_,_,_=stream.update(float(t),ideal)
  if t>=delay:
   errors.extend((command[:3]-ideal[:3]).tolist());expired.append(stream.exhausted)
 rows.append({'delay_s':delay,'period_s':period,'horizon_s':horizon*.01,'method':method,'command_rmse_deg':float(np.rad2deg(np.sqrt(np.mean(np.array(errors)**2)))),'exhausted_fraction':float(np.mean(expired)),'coverage_margin_s':horizon*.01-delay-period})
(R/'stream_sweep.json').write_text(json.dumps(rows,indent=2))
fig,ax=plt.subplots(figsize=(9,4))
for horizon in [.2,.4,.8]:
 values=[row for row in rows if row['method']=='delayed_timestamp' and row['period_s']==.2 and row['horizon_s']==horizon]
 ax.plot([v['delay_s']*1000 for v in values],[v['exhausted_fraction']*100 for v in values],marker='o',label=f'Horizon {horizon:.1f}s')
ax.set(xlabel='Injected delay (ms)',ylabel='Expired chunk time (%)');ax.grid(alpha=.2);ax.legend();fig.tight_layout();fig.savefig(R/'figures'/'stream_coverage.png',dpi=160)
print('54 interface configurations; 7 executed-command replay checks passed')
