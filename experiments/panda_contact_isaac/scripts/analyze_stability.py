"""Evaluate all stability experiments on identical unfiltered 10ms force bins."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1]
NAMES=['pid_contact_gate','stability_freeze','stability_freeze_ema','stability_freeze_last','stability_freeze_5ms','stability_freeze_20ms','stability_ik_previous','stability_ik_tight','stability_differential_moving','stability_differential_frozen','stability_differential_ema','stability_differential_5ms','stability_validation_repeat','stability_validation_higher','stability_validation_lower']
results={};signals={}
for name in NAMES:
 d=R/'runs'/name
 if not (d/'COMPLETE').exists():continue
 raw=np.genfromtxt(d/'physics_force.csv',delimiter=',',names=True)
 x=np.genfromtxt(d/'trace.csv',delimiter=',',names=True)
 meta=json.loads((d/'metadata.json').read_text());clock=json.loads((d/'time_audit.json').read_text())
 assert abs(clock['measured_duration']-clock['expected_duration'])<1e-5
 assert len(raw)%10==0
 force=raw['fz_N'].reshape(-1,10).mean(axis=1)
 time=raw['t'].reshape(-1,10)[:,-1]
 signals[name]=(time,force)
 tail=time>time[-1]-2
 band=np.abs(force-5)<=.5
 hit=np.flatnonzero((time>=5)&(force>.5))
 stable=None;longest=0;length=0
 for i in range(len(force)):
  length=length+1 if band[i] else 0
  longest=max(longest,length)
  if length>=51 and stable is None and len(hit):stable=float(time[i-50]-time[hit[0]])
 dt=float(meta['control_dt']);assert len(x)==round(clock['expected_duration']/dt)
 qref=np.stack([x[f'ref{i}'] for i in range(7)],axis=1)
 joint_step=np.rad2deg(np.max(np.abs(np.diff(qref[x['t']>18],axis=0))))
 results[name]={'native_control_dt_s':dt,'force_evaluation_bin_s':.01,'raw_peak_N':float(raw['fz_N'].max()),'last2s_mean_N':float(np.mean(force[tail])),'last2s_rmse_N':float(np.sqrt(np.mean((force[tail]-5)**2))),'last2s_fraction_in_band':float(np.mean(band[tail])),'strict_settle_time_s':stable,'longest_band_span_s':max(0,(longest-1)*.01),'last2s_joint_command_max_step_deg':float(joint_step),'max_joint_reference_step_deg':float(np.rad2deg(np.max(np.abs(np.diff(qref,axis=0))))),'ik_seed':meta.get('ik_seed','measured'),'ik_tolerance_m':meta.get('ik_position_tolerance_m'),'height_shift_m':meta.get('plane_height_shift_m',0.)}
(R/'stability_results.json').write_text(json.dumps(results,indent=2))
for stem,names in [('stability_factors',NAMES[:6]),('stability_ik',['stability_freeze','stability_ik_previous','stability_differential_moving','stability_differential_frozen'])]:
 fig,axs=plt.subplots(2,1,figsize=(10,7))
 for name in names:
  if name not in signals:continue
  t,f=signals[name];axs[0].plot(t,f,label=name,lw=.7)
  mask=t>18;axs[1].plot(t[mask],f[mask],label=name,lw=.8)
 for ax in axs:
  ax.axhspan(4.5,5.5,color='green',alpha=.12);ax.axhline(5,color='black',ls='--');ax.grid(alpha=.2);ax.set_ylabel('10ms mean of raw force (N)');ax.legend(fontsize=7)
 axs[1].set_xlabel('Simulation time (s)');fig.tight_layout();fig.savefig(R/'figures'/f'{stem}.png',dpi=160);plt.close(fig)
print(json.dumps(results,indent=2))

fig,axs=plt.subplots(2,1,figsize=(10,6))
for name in ['stability_differential_ema','stability_validation_repeat','stability_validation_higher','stability_validation_lower']:
 if name not in signals:continue
 t,f=signals[name];axs[0].plot(t,f,label=name,lw=.7);mask=t>18;axs[1].plot(t[mask],f[mask],label=name,lw=.7)
for ax in axs:
 ax.axhspan(4.5,5.5,color='green',alpha=.12);ax.axhline(5,color='black',ls='--');ax.grid(alpha=.2);ax.set_ylabel('10ms mean of raw force (N)');ax.legend(fontsize=7)
axs[1].set_xlabel('Simulation time (s)');fig.tight_layout();fig.savefig(R/'figures/stability_validation.png',dpi=160);plt.close(fig)
