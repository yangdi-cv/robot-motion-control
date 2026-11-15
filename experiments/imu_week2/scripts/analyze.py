"""Analyze recorded Isaac sensor data and real ROS feedback; injected noise is explicit."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from filters import FilterBank
p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();out=args.run;fig=out/'figures';fig.mkdir(exist_ok=True)
def read(name):return np.genfromtxt(out/name,delimiter=',',names=True)
track=read('tracking.csv');relay=read('relay.csv');imu=read('sim_imu.csv');cfg=json.loads((out/'config.json').read_text())
assert np.isfinite(np.column_stack([imu[n] for n in imu.dtype.names])).all()
assert np.max(np.linalg.norm(np.column_stack([imu[f'gyro_{a}_rad_s'] for a in 'xyz']),axis=1))>.01,'IMU motion missing'
joined={int(r['seq']):r for r in relay};metrics=[]
for phase,rate in enumerate(cfg['feedback_rates_hz']):
 d=track[track['phase']==phase];assert len(d)>rate*8
 times=d['host_receive_ns']/1e9;delta=np.diff(times);motion=(d['phase_time_s']>=2)&(d['phase_time_s']<=7)
 err=np.column_stack([d[f'q_ref_{i}']-d[f'q_actual_{i}'] for i in range(1,4)])
 latency=np.array([(r['host_receive_ns']-joined[int(r['seq'])]['relay_publish_ns'])/1e6 for r in d])
 assert np.min(latency)>=0
 row={'phase':phase,'requested_hz':rate,'samples':len(d),'observed_wall_hz':(len(d)-1)/(times[-1]-times[0]),'sim_hz':(len(d)-1)/(d['sim_time_s'][-1]-d['sim_time_s'][0]),'interval_p50_ms':np.median(delta)*1e3,'interval_p95_ms':np.percentile(delta,95)*1e3,'relay_latency_p50_ms':np.median(latency),'relay_latency_p95_ms':np.percentile(latency,95),'rmse_joint1_rad':np.sqrt(np.mean(err[motion,0]**2)),'rmse_joint2_rad':np.sqrt(np.mean(err[motion,1]**2)),'rmse_joint3_rad':np.sqrt(np.mean(err[motion,2]**2)),'max_abs_joint1_rad':np.max(abs(err[motion,0]))}
 # Apparent plant/command tracking delay: a shift-fit descriptor, not transport latency.
 q0=np.array(cfg['initial_positions_rad']);qf=np.array(cfg['target_positions_rad'])
 if phase%2:q0,qf=qf,q0
 from trajectory import reference
 lag_grid=np.arange(0,.401,.005);scores=[]
 for lag in lag_grid:
  expected=np.array([reference(pt-2-lag,5,q0,qf)[0][0] for pt in d['phase_time_s'][motion]])
  scores.append(np.mean((expected-d['q_actual_1'][motion])**2))
 row['apparent_joint1_tracking_lag_ms']=float(lag_grid[int(np.argmin(scores))]*1000)
 row['motion_direction']='forward' if phase%2==0 else 'reverse'
 metrics.append(row)
with (out/'frequency_metrics.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(metrics[0]));w.writeheader();w.writerows(metrics)
seq=track['seq'].astype(int);loss=len(set(joined)-set(seq));duplicates=len(seq)-len(set(seq))
# Decimate physics IMU to the existing hardware's approximately25Hz as a modeled operating point.
imu=imu[::4];t=imu['sim_time_s'];clean=np.column_stack([imu[f'accel_{a}_m_s2'] for a in 'xyz']+[imu[f'gyro_{a}_rad_s'] for a in 'xyz'])
rng=np.random.default_rng(20261013);sigma=np.array([.15]*3+[.02]*3);noisy=clean+rng.normal(size=clean.shape)*sigma
bank=FilterBank(rate=25,cutoff=3,window=5);outputs={key:[] for key in ('raw','moving_average','ema','butterworth')}
for n,x in enumerate(noisy):
 for key,value in bank.update(x,None if n==0 else t[n]-t[n-1]).items():outputs[key].append(value)
outputs={k:np.array(v) for k,v in outputs.items()};anchor=track['sim_time_s'][0];local=(t-anchor)%9
static=((t>=anchor)&(t<anchor+len(cfg['feedback_rates_hz'])*cfg['phase_duration_s'])&(((local>=1)&(local<2))|((local>=8)&(local<9))))
motion=(t>=anchor)&(t<anchor+len(cfg['feedback_rates_hz'])*cfg['phase_duration_s'])&(local>=2)&(local<=7)
assert static.sum()>200 and motion.sum()>900
filtermetrics=[]
for name,y in outputs.items():
 e=y-clean
 for kind,sl in [('acceleration',slice(0,3)),('gyro',slice(3,6))]:
  base=np.sqrt(np.mean((noisy-clean)[static,sl]**2));staticrmse=np.sqrt(np.mean(e[static,sl]**2));rmse=np.sqrt(np.mean(e[motion,sl]**2))
  # Descriptive delay: shift output against earlier clean values. Never used for the RMSE reported above.
  scores=[]
  for lag in range(11):
   indices=np.flatnonzero(motion);indices=indices[indices>=lag];scores.append(np.mean((y[indices,sl]-clean[indices-lag,sl])**2))
  lag=int(np.argmin(scores));filtermetrics.append({'filter':name,'signal':kind,'static_error_rmse':staticrmse,'static_error_reduction_pct':100*(1-staticrmse/base),'motion_error_rmse':rmse,'best_shift_delay_ms':lag*40})
with (out/'filter_metrics.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(filtermetrics[0]));w.writeheader();w.writerows(filtermetrics)
with (out/'imu_filter_data.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['sim_time_s']+[f'{name}_{a}' for name in ('clean',*outputs) for a in ('ax','ay','az','gx','gy','gz')]);w.writerows(np.column_stack([t,clean,*outputs.values()]))
summary={'frequency_trials':metrics,'feedback_missing_sequences':loss,'feedback_duplicates':duplicates,'filter_metrics':filtermetrics,'imu_samples_25hz':len(t),'static_samples':int(static.sum()),'motion_samples':int(motion.sum()),'noise':{'seed':20261013,'acceleration_sigma_m_s2':.15,'gyro_sigma_rad_s':.02,'type':'independent zero-mean Gaussian added offline; NOT physical IMU noise measurement'},'filter_settings':{'rate_hz':25,'cutoff_hz':3,'moving_average_window':5,'butterworth_order':2},'notes':'Actual Isaac virtual sensor truth + explicit synthetic noise; rate trials use real ROS Bridge feedback. Relay latency excludes Isaac-to-relay transport and is not end-to-end command response.'}
(out/'metrics.json').write_text(json.dumps(summary,indent=2))
fig1,axes=plt.subplots(1,3,figsize=(13,4))
for axis,key,label in zip(axes,['observed_wall_hz','relay_latency_p95_ms','rmse_joint1_rad'],['Observed host feedback Hz','Relay DDS latency P95 ms','Joint1 motion RMSE rad']):
 for rate in (25,50,100):
  vals=[r[key] for r in metrics if r['requested_hz']==rate];axis.scatter([rate]*len(vals),vals);axis.plot(rate,np.mean(vals),'kx')
 axis.set_xticks([25,50,100]);axis.set_xlabel('Requested feedback Hz');axis.set_ylabel(label);axis.grid(alpha=.3)
fig1.tight_layout();fig1.savefig(fig/'frequency_comparison.png',dpi=150);plt.close(fig1)
fig1,axes=plt.subplots(2,1,figsize=(12,7),sharex=True);mask=(t>=anchor)&(t<=anchor+9)
for ax,col,title in zip(axes,[0,5],['Acceleration X (m/s2)','Gyroscope Z (rad/s)']):
 ax.plot(t[mask]-anchor,clean[mask,col],color='black',label='Clean Isaac IMU',lw=2)
 for name,y in outputs.items():ax.plot(t[mask]-anchor,y[mask,col],label=name,alpha=.7,lw=1)
 ax.set_ylabel(title);ax.grid(alpha=.3);ax.legend(ncol=3)
axes[-1].set_xlabel('Trial time s');fig1.tight_layout();fig1.savefig(fig/'imu_filter_comparison.png',dpi=150);plt.close(fig1)
print(json.dumps({'trials':len(metrics),'missing':loss,'filter_samples':len(t)},indent=2))
