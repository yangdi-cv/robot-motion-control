"""Separate statistics for sensing, reduced planning and physical execution."""
from pathlib import Path
from collections import Counter
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1];F=R/'figures';F.mkdir(exist_ok=True)
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'font.size':10})
LABEL={'capture':'落脚调整','lqr_predict':'仅状态预测','capture_predict':'预测＋落脚调整','fixed':'固定落脚/时刻','spatial':'仅空间调整','timing':'时空调整'}
def select(data,**criteria):return [v for v in data if all(v['config'].get(k)==x for k,x in criteria.items())]
def stats(values):return {'n':len(values),'passed':sum(v['success'] for v in values),'reasons':dict(Counter(v.get('reason',v.get('failure')) for v in values if not v['success']))}
def figsave(fig,name):fig.tight_layout();fig.savefig(F/name,dpi=165);plt.close(fig)
if __name__=='__main__':
 data={k:json.loads((R/f'{k}_results.json').read_text()) for k in ['network','gait','biped']}
 manifests={k:json.loads((R/f'{k}_manifest.json').read_text()) for k in data}
 groups={k:{g:[data[k][i] for i in ids] for g,ids in manifests[k]['groups'].items()} for k in data}
 summary={'totals':{k:stats(list(v.values())) for k,v in data.items()},'network_rates':[],'traffic_methods':{},'noise':[],'bursts':[],'support':[],'swing_limits':[],'timing_samples':[],'biped_cases':[]}
 for method in ['capture','capture_predict']:
  for sensor in [25,100,200]:
   for hz in [100,200]:
    vals=select(groups['network']['rates'],controller=method,sensor_hz=sensor,hz=hz,delay=.02,jitter=0.,loss=0.,impulse_x=6.)
    summary['network_rates'].append({'method':method,'sensor':sensor,'hz':hz,**stats(vals),'repeat_pct':float(np.mean([v['observer']['reused_fraction'] for v in vals])*100),'age95_ms':float(np.mean([v['observer']['age_p95_s'] for v in vals])*1000)})
 for method in ['capture','lqr_predict','capture_predict']:
  summary['traffic_methods'][method]=stats(select(groups['network']['rates'],controller=method))
  for tau,drift,label in [(0.,0.,'white'),(.08,0.,'colored'),(.08,.005,'drift')]:
   vals=select(groups['network']['colored'],controller=method,color_tau=tau,drift_position=drift)
   summary['noise'].append({'method':method,'mode':label,**stats(vals)})
  for threshold in [.15,.30,1.]:
   vals=select(groups['network']['bursts'],controller=method,burst_duration=.4,impulse_x=12.,max_age=threshold)
   summary['bursts'].append({'method':method,'max_age':threshold,**stats(vals),'stop_times_s':[v['duration_s'] for v in vals]})
 for method in ['fixed','spatial','timing']:
  for ds in [0.,.12,.20]:
   vals=select(groups['gait']['support'],method=method,ds=ds,impulse_x=12.,push_time=2.70)
   success=[v for v in vals if v['success']]
   summary['support'].append({'method':method,'ds':ds,**stats(vals),'duration_mean_s':float(np.mean([v['duration_s'] for v in success])) if success else None,'route_error_mean_m':float(np.mean([v['terminal_original_route_error_m'] for v in success])) if success else None})
  for speed in [1.2,1.8,2.4]:
   vals=select(groups['gait']['swing_limits'],method=method,swing_speed=speed)
   summary['swing_limits'].append({'method':method,'speed':speed,**stats(vals)})
  vals=select(groups['gait']['support'],method=method,ds=.12,impulse_x=12.,push_time=2.70,seed=0)
  if vals:summary['timing_samples'].append({'method':method,**vals[0]})
 for key,v in data['biped'].items():
  x=np.load(R/manifests['biped']['root']/key/'trace.npz')['data'];c=v['config']
  useful=x[:,0]>.5
  deltas=np.max(np.abs(np.diff(x[useful,10:16],axis=0))) if useful.sum()>1 else 0.
  feet=np.stack([x[:,4:6],x[:,6:8]],axis=1)
  speed=np.linalg.norm(np.diff(feet,axis=0)/c['dt'],axis=2).max() if len(x)>1 else 0.
  summary['biped_cases'].append({'id':key,**v,'peak_applied_torque_Nm':np.max(np.abs(x[:,10:16]),axis=0).tolist() if len(x) else [],'max_poststart_torque_step_Nm':float(deltas),'max_measured_foot_speed_m_s':float(speed)})
 (R/'summary.json').write_text(json.dumps(summary,indent=2))
 fig,axs=plt.subplots(1,2,figsize=(11,4.5))
 for sensor in [25,100,200]:
  values=[v for v in summary['network_rates'] if v['method']=='capture_predict' and v['sensor']==sensor]
  axs[0].plot([v['hz'] for v in values],[v['repeat_pct'] for v in values],'o-',label=f'传感器{sensor}Hz')
  axs[1].plot([v['hz'] for v in values],[v['age95_ms'] for v in values],'o-',label=f'传感器{sensor}Hz')
 axs[0].set(xlabel='控制频率 / Hz',ylabel='重复使用旧测量 / %',title='高控制频率不会生成新测量')
 axs[1].set(xlabel='控制频率 / Hz',ylabel='控制时刻数据年龄P95 / ms',title='传输延迟20ms；无丢帧/抖动')
 for a in axs:a.legend();a.grid(alpha=.2)
 figsave(fig,'sensor_rates.png')
 fig,axs=plt.subplots(1,2,figsize=(11,4.5))
 modes=['white','colored','drift']
 for i,method in enumerate(['capture','lqr_predict','capture_predict']):
  vals=[next(v for v in summary['noise'] if v['method']==method and v['mode']==mode) for mode in modes]
  axs[0].bar(np.arange(3)+i*.25,[v['passed']/v['n'] for v in vals],width=.25,label=LABEL[method])
 axs[0].set_xticks(np.arange(3)+.25,['白噪声','有色噪声','有色＋漂移']);axs[0].set(ylabel='固定测试组合通过比例',ylim=(0,1.05));axs[0].legend(fontsize=8)
 for method in ['capture','lqr_predict','capture_predict']:
  v=summary['traffic_methods'][method];axs[1].bar(LABEL[method],v['passed']/v['n'])
 axs[1].set(ylabel='时序主网格通过比例',ylim=(0,1.05));axs[1].tick_params(axis='x',labelsize=8)
 figsave(fig,'noise_traffic.png')
 fig,axs=plt.subplots(1,2,figsize=(11,4.5))
 for method in ['fixed','spatial','timing']:
  values=[v for v in summary['support'] if v['method']==method]
  axs[0].plot([v['ds'] for v in values],[v['passed']/v['n'] for v in values],'o-',label=LABEL[method])
  values=[v for v in summary['swing_limits'] if v['method']==method]
  axs[1].plot([v['speed'] for v in values],[v['passed']/v['n'] for v in values],'o-',label=LABEL[method])
 axs[0].set(xlabel='双支撑时长 / s',ylabel='12N·s、2.70s推扰：通过比例',ylim=(-.05,1.05))
 axs[1].set(xlabel='连续摆脚速度上界 / m/s',ylabel='固定组合通过比例',ylim=(-.05,1.05))
 for a in axs:a.legend(fontsize=8);a.grid(alpha=.2)
 figsave(fig,'support_timing.png')
 # Matched physical successful walk and the insufficient-torque case.
 fig,axs=plt.subplots(4,1,figsize=(10,10))
 for scale,color in [(1.,'#218259'),(.5,'#b65543')]:
  values=[v for v in summary['biped_cases'] if v['config']['walk'] and v['config']['step_length']==.12 and v['config']['hz']==100 and v['config']['push']==40. and v['config']['friction']==.8 and v['config']['ds']==.12 and v['config']['dt']==.001 and v['config']['torque_scale']==scale]
  v=values[0];x=np.load(R/manifests['biped']['root']/v['id']/'trace.npz')['data']
  axs[0].plot(x[:,0],x[:,1],label=f'力矩上限×{scale}',color=color)
  axs[1].plot(x[:,0],np.rad2deg(x[:,3]),color=color)
  axs[2].plot(x[:,0],x[:,4],color=color,label='左脚');axs[2].plot(x[:,0],x[:,6],color=color,ls='--',label='右脚')
  axs[3].plot(x[:,0],np.max(np.abs(x[:,10:16]),axis=1),color=color)
 for a in axs:a.axvspan(2.7,2.8,color='red',alpha=.1);a.grid(alpha=.2)
 axs[0].legend();axs[0].set(ylabel='质心前向位置 / m',title='真实接触平面双足：相同40N、0.1s推扰')
 axs[1].set_ylabel('基座俯仰 / °');axs[2].set_ylabel('实际足底前向位置 / m');axs[3].set(ylabel='关节力矩绝对峰值 / Nm',xlabel='物理时间 / s');figsave(fig,'biped_constraints.png')
 nominal=[v for v in summary['biped_cases'] if v['config']==dict(walk=True,step_length=.12,step_time=.6,ds=.12,steps=8,hz=100,dt=.001,duration=6.,push=0.,friction=.8,torque_scale=1.,record=False,seed=0)][0]
 x=np.load(R/manifests['biped']['root']/nominal['id']/'trace.npz')['data']
 fig,axs=plt.subplots(3,1,figsize=(10,8))
 axs[0].plot(x[:,0],x[:,4],label='左足');axs[0].plot(x[:,0],x[:,6],label='右足');axs[0].set(ylabel='足底前向位置 / m',title='8次实测落脚：足位、离地与地面反力');axs[0].legend()
 axs[1].plot(x[:,0],x[:,5],label='左足');axs[1].plot(x[:,0],x[:,7],label='右足');axs[1].set_ylabel('足底离地高度 / m')
 axs[2].plot(x[:,0],x[:,8],label='左足');axs[2].plot(x[:,0],x[:,9],label='右足');axs[2].set(ylabel='实测法向反力 / N',xlabel='物理时间 / s')
 for a in axs:a.grid(alpha=.2)
 figsave(fig,'measured_steps.png')
 print({k:v for k,v in summary['totals'].items()})
