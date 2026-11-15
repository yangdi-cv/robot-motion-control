"""Body-frame command tracking, world path, support and stop metrics."""
from pathlib import Path
import argparse,json,csv,sys
import numpy as np
from core import ROOT,atomic_json
from protocol import THRESHOLDS

def arrays(case):
 paths=sorted(case.glob('part_*.npz'));keys=np.load(paths[0]).files
 return {k:np.concatenate([np.load(p)[k] for p in paths]) for k in keys}
def rms(x):return np.sqrt(np.mean(np.square(x),axis=0))
def touchdown_events(t,force,clearance,dt):
 events=[]
 for side in range(2):
  air=0.;loaded=0.;armed=False
  for i in range(len(t)):
   off=force[i,side]<10 and clearance[i,side]>.005
   air=air+dt if off else 0.
   if air>=.02:armed=True
   loaded=loaded+dt if force[i,side]>10 else 0.
   if armed and loaded>=.016:
    events.append(dict(side=side,time=float(t[i])));armed=False;air=0.
 return sorted(events,key=lambda x:x['time'])
def stop_metrics(t,v,w,start,end,dt):
 mask=(t>=start)&(t<end);idx=np.where(mask)[0];run=0.;settle=None
 for i in idx:
  ok=np.linalg.norm(v[i,:2])<=THRESHOLDS['stop_speed_mps'] and abs(w[i])<=THRESHOLDS['stop_yaw_rate_radps']
  run=run+dt if ok else 0.
  if run>=THRESHOLDS['stop_hold_s']:
   settle=max(0.,float(t[i]-start-run+dt));break
 last=idx[t[idx]>=end-2.]
 return dict(command_start_s=start,settle_delay_s=settle,final2_speed_mean_mps=float(np.mean(np.linalg.norm(v[last,:2],axis=1))),final2_yaw_rate_abs_mean_radps=float(np.mean(np.abs(w[last]))))
def score(folder):
 r=json.loads((folder/'result.json').read_text());a=arrays(folder);t=a['time'];dt=float(np.median(np.diff(t)));case=r['case'];N=len(t)
 assert np.max(np.abs(np.diff(t)-dt))<1e-9
 steady=np.zeros(N,bool)
 for j,(start,cmd) in enumerate(case['segments']):
  end=case['segments'][j+1][0] if j+1<len(case['segments']) else case['duration'];steady|=(t>=start+THRESHOLDS['settle_s']-1e-8)&(t<end)
 if not steady.any():steady=t>=max(0.,t[-1]-1)
 v=a['base_body_velocity'];omega=a['qvel'][:,5];command=a['command'];error=np.c_[v[:,:2]-command[:,:2],omega-command[:,2]];rmse=rms(error[steady]);yaw=np.unwrap(a['rpy_tilt'][:,2]);heading=yaw-yaw[0]
 loaded=a['foot_fz']>THRESHOLDS['contact_force_N'];weights=a['slip_weight'][steady];slip=a['contact_slip'][steady]
 nominal=np.zeros((N,3))
 for i in range(1,N):
  c,s=np.cos(nominal[i-1,2]),np.sin(nominal[i-1,2]);vx,vy,wz=command[i-1];nominal[i]=nominal[i-1]+dt*np.array([c*vx-s*vy,s*vx+c*vy,wz])
 events=touchdown_events(t,a['foot_fz'],a['foot_lowest_z'],dt)
 stop=[]
 for j,(start,cmd) in enumerate(case['segments']):
  if j and np.linalg.norm(cmd)<1e-12 and np.linalg.norm(case['segments'][j-1][1])>0:
   end=case['segments'][j+1][0] if j+1<len(case['segments']) else case['duration'];stop.append(stop_metrics(t,v,omega,start,end,dt))
 q0=np.asarray(r['base_start']);q1=np.asarray(r['base_end']);straight=len(case['segments'])==1 and case['segments'][0][1][1:]==[0.,0.] and case['segments'][0][1][0]!=0
 speed_check=bool(r['completed'] and (rmse[:2]<=THRESHOLDS['linear_rmse_mps']).all() and rmse[2]<=THRESHOLDS['yaw_rate_rmse_radps'])
 result=dict(id=case['id'],completed=r['completed'],termination=r['termination'],duration_s=r['sim_time'],base_delta_m=(q1-q0).tolist(),base_velocity_body_mean=v[steady].mean(axis=0).tolist(),command_tracking_rmse=rmse.tolist(),velocity_target_met=speed_check,heading_change_deg=float(np.rad2deg(heading[-1])),tilt_max_deg=float(np.rad2deg(a['rpy_tilt'][:,3].max())),tilt_p95_deg=float(np.rad2deg(np.quantile(a['rpy_tilt'][steady,3],.95))),base_height_range_steady_m=[float(a['qpos'][steady,2].min()),float(a['qpos'][steady,2].max())],base_height_p95_p05_span_m=float(np.ptp(np.quantile(a['qpos'][steady,2],[.05,.95]))),whole_body_com_height_range_steady_m=[float(a['com'][steady,2].min()),float(a['com'][steady,2].max())],support_fraction=dict(left=float(loaded[steady,0].mean()),right=float(loaded[steady,1].mean()),both=float(loaded[steady].all(axis=1).mean()),neither=float((~loaded[steady].any(axis=1)).mean())),loaded_contact_tangent_speed_rms_mps=float(np.sqrt(np.sum(weights*slip**2)/max(float(weights.sum()),1e-12))),foot_max_clearance_m=a['foot_lowest_z'][steady].max(axis=0).tolist(),touchdowns_after_clear_liftoff=len(events),touchdown_events=events,nonfoot_ground_force_max_N=float(a['nonfoot_fz'].max()),steady_samples=int(steady.sum()),steady_definition='3 seconds after start of each command segment; raw start/end states retained',stop=stop,nominal_integrated_command_end=nominal[-1].tolist(),nominal_path_note='illustrative integrated command; policy has no absolute position or heading target',straight_world_y_displacement_m=float(q1[1]-q0[1]) if straight else None,policy_wall_p50_ms=float(np.median(np.load(folder/'policy_calls.npz')['seconds'])*1e3),policy_wall_p95_ms=float(np.quantile(np.load(folder/'policy_calls.npz')['seconds'],.95)*1e3))
 atomic_json(folder/'metrics.json',result);return result,a,nominal

def figures(out,rows,cases):
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 (out/'figures').mkdir(exist_ok=True)
 fig,axes=plt.subplots(3,4,figsize=(15,10))
 for ax,(r,a,p) in zip(axes.flat,cases):
  ax.plot(a['qpos'][:,0],a['qpos'][:,1],label='actual base');ax.plot(p[:,0],p[:,1],'--',label='integrated command');ax.set_title(r['id']);ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.3);ax.set_xlabel('world x [m]');ax.set_ylabel('world y [m]')
 axes[0,0].legend(fontsize=7);fig.tight_layout();fig.savefig(out/'figures/world_paths.png',dpi=150);plt.close(fig)
 fig,axes=plt.subplots(3,1,figsize=(13,8));ids=np.arange(len(rows));labels=[r['id'] for r in rows]
 for k,ax in enumerate(axes):
  ax.bar(ids,[r['command_tracking_rmse'][k] for r in rows]);ax.axhline([.15,.15,.2][k],color='r',ls='--',label='declared review target');ax.set_ylabel(['body vx RMSE [m/s]','body vy RMSE [m/s]','body wz RMSE [rad/s]'][k]);ax.grid(axis='y',alpha=.3)
 axes[0].legend();axes[-1].set_xticks(ids,labels,rotation=35,ha='right');fig.tight_layout();fig.savefig(out/'figures/tracking_rmse.png',dpi=150);plt.close(fig)
 fig,axes=plt.subplots(3,1,figsize=(11,9))
 for r,a,p in cases:
  if r['id']!='forward_05':continue
  t=a['time'];axes[0].plot(t,a['qpos'][:,1]);axes[0].set_ylabel('world y [m]');axes[1].plot(t,np.rad2deg(np.unwrap(a['rpy_tilt'][:,2])));axes[1].set_ylabel('heading [deg]');axes[2].plot(t,a['base_body_velocity'][:,1]);axes[2].set_ylabel('body vy [m/s]')
 for ax in axes:ax.grid(alpha=.3);ax.set_xlabel('simulation time [s]')
 fig.suptitle('Forward 0.5 m/s: local velocity and world heading are different');fig.tight_layout();fig.savefig(out/'figures/forward_drift.png',dpi=150);plt.close(fig)
 fig,axes=plt.subplots(3,1,figsize=(12,9))
 for r,a,p in cases:
  if r['id']!='walk_stop':continue
  t=a['time'];axes[0].plot(t,a['base_body_velocity'][:,0],label='actual body vx');axes[0].plot(t,a['command'][:,0],'--',label='command');axes[0].legend();axes[0].set_ylabel('m/s');axes[1].plot(t,a['foot_fz']);axes[1].set_ylabel('foot vertical force [N]');axes[2].plot(t,a['foot_lowest_z']);axes[2].set_ylabel('lowest foot surface [m]')
 for ax in axes:ax.axvline(12,color='r',ls=':');ax.grid(alpha=.3);ax.set_xlabel('simulation time [s]')
 fig.tight_layout();fig.savefig(out/'figures/walk_stop.png',dpi=150);plt.close(fig)

def main():
 p=argparse.ArgumentParser();p.add_argument('tag');args=p.parse_args();out=ROOT/'studies'/args.tag;rows=[];cases=[]
 for c in json.loads((out/'PROTOCOL.json').read_text())['cases']:
  r,a,n=score(out/'cases'/c['id']);rows.append(r);cases.append((r,a,n))
 atomic_json(out/'SUMMARY.json',dict(conditions=len(rows),physical_complete=sum(r['completed'] for r in rows),velocity_targets_met=sum(r['velocity_target_met'] for r in rows),rows=rows,scope='one deterministic run per command condition; not deployment success probability'))
 figures(out,rows,cases)
 with (out/'metrics.csv').open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['case','completed','body_vx_rmse_mps','body_vy_rmse_mps','body_wz_rmse_radps','world_dx_m','world_dy_m','heading_change_deg','tilt_p95_deg','contact_tangent_rms_mps','touchdowns'])
  for r in rows:w.writerow([r['id'],r['completed'],*r['command_tracking_rmse'],*r['base_delta_m'][:2],r['heading_change_deg'],r['tilt_p95_deg'],r['loaded_contact_tangent_speed_rms_mps'],r['touchdowns_after_clear_liftoff']])
 for r in rows:print(r['id'],r['completed'],'RMSE',np.round(r['command_tracking_rmse'],3).tolist(),'world y',round(r['base_delta_m'][1],3),'heading',round(r['heading_change_deg'],1),'stop',r['stop'])
if __name__=='__main__':main()
