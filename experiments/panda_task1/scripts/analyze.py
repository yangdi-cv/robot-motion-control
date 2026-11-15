"""Analyze actual ROS feedback against the analytical reference at its simulation timestamp."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args();run=args.run
with (run/'tracking.csv').open() as f:rows=list(csv.DictReader(f))
if not rows:raise ValueError('No tracking records')
def column(name):return np.array([float(r[name]) for r in rows])
t=column('trajectory_time_s');host=column('host_receive_ns')*1e-9
if not np.all(np.diff(t)>0):raise ValueError('Simulation timestamps must strictly increase')
result={'samples':len(rows),'recorded_sim_duration_s':float(t[-1]-t[0]),'recorded_wall_duration_s':float(host[-1]-host[0]),
        'sim_period_ms_p50':float(np.median(np.diff(t))*1000),'sim_period_ms_max':float(np.max(np.diff(t))*1000),
        'host_receive_period_ms_p50':float(np.median(np.diff(host))*1000),'host_receive_period_ms_p95':float(np.percentile(np.diff(host),95)*1000),
        'feedback_wall_hz':float((len(rows)-1)/(host[-1]-host[0])),
        'real_time_factor':float((t[-1]-t[0])/(host[-1]-host[0])),'time_basis':'simulation time, host receipt timing reported separately',
        'error_definition':'analytical target(t_state) minus actual(t_state); no lag correction','joints':{}}
fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True)
errors=[]
for i,axis in enumerate(axes,1):
 desired=column(f'q_ref_{i}');actual=column(f'q_actual_{i}');error=desired-actual;errors.append(error)
 if not np.isfinite(actual).all():raise ValueError('Non-finite feedback')
 result['joints'][f'panda_joint{i}']={'max_abs_error_rad':float(np.abs(error).max()),'rmse_rad':float(np.sqrt(np.mean(error**2))),
                                 'max_abs_error_deg':float(np.degrees(np.abs(error).max())),'rmse_deg':float(np.degrees(np.sqrt(np.mean(error**2)))),
                                 'initial_actual_rad':float(actual[0]),'final_actual_rad':float(actual[-1]),'final_target_rad':float(desired[-1]),'final_error_rad':float(error[-1])}
 axis.plot(t,desired,label='Quintic reference',linewidth=2);axis.plot(t,actual,label='Isaac / ROS feedback',linewidth=1.5,linestyle='--')
 axis.set_ylabel(f'Joint {i} [rad]');axis.grid(alpha=.3);axis.legend()
axes[-1].set_xlabel('Trajectory simulation time [s]');fig.suptitle('Franka Panda: 5 s quintic trajectory tracking');fig.tight_layout();fig.savefig(run/'trajectory_tracking.png',dpi=180);plt.close(fig)
fig,axis=plt.subplots(figsize=(10,4.5))
for i,error in enumerate(errors,1):axis.plot(t,np.degrees(error),label=f'panda_joint{i}')
axis.set_xlabel('Trajectory simulation time [s]');axis.set_ylabel('Target - actual [degrees]');axis.grid(alpha=.3);axis.legend();fig.tight_layout();fig.savefig(run/'tracking_error.png',dpi=180);plt.close(fig)
with (run/'metrics.csv').open('w',newline='') as f:
 writer=csv.writer(f);writer.writerow(['joint','max_abs_error_rad','rmse_rad','max_abs_error_deg','rmse_deg','final_error_rad'])
 for name,stats in result['joints'].items():writer.writerow([name,*[stats[k] for k in ['max_abs_error_rad','rmse_rad','max_abs_error_deg','rmse_deg','final_error_rad']]])
(run/'metrics.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
