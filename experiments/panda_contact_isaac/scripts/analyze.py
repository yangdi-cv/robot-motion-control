"""Compute tracking and contact metrics from Isaac simulation logs."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root = Path(__file__).resolve().parents[1]
fig = root / 'figures'
fig.mkdir(exist_ok=True)
results = {}
runs = {}
for d in sorted((root / 'runs').iterdir()):
    if not d.is_dir() or d.name not in {'position_final', 'admittance_final', 'admittance_slow', 'admittance_B20', 'admittance_limited', 'admittance_repeat', 'admittance_B80', 'admittance_contact_gate', 'admittance_force_ramp','pid_position','pid_admittance','pid_contact_gate','pid_gate_slow'} or (not (d / 'COMPLETE').exists()):
        continue
    audit=json.loads((d/'time_audit.json').read_text())
    if abs(audit['measured_duration']-audit['expected_duration'])>1e-5: raise ValueError('Invalid simulation time')
    x = np.genfromtxt(d / 'trace.csv', delimiter=',', names=True)
    f = np.genfromtxt(d / 'physics_force.csv', delimiter=',', names=True)
    runs[d.name] = x
    import csv
    with (d / 'joint_tracking.csv').open('w') as output:
        writer = csv.writer(output)
        writer.writerow(['t_s'] + [f'joint{i + 1}_{v}' for i in range(3) for v in ['target_rad', 'actual_rad', 'error_rad']])
        for row in x:
            writer.writerow([row['t']] + [v for i in range(3) for v in (row[f'ref{i}'], row[f'q{i}'], row[f'ref{i}'] - row[f'q{i}'])])
    contact = x['t'] >= 5
    tail = x['t'] >= float(x['t'][-1]) - 2
    hit = np.flatnonzero(contact & (x['fz'] > 0.5))
    stable = None
    if len(hit):
        good = np.abs(x['fz'] - 5) <= 0.5
        for j in range(hit[0], len(x) - 50):
            if np.all(good[j:j + 51]):
                stable = float(x['t'][j] - x['t'][hit[0]])
                break
    m = {'duration_s': float(x['t'][-1] + 0.01), 'control_samples': len(x), 'physics_samples': len(f), 'pre_contact_force_peak_N': float(np.max(np.abs(f['fz_N'][f['t'] <= 5]))), 'control_dt_max_error_s': float(np.max(np.abs(np.diff(x['t']) - 0.01))), 'joint_first3_rmse_deg': [float(np.rad2deg(np.sqrt(np.mean((x[f'q{i}'][~contact] - x[f'ref{i}'][~contact]) ** 2)))) for i in range(3)], 'raw_force_peak_N': float(np.max(f['fz_N'])), 'control_mean_force_peak_N': float(np.max(x['fz'])), 'last2s_mean_N': float(np.mean(x['fz'][tail])), 'last2s_force_rmse_N': float(np.sqrt(np.mean((x['fz'][tail] - 5) ** 2))), 'stable_contact_after_first_hit_s': stable, 'stability_definition': '100Hz cycle-mean force within 5±0.5 N continuously for 0.5 s', 'normal_position_rmse_mm': float(1000 * np.sqrt(np.mean((x['z'][contact] - x['z_ref'][contact]) ** 2)))}
    if all(k in x.dtype.names for k in ['x','y','x_ref','y_ref']):
        m['position_3d_rmse_mm']=float(1000*np.sqrt(np.mean((x['x'][contact]-x['x_ref'][contact])**2+(x['y'][contact]-x['y_ref'][contact])**2+(x['z'][contact]-x['z_ref'][contact])**2)))
    m['first_contact_time_s']=float(x['t'][hit[0]]) if len(hit) else None
    m['pre_contact_downward_speed_mm_s']=float(-1000*np.gradient(x['z'],x['t'])[max(0,hit[0]-2)]) if len(hit) else None
    results[d.name] = m
    if (d / 'frames').exists():
        import cv2
        paths = sorted((d / 'frames').glob('*.png'))
        if paths:
            im = cv2.imread(str(paths[0]))
            v = cv2.VideoWriter(str(d / 'simulation.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 10, (im.shape[1], im.shape[0]))
            for index,p in enumerate(paths):
                frame=cv2.imread(str(p))
                row=x[min(index*10,len(x)-1)]
                cv2.rectangle(frame,(0,0),(frame.shape[1],65),(30,35,43),-1)
                cv2.putText(frame,d.name,(20,26),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1,cv2.LINE_AA)
                text=f't={row["t"]:.1f}s  mean contact={row["fz"]:.2f} N  target=5 N'
                cv2.putText(frame,text,(20,53),cv2.FONT_HERSHEY_SIMPLEX,.6,(230,230,230),1,cv2.LINE_AA)
                v.write(frame)
            v.release()
import shutil
scene=root/'runs/position_final/frames/00100.png'
if scene.exists():shutil.copy2(scene,fig/'scene.png')
(root / 'results.json').write_text(json.dumps(results, indent=2))
for (field, label) in [('fz', 'Contact force (N)'), ('z', 'Hand height (m)')]:
    (figobj, ax) = plt.subplots(figsize=(10, 4))
    for (name, x) in runs.items():
        if name not in ['pid_position','pid_admittance']: continue
        line=ax.plot(x['t'], x[field], label=name, linewidth=1)[0]
        if field=='z': ax.plot(x['t'],x['z_ref'],ls='--',color=line.get_color(),label=name+' reference',linewidth=.8)
    if field == 'fz':
        ax.axhline(5, color='black', ls='--', label='5 N target')
    ax.set(xlabel='Simulation time (s)', ylabel=label)
    ax.grid(alpha=0.2)
    ax.legend(fontsize=8)
    figobj.tight_layout()
    figobj.savefig(fig / f'{field}.png', dpi=160)
    plt.close(figobj)
(figobj, axs) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
for (i, ax) in enumerate(axs):
    for (name, x) in runs.items():
        if name not in ['pid_position','pid_admittance']: continue
        mask = x['t'] <= 5
        ax.plot(x['t'][mask], np.rad2deg(x[f'q{i}'][mask]), label=name)
    if runs:
        x = next(iter(runs.values()))
        mask = x['t'] <= 5
        ax.plot(x['t'][mask], np.rad2deg(x[f'ref{i}'][mask]), 'k--', label='Reference')
    ax.set_ylabel(f'Joint {i + 1} (deg)')
    ax.grid(alpha=0.2)
axs[0].legend()
axs[-1].set_xlabel('Simulation time (s)')
figobj.tight_layout()
figobj.savefig(fig / 'joints.png', dpi=160)
plt.close(figobj)
print(json.dumps(results, indent=2))

figobj,axs=plt.subplots(2,1,figsize=(10,7),sharex=True)
for name in ['position_final','admittance_final','admittance_contact_gate','admittance_force_ramp']:
 if name not in runs:continue
 x=runs[name]
 axs[0].plot(x['t'],x['fz'],label=name,linewidth=.8)
 raw=np.genfromtxt(root/'runs'/name/'physics_force.csv',delimiter=',',names=True)
 axs[1].plot(raw['t'],raw['fz_N'],label=name,linewidth=.6,alpha=.8)
axs[0].axhline(5,color='black',ls='--');axs[0].set_ylabel('Cycle-mean force (N)');axs[1].set_ylabel('Raw 1ms force (N)');axs[1].set_xlabel('Simulation time (s)')
for ax in axs:ax.grid(alpha=.2);ax.legend(fontsize=7);ax.set_xlim(5,12)
figobj.tight_layout();figobj.savefig(fig/'transition.png',dpi=160);plt.close(figobj)
