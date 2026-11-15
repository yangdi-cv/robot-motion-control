"""Audit saved aggregates only; no simulator, ROS node or hardware is started."""
from pathlib import Path
import json,hashlib
D=Path(__file__).resolve().parents[1];R=D.parents[1]
checks=[]
def load(rel):
 p=R/rel;return json.loads(p.read_text()),dict(path=rel,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
for name,file,n,passed in [('lipm_balance','results.json',1458,None),('lipm_balance','physical_results.json',333,None),('lipm_extension','network_results.json',1188,692),('lipm_extension','gait_results.json',423,175),('lipm_extension','biped_results.json',51,50),('biped_feedback','results.json',94,79),('humanoid_control','results.json',52,51),('humanoid_control','stress_results.json',8,2)]:
 r,src=load(f'experiments/{name}/{file}');assert len(r)==n,(name,len(r),n)
 count=sum(x.get('success') is True for x in r.values());
 if passed is not None:assert count==passed,(name,count,passed)
 checks.append(dict(name=name,file=file,conditions=len(r),successes=count,source=src))
r,src=load('experiments/panda_contact_optimization/results.json');assert len(r)==44
selected=r['final_selected'];half=r['final_half_step'];assert abs(selected['raw_peak_N']-11.112)<.001;assert abs(half['raw_peak_N']-20.43)<.01
checks.append(dict(name='panda_contact_optimization',conditions=len(r),selected_raw_peak_N=selected['raw_peak_N'],selected_last2_mean_N=selected['last2_mean_N'],selected_last2_rmse_N=selected['last2_rmse_N'],half_dt_raw_peak_N=half['raw_peak_N'],source=src))
r,src=load('experiments/panda_contact_isaac/study_summary.json');assert r['full_length_runs']==34 and r['full_force_samples']==599000;checks.append(dict(name='panda_contact_isaac',summary=r,source=src))
(D/'RESULT_AUDIT.json').write_text(json.dumps(dict(scope='Existing saved condition counts and selected contact metrics; not a new experiment or complete re-verification of every trace.',checks=checks),ensure_ascii=False,indent=2))
for x in checks:print(x['name'],x.get('file',''),x.get('conditions',x.get('summary',{}).get('full_length_runs')),x.get('successes',''))
