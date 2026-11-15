"""Post-hoc explanations, kept separate from frozen thresholds and official control."""
from pathlib import Path
import argparse,json
import numpy as np
from core import ROOT,quaternion_matrix,atomic_json
from analyze import arrays,touchdown_events

def one_second_stop(a,start):
 dt=float(np.median(np.diff(a['time'])));span=int(round(1/dt));t=a['time'];p=a['qpos'][:,:2];yaw=np.unwrap(a['rpy_tilt'][:,2]);net=np.linalg.norm((p[span:]-p[:-span])/(span*dt),axis=1);rate=np.abs((yaw[span:]-yaw[:-span])/(span*dt));ends=t[span:]
 valid=ends>=start+1-1e-8;met=np.where(valid&(net<=.1)&(rate<=.15))[0]
 return dict(window_s=1.,first_qualifying_window_end_after_stop_s=float(ends[met[0]]-start) if len(met) else None,final_window_net_speed_mps=float(net[-1]),final_window_abs_heading_rate_radps=float(rate[-1]),note='Net displacement over 1s, not instantaneous rest; added diagnostic does not replace frozen stop criterion')
def main():
 ap=argparse.ArgumentParser();ap.add_argument('tag');args=ap.parse_args();out=ROOT/'studies'/args.tag;result={}
 a=arrays(out/'cases/forward_05');r=json.loads((out/'cases/forward_05/result.json').read_text());end=np.vstack([a['qpos'][1:,:3],np.asarray(r['base_end'])]);delta=end-a['qpos'][:,:3];decomp=np.zeros(3)
 for q,dp in zip(a['qpos'],delta):
  rotation=quaternion_matrix(q[3:7]);local=rotation.T@dp;decomp+=rotation[1,:]*local
 result['forward_05']=dict(world_y_m=float(delta[:,1].sum()),world_y_from_body_forward_m=float(decomp[0]),world_y_from_body_lateral_m=float(decomp[1]),world_y_from_body_vertical_m=float(decomp[2]),reconstruction_residual_m=float(abs(decomp.sum()-delta[:,1].sum())),heading_at_3s_deg=float(np.rad2deg(a['rpy_tilt'][np.argmin(abs(a['time']-3)),2])),heading_final_deg=float(np.rad2deg(a['rpy_tilt'][-1,2])),note='Kinematic decomposition of measured displacements; not a causal explanation of policy drift')
 for name,start in [('zero',3.),('walk_stop',12.),('mixed_commands',18.)]:
  a=arrays(out/'cases'/name);r=json.loads((out/'cases'/name/'result.json').read_text());i=int(np.argmin(abs(a['time']-start)));events=touchdown_events(a['time'],a['foot_fz'],a['foot_lowest_z'],float(np.median(np.diff(a['time']))))
  result[name]=dict(window_start_s=start,base_displacement_after_start_m=(np.asarray(r['base_end'])-a['qpos'][i,:3]).tolist(),touchdowns_after_start=sum(e['time']>=start for e in events),net_stop=one_second_stop(a,start))
 atomic_json(out/'POSTHOC_DIAGNOSTICS.json',result);print(json.dumps(result,indent=2))
if __name__=='__main__':main()
