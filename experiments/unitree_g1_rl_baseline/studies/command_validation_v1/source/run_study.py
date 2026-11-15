"""Record official-policy runs without rendering or buffering video frames."""
from pathlib import Path
import argparse,time,json,hashlib,shutil,sys,platform,os
import numpy as np
import mujoco,torch
from core import ROOT,UPSTREAM,REVISION,POLICY_SHA,load,geometry,gravity,pd,observation,quaternion_matrix,orientation,contacts,atomic_json,digest
from protocol import CASES,THRESHOLDS,command_at

def run_case(case,out):
 out.mkdir(parents=True,exist_ok=False)
 cfg,m,d,policy,pp,xml=load();pelvis,feet,floor,gs=geometry(m);dt=float(cfg['simulation_dt']);dec=int(cfg['control_decimation']);N=int(round(case['duration']/dt))
 kp=np.asarray(cfg['kps'],np.float32);kd=np.asarray(cfg['kds'],np.float32);default=np.asarray(cfg['default_angles'],np.float32)
 target=default.copy();action=np.zeros(cfg['num_actions'],np.float32);applied_cmd=np.zeros(3,np.float32);initial=d.qpos.copy()
 buffers={k:dict(shape=list(v.shape),max_abs=float(v.detach().abs().max())) for k,v in policy.named_buffers()}
 fields={};part=[];part_id=0;global_count=0;obs_records=[];action_records=[];query_times=[];obs_cmds=[];guard={'tilt':0.,'nonfoot':0.};reason=None
 def flush():
  nonlocal part,part_id
  if not part:return
  arrays={k:np.asarray([r[k] for r in part]) for k in part[0]};np.savez_compressed(out/f'part_{part_id:04d}.npz',**arrays);part_id+=1;part=[]
 t0=time.perf_counter()
 with torch.inference_mode():
  for counter in range(1,N+1):
   before=d.qpos.copy();vel=d.qvel.copy();t=float(d.time);cmd=np.asarray(command_at(case,t),np.float32)
   torque=pd(target,d.qpos[7:],kp,d.qvel[6:],kd);d.ctrl[:]=torque
   mujoco.mj_step(m,d) # same order as official deployment; no extra forward on live state
   fz,slip,weights,other,ncon=contacts(m,d,feet,floor,vel)
   # mj_step leaves kinematic fields/contact solve at pre-integration state.
   rot=quaternion_matrix(before[3:7]);footlow=np.array([np.min(d.geom_xpos[g,2]-m.geom_size[g,0]) for g in gs])
   row=dict(time=t,qpos=before,qvel=vel,command=cmd,policy_command=applied_cmd.copy(),action_applied=action.copy(),target_applied=target.copy(),ctrl=torque.copy(),actuator_force=d.actuator_force.copy(),joint_actuation=d.qfrc_actuator.copy(),com=d.subtree_com[pelvis].copy(),base_body_velocity=rot.T@vel[:3],rpy_tilt=orientation(before[3:7]),feet=d.xpos[feet].copy(),foot_lowest_z=footlow,foot_fz=fz,contact_slip=slip,slip_weight=weights,nonfoot_fz=other,foot_contact_count=ncon)
   part.append(row);global_count+=1
   if counter%dec==0:
    nextcmd=np.asarray(command_at(case,float(d.time)),np.float32);obs=observation(d.qpos,d.qvel,cfg,action,nextcmd,counter)
    qt=time.perf_counter();action[:]=policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze();query_times.append(time.perf_counter()-qt)
    target=action*cfg['action_scale']+default;applied_cmd=nextcmd
    obs_records.append(obs);action_records.append(action.copy());obs_cmds.append(nextcmd)
   tilt=orientation(d.qpos[3:7])[3];guard['tilt']=guard['tilt']+dt if tilt>THRESHOLDS['tilt_rad'] else 0.;guard['nonfoot']=guard['nonfoot']+dt if other>THRESHOLDS['nonfoot_force_N'] else 0.
   if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():reason='nonfinite_state'
   elif d.qpos[2]<THRESHOLDS['low_base_height_m']:reason='base_height_below_0p4'
   elif guard['tilt']>=THRESHOLDS['tilt_hold_s']:reason='sustained_tilt_over_60deg'
   elif guard['nonfoot']>=THRESHOLDS['nonfoot_hold_s']:reason='sustained_nonfoot_floor_contact'
   if counter%500==0:
    flush();atomic_json(out/'progress.json',dict(steps=counter,simulation_time=float(d.time),complete=False))
   if reason:break
 flush();wall=time.perf_counter()-t0
 np.savez_compressed(out/'policy_calls.npz',observation=np.asarray(obs_records),action=np.asarray(action_records),command=np.asarray(obs_cmds),seconds=np.asarray(query_times))
 # Final whole-body CoM is computed in a spare data object, not in the policy loop.
 final=mujoco.MjData(m);final.qpos[:]=d.qpos;final.qvel[:]=d.qvel;mujoco.mj_forward(m,final)
 atomic_json(out/'result.json',dict(case=case,completed=reason is None and global_count==N,termination=reason,steps=global_count,sim_time=float(d.time),rollout_recording_wall_s=wall,base_start=initial[:3].tolist(),base_end=d.qpos[:3].tolist(),whole_body_com_end=final.subtree_com[pelvis].tolist(),initial_qpos=initial.tolist(),final_qpos=d.qpos.tolist(),final_qvel=d.qvel.tolist(),initial_recurrent_buffers=buffers,policy_calls=len(query_times),policy_sha256=digest(pp),model_scene_sha256=digest(xml),upstream_revision=REVISION,model_dimensions=dict(nq=m.nq,nv=m.nv,nu=m.nu,mass_kg=float(m.body_mass.sum())),recording_phase='time/qpos/qvel before integration; contacts, actuator force and kinematics from same solved step; final state separate',parts=part_id))
 atomic_json(out/'progress.json',dict(steps=global_count,simulation_time=float(d.time),complete=True,completed=reason is None))
 print(case['id'],'COMPLETE' if reason is None else reason,'time',round(d.time,3),'base',np.round(d.qpos[:3],4).tolist(),flush=True)

def main():
 p=argparse.ArgumentParser();p.add_argument('--tag',required=True);p.add_argument('--only',nargs='*');p.add_argument('--smoke-duration',type=float);p.add_argument('--resume',action='store_true');a=p.parse_args()
 torch.set_num_threads(1);torch.set_num_interop_threads(1)
 cases=[x.copy() for x in CASES if not a.only or x['id'] in a.only]
 if a.smoke_duration:
  for c in cases:c['duration']=a.smoke_duration
 out=ROOT/'studies'/a.tag
 code={str(x.relative_to(ROOT)):digest(x) for x in sorted((ROOT/'validation').glob('*.py'))};proto=dict(cases=cases,thresholds=THRESHOLDS,source_sha256=code,policy_sha256=POLICY_SHA,upstream_revision=REVISION,python=sys.version,torch=torch.__version__,mujoco=mujoco.__version__,numpy=np.__version__,torch_threads=1,description='fixed deterministic command conditions; no retraining, no control changes, no real hardware')
 if out.exists():
  assert a.resume and json.loads((out/'PROTOCOL.json').read_text())==proto,'Output exists or protocol changed'
 else:
  out.mkdir(parents=True);atomic_json(out/'PROTOCOL.json',proto)
  for f in (ROOT/'validation').glob('*.py'):
   dst=out/'source'/f.name;dst.parent.mkdir(exist_ok=True);shutil.copy2(f,dst)
 for c in cases:
  d=out/'cases'/c['id']
  if d.exists():
   assert a.resume and (d/'result.json').exists(),'Partial case retained; use a new tag'
   continue
  run_case(c,d)
 atomic_json(out/'STATUS.json',dict(complete=True,cases=len(cases)))
if __name__=='__main__':main()
