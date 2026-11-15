"""Unitree BSD-3 deployment semantics plus read-only measurement helpers.

Control/observation layout follows pinned unitree_rl_gym deploy_mujoco.py.
No policy, gains, model, action clipping or initialization changes are made.
"""
from pathlib import Path
import hashlib,json,subprocess
import numpy as np
import mujoco,yaml,torch
ROOT=Path(__file__).resolve().parents[1]
UPSTREAM=ROOT/'upstream'
REVISION='276801e46c5d433564f24658bac64f254b7d2d4b'
POLICY_SHA='cf668f75b90d1abf73d2b87612a6e76bccc61ff7e083b63582d3f6aaa3c1759d'

def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def gravity(quat):
 w,x,y,z=quat
 return np.array([2*(-z*x+w*y),-2*(z*y+w*x),1-2*(w*w+z*z)])
def pd(target,q,kp,dq,kd):return (target-q)*kp-dq*kd

def quaternion_matrix(q):
 w,x,y,z=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
 [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
 [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def orientation(q):
 r=quaternion_matrix(q)
 return np.array([np.arctan2(r[2,1],r[2,2]),np.arcsin(np.clip(-r[2,0],-1,1)),np.arctan2(r[1,0],r[0,0]),np.arccos(np.clip(r[2,2],-1,1))])
def observation(q,dq,cfg,previous_action,cmd,counter):
 n=cfg['num_actions'];dt=cfg['simulation_dt'];obs=np.zeros(cfg['num_obs'],np.float32)
 obs[:3]=dq[3:6]*cfg['ang_vel_scale'];obs[3:6]=gravity(q[3:7]);obs[6:9]=np.asarray(cmd,dtype=np.float32)*np.asarray(cfg['cmd_scale'],np.float32)
 obs[9:9+n]=(q[7:]-np.asarray(cfg['default_angles'],np.float32))*cfg['dof_pos_scale']
 obs[9+n:9+2*n]=dq[6:]*cfg['dof_vel_scale'];obs[9+2*n:9+3*n]=previous_action
 phase=(counter*dt)%0.8/0.8;obs[9+3*n:9+3*n+2]=[np.sin(2*np.pi*phase),np.cos(2*np.pi*phase)]
 return obs

def load():
 cfg=yaml.safe_load((UPSTREAM/'deploy/deploy_mujoco/configs/g1.yaml').read_text())
 policy_path=Path(cfg['policy_path'].replace('{LEGGED_GYM_ROOT_DIR}',str(UPSTREAM)))
 xml=Path(cfg['xml_path'].replace('{LEGGED_GYM_ROOT_DIR}',str(UPSTREAM)))
 assert digest(policy_path)==POLICY_SHA,'Policy differs from pinned official file'
 m=mujoco.MjModel.from_xml_path(str(xml));m.opt.timestep=float(cfg['simulation_dt']);d=mujoco.MjData(m)
 policy=torch.jit.load(str(policy_path),map_location='cpu');policy.eval() # fresh recurrent buffers every case, no warm-up calls
 return cfg,m,d,policy,policy_path,xml

def geometry(m):
 pelvis=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'pelvis')
 feet=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,s+'_ankle_roll_link') for s in ['left','right']]
 floor=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'floor')
 geoms=[np.where((m.geom_bodyid==b)&((m.geom_contype!=0)|(m.geom_conaffinity!=0)))[0] for b in feet]
 assert all(len(g)==4 for g in geoms)
 assert all(np.all(m.geom_type[g]==mujoco.mjtGeom.mjGEOM_SPHERE) for g in geoms)
 return pelvis,feet,floor,geoms

def contacts(m,d,feet,floor,qvel_before):
 """Use pre-integration contact geometry with its solved force after mj_step.

 Positive force is force on the foot. Slip proxy is force-weighted RMS material
 contact-point tangential speed against the stationary floor, not ankle speed.
 """
 fz=np.zeros(2);slip_num=np.zeros(2);slip_den=np.zeros(2);other=0.;count=np.zeros(2,np.int32)
 wrench=np.zeros(6);jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
 for i in range(d.ncon):
  c=d.contact[i];g1,g2=int(c.geom1),int(c.geom2)
  if floor not in [g1,g2]:continue
  g=g2 if g1==floor else g1;b=int(m.geom_bodyid[g]);mujoco.mj_contactForce(m,d,i,wrench)
  f=c.frame.reshape(3,3).T@wrench[:3]
  if g==g1:f=-f
  vertical=max(0.,float(f[2]))
  if b not in feet:other+=vertical;continue
  side=feet.index(b);fz[side]+=vertical;count[side]+=1
  if vertical>1.:
   mujoco.mj_jac(m,d,jp,jr,c.pos,b);velocity=jp@qvel_before
   slip_num[side]+=vertical*float(velocity[:2]@velocity[:2]);slip_den[side]+=vertical
 return fz,np.sqrt(np.divide(slip_num,slip_den,out=np.zeros(2),where=slip_den>0)),slip_den,other,count

def atomic_json(path,value):
 path=Path(path);tmp=path.with_name(path.name+'.tmp');tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2));tmp.replace(path)
