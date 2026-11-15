"""Compare against the pinned upstream AST, not another copy of our equations."""
from pathlib import Path
import ast,sys,unittest,types,json
import numpy as np
import mujoco,torch
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'validation'))
from core import load,observation,pd,gravity,quaternion_matrix,UPSTREAM

source=ast.parse((UPSTREAM/'deploy/deploy_mujoco/deploy_mujoco.py').read_text())
g={"np":np,"torch":torch};functions=[x for x in source.body if isinstance(x,ast.FunctionDef)];exec(compile(ast.Module(body=functions,type_ignores=[]),'upstream','exec'),g)
block=next(x for x in ast.walk(source) if isinstance(x,ast.If) and isinstance(x.test,ast.Compare) and 'control_decimation' in ast.unparse(x.test))
prefix=[]
for node in block.body:
 prefix.append(node)
 if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='obs_tensor' for t in node.targets):break
OBS=compile(ast.Module(body=prefix,type_ignores=[]),'upstream-observation','exec')
def official_obs(d,cfg,action,cmd,counter):
 ns={**g,**cfg,'d':d,'counter':counter,'default_angles':np.asarray(cfg['default_angles'],np.float32),'cmd_scale':np.asarray(cfg['cmd_scale'],np.float32),'cmd':np.asarray(cmd,np.float32),'action':action.copy(),'obs':np.zeros(cfg['num_obs'],np.float32)}
 exec(OBS,ns);return ns['obs']
class OfficialContracts(unittest.TestCase):
 @classmethod
 def setUpClass(cls):torch.set_num_threads(1)
 def test_observation_matches_official_randomized_states(self):
  cfg,m,d,p,_,_=load();rng=np.random.default_rng(42)
  for counter in [10,35,400,977]:
   q=d.qpos.copy();q[:3]=rng.normal(size=3);q[3:7]=rng.normal(size=4);q[3:7]/=np.linalg.norm(q[3:7]);q[7:]=rng.normal(size=12);v=rng.normal(size=m.nv);a=rng.normal(size=12).astype(np.float32);cmd=rng.normal(size=3).astype(np.float32)
   np.testing.assert_array_equal(observation(q,v,cfg,a,cmd,counter),official_obs(types.SimpleNamespace(qpos=q,qvel=v),cfg,a,cmd,counter))
 def test_pd_matches_official(self):
  rng=np.random.default_rng(4);a=[rng.normal(size=12) for _ in range(5)]
  np.testing.assert_array_equal(pd(a[0],a[1],a[2],a[3],a[4]),g['pd_control'](a[0],a[1],a[2],np.zeros(12),a[3],a[4]))
 def test_gravity_and_frame_convention(self):
  q=np.array([np.sqrt(.5),0,0,np.sqrt(.5)]);np.testing.assert_allclose(quaternion_matrix(q).T@np.array([0,1,0]),[1,0,0],atol=1e-15);np.testing.assert_array_equal(gravity(q),g['get_gravity_orientation'](q))
 def test_recurrent_memory_is_reset_by_fresh_load(self):
  c,m,d,p,_,_=load();before=p.hidden_state.clone();self.assertEqual(float(before.abs().max()),0)
  with torch.no_grad():p(torch.ones(1,c['num_obs']))
  self.assertGreater(float(p.hidden_state.abs().max()),0);p2=load()[3];self.assertEqual(float(p2.hidden_state.abs().max()),0)
 def test_full_loop_matches_official_500_steps(self):
  cfg,m,d,p,_,_=load();dr=mujoco.MjData(m);pr=load()[3];a=np.zeros(12,np.float32);ar=a.copy();default=np.asarray(cfg['default_angles'],np.float32);target=default.copy();tr=default.copy();kp=np.asarray(cfg['kps'],np.float32);kd=np.asarray(cfg['kds'],np.float32);cmd=np.asarray(cfg['cmd_init'],np.float32);maximum=0.
  with torch.no_grad():
   for count in range(1,501):
    d.ctrl[:]=pd(target,d.qpos[7:],kp,d.qvel[6:],kd);dr.ctrl[:]=g['pd_control'](tr,dr.qpos[7:],kp,np.zeros_like(kd),dr.qvel[6:],kd)
    mujoco.mj_step(m,d);mujoco.mj_step(m,dr)
    maximum=max(maximum,float(np.max(np.abs(d.qpos-dr.qpos))))
    if count%cfg['control_decimation']==0:
     a[:]=p(torch.from_numpy(observation(d.qpos,d.qvel,cfg,a,cmd,count)).unsqueeze(0)).numpy().squeeze();ar[:]=pr(torch.from_numpy(official_obs(dr,cfg,ar,cmd,count)).unsqueeze(0)).detach().numpy().squeeze()
     target=a*cfg['action_scale']+default;tr=ar*cfg['action_scale']+default
  self.assertLessEqual(maximum,1e-12)
if __name__=='__main__':unittest.main()
