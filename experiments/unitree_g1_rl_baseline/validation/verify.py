"""Independently reconstruct kinematics, body velocity and archived command alignment."""
from pathlib import Path
import argparse,json
import numpy as np
import mujoco
from core import ROOT,UPSTREAM,geometry,atomic_json
from protocol import command_at
from analyze import arrays

def main():
 ap=argparse.ArgumentParser();ap.add_argument('tag');args=ap.parse_args();out=ROOT/'studies'/args.tag
 m=mujoco.MjModel.from_xml_path(str(UPSTREAM/'resources/robots/g1_description/scene.xml'));d=mujoco.MjData(m);pelvis,feet,floor,gs=geometry(m);records=[]
 for case in json.loads((out/'PROTOCOL.json').read_text())['cases']:
  folder=out/'cases'/case['id'];r=json.loads((folder/'result.json').read_text());a=arrays(folder);maximum=0.;v_error=0.;phase_error=0.;force_load=[]
  for i in np.linspace(0,len(a['time'])-1,31,dtype=int):
   d.qpos[:]=a['qpos'][i];d.qvel[:]=a['qvel'][i];mujoco.mj_forward(m,d)
   maximum=max(maximum,float(np.abs(d.subtree_com[pelvis]-a['com'][i]).max()),float(np.abs(d.xpos[feet]-a['feet'][i]).max()))
   # Quaternion-to-matrix comes from MuJoCo, independent of the recorded helper.
   matrix=np.zeros(9);mujoco.mju_quat2Mat(matrix,d.qpos[3:7]);v_error=max(v_error,float(np.abs(matrix.reshape(3,3).T@d.qvel[:3]-a['base_body_velocity'][i]).max()))
  assert maximum<1e-10 and v_error<1e-12
  np.testing.assert_allclose(a['command'],np.array([command_at(case,t) for t in a['time']],np.float32),atol=0,rtol=0)
  assert np.abs(a['joint_actuation'][:,:6]).max()==0
  np.testing.assert_allclose(np.diff(a['time']),m.opt.timestep,atol=1e-10)
  assert len(a['time'])==r['steps'] and np.isfinite(a['qpos']).all()
  p=np.load(folder/'policy_calls.npz');assert len(p['observation'])==r['steps']//10;assert p['observation'].shape[1]==47
  rows=dict(id=case['id'],saved_steps=r['steps'],kinematic_max_error_m=maximum,body_velocity_max_error_mps=v_error,unactuated_root=True,command_alignment=True,policy_calls=len(p['observation']))
  records.append(rows)
 atomic_json(out/'VERIFICATION.json',dict(cases=records,scope='all timestamps, commands and root actuation; 31 sampled kinematic states per case; official interface checked separately'))
 print('Verified',len(records),'cases; max kinematic difference',max(x['kinematic_max_error_m'] for x in records))
if __name__=='__main__':main()
