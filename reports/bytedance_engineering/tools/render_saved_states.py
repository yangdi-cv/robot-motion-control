"""Render archived G1 states only. No controller, simulation step, or file mutation upstream."""
from pathlib import Path
import os,json,hashlib
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
D=Path(__file__).resolve().parents[1];R=D.parents[1];S=R/'experiments/humanoid_control'
font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
F=lambda n:ImageFont.truetype(font,n)
m=mujoco.MjModel.from_xml_path(str(S/'models/scene.xml'));data=mujoco.MjData(m)
renderer=mujoco.Renderer(m,height=580,width=720)
manifest=json.loads((S/'manifest.json').read_text());stress=json.loads((S/'stress_manifest.json').read_text())
records=[]
def one(source,key,config,times,title,name):
 p=S/source['root']/key/'trace.npz';a=np.load(p)
 shots=[];indices=[]
 for requested in times:
  i=len(a['time'])-1 if requested is None else int(np.argmin(np.abs(a['time']-requested)))
  t=float(a['time'][i]);data.qpos[:]=a['qpos'][i];data.qvel[:]=a['qvel'][i];data.ctrl[:]=0
  # Reconstruct transforms for rendering only; the historical contacts below come from the saved array.
  mujoco.mj_forward(m,data)
  panel=Image.new('RGB',(1440,710),'white');draw=ImageDraw.Draw(panel)
  draw.text((20,8),title+f'　t={t:.3f} s',fill='black',font=F(27))
  for j,az in enumerate([135,180]):
   camera=mujoco.MjvCamera();camera.distance=2.1;camera.elevation=-15;camera.azimuth=az;camera.lookat[:]=[data.qpos[0],data.qpos[1],.65]
   renderer.update_scene(data,camera);panel.paste(Image.fromarray(renderer.render()),(720*j,52))
  feet=a['feet'][i,:,2];fz=a['actual_wrench'][i,:,2]
  draw.text((20,643),f'保存记录：左/右足高度 {feet[0]*100:.2f}/{feet[1]*100:.2f} cm；法向力 {fz[0]:.1f}/{fz[1]:.1f} N',fill='black',font=F(24))
  draw.text((20,678),'从已保存qpos/qvel重建画面；未运行控制器、未推进物理仿真。',fill='#333333',font=F(20))
  shots.append(panel);indices.append(dict(index=i,time=t,feet_z_m=feet.tolist(),saved_fz_N=fz.tolist()))
 canvas=Image.new('RGB',(1440,710*len(shots)),'white')
 for j,shot in enumerate(shots):canvas.paste(shot,(0,j*710))
 out=D/'figures/ch08'/name;out.parent.mkdir(exist_ok=True,parents=True);canvas.save(out)
 records.append(dict(local=str(out.relative_to(D)),source=str(p.relative_to(R)),source_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),model=str((S/'models/scene.xml').relative_to(R)),frames=indices,config=config,method='mj_forward + render; no mj_step or control; contact values read from saved arrays'))
for side,label in [(0,'左脚支撑、右脚抬起'),(1,'右脚支撑、左脚抬起')]:
 key=next(k for k,c in manifest['configs'].items() if c['scenario']=='single' and c['support_side']==side and c['hz']==100 and not c['push_x'] and not c['push_y'] and c['backend']=='clarabel')
 one(manifest,key,manifest['configs'][key],[.5,2.8,6.8],label,f'g1_single_support_side{side}_saved.png')
for force in [150,250]:
 key=next(k for k,c in stress['configs'].items() if c['scenario']=='stand' and c['push_y']==force)
 one(stress,key,stress['configs'][key],[2.65,2.85,None],f'站立侧向推扰 {force} N'+('（完成条件）' if force==150 else '（约束冲突终止）'),f'g1_stress_{force}n_saved.png')
key=next(k for k,c in stress['configs'].items() if c['scenario']=='walk' and c['push_y']==40)
one(stress,key,stress['configs'][key],[9.8,10.2,None],'行走侧向推扰40 N（约束冲突终止）','g1_walk_stress_40n_saved.png')
renderer.close();(D/'STATE_RENDER_MANIFEST.json').write_text(json.dumps(records,ensure_ascii=False,indent=2));print('Saved-state figures:',len(records))
