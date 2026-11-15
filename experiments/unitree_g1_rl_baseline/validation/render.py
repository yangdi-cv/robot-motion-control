"""Render recorded states at real simulation-time playback; never call mj_step."""
from pathlib import Path
import os,argparse,json,subprocess,time
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np
import mujoco
import imageio.v2 as imageio
import imageio_ffmpeg
from PIL import Image,ImageDraw,ImageFont,ImageOps
from core import ROOT,UPSTREAM,digest,atomic_json
from analyze import arrays
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
def font(n):return ImageFont.truetype(FONT,n)
def encoder(path,w,h,fps):
 return subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-f','rawvideo','-vcodec','rawvideo','-s',f'{w}x{h}','-pix_fmt','rgb24','-r',str(fps),'-i','-','-an','-vcodec','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(path)],stdin=subprocess.PIPE,stderr=subprocess.DEVNULL)

def render(out,caseid):
 folder=out/'cases'/caseid;a=arrays(folder);result=json.loads((folder/'result.json').read_text());dt=float(np.median(np.diff(a['time'])));fps=20;N=int(round(result['sim_time']*fps));target=folder/'rollout_20fps.mp4'
 m=mujoco.MjModel.from_xml_path(str(UPSTREAM/'resources/robots/g1_description/scene.xml'));d=mujoco.MjData(m);renderer=mujoco.Renderer(m,height=480,width=640);proc=encoder(target,1280,760,fps);stills=[];snapshots={max(0,min(N-1,int(N*f))) for f in [0.,.25,.5,.75,.999]};t0=time.perf_counter()
 for k in range(N):
  i=min(len(a['time'])-1,int(round(k/fps/dt)));d.qpos[:]=a['qpos'][i];d.qvel[:]=a['qvel'][i];mujoco.mj_forward(m,d)
  im=Image.new('RGB',(1280,760),'white');draw=ImageDraw.Draw(im);draw.text((16,7),f'官方G1 RL基线｜{caseid}｜仿真时间 {a["time"][i]:.2f} s',font=font(26),fill='black')
  for j,az in enumerate([135,180]):
   cam=mujoco.MjvCamera();cam.distance=2.4;cam.elevation=-15;cam.azimuth=az;cam.lookat[:]=[d.qpos[0],d.qpos[1],.65];renderer.update_scene(d,cam);im.paste(Image.fromarray(renderer.render()),(j*640,52))
  cmd=a['command'][i];v=a['base_body_velocity'][i];w=a['qvel'][i,5];loads=a['foot_fz'][i]
  lines=[f'指令 vx/vy/wz：{cmd[0]:+.2f} / {cmd[1]:+.2f} m/s / {cmd[2]:+.2f} rad/s',f'实测机身速度：{v[0]:+.2f} / {v[1]:+.2f} m/s；角速度 {w:+.2f} rad/s',f'世界位置 x/y：{d.qpos[0]:+.2f} / {d.qpos[1]:+.2f} m；基座高 {d.qpos[2]:.3f} m',f'左右足法向力：{loads[0]:.1f} / {loads[1]:.1f} N；机身倾角 {np.rad2deg(a["rpy_tilt"][i,3]):.1f}°']
  for j,line in enumerate(lines):draw.text((18,546+j*35),line,font=font(23),fill='black')
  draw.text((18,700),'12电机｜策略50Hz / 物理500Hz｜两视角均跟随机身｜按保存状态渲染，非新回合',font=font(20),fill='#333333')
  proc.stdin.write(im.tobytes())
  if k in snapshots:
   p=folder/f'frame_{k:04d}.png';im.save(p);stills.append(dict(file=p.name,video_frame=k,state_index=i,simulation_time=float(a['time'][i])))
 proc.stdin.close();assert proc.wait()==0;renderer.close()
 tiles=[]
 for s in stills:
  im=Image.open(folder/s['file']).crop((0,52,640,532));tile=Image.new('RGB',(640,530),'white');tile.paste(im,(0,45));ImageDraw.Draw(tile).text((12,7),f't = {s["simulation_time"]:.2f} s',font=font(23),fill='black');tiles.append(tile)
 sheet=Image.new('RGB',(1280,1590),'white')
 for j,im in enumerate(tiles):sheet.paste(im,((j%2)*640,(j//2)*530))
 sheet.save(folder/'contact_sheet.png')
 atomic_json(folder/'video.json',dict(file=target.name,frames=N,fps=fps,simulation_duration=result['sim_time'],render_and_encode_wall_s=time.perf_counter()-t0,trace_sha256=[digest(p) for p in sorted(folder.glob('part_*.npz'))],stills=stills,method='saved qpos/qvel + mj_forward for rendering only; original logged forces; no policy calls or physics stepping'))
 print('rendered',caseid,N,'frames',flush=True)

def comparison(out):
 wbc=ROOT.parents[1]/'experiments/humanoid_control/G1_三维四步准静态行走.mp4';rl=out/'cases/forward_05/rollout_20fps.mp4';
 if not wbc.exists(): wbc=ROOT.parents[1]/'reports/bytedance_engineering/videos/ch08/g1_quasistatic_four_steps.mp4'
 left=imageio.get_reader(wbc);right=imageio.get_reader(rl);fps=20;N=min(left.count_frames(),right.count_frames(),400);path=out/'wbc_vs_official_rl_first20s.mp4';p=encoder(path,1280,680,fps)
 for k in range(N):
  im=Image.new('RGB',(1280,680),'white');draw=ImageDraw.Draw(im);draw.text((15,8),'准静态WBC 与 官方RL：同一播放时长，不同模型和任务',font=font(27),fill='black')
  clips=[Image.fromarray(left.get_data(k)).crop((0,75,720,715)),Image.fromarray(right.get_data(k)).crop((0,52,640,532))]
  for j,clip in enumerate(clips):
   clip=ImageOps.contain(clip,(630,480));im.paste(clip,(j*640+(640-clip.width)//2,95+(480-clip.height)//2))
  draw.text((16,54),'自编WBC：29电机，准静态参考',font=font(23),fill='black');draw.text((650,54),'官方RL：12电机，速度指令0.5 m/s',font=font(23),fill='black')
  draw.text((16,588),f'仿真时间 {k/fps:.2f} s；左侧仅展示原29秒录屏的前20秒，双方均未加速播放。',font=font(21),fill='black')
  draw.text((16,627),'该视频说明步态与任务设置的差异，不能作为同模型、同目标的算法性能排名。',font=font(21),fill='black');p.stdin.write(im.tobytes())
 p.stdin.close();assert p.wait()==0;left.close();right.close();atomic_json(out/'comparison_video.json',dict(file=path.name,frames=N,fps=fps,wbc_source=str(wbc),wbc_sha256=digest(wbc),rl_source=str(rl),rl_sha256=digest(rl),same_playback_rate=True,algorithm_ablation=False));print('comparison video saved',flush=True)

def main():
 ap=argparse.ArgumentParser();ap.add_argument('tag');ap.add_argument('--only',nargs='*');ap.add_argument('--comparison',action='store_true');args=ap.parse_args();out=ROOT/'studies'/args.tag
 for c in json.loads((out/'PROTOCOL.json').read_text())['cases']:
  if not args.only or c['id'] in args.only:render(out,c['id'])
 if args.comparison:comparison(out)
if __name__=='__main__':main()
