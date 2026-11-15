"""Add saved screenshots and video/state-derived figures. Never run a simulation."""
from pathlib import Path
import json,hashlib,re,shutil
import imageio.v2 as iio
from PIL import Image,ImageDraw,ImageFont
D=Path(__file__).resolve().parents[1];R=D.parents[1]
assets=json.loads((D/'asset_manifest.json').read_text());records=[];blocks={}
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def font(n):return ImageFont.truetype(FONT,n)
def register(sid,p,source,caption,method='copied original screenshot',detail=None,kind='figure'):
 rel=str(p.relative_to(D));item=dict(kind=kind,stage=sid,source=str(source.relative_to(R)),local=rel,sha256=sha(p),caption=caption,derivation=method)
 if detail:item['detail']=detail
 for i,a in enumerate(assets):
  if a['local']==rel:assets[i]=item;break
 else:assets.append(item)
 records.append(item);return rel

def add(sid,rel,cap):
 blocks.setdefault(sid,[]).append(r'\motionfigure{'+rel+'}{'+cap+'}{fig:motion-'+Path(rel).stem+'}')
def copy(sid,src,name,cap):
 source=R/src;ch=next(x['chapter'] for x in stages if x['id']==sid);p=D/f'figures/ch{ch:02d}'/name;p.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(source,p);add(sid,register(sid,p,source,cap),cap)
def sheet(sid,video,name,times,crop,cols,cap,clock='视频时间'):
 ch=next(x['chapter'] for x in stages if x['id']==sid);source=R/video;r=iio.get_reader(source);fps=r.get_meta_data()['fps'];n=r.count_frames();tiles=[];meta=[]
 for t in times:
  idx=min(int(round(t*fps)),n-1);im=Image.fromarray(r.get_data(idx));original=list(im.size)
  if crop:im=im.crop(crop)
  w=800;h=round(w*im.height/im.width);im=im.resize((w,h));tile=Image.new('RGB',(w,h+55),'white');tile.paste(im,(0,55));ImageDraw.Draw(tile).text((12,9),f'{clock} {idx/fps:.2f} s',font=font(26),fill='black');tiles.append(tile);meta.append(dict(frame=idx,video_time_s=idx/fps,crop=crop,original_size=original))
 r.close();tw=max(x.width for x in tiles);th=max(x.height for x in tiles);out=Image.new('RGB',(cols*tw,((len(tiles)+cols-1)//cols)*th),'white')
 for i,im in enumerate(tiles):out.paste(im,((i%cols)*tw,(i//cols)*th))
 p=D/f'figures/ch{ch:02d}'/name;out.save(p);rel=register(sid,p,source,cap,'video frame extraction; fixed crop and resize; no pose synthesis',dict(fps=fps,frames=meta));add(sid,rel,cap)

stages=json.loads((D/'outline.json').read_text())
introductions={
's03':('轨迹运动的实际场景','下图从原始Isaac录屏按固定视频时刻提取，裁去侧栏和资源面板，保留机器人视口。视频时间不等于五次轨迹的相对时间；图像说明实际模型与姿态变化，跟踪精度仍由前面的时戳和关节记录评价。'),
's11':('接触几何与姿态变化','补充原报告保存的实际仿真关键帧。图中为补偿内环示例，用于辨认球形探针、平面与接触位置；普通PID主结果仍采用本节对应曲线和记录，不把不同内环的画面混作同条件对照。'),
's14':('带重力接触的画面与阶段','两侧均来自同一场景下保存的实际相机帧。相机画面能说明夹爪与平面的相对位置，不能独立分辨数牛的力差；因此保留同步力曲线，并在下面另外给出不含曲线的接近与接触画面。'),
's18':('简化模型的运动示意','下图显示LIPM参考、支撑点与控制结果的关系，属于数学模型可视化。它没有真实摆腿惯量或关节力矩，不能作为多关节动力学步行的画面。'),
's19':('有限脚掌站立与推扰画面','本阶段的物理模型只有有限脚掌、踝关节与上部集中质量。画面呈现PD和LQR的站立响应，与后续平面双足连续迈步分开。'),
's21':('实际摆腿与失稳对照','保留完整力矩和半力矩两侧的原始对照画面。右侧在失败后停留于最后保存姿态，不继续补出运动；“仍显示机器人”不等于该条件继续完成了行走。'),
's23':('二十步实际行走的连续画面','下图来自已完成二十次实测落脚的保存录屏，包含施加40牛、持续0.1秒推力的回合。相机跟随机器人，画面中的地面位置不能直接用来估计前进量；距离和落脚数以前文保存状态与接触记录为准。'),
's25':('左右单脚支撑的双视角过程','以下画面从已保存的关节位置与速度重建，未执行控制器或推进物理积分。各图同时列出原日志中的足高与法向力；初始和末态为双支撑，中间帧展示实际单脚承重。重建时的接触求解值不用于替代原日志的力。'),
's26':('三维步行的双视角关键帧','补入原始四步录屏中的斜前方与正前方双视角，可观察横向重心转移和交替离地。图中的“经过的步态周期”是名义时间进度，实际完成仍依据保存的足位和承重确认。'),
's28':('强扰动完成与终止姿态','下面从冻结控制器测试的已保存状态重建画面。150牛站立条件完成，250牛站立及40牛侧向行走条件因约束冲突提前终止；图像只画到保存时刻，没有让终止后的模型继续下落，也不把约束冲突自动称作倒地。')}

sheet('s03','reports/bytedance_engineering/videos/ch02/panda_joint_trajectory.mp4','panda_trajectory_viewport_sequence.png',[.5,2.5,5.,8.],(50,30,1000,605),2,'Isaac Panda实际轨迹录屏的四个时刻；裁剪仅去除界面侧栏，时间为视频播放时刻。')
copy('s11','experiments/panda_contact_isaac/figures/keyframes.png','panda_contact_compensated_keyframes.png','Isaac球形探针接触的保存关键帧；这是补偿内环示例，不替代普通PID主实验的条件标记。')
copy('s14','experiments/panda_contact_optimization/figures/comparison_keyframe.png','panda_gravity_comparison_full_frame.png','相同重力、几何与内环下的位置控制和优化导纳：实际画面与同步均力曲线。')
# Preserve native camera pairs, no graph panel in these extracts.
for idx in [0,140,300]:
 imgs=[];sources=[]
 for mode in ['video_position','video_admittance']:
  source=R/f'experiments/panda_contact_optimization/runs/{mode}/frames/{idx:05d}.png';assert source.exists(),source
  imgs.append(Image.open(source).convert('RGB').resize((800,600)));sources.append(str(source.relative_to(R)))
 canvas=Image.new('RGB',(1600,655),'white');draw=ImageDraw.Draw(canvas)
 for j,(im,label) in enumerate(zip(imgs,['位置控制','优化导纳控制'])):canvas.paste(im,(j*800,55));draw.text((j*800+15,10),label,font=font(26),fill='black')
 p=D/f'figures/ch05/panda_contact_camera_pair_frame{idx:03d}.png';canvas.save(p)
 cap=f'两种控制条件的纯相机画面，保存帧序号{idx}（约{idx*.1+.001:.3f}秒仿真时间）。'
 rel=register('s14',p,R/sources[0],cap,'pair of saved camera images; resize only',dict(sources=sources,source_sha256=[sha(R/x) for x in sources],frame_index=idx));add('s14',rel,cap)
copy('s18','experiments/lipm_balance/figures/lipm_keyframe.png','lipm_reference_support_visualization.png','线性倒立摆的状态与支撑示意，不属于多关节物理相机画面。')
copy('s19','experiments/lipm_balance/figures/physical_keyframe.png','finite_foot_standing_comparison.png','有限脚掌非线性站立模型的实际物理画面与对应姿态记录；左PD、右LQR。')
sheet('s19','experiments/lipm_balance/physical_comparison.mp4','finite_foot_push_camera_sequence.png',[2.65,2.85,4.5],(20,98,1390,603),1,'有限脚掌站立推扰前、推扰后及后续恢复画面；每行左PD、右LQR，时间为视频时刻。')
# Existing early biped snapshots are already in the main text; add an explicit failure-time video extract.
sheet('s21','reports/bytedance_engineering/videos/ch07/planar_biped_torque_comparison.mp4','planar_biped_failure_camera_sequence.png',[2.5,3.5,4.5],(20,98,1390,603),1,'完整力矩与半力矩的实际相机画面对照；后者终止后冻结末帧，不能将其显示时长当成有效运行时长。')
copy('s23','experiments/biped_feedback/figures/frame_142.png','planar_twenty_steps_full_frame.png','二十步行走实际相机帧与同期质心、接触记录；画面显示的名义经过时段不代替实测落脚判据。')
for num,times in enumerate([[.5,1.2,2.7,3.25],[7.1,9.0,11.0,13.5]],1):
 sheet('s23','reports/bytedance_engineering/videos/ch07/planar_biped_twenty_steps.mp4',f'planar_twenty_steps_camera_sequence_{num}.png',times,(40,66,1080,704),2,'二十步实际动力学录屏的'+('前段' if num==1 else '后段')+'相机画面；为保留机器人和脚掌而裁去文字栏。')
# Split tall saved-state contact sheets into legible full-width snapshots.
state=json.loads((D/'STATE_RENDER_MANIFEST.json').read_text())
for entry in state:
 name=Path(entry['local']).stem;side='single_support' in name;walk='walk_stress' in name;indices=range(3) if side or walk else [2]
 sid='s25' if side else 's28';im=Image.open(D/entry['local'])
 for j in indices:
  panel=im.crop((0,710*j,1440,710*(j+1)));p=D/'figures/ch08'/f'{name}_view{j}.png';panel.save(p);t=entry['frames'][j]['time']
  if side:
   label='左脚支撑流程' if 'side0' in name else '右脚支撑流程';phase=['初始双支撑','单脚承重','落脚后双支撑'][j]
  else:label='40牛侧向行走扰动' if walk else ('150牛站立扰动' if '150n' in name else '250牛站立扰动');phase=['推扰前','推扰后','最后保存状态'][j]
  header=ImageDraw.Draw(panel);header.rectangle((0,0,1440,51),fill='white');header.text((20,8),f'{label}：{phase}　t={t:.3f} s',font=font(27),fill='black');panel.save(p)
  cap=f'G1{label}：{phase}，{t:.3f}秒。依据保存状态重建，图下的足高与法向力取自历史日志。'
  rel=register(sid,p,R/entry['source'],cap,'saved-state render; no physics stepping',dict(state_render_manifest='STATE_RENDER_MANIFEST.json',frame=entry['frames'][j],trace_sha256=entry['source_sha256']));add(sid,rel,cap)
for frame in [18,60,180,340,560]:
 copy('s26',f'experiments/humanoid_control/figures/camera_{frame:03d}.png',f'g1_walk_dual_camera_{frame:03d}.png',f'G1四步准静态行走的原始双视角画面，仿真时间{frame/20:.2f}秒；保留原始状态说明。')
for sid,src,dst in [('s18','experiments/lipm_balance/lipm_comparison.mp4','videos/ch06/lipm_model_comparison.mp4'),('s19','experiments/lipm_balance/physical_comparison.mp4','videos/ch06/finite_foot_standing_comparison.mp4')]:
 p=D/dst;p.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(R/src,p);register(sid,p,R/src,'倒立摆模型示意' if sid=='s18' else '有限脚掌物理站立','copied existing video',kind='video');blocks[sid].append(r'\stagevideo{'+dst+'}')
for s in stages:
 sid=s['id']
 if sid not in blocks:continue
 p=next((D/'chapters').glob(f"{s['chapter']:02d}_*.tex"));text=p.read_text();start=f'% BEGIN SIM_VISUALS {sid}';end=f'% END SIM_VISUALS {sid}'
 text=re.sub(re.escape(start)+r'[\s\S]*?'+re.escape(end)+'\n?','',text)
 heading,intro=introductions[sid];block='\n'+start+'\n'+r'\subsection{'+heading+'}\n'+intro+'\n\n'+'\n\n'.join(blocks[sid])+'\n'+end+'\n'
 text=text.replace(r'\stagecode{'+sid+'}',block+r'\stagecode{'+sid+'}');p.write_text(text)
# Add a concise visual road map early in the document using the same actual assets.
p=D/'chapters/01_overview.tex';text=p.read_text();start='% BEGIN SIM_VISUAL_OVERVIEW';end='% END SIM_VISUAL_OVERVIEW';text=re.sub(re.escape(start)+r'[\s\S]*?'+re.escape(end)+'\n?','',text)
block='\n'+start+'\n'+r'\section{主要仿真场景与阅读顺序}'+'\n以下三幅图先说明实验对象。后续各章同时给出相机画面和定量记录；模型示意、物理录屏和已保存状态重建分别标注。\n'
for rel,cap,label in [('figures/ch05/panda_gravity_comparison_full_frame.png','Panda固定基座接触：比较夹爪与平面的实际相对运动。','overview-panda'),('figures/ch07/planar_twenty_steps_full_frame.png','平面双足连续行走：躯干、双腿和实际脚地接触共同推进。','overview-biped'),('figures/ch08/g1_walk_dual_camera_180.png','G1三维浮动基座：斜前方和正前方视角展示横向重心转移。','overview-g1')]:block+='\n'+r'\motionfigure{'+rel+'}{'+cap+'}{fig:'+label+'}\n'
block+='\n'+end+'\n';text+=block;p.write_text(text)
(D/'asset_manifest.json').write_text(json.dumps(assets,ensure_ascii=False,indent=2));(D/'SIMULATION_VISUALS.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
print('New visual figures',sum(x['kind']=='figure' for x in records),'total figure assets',sum(x['kind']=='figure' for x in assets),'videos',sum(x['kind']=='video' for x in assets))
