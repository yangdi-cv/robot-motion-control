"""Curate archived control reports. Run only for migration, not normal PDF builds."""
from pathlib import Path
import re,json,hashlib,subprocess,shutil,math
R=Path(__file__).resolve().parents[3];D=R/'reports/bytedance_engineering'
PANDOC=Path('/home/yd/.local/share/vla-report-tools/pandoc/pandoc')
# Local reviewed conversion helpers; normal PDF build does not execute migration.
stages=[]
exec((Path(__file__).with_name('markdown_support.py')).read_text())
parts={};stages=[];assets=[];coverage=[];missing=[];commands=[];stats={};known={};used_figures={};used_videos={}
chapters={2:('02_platform','仿真平台与机械臂轨迹跟踪','首先用真实ROS2消息连接轨迹节点与物理仿真，明确时间戳、关节单位和控制接口，再比较驱动增益与参考速度。'),3:('03_feedback','状态测量、滤波与反馈时序','把虚拟传感器、公开实测数据和自有设备采集分别记录。采样、发布、控制与物理积分的频率不同，滤波平滑也可能增加数据滞后。'),4:('04_contact','末端接触与导纳控制','自由空间跟踪精度不决定接触表现。本章先用MuJoCo研究法向柔顺，再在Isaac中核对普通关节内环、原始冲击与有限窗口稳定。'),5:('05_optimization','更完整物理条件下的控制优化','恢复重力和手部碰撞后重新验证接触方案，同时明确物理步长敏感性。动作块执行接口独立分析，不将解析轨迹当成学习策略。'),6:('06_lipm','倒立摆规划与平衡恢复','简化模型便于分析支撑区域、落脚和延迟，但忽略了完整摆腿动力学。本章先验证模型内恢复，再用有限脚掌接触和有约束的步态说明边界。'),7:('07_biped','平面双足连续行走','将规划参考、实际摆腿、接触和有限电机力矩放入同一物理系统。比较统一的质心与零力矩点接口，同时记录误差与完成率之间的折中。'),8:('08_humanoid','三维人形全身控制','去掉平面自由度约束，在G1完整浮动基座上验证横向重心转移、单脚承重和准静态交替迈步。强扰动失败与运行时尾部同时保留。')}
def add(ch,sid,title,items,conclusion):
 s=dict(chapter=ch,report_chapter=ch,id=sid,title=title,sources=list(dict.fromkeys('experiments/'+name+'/REPORT.md' for name,keys in items)),code=list(dict.fromkeys('experiments/'+name+'/scripts' for name,keys in items)),conclusion=conclusion)
 stages.append(s);parts[sid]=[];used_figures[sid]=set();used_videos[sid]=set()
 for name,keys in items:select(sid,'experiments/'+name+'/REPORT.md',keys)
add(2,'s01','运行环境与隔离',[('panda_task1',[1,2])],'Isaac独立Python与系统ROS2分别运行，通过专用通信域连接；没有修改系统Python或原训练环境。')
add(2,'s02','Panda模型、关节与ROS2通信',[('panda_task1',[3])],'关节名称、限位和初态来自实际加载模型；初期验证关节空间，不额外声称末端位姿或完整坐标树已验收。')
add(2,'s03','五次轨迹与默认驱动基准',[('panda_task1',[4,5])],'5秒轨迹最终能到达目标，中间存在约1.36至2.00度峰值误差，为后续调参提供基准。')
add(2,'s04','驱动增益、参考速度与幅度验证',[('control_feedback_task2',[2,3,6])],'独立幅度验证的平均运动误差由0.864度降至0.082度。参考速度收益明显，但不是逆动力学力矩前馈，也不是硬件性能。')
add(3,'s05','反馈频率与虚拟传感器链路',[('imu_week2',[1,2,3])],'25/50/100Hz比较改变外环状态与命令更新，不改变100Hz物理驱动；频率提高没有消除默认驱动滞后。')
add(3,'s06','因果滤波与ROS2消息验证',[('imu_week2',[4,5])],'移动平均降低设定噪声但带来约80毫秒滞后；消息回放证明接口连通，与实体传感器联调分开。')
add(3,'s07','公开真实IMU与噪声分析',[('control_feedback_task2',[4,7])],'TUM-VI六轴静止标准差下降约49.2%至52.6%；缺运动真值，不能把运动波形变平解释成测量精度提高。')
add(3,'s08','发布频率、数据年龄与Jetson实体采集',[('control_feedback_task2',[5,8])],'实体IMU约25Hz产生新测量，200Hz发布时约87.52%重复；传输P95不足以代表包含滤波的端到端时延。')
add(4,'s09','MuJoCo法向接触模型与导纳设计',[('bytedance_week3',[1,2])],'采用真实动力学和有限关节力矩，但仅单法向探头接触；接触参数档不是已标定材料刚度。')
add(4,'s10','力跟随、噪声与首次碰撞',[('bytedance_week3',[3,4,5])],'稳定力误差可大幅下降，首次冲击仍需接近速度和阶段切换共同处理。')
add(4,'s11','Isaac普通PID与接触对照',[('panda_contact_isaac',[1,2,3,4])],'原始1毫秒峰值与10毫秒均力必须分开；关闭重力和简化碰撞的结果不能直接迁移到完整物理场景。')
add(4,'s12','恒力稳定性诊断与连续逆解',[('panda_contact_isaac',[7])],'组合方案通过连续0.5秒有限窗口标准，但末段仍有部分周期均力越界；这不是全程恒力。')
add(5,'s13','恢复重力与手部碰撞',[('panda_contact_optimization',[1,2,3])],'重力缺失补偿造成初态下坠；恢复手部与双指碰撞后重新建立对照，其他连杆碰撞仍未全面恢复。')
add(5,'s14','接近减速、目标力缓升与修正速度',[('panda_contact_optimization',[4])],'选用统一方案原始峰值11.11牛，末段均力约5.007牛；单因素和版本变化分别说明，不用较好先导替代复测。')
add(5,'s15','扰动、接触材质与数值敏感性',[('panda_contact_optimization',[5])],'刚性接触减半步长后峰值由11.11增至20.43牛，瞬时峰值未数值收敛；柔顺材质另列，不作跨材质算法提升比较。')
add(5,'s16','动作块执行、延迟与跨进程通信',[('panda_contact_isaac',[6])],'动作源是解析参考，验证时间戳、限幅、超时与恢复；这些结果不能解释为VLA网络控制成功。')
add(6,'s17','倒立摆模型、步态与控制器',[('lipm_balance',[1,2,3])],'LIPM将机器人简化为恒高质点，支撑输入是地面位置；PD和LQR应在相同约束与恢复判据下比较。')
add(6,'s18','推扰恢复、落脚调整与延迟预测',[('lipm_balance',[4,5,6])],'12牛秒与80毫秒条件可通过组合策略恢复，但伴随路线偏移；预测只使用已收测量和已发输入。')
add(6,'s19','有限脚掌非线性站立验证',[('lipm_balance',[7])],'MuJoCo验证脚掌接触与站立恢复，不能与简化模型步态拼接成已完成多关节行走。')
add(6,'s20','独立测量时钟与受限摆脚规划',[('lipm_extension',[2,3])],'有色噪声加漂移明显降低通过数；双支撑会挤占摆脚时间，联合调整并非在全部条件更好。')
add(7,'s21','力矩驱动的平面双足与八步验证',[('lipm_extension',[4,5])],'51个唯一动力学条件通过50个；标准八步前进约0.937米，但尚未统一全部反馈时序实验。')
add(7,'s22','质心与零力矩点到关节力矩的统一接口',[('biped_feedback',[1,2])],'统一水平加速度目标改善实际压力中心跟踪，同时增加质心误差，不是所有指标一起提高。')
add(7,'s23','连续摆腿、双支撑与二十步行走',[('biped_feedback',[3])],'两组二十步均完成实际落脚并前进约2.323米；名义支撑与实测接触有核对，尚无触地事件重排步时。')
add(7,'s24','频率、延迟与动力学独立核验',[('biped_feedback',[4,5])],'94个去重条件79个通过；80毫秒延迟多数失败，常速度预测不能解决全部接触切换问题。')
add(8,'s25','G1浮动基座、站立与双侧单脚支撑',[('humanoid_control',[1,2])],'真实三维浮动基座与29个受限电机完成重心转移和双侧单脚承重，未用外部支撑锁定侧向自由度。')
add(8,'s26','三维准静态交替步行',[('humanoid_control',[3])],'四步参考29秒前进约0.210米，六步43秒约0.330米，约7秒一步，属于准静态而非动态高速步行。')
add(8,'s27','QP求解器对照与耗时尾部',[('humanoid_control',[4])],'Clarabel在所测控制调用中尾部更短；这不是包含传感、通信和调度的硬实时保证。')
add(8,'s28','冻结条件、强扰动与保存核验',[('humanoid_control',[5,6,7])],'正式52个条件51个通过，另八个强扰动仅两个通过；状态估计、触地时序、自碰撞和真实执行器仍待补齐。')
# Separate optimization conclusions from its new UDP experiment.
full,blocks=section_blocks('experiments/panda_contact_optimization/REPORT.md')
h,b=next((h,b) for h,b in blocks if h.startswith('6.'))
main,udp=b.split('### 补充：真实本机 UDP 跨进程通信')
parts['s15'].append(dict(source='experiments/panda_contact_optimization/REPORT.md',markdown=main))
parts['s16'].append(dict(source='experiments/panda_contact_optimization/REPORT.md',markdown='## 本机UDP跨进程通信\n'+udp))
coverage.extend([dict(stage=i,source='experiments/panda_contact_optimization/REPORT.md',section=h+'（分段）',source_sha256=digest(R/'experiments/panda_contact_optimization/REPORT.md')) for i in ['s15','s16']])
for i in ['s15','s16']:
 if 'experiments/panda_contact_optimization/REPORT.md' not in next(s for s in stages if s['id']==i)['sources']:next(s for s in stages if s['id']==i)['sources'].append('experiments/panda_contact_optimization/REPORT.md')
# Representative saved recordings. Deliberately retain actual physical runs, not generated motion.
video_specs=[('s03','experiments/panda_task1/runs/run_20261013_003/simulation_recording.mp4','panda_joint_trajectory'),('s10','experiments/bytedance_week3/runs/contact_suite_v3/position.mp4','panda_position_contact'),('s10','experiments/bytedance_week3/runs/contact_suite_v3/admittance.mp4','panda_admittance_contact'),('s21','experiments/lipm_extension/biped_comparison.mp4','planar_biped_torque_comparison'),('s23','experiments/biped_feedback/连续二十步_实际动力学.mp4','planar_biped_twenty_steps'),('s26','experiments/humanoid_control/G1_三维四步准静态行走.mp4','g1_quasistatic_four_steps')]
for sid,source,name in video_specs:
 s=next(s for s in stages if s['id']==sid);src=R/source
 if not src.exists():missing.append(dict(stage=sid,kind='video',path=source));continue
 rel=f"videos/ch{s['chapter']:02d}/{name}.mp4";(D/rel).parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(src,D/rel)
 assets.append(dict(kind='video',stage=sid,source=source,local=rel,sha256=digest(src),caption=name));used_videos[sid].add(rel);known[(sid,digest(src))]=rel
for ch,(file,title,intro) in chapters.items():
 out=[r'\chapter{'+title+'}',intro]
 for s in [x for x in stages if x['chapter']==ch]:
  sid=s['id'];out.extend([r'\section{'+s['title']+'}',r'\label{stage:'+sid+'}']);current={'heading':'','header_count':0,'tables':0,'images':0};combined=[]
  for part in parts[sid]:
   body=normalize(part['markdown'])
   body=re.sub(r'<a\s+id="[^"]+"></a>','',body)
   combined.append(body)
   doc=json.loads(subprocess.check_output([str(PANDOC),'-f','markdown+pipe_tables+tex_math_dollars+tex_math_single_backslash','-t','json'],input=body,text=True));doc['blocks']=walk(doc['blocks'],s,part['source'])
   rendered=subprocess.check_output([str(PANDOC),'-f','json','-t','latex','--wrap=none','--syntax-highlighting=none'],input=json.dumps(doc),text=True)
   rendered=re.sub(r'\\begin\{figure\}.*?(\\stagefigure\{[^\n]+)\n\\caption\{[^\n]*\}\n\\end\{figure\}',r'\1',rendered,flags=re.S)
   rendered=rendered.replace(r'\begin{longtable}',r'\FloatBarrier\begingroup\small\setlength{\tabcolsep}{2.5pt}'+'\n'+r'\begin{longtable}').replace(r'\end{longtable}',r'\end{longtable}\endgroup')
   rendered=rendered.replace(r'\(\pi\)0.5',r'\pifive{}')
   out.append(rendered)
  out.extend([r'\paragraph{阶段结论}'+tex(s['conclusion']),r'\stagecode{'+sid+'}'])
  for v in sorted(used_videos[sid]):out.append(r'\stagevideo{'+v+'}')
  out.append(r'\FloatBarrier');(D/'source_fragments'/f'{sid}.md').write_text('\n\n'.join(combined));stats[sid]=dict(source_pieces=len(combined),source_characters=sum(len(x) for x in combined),tables=current['tables'],figures=current['images'])
 (D/'chapters'/f'{file}.tex').write_text('\n\n'.join(out)+'\n')
app=[r'\chapter{运行命令与目录示例}','以下为对应阶段的历史运行入口。复现前应阅读源目录README与依赖要求，新实验使用新输出目录。']
for c in commands:app.extend([r'\section*{'+c['stage'].upper()+'运行示例}',r'\noindent\path|'+str(c['source'])+'|',r'\hypertarget{'+c['id']+'}{}','\\begin{Verbatim}[breaklines=true,breakanywhere=true,fontsize=\\footnotesize]\n'+c['text']+'\n\\end{Verbatim}'])
(D/'appendices/reproduction.tex').write_text('\n\n'.join(app)+'\n')
for name,obj in [('outline',stages),('asset_manifest',assets),('source_coverage',coverage),('expansion_stats',stats),('missing_assets',missing)]: (D/(name+'.json')).write_text(json.dumps(obj,ensure_ascii=False,indent=2))
print('Stages',len(stages),'selected sections',len(coverage),'tables',sum(x['tables'] for x in stats.values()),'assets',len(assets),'missing',missing)
