from pathlib import Path
import json, html, shutil, zipfile, hashlib, subprocess
R = Path(__file__).resolve().parents[1]
root = R.parents[1]
result = json.loads((R / 'results.json').read_text())
S=json.loads((R/'stability_results.json').read_text())
E_all=json.loads((R/'execution_results.json').read_text())
all_names=sorted(set(result)|set(S)|{'execution_'+n for n in E_all})
full_samples=sum(sum(1 for _ in (R/'runs'/n/'physics_force.csv').open())-1 for n in all_names)
partial=[p for p in (R/'runs').glob('stability_*/FAILED.json') if not (p.parent/'COMPLETE').exists()]
study_summary={'full_length_runs':len(all_names),'full_force_samples':full_samples,'full_duration_s':full_samples*.001,'guarded_partial_runs':len(partial),'full_run_names':all_names}
(R/'study_summary.json').write_text(json.dumps(study_summary,indent=2))
md = []
doc = []

def h(s, l=2):
    md.extend(['', '#' * l + ' ' + s, ''])
    doc.append(f'<h{l}>{html.escape(s)}</h{l}>')

def p(s):
    md.extend([s, ''])
    doc.append('<p>' + html.escape(s) + '</p>')

def table(head, rows):
    md.extend(['| ' + ' | '.join(head) + ' |', '|' + '|'.join(['---'] * len(head)) + '|'] + ['| ' + ' | '.join(map(str, row)) + ' |' for row in rows] + [''])
    doc.append('<table><tr>' + ''.join(('<th>' + html.escape(x) + '</th>' for x in head)) + '</tr>' + ''.join(('<tr>' + ''.join(('<td>' + html.escape(str(x)) + '</td>' for x in row)) + '</tr>' for row in rows)) + '</table>')

def code(s):
    md.extend(['```python', s, '```', ''])
    doc.append('<pre>' + html.escape(s) + '</pre>')

def pic(name, caption):
    md.extend([f'![{caption}](figures/{name})', ''])
    doc.append(f'<figure><img src="figures/{name}"><figcaption>{html.escape(caption)}</figcaption></figure>')

def link(path, text):
    md.extend([f'[{text}]({path})', ''])
    doc.append(f'<p><a href="{path}">{html.escape(text)}</a></p>')

def num(x):
    return '未达到' if x is None else f'{x:.3f}'
base = result['pid_admittance']
position = result['pid_position']
h('Franka Panda 多关节轨迹跟踪与末端柔顺控制', 1)
p('2026年10月25—26日｜Isaac Sim 5.1.0｜Ubuntu 22.04 / RTX 4090')
h('摘要')
p(f'累计{len(all_names)}组全长物理试验、{full_samples*.001:.0f}秒、{full_samples}条1ms力记录；另有{len(partial)}组由逆解保护提前终止。54组纯接口回放单独统计。全长试验包含失败对照，不代表全部控制方案合格。')
p('本报告分为基础控制实验与执行接口扩展两部分。主实验采用普通关节PID，扩展保留质量矩阵补偿对照，并测试动作分块、延迟、目标跳变和断流恢复。全部物理实验在PC端Isaac Sim完成。')
p(f"本次完成前三关节5秒五次轨迹和固定平面接触实验。采用相同普通PID关节内环比较位置控制与5N导纳控制，主条件为M=1、B=40。位置控制末两秒平均接触力为{position['last2s_mean_N']:.2f}N，导纳为{base['last2s_mean_N']:.2f}N；1ms原始接触力峰值分别为{position['raw_force_peak_N']:.2f}N和{base['raw_force_peak_N']:.2f}N。导纳改善了平均力误差，但并没有消除瞬时冲击。")
p('实验使用Panda手部球形探针与水平面，关闭原装夹爪和其他连杆的碰撞，并统一关闭重力，以隔离法向接触控制。主实验的关节力矩由比例、积分和实测速度阻尼直接计算，不使用质量矩阵或科氏项补偿；补偿控制另列为扩展。全部结果来自实际运行的PhysX接触和关节状态，不包含实体机器人实验。')
p('主实验之后，分别验证接触阶段切换与学习策略执行接口。接口实验使用已知五次参考生成动作块，不运行VLA网络；因此结果衡量延迟、限幅和超时机制，不能解释为VLA取放成功率。')
p('稳定性改进后，在M=1/B=40和10ms普通PID内环保持不变的条件下，连续微分逆解、接触后冻结名义位置和指数反馈这一组合达到原有连续0.5秒标准。默认场景末2秒力RMSE约0.244N，最长带内跨度0.98秒；重复及平面高度±1mm也通过。后段仍有6%—18%的周期均力越界，尚未实现全程恒力。')
h('目录')
p('1. 场景和参数\n2. 控制方法与代码\n3. 主要对比结果\n4. 参数对照与问题分析\n5. 复现方式与代码目录\n6. 扩展：学习策略动作执行接口\n7. 恒力稳定性排查与改进\n8. 结论与后续工作')
h('1. 场景和参数')
table(['项目', '设置'], [['软件', 'Isaac Sim 5.1.0；独立Python 3.11环境'], ['物理与控制周期', '物理1ms；控制10ms，仿真时间基准'], ['关节初态/rad', '[0, -0.785398, 0, -2.356194, 0, 1.570796, 0.785398]'], ['五次轨迹', '前三关节增量[0.06, 0.04, -0.04]rad；持续5s'], ['探针与平面', '手部原点半径25mm球；固定平面220×220mm，厚20mm'], ['接近轨迹', '5s后沿世界-Z方向以5mm/s接近，名义最大位移55mm'], ['力目标与主导纳参数', '5N；M=1kg，B=40N·s/m，无弹簧项'], ['导纳限幅', '位移±80mm，速度±30mm/s；另测位移±30mm'], ['普通PID参数', '七轴Kp=[200,200,150,150,10,8,1]；Kd=[10,10,8,8,0.3,0.2,0.02]；Ki=[5,5,5,5,0.2,0.2,0.05]；积分±0.2rad·s'], ['力矩限制', '前四轴87Nm，后三轴12Nm；夹爪5N'], ['测力', 'PhysX手部—平面接触矩阵；传入dt=0.001换算N'], ['力信号', '保存1kHz原始值；100Hz控制使用前一周期10样本平均']])
p('0—5秒完成关节轨迹，之后固定末端姿态，由Lula逆运动学求沿平面法向运动的关节目标。两个主条件的初态、名义接近轨迹、碰撞几何、内环和力矩限幅相同，仅导纳条件叠加力反馈位移。控制周期指仿真中的10ms，不代表主机实时计算保证。')
pic('scene.png','Panda球形探针与固定水平接触面；原夹爪几何不参与本轮实验。')
h('2. 控制方法与代码')
p('五次插值s(u)=10u³−15u⁴+6u⁵，u=t/5。普通PID输出直接作为关节力矩，D项使用实测角速度，避免目标跳变引起微分冲击。七轴增益按惯量差异配置，两种主条件完全相同。重力统一关闭，以隔离本轮轨迹与接触行为。增益单位为Kp：Nm/rad、Kd：Nm·s/rad、Ki：Nm/(rad·s)。')
code('error = q_reference - q\nintegral = clip(integral + error * 0.01, -0.2, 0.2)\ntorque = kp * error - kd * dq + ki * integral\ntorque = clip(torque, -torque_limit, torque_limit)')
p('导纳坐标的正方向取向下压入平面，PhysX向上的反力取正。用目标力减实测力驱动向下的附加位移；力偏大时附加位移减小。因此在世界Z坐标中需要减去导纳位移，不能直接照搬正负号。')
code('acc = (5.0 - measured_force - B * velocity) / M\nvelocity = clip(velocity + acc * 0.01, -0.03, 0.03)\noffset = clip(offset + velocity * 0.01, -limit, limit)\nz_reference = z_nominal - offset')
p('相关代码在 scripts/run.py。分析与图表在 scripts/analyze.py，运行条件在各 runs/<条件>/metadata.json。')
h('3. 主要对比结果')
table(['指标', '位置PID', '导纳 M1/B40'], [['前三轴5s RMSE/度', '/'.join((f'{v:.4f}' for v in position['joint_first3_rmse_deg'])), '/'.join((f'{v:.4f}' for v in base['joint_first3_rmse_deg']))], ['1ms原始力峰值/N', num(position['raw_force_peak_N']), num(base['raw_force_peak_N'])], ['10ms平均力峰值/N', num(position['control_mean_force_peak_N']), num(base['control_mean_force_peak_N'])], ['末2s平均力/N', num(position['last2s_mean_N']), num(base['last2s_mean_N'])], ['末2s目标力RMSE/N', num(position['last2s_force_rmse_N']), num(base['last2s_force_rmse_N'])], ['法向位置跟踪RMSE/mm', num(position['normal_position_rmse_mm']), num(base['normal_position_rmse_mm'])], ['三维位置跟踪RMSE/mm', num(position['position_3d_rmse_mm']), num(base['position_3d_rmse_mm'])], ['严格稳定接触时间/s', num(position['stable_contact_after_first_hit_s']), num(base['stable_contact_after_first_hit_s'])]])
p('严格稳定接触时间定义为：从首次周期平均力超过0.5N起，到连续0.5秒内每个100Hz力样本均落在5±0.5N的时间。未达到表示本轮运行内未满足此条件，不填写0，也不以平均力接近5N代替。法向位置误差相对于各自实际发送的末端位置目标计算，导纳目标会随力反馈改变，因此不能把该误差单独作为优劣判据。')
pic('joints.png', '前三关节5秒轨迹跟踪，角度单位为度。')
pic('fz.png', '周期平均接触力；原始1ms峰值另列于结果表。')
pic('z.png', '末端高度变化；刚性平面限制实际高度继续下降。')
p(f'主条件的平均力虽接近5N，原始冲击峰值却高于位置控制。首次周期平均力超过0.5N之前约20ms，根据高度差分估算的下降速度约为位置控制{position["pre_contact_downward_speed_mm_s"]:.1f}mm/s、直接导纳{base["pre_contact_downward_speed_mm_s"]:.1f}mm/s。原因是空中实测力为0，恒力导纳也在不断增加向下位移，叠加到名义接近速度上。')
h('4. 参数对照与问题分析')
p('先在普通PID内环下比较接触切换和接近速度，保持M=1/B=40。三维位置误差由真实末端XYZ与发送的XYZ目标计算；不同控制条件的目标会变化，位置误差与接触力需一起看。')
table(['普通PID条件','原始峰值/N','末2s均力/N','力RMSE/N'],[[n,num(result[k]['raw_force_peak_N']),num(result[k]['last2s_mean_N']),num(result[k]['last2s_force_rmse_N'])] for k,n in [('pid_position','位置控制'),('pid_admittance','直接导纳'),('pid_contact_gate','接触后启用'),('pid_gate_slow','接触后启用、2.5mm/s')]])
p('接触检测发生在首次碰撞之后，因此门控本身不能预先消除初次冲击。降低接近速度与接触后退让是两件事，需要分别验证。下面保留补偿PID的九组结果，内环为质量矩阵乘以PID加速度，再补偿科氏/离心项；加速度形式Kp=100/s²、Kd=20/s、Ki=5/s³。两组增益参数化不同，不能单独据此归因补偿算法的优劣；与普通PID结果分组展示，不跨组混算改善比例。')
labels = {'position_final':'补偿PID位置控制','admittance_final': '补偿PID M1/B40', 'admittance_slow': '接近速度2.5mm/s', 'admittance_B20': '阻尼B20', 'admittance_B80': '阻尼B80', 'admittance_limited': '位移限幅±30mm', 'admittance_repeat': '补偿PID重复运行', 'admittance_contact_gate':'接触后启用导纳', 'admittance_force_ramp':'接触后0.5s力渐增'}
table(['条件', '原始峰值/N', '末2s平均/N', '末2s力RMSE/N', '严格稳定时间/s'], [[labels[n], num(v['raw_force_peak_N']), num(v['last2s_mean_N']), num(v['last2s_force_rmse_N']), num(v['stable_contact_after_first_hit_s'])] for (n, v) in result.items() if n in labels])
p('慢速条件保持相同55mm最大名义接近位移，运行32秒，其余20秒。阻尼对照只改B，基准条件保持B=40；该表各项均使用补偿PID内环。位移限幅对照检验控制器是否因无法继续退让而产生稳态力偏差。重复运行用于检查本机确定性，不是不同初态的独立泛化样本。')
p('B=20、40、80的首次峰值相同，因为接触前都已触及30mm/s导纳速度限幅；减慢名义接近速度虽降低冲击，仍未消除额外下压。接触后启用导纳直接处理了这个阶段切换问题。0.5秒目标力渐增只带来小幅进一步改善，不能据此声称优于其他方法或具有统计显著性。主条件重复运行的力序列一致，两个主条件前5秒的七轴实测轨迹也完全一致。')
pic('transition.png','直接导纳与接触后切换的对照：上图周期平均力，下图1ms原始力。')
p('本轮触及的主要问题包括碰撞体遍历、时间步进和力信号采样。调试中发现带渲染的step与无渲染step会推进不同数量的物理步，已将旧运行移出正式统计；现改为每次只推进1ms物理步，再单独渲染，并逐步校验引擎时钟。20秒运行累计时钟差约1微秒，来自浮点步长表示。第一，USD实例化碰撞体需要先展开再检查，否则普通遍历会漏掉原夹爪碰撞，得到与设计不符的接触。第二，100Hz单点抽取会漏掉1kHz尖峰，所以同时保留原始峰值和周期平均力。第三，导纳输出存在速度、位移限幅，平均力接近目标并不能说明刚性接触完全稳定。')
p('导纳主动让出位置精度来控制接触力。位置控制坚持向平面内部走，会持续增加反力；导纳会退让，但离散更新、刚性接触、逆解变化及关节内环共同影响波动。本轮尚未逐项分离这些因素，不能把所有波动归因于单一参数。')
h('5. 复现方式与代码目录')
code('experiments/panda_contact_isaac/\n  scripts/run.py           场景、PID、导纳和日志\n  scripts/isaac-python     独立Isaac环境入口\n  scripts/pid_suite.sh     普通PID主实验与切换对照\n  scripts/suite.sh         补偿PID参数对照\n  scripts/revision_suite.sh 动作执行接口七组测试\n  scripts/action_stream.py 时标、动作块、限幅和超时\n  scripts/analyze_execution.py 执行接口指标与曲线\n  scripts/sweep_stream.py  窗口覆盖回放\n  scripts/finish.sh        切换策略与重复运行\n  scripts/analyze.py       指标、曲线、视频编码\n  scripts/verify.py        时钟、轨迹和成对一致性检查\n  scripts/report.py        本报告与提交文件\n  runs/<条件>/\n    scene.usda            场景覆盖层，引用官方Panda资源\n    metadata.json         条件参数\n    trace.csv             100Hz关节与末端记录\n    physics_force.csv     1kHz原始接触力\n    joint_tracking.csv    前三轴目标、实际与误差\n    time_audit.json       引擎时钟与总时长\n    simulation.mp4        可选录屏，不随提交\n  figures/\n  results.json\n  REPORT.md')
code('experiments/panda_contact_isaac/scripts/isaac-python \\\n  experiments/panda_contact_isaac/scripts/run.py \\\n  --inner pid --mode admittance \\\n  --out experiments/panda_contact_isaac/runs/new_run')
pic('keyframes.png','同场景补偿PID运行关键帧，用于展示几何和接触位置；普通PID结果见前文曲线和数据表。')
p('本次提交包含本PDF和源码ZIP。关键帧、参数及主要结果已嵌入PDF，无需额外视频文件。ZIP包含源码、依赖说明、报告MD及报告配图，不附原始CSV、录屏、权重或完整软件环境。目录中runs为运行后生成的文件，原始记录保留在本机。scene.usda引用NVIDIA官方Franka资源，首次加载需要访问资源服务器或已有缓存。Python环境使用现有.venvs/isaacsim51，没有改动系统Python、ROS2或Jetson环境。')
h('6. 扩展：学习策略动作执行接口')
p('扩展目标是检查策略返回动作以后，执行端如何处理动作时标、突变和断流。参考LeRobot预测与执行解耦、动作队列的设计；本轮为独立的确定性接口测试，没有接入ACT、SmolVLA或WAM推理，也没有实现训练式RTC。')
link('https://huggingface.co/docs/lerobot/async','参考：LeRobot异步推理与动作队列')
p('七组测试均在Isaac Panda实际动力学中执行，普通PID内环不变。前三轴使用三倍幅度的5秒五次参考，固定面下移以排除接触。模拟策略每200ms生成40步未来动作，动作间隔10ms，固定送达延迟120ms。这样可以把执行机制的影响与模型预测错误分开。')
table(['条件','注入或执行方式'],[['oracle','100Hz直接发送理想参考'],['delayed_naive','动作块到达后从第0步执行，忽略生成时间'],['delayed_timestamp','按当前时间减生成时间选择动作，跳过过期前缀'],['jitter_raw','每个块在第一轴交替叠加±0.04rad偏差'],['jitter_limited','同样偏差，命令速度限制0.35rad/s、加速度1rad/s²'],['outage_stale','2—3秒不生成新块，继续消费旧块并停留在最后目标'],['outage_hold','同样断流，源数据年龄超过0.5秒后保持一次读取的实测姿态，恢复时仍限速限加速度']])
E=json.loads((R/'execution_results.json').read_text())
table(['执行条件','实测对理想RMSE/度','最大命令跳变/度','峰值力矩/Nm'],[[k,num(v['actual_oracle_rmse_deg']),num(v['command_max_step_deg']),num(v['peak_effort_Nm'])] for k,v in E.items()])
p('RMSE统计前三轴相对理想参考的误差；跳变及峰值力矩统计全部七轴。所有条件使用同一条已知轨迹，没有多随机种子统计。命令限幅经过逐样本检查，但这不等于实际关节速度限幅或碰撞安全证明。')
pic('execution_delay.png','固定120ms延迟下，忽略时标与按时标选取动作的比较。')
p(f"忽略时标时，实际轨迹RMSE为{E['delayed_naive']['actual_oracle_rmse_deg']:.3f}°；按时标取动作后为{E['delayed_timestamp']['actual_oracle_rmse_deg']:.3f}°。这里的未来动作来自已知解析参考，时标对齐能够恢复理想执行；真实VLA预测误差、观察过时和环境变化并不能靠跳过前缀消除。")
pic('execution_jitter.png','同样的动作块偏差下，直接执行与命令限速限加速度。')
p(f"限幅将最大命令跳变从{E['jitter_raw']['command_max_step_deg']:.3f}°降至{E['jitter_limited']['command_max_step_deg']:.3f}°，峰值力矩从{E['jitter_raw']['peak_effort_Nm']:.3f}Nm降至{E['jitter_limited']['peak_effort_Nm']:.3f}Nm。平滑会改变原动作时序，因此不能只看动作更顺，还应核对跟踪误差及任务阶段。")
pic('execution_outage.png','橙色区间为停止产生动作块的2—3秒；包含送达延迟与超时等待。')
p(f"超时保持与受限恢复使最大命令跳变从{E['outage_stale']['command_max_step_deg']:.3f}°降至{E['outage_hold']['command_max_step_deg']:.3f}°，但轨迹RMSE从{E['outage_stale']['actual_oracle_rmse_deg']:.3f}°增加到{E['outage_hold']['actual_oracle_rmse_deg']:.3f}°。这是有意暂停追赶带来的代价，不能写成所有指标都改善。超时触发{E['outage_hold']['timeout_events']}次；该保持机制尚未在重力、接触负载或真机上验证。")
h('6.1 动作窗口覆盖的接口回放',3)
p('另外进行54组接口级回放：延迟40/120/240ms、块更新周期100/200/400ms、动作窗口200/400/800ms，分别比较忽略时标和按时标消费。这里只计算动作命令与窗口覆盖，不再次运行机器人动力学，不计入物理仿真次数。')
W=json.loads((R/'stream_sweep.json').read_text())
table(['延迟/ms','更新周期/ms','窗口/ms','窗口过期时间占比'],[[int(v['delay_s']*1000),int(v['period_s']*1000),int(v['horizon_s']*1000),f"{v['exhausted_fraction']*100:.1f}%"] for v in W if v['method']=='delayed_timestamp' and v['period_s']==.2 and (v['horizon_s']==.4 or (v['horizon_s']==.2 and v['delay_s']==.12))])
pic('stream_coverage.png','动作窗口不足时，即使按时标消费也会用完可执行动作。')
p('在固定延迟、无丢包的本模型下，窗口至少覆盖送达延迟与下一次块更新的间隔，才能持续提供未过期动作。真实系统还需要为抖动和计算时间变化留余量。扩大窗口可能降低反应速度，不能无限增加。配置化后的执行器还逐帧重放了七组已执行命令，与原命令一致。')
p('相关代码在 scripts/action_stream.py、scripts/analyze_execution.py 和 scripts/sweep_stream.py。结果分别在 execution_results.json、stream_sweep.json。下一步接入实际策略的带时标动作块，在相同扰动下复测；本轮不据此声称VLA已经能够完成操作任务。')
h('7. 恒力稳定性排查与改进')
p('本章保留原有标准：未经指数滤波的接触力，按10ms时间箱取均值，连续0.5秒内每个样本均在5±0.5N内。控制周期改为5ms或20ms时，评价仍使用相同的10ms箱。指数滤波仅进入控制反馈，不用于生成评价曲线。')
h('7.1 参考实现与适配范围',3)
table(['参考项目','许可证与最近推送','借鉴和限制'],[['libfranka','Apache-2.0；2026-09-07','力控示例假设起始已接触，采用平滑目标；依赖真实Franka FCI，不直接替换本仿真接口'],['ros2_controllers','Apache-2.0；2026-10-19','测力、运动学、位置内环分层，支持力滤波；当前Rolling接口不直接安装到已有Humble环境'],['FZI cartesian_controllers','BSD-3-Clause；2024-10-30','运动/力/柔顺组合与前向动力学设计；更新较早，仅作设计对照']])
link('https://frankarobotics.github.io/libfranka/0.16.1/force_control_8cpp-example.html','libfranka力控示例与接触初态约束')
link('https://control.ros.org/rolling/doc/ros2_controllers/admittance_controller/doc/userdoc.html','ROS2导纳控制器：运动学与滤波接口')
link('https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers','FZI笛卡尔控制器')
p('核对了许可证、仓库提交号和相关issue。ros2_controllers #2670涉及非激活状态参考被带入激活阶段，libfranka #241涉及真实网络切换超时；它们提醒切换初始化与硬件依赖需独立检查，不能拿来解释本地PhysX的波动。本轮只借鉴设计，通过现有Isaac SDK实现，不引入新的ROS2或硬件库。完整比较在OPEN_SOURCE_REVIEW.md，提交号及issue状态在open_source_review.json。')
h('7.2 单因素排查',3)
labels={'pid_contact_gate':'接触门控基线','stability_freeze':'冻结名义下压','stability_freeze_ema':'冻结+指数滤波','stability_freeze_last':'冻结+末次力采样','stability_freeze_5ms':'冻结+5ms控制','stability_freeze_20ms':'冻结+20ms控制','stability_ik_previous':'冻结+上次命令作逆解初值','stability_ik_tight':'上次初值+1μm容差（失稳）','stability_differential_moving':'微分逆解、继续名义下压','stability_differential_frozen':'微分逆解、冻结名义下压','stability_differential_ema':'微分逆解+冻结+指数滤波','stability_differential_5ms':'微分逆解+冻结+5ms控制'}
table(['条件','末2s力RMSE/N','最长带内跨度/s','达到0.5s标准'],[[label,num(S[n]['last2s_rmse_N']),num(S[n]['longest_band_span_s']),'是' if S[n]['strict_settle_time_s'] is not None else '否'] for n,label in labels.items() if n in S])
p('冻结名义下压本身没有消除波动；仅加滤波或缩短周期也未使原逆解链路通过标准。默认Lula逆解位置容差为1mm，实测关节状态不断变化会影响每轮求解结果。改用上次命令作初值也不是充分条件，直接收紧容差还可能切换到远处的另一组关节解。')
if 'stability_ik_tight' in S:
 p(f"1μm容差的保护前测试出现最大约{S['stability_ik_tight']['max_joint_reference_step_deg']:.1f}°的关节参考跳变，原始峰值力约{S['stability_ik_tight']['raw_peak_N']:.1f}N。该条件属于失败对照，不作为可用方案。当前代码增加有限值检查和每轮0.05rad参考跳变保护。")
guard=json.loads((R/'runs/stability_ik_guard/FAILED.json').read_text())
p(f"回归测试在{guard['t']:.2f}秒拦截了约{guard['max_jump_rad']*180/3.141592653589793:.1f}°的拟议跳变，未继续执行；该测试没有正常完成标记。它与此前无保护的失败记录分开保存。")
pic('stability_factors.png','冻结、滤波与控制周期对照；全部从1ms原始力计算相同10ms均值，不使用控制器指数滤波结果。')
h('7.3 连续微分逆运动学',3)
p('使用Isaac关节雅可比和Lula正运动学，按相邻目标的位姿误差做阻尼最小二乘增量更新，避免每轮重新选择一整组逆解。正则系数0.01，每轴每轮增量不超过0.003rad，并按模型关节限位裁剪。导纳保持M=1/B=40，关节内环仍是普通PID。')
code('pose_error = concatenate([target_position - FK_position(q_reference), rotation_error])\ndq = J.T @ solve(J @ J.T + 0.01**2 * I, pose_error)\nq_reference = clip(q_reference + clip(dq, -0.003, 0.003), lower, upper)')
p('该雅可比来自当前实测姿态，正运动学在上次参考处计算，是局部近似；不保证大范围路径或奇异位形下仍适用。它沿用成熟控制链的连续增量设计，通过当前SDK公开接口实现，没有复制外部项目源码。')
pic('stability_ik.png','原逆解、上次命令初值和连续微分逆解对照。')
chosen=S['stability_differential_ema']
p(f"选择10ms控制、接触后冻结名义位置、连续微分逆解和α=0.2指数反馈这一组合。末2秒均力{chosen['last2s_mean_N']:.3f}N，力RMSE为{chosen['last2s_rmse_N']:.3f}N，最长连续带内跨度{chosen['longest_band_span_s']:.2f}秒，达到原0.5秒标准。默认几何下末2秒{chosen['last2s_fraction_in_band']*100:.1f}%样本在带内，仍存在越界。")
h('7.4 重复运行与高度扰动',3)
names=[('stability_differential_ema','所选组合'),('stability_validation_repeat','相同条件重复'),('stability_validation_higher','平面升高1mm'),('stability_validation_lower','平面降低1mm')]
table(['条件','原始峰值/N','末2s力RMSE/N','带内占比','最长带内/s'],[[label,num(S[n]['raw_peak_N']),num(S[n]['last2s_rmse_N']),f"{S[n]['last2s_fraction_in_band']*100:.1f}%",num(S[n]['longest_band_span_s'])] for n,label in names])
pic('stability_validation.png','所选组合、同条件重复和平面高度±1mm；绿色为4.5—5.5N。')
p('这四组都出现了连续0.5秒的合格区间。同条件重复用于检查确定性，不算独立随机样本。平面降低1mm时，末2秒仍有约18%的周期均力越界，首次冲击也增大。因此当前结论是通过既定有限窗口标准，尚未实现整个接触阶段持续无越界，更不能称为工业现场恒力性能。')
p('相关代码在 scripts/analyze_stability.py、scripts/stability_*suite.sh 和 scripts/run.py。刚度、摩擦、重力、完整夹爪碰撞及真实力传感器噪声尚未系统测试。现阶段最重要的改进是参考连续性与输入链路联合验证，而不是仅凭一次参数调整宣称解决了所有波动。')
h('8. 结论与后续工作')
table(['下一步检查','当前尚未覆盖'],[['接触控制','重力、完整碰撞、接触刚度与摩擦变化，多初态重复测试'],['通信异常','乱序、重复、无效数值、时钟偏移、延迟抖动与传输带宽'],['模型接入','同一机器人和动作定义的真实策略；关节顺序、单位、绝对目标与增量的核对'],['实机','当前限幅作用于命令；实际跟随、负载与碰撞边界需要独立验证']])
p('普通PID轨迹、接触力读取和M=1/B=40导纳对照已实际完成。M=1/B=40改善了平均接触力和接触阶段的位置跟踪误差，但原始尖峰仍明显，原基线未达到严格稳定条件；新组合在默认场景和±1mm平面扰动中通过了连续0.5秒标准，仍存在其他时段越界。当前结果支持“导纳能限制持续压入力”，不支持“已经实现无冲击精确恒力”。')
p('下一步在接触控制中恢复重力与完整碰撞模型，并把真实策略动作接到已验证的时标、限幅和超时接口。当前仍是受控仿真原型，尚未达到生产部署条件。实体IMU高采样率、设备时间戳与Jetson测试属于另一组工作，不用本仿真结果替代。')
link('https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.api/docs/index.html', 'NVIDIA Isaac Sim 5.1接触力接口文档')
(R / 'REPORT.md').write_text('\n'.join(md))
style = '@page{size:A4;margin:18mm 16mm}body{font-family:"Noto Sans CJK SC",sans-serif;font-size:10.5pt;line-height:1.65;color:#202b38}h1{font-size:22pt}h2{font-size:15pt;break-after:avoid}table{width:100%;border-collapse:collapse;font-size:9pt}td,th{padding:6px;border:1px solid #ccd4df;text-align:left}th{background:#eef2f7}pre{font-size:8pt;white-space:pre-wrap;padding:12px;background:#f1f4f8}figure{margin:14px 0;break-inside:avoid}img{max-width:100%}figcaption{font-size:9pt;color:#526075}p{white-space:pre-line}'
(R / 'REPORT.html').write_text('<html lang="zh"><meta charset="utf-8"><title>Panda轨迹跟踪与柔顺接触实验</title><style>' + style + '</style>' + ''.join(doc) + '</html>')
subprocess.run(['/usr/bin/google-chrome', '--headless', '--no-sandbox', '--disable-gpu', '--no-pdf-header-footer', f'--print-to-pdf={R}/REPORT.pdf', (R / 'REPORT.html').as_uri()], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
from pypdf import PdfReader, PdfWriter
from pypdf.generic import NameObject, TextStringObject
reader=PdfReader(R/'REPORT.pdf');writer=PdfWriter();writer.clone_document_from_reader(reader)
for page in writer.pages:
 for reference in page.get('/Annots',[]):
  action=reference.get_object().get('/A')
  if action and str(action.get('/URI','')).startswith(R.as_uri()+'/'):
   action[NameObject('/URI')]=TextStringObject(str(action['/URI'])[len(R.as_uri())+1:])
with (R/'REPORT_portable.pdf').open('wb') as output:writer.write(output)
(R/'REPORT_portable.pdf').replace(R/'REPORT.pdf')
D = root / 'deliverables/20261025_字节_轨迹与柔顺控制'
D.mkdir(parents=True, exist_ok=True)
shutil.copy2(R / 'REPORT.pdf', D / 'Panda轨迹与柔顺控制报告.pdf')
shutil.copy2(R / 'REPORT.md', D / 'REPORT.md')
shutil.copy2(R/'REPORT.html',D/'REPORT.html')
shutil.copytree(R / 'figures', D / 'figures', dirs_exist_ok=True)
for name in ['position_final', 'admittance_final']:
    d = D / 'runs' / name
    d.mkdir(parents=True, exist_ok=True)
    shutil.copy2(R / 'runs' / name / 'simulation.mp4', d / 'simulation.mp4')
for name in result:
    d=D/'runs'/name
    d.mkdir(parents=True,exist_ok=True)
    for filename in ['metadata.json','time_audit.json','trace.csv','physics_force.csv','joint_tracking.csv','scene.usda']:
        shutil.copy2(R/'runs'/name/filename,d/filename)
for filename in ['results.json','verification.json','execution_results.json','stream_sweep.json','stream_replay_verification.json','stability_results.json','stability_verification.json','study_summary.json','open_source_review.json']:
    shutil.copy2(R/filename,D/filename)
for d in (R/'runs').glob('execution_*'):
    if d.is_dir() and (d/'COMPLETE').exists():
        target=D/'runs'/d.name;target.mkdir(parents=True,exist_ok=True)
        for filename in ['execution.csv','metadata.json','stream_summary.json','time_audit.json']:
            shutil.copy2(d/filename,target/filename)
for name in S:
    if name in result:continue
    target=D/'runs'/name;target.mkdir(parents=True,exist_ok=True)
    for filename in ['metadata.json','trace.csv','physics_force.csv','time_audit.json']:
        shutil.copy2(R/'runs'/name/filename,target/filename)
for item in partial:
    target=D/'runs'/item.parent.name;target.mkdir(parents=True,exist_ok=True)
    shutil.copy2(item,target/'FAILED.json')
with zipfile.ZipFile(D / 'Panda轨迹与柔顺控制源码.zip', 'w', zipfile.ZIP_DEFLATED) as z:
    for path in sorted((R / 'scripts').glob('*')):
        if path.is_file():
            z.write(path, 'panda_contact_isaac/scripts/' + path.name)
    z.write(R / 'README.md', 'panda_contact_isaac/README.md')
    z.write(R/'REPORT.md','panda_contact_isaac/REPORT.md')
    for image in (R/'figures').glob('*.png'):z.write(image,'panda_contact_isaac/figures/'+image.name)
    z.write(R/'requirements-analysis.txt','panda_contact_isaac/requirements-analysis.txt')
    z.write(R/'OPEN_SOURCE_REVIEW.md','panda_contact_isaac/OPEN_SOURCE_REVIEW.md')
    z.write(R/'open_source_review.json','panda_contact_isaac/open_source_review.json')
print(D)
