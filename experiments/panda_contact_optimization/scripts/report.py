"""Build a self-contained Chinese report and a source-only submission archive."""

from pathlib import Path
import json, html, subprocess, zipfile, hashlib, shutil

R = Path(__file__).resolve().parents[1]
ROOT = R.parents[1]
D = ROOT / "deliverables/20261112_字节_接触控制优化"
D.mkdir(parents=True, exist_ok=True)
A = json.loads((R / "results.json").read_text())
V = json.loads((R / "verification.json").read_text())
selected = (
    json.loads((R / "selection.json").read_text())["selected"]
    if (R / "selection.json").exists()
    else "validation_repeat"
)
md = []
doc = []


def h(t, n=2):
    md.extend(["", "#" * n + " " + t, ""])
    doc.append(f"<h{n}>{html.escape(t)}</h{n}>")


def p(t):
    md.extend([t, ""])
    doc.append("<p>" + html.escape(t) + "</p>")


def table(head, rows):
    md.extend(
        ["| " + " | ".join(head) + " |", "|" + "|".join(["---"] * len(head)) + "|"]
        + ["| " + " | ".join(map(str, r)) + " |" for r in rows]
        + [""]
    )
    doc.append(
        "<table><thead><tr>"
        + "".join("<th>" + html.escape(c) + "</th>" for c in head)
        + "</tr></thead><tbody>"
        + "".join(
            "<tr>"
            + "".join("<td>" + html.escape(str(c)) + "</td>" for c in row)
            + "</tr>"
            for row in rows
        )
        + "</tbody></table>"
    )


def code(t):
    md.extend(["```text", t, "```", ""])
    doc.append("<pre>" + html.escape(t) + "</pre>")


def pic(n, c):
    if not (R / "figures" / n).exists():
        return
    md.extend([f"![{c}](figures/{n})", ""])
    doc.append(
        f'<figure><img src="figures/{n}"><figcaption>{html.escape(c)}</figcaption></figure>'
    )


LABELS = {
    "no_gravity_probe": "无重力·探针",
    "gravity_uncompensated": "带重力·无补偿（失败）",
    "gravity_probe": "带重力补偿·探针",
    "gravity_gripper": "带重力补偿·夹爪",
    "gripper_ramp": "夹爪＋目标力缓升",
    "gripper_brake": "接近减速初测",
    "gripper_soft": "柔顺材质10kN/m",
    "validation_repeat": "统一高度算法复测",
    "validation_higher": "默认方案·平面+1mm",
    "validation_lower": "默认方案·平面−1mm",
    "validation_higher5mm": "默认方案·平面+5mm",
    "validation_lower5mm": "默认方案·平面−5mm",
    "validation_friction_low": "默认方案·摩擦0.1",
    "validation_friction_high": "默认方案·摩擦1.0",
    "validation_soft_2k": "柔顺材质2kN/m",
    "validation_soft_50k": "柔顺材质50kN/m",
    "validation_solver4": "4次位置求解迭代",
    "validation_mass_matrix": "质量矩阵补偿内环",
    "validation_5ms": "5ms控制周期",
    "validation_20ms": "20ms控制周期",
    "refine_velocity": "修正速度3mm/s",
    "refine_braking": "提前15mm减速",
    "refine_combined": "减速＋修正速度3mm/s",
    "refine_damping100": "附加法向阻尼100",
    "refine_damping300": "附加法向阻尼300",
    "refine_damping100_slow": "阻尼100＋修正速度1mm/s",
    "candidate_velocity3": "统一候选·修正3mm/s",
    "candidate_velocity30": "统一候选·修正30mm/s",
    "final_default": "统一候选·修正1mm/s",
    "final_selected": "最终方案",
    "final_higher": "最终·平面+1mm",
    "final_lower": "最终·平面−1mm",
    "final_higher5mm": "最终·平面+5mm",
    "final_lower5mm": "最终·平面−5mm",
    "final_friction_low": "最终·摩擦0.1",
    "final_friction_high": "最终·摩擦1.0",
    "final_repeat": "最终·重复",
    "final_half_step": "最终·物理步长0.5ms",
    "video_admittance": "录屏·导纳",
    "video_position": "录屏·位置控制",
}

LABELS.update(
    {
        "final_soft": "最终·柔顺50kN/m",
        "final_soft_higher5mm": "柔顺·平面+5mm",
        "final_soft_lower5mm": "柔顺·平面−5mm",
    }
)


LABELS["final_soft_half_step"] = "柔顺·物理步长0.5ms"


def rows(names):
    return [
        [
            LABELS.get(n, n),
            f"{A[n]['raw_peak_N']:.2f}",
            f"{A[n]['last2_mean_N']:.3f}",
            f"{A[n]['last2_rmse_N']:.3f}",
            f"{A[n]['last2_in_band_pct']:.1f}%",
            f"{A[n]['last2_raw_in_band_pct']:.1f}%",
            "通过" if A[n]["all_targets_met"] else "未全部通过",
        ]
        for n in names
        if n in A
    ]


head = [
    "条件",
    "原始峰值/N",
    "末段均力/N",
    "力RMSE/N",
    "周期带内率",
    "原始带内率",
    "四项检查",
]
s = A[selected]
b = A["gravity_gripper"]
count = len(A)
duration = sum(x["duration_s"] for x in A.values())
h("Panda 带重力轨迹跟踪与接触控制优化", 1)
p("2026年11月12日｜PC 仿真｜Isaac Sim 5.1 / RTX 4090")
h("摘要")
p(
    f"针对前一阶段重力关闭、夹爪碰撞简化和接触冲击偏大的问题，本轮完成{count}组全长仿真，总时长{duration:.0f}秒。保留M=1、B=40的导纳参数，在普通关节PID上加入重力补偿，恢复手部与双指碰撞及接触材质，并逐项验证目标力缓升、接近减速和修正速度限制。"
)
p(
    f'本轮选用条件为 {LABELS.get(selected,selected)}：1ms原始力峰值{s["raw_peak_N"]:.2f}N，末尾两秒平均力{s["last2_mean_N"]:.3f}N，固定10ms均力RMSE为{s["last2_rmse_N"]:.3f}N，周期带内率{s["last2_in_band_pct"]:.1f}%。原始峰值和周期均力分开评价，不用反馈滤波后的曲线代替测量。'
)
p(
    "本轮仍是固定基座、固定平面接触实验，没有实体机器人、IMU或视觉模型参与控制。末段周期均力合格并不等于所有1ms瞬时力都在目标范围内。对材质、几何和内环参数不同的条件分别解释，不把它们合并为同一算法的改善百分比。"
)
h("目录")
p(
    "1. 反馈对应的改动\n2. 控制与接触模型\n3. 重力及几何对照\n4. 接近与力切换优化\n5. 环境与周期扰动\n6. 结果边界和失败分析\n7. 复现及代码目录\n8. 后续工作"
)
p(
    "四项工程检查分别为原始峰值≤15N、末段周期带内率≥99%、力RMSE≤0.25N和前三轴各自轨迹RMSE≤0.1°。原始采样带内率额外单列，不用周期均值代替瞬时表现。"
)
h("1. 反馈对应的改动")
table(
    ["上次问题", "本轮处理", "评价方式"],
    [
        [
            "无重力工况",
            "启用重力；关节PID增加PhysX重力补偿向量；保留无补偿失败对照",
            "前三轴5s误差、开始阶段接触情况",
        ],
        [
            "只有球形探针",
            "恢复panda_hand与左右手指碰撞；求和三刚体对平面的接触力",
            "核对场景、接触体和合力",
        ],
        [
            "接触瞬间峰值偏大",
            "接近减速、接触门控、冻结名义位置、平滑力目标、修正速度限制",
            "1ms原始峰值、相对首次接触曲线",
        ],
        [
            "对高度变化敏感",
            "实际平面±1/±5mm扰动，减速器仍使用名义高度",
            "同口径峰值和末段带内率",
        ],
        [
            "材质过于理想",
            "刚性与等效柔顺材质分组；摩擦系数变化",
            "不把软材质结果当作刚性控制提升",
        ],
        [
            "只满足短窗口",
            "同时统计末2s均力带内率、原始力带内率和RMSE",
            "保留连续0.5s指标，不再只靠它下结论",
        ],
    ],
)
h("2. 控制与接触模型")
p(
    "前5秒仍完成前三轴五次轨迹，增量[0.06,0.04,-0.04]rad。普通PID增益沿用上阶段配置，力矩限幅前四轴87Nm、后三轴12Nm，夹爪5N。主要条件控制周期10ms、物理1ms；后续单独改变控制周期，另有物理步长0.5ms的数值检查，评价仍按10ms分箱。"
)
code(
    "tau = Kp*(q_ref-q) - Kd*dq + Ki*integral + g(q)\nM_adm * x_ddot + B_adm * x_dot = F_ref - F_feedback\nz_ref = z_frozen - x\nF_ref = 5 * (3*u**2 - 2*u**3),  u=clip((t-t_contact)/1s,0,1)"
)
p(
    "重力补偿使用本地Isaac 5.1关节接口返回的PhysX补偿向量，正号加入输出力矩。新增日志保存各关节补偿力矩，并检查调用前后关节速度未被改变。没有将真实Franka接口的隐含补偿假设直接套用进仿真。"
)
p(
    "接触发生后以实测手部高度冻结名义位置，沿法向叠加导纳位移。微分逆解正则化系数0.01，每次每轴参考增量限制0.003rad，另设总逆解跳变保护。三个接触刚体共用平面过滤器，力取世界Z分量之和。夹爪保持张开，不执行夹物动作；其他连杆碰撞尚未恢复。"
)
p(
    "减速器利用已知场景的名义平面高度和初始碰撞体边界估算间隙，接近速度随间隙降低，保留0.3mm/s下限。高度扰动不传给减速器。由于手指姿态会改变最低点，这个估计不是精确距离传感器，测试结果必须包含几何先验误差的影响。"
)
p(
    "初始对照采用16次位置求解迭代、4次速度迭代；最终统一方案改用4次位置迭代，并在相同设置下做扰动验证。迭代次数的影响单列讨论，不将这一变化归因于控制算法。球形探针与原装夹爪的接触几何不同，因此不将两组力误差的差值单独归因于算法。柔顺接触参数为等效刚度和阻尼，未经真实材料标定。"
)
p(
    "相关代码在 scripts/run.py；实验入口为 scripts/suite.py、scripts/validation_suite.py 和 scripts/refine_suite.py。开源设计对照见 OPEN_SOURCE_REVIEW.md，主要参考Franka官方力控制示例和NVIDIA稳定性、接触材质文档；未增加ROS控制器依赖。"
)
h("3. 重力及几何对照")
table(
    head,
    rows(
        [
            "no_gravity_probe",
            "gravity_uncompensated",
            "gravity_probe",
            "gravity_gripper",
        ]
    ),
)
table(
    ["条件", "前三轴5s RMSE/度", "首次周期接触/s"],
    [
        [
            LABELS.get(n, n),
            " / ".join(f"{v:.4f}" for v in A[n]["first5_joint_rmse_deg"]),
            (
                f"{A[n]['first_contact_s']:.2f}"
                if A[n]["first_contact_s"] is not None
                else "未接触"
            ),
        ]
        for n in [
            "no_gravity_probe",
            "gravity_uncompensated",
            "gravity_probe",
            "gravity_gripper",
        ]
        if n in A
    ],
)
p(
    "无补偿条件在初始轨迹阶段即下坠并碰到平面，属于控制失败，不能因为最后均力接近5N就判定可用。该条件的巨大峰值仅用于说明重力补偿缺失的后果，不纳入正常接触方案的改善百分比。补偿条件的前三轴误差保持在较小范围。"
)
pic("gravity.png", "相同探针条件下，关闭重力与开启重力并补偿的对照。")
h("4. 接近与力切换优化")
table(
    head,
    rows(
        [
            "gravity_gripper",
            "gripper_ramp",
            "gripper_brake",
            "validation_repeat",
            "refine_velocity",
            "refine_braking",
            "refine_combined",
            "refine_damping100",
            "refine_damping300",
            "refine_damping100_slow",
            "candidate_velocity3",
            "candidate_velocity30",
            "final_selected",
        ]
    ),
)
p(
    "首次接触之前没有可用接触力，单靠接触门控无法预防碰撞。目标力缓升避免切换瞬间直接要求5N，提前减速则降低接触前速度，两者作用阶段不同。后续限制导纳修正速度和扩大减速范围，检查能否减少切换后的再次撞击。"
)
p(
    "初测gripper_brake峰值较低，但名义平面高度计算改为不读取扰动值、增加运行检查后，默认条件峰值重新升高。两个条件并非逐位相同的复现，不能把初测较好数值当作可重复性能；保留两组结果以展示刚性接触对微小差异的敏感性。最终验证以统一代码下的新条件为准。"
)
pic("approach.png", "相同刚性夹爪场景下的接近与目标力切换对照。")
pic("refinement.png", "修正速度与提前减速范围的后续对照。")
h("5. 环境与周期扰动")
names = [n for n in A if n.startswith("validation_") or n.startswith("final_")]
table(head, rows(names))
p(
    "高度扰动相对同一几何条件的名义平面施加，不给控制器额外告知实际偏差。摩擦变化只检验当前法向接触工况，不能证明切向擦拭或滑动稳定性。质量矩阵补偿使用加速度形式增益，与普通PID的单位和数值不同；这一组仅作为备选内环，不直接归因补偿方法优劣。5/20ms结果为周期敏感性测试，主实验仍以10ms为准。"
)
table(
    head,
    rows(
        ["gripper_brake", "gripper_soft", "validation_soft_2k", "validation_soft_50k"]
    ),
)
p(
    "柔顺材质的峰值与末段波动较小，说明接触物理模型会明显影响结果。软化环境不等于刚性控制问题已解决，最终刚性与柔顺条件分别给出。"
)
pic("material.png", "刚性与柔顺材质对照；此处改变环境，不计算跨材质算法改善比例。")
pic("validation.png", "默认减速方案的高度与摩擦扰动。")
pic("timestep.png", "物理步长减半的数值检查；原始峰值与周期均力分别解释。")
pic("final_validation.png", "统一修正方案的独立条件验证。")

if "final_soft" in A:
    table(head, rows(["final_soft", "final_soft_higher5mm", "final_soft_lower5mm"]))
    p(
        "在最终控制器保持不变的情况下另测50kN/m、150N·s/m的等效柔顺接触，表示可能通过末端薄缓冲层改变接触物理。该结果需要真实材料与接触面积标定后才能迁移，不能作为刚性环境问题已经解决的依据。"
    )


if "final_half_step" in A:
    h("物理步长与冲量检查", 3)
    table(
        ["条件", "物理步长/ms", "原始峰值/N", "最大单步冲量/N·s", "10ms均力峰值/N"],
        [
            [
                LABELS.get(n, n),
                f"{A[n]['time_audit']['expected_duration']/V[n]['physics_rows']*1000:.1f}",
                f"{A[n]['raw_peak_N']:.2f}",
                f"{A[n]['max_single_step_impulse_Ns']:.5f}",
                f"{A[n]['cycle_peak_N']:.3f}",
            ]
            for n in [
                "final_selected",
                "final_half_step",
                "final_soft",
                "final_soft_half_step",
            ]
            if n in A
        ],
    )
    p(
        "刚性条件把物理步长从1ms减到0.5ms后，原始峰值从11.11N变成20.43N，末段原始带内率也下降，说明瞬时峰值尚未数值收敛。单步接触力来自求解器冲量除以步长，因而还需比较冲量和固定分箱均力，不能把离散尖峰直接解释为真实传感器的力上界。改变步长也会改变启动预步进的微小初始误差，这一组属于数值敏感性检查，不是严格的连续系统收敛证明。"
    )
    if "final_soft_half_step" in A:
        p(
            f"有限刚度条件下，1ms与0.5ms的原始峰值分别为{A['final_soft']['raw_peak_N']:.2f}N、{A['final_soft_half_step']['raw_peak_N']:.2f}N，说明在本轮范围内对步长更不敏感。真实迁移仍需要标定接触刚度、阻尼、接触面积和测力带宽。"
        )

h("6. 结果边界和失败分析")
table(
    ["选用条件的指标", "结果"],
    [
        ["条件", selected],
        ["1ms原始力峰值", f"{s['raw_peak_N']:.3f} N"],
        ["末2s周期均力带内率", f"{s['last2_in_band_pct']:.1f}%"],
        ["末2s原始力带内率", f"{s['last2_raw_in_band_pct']:.1f}%"],
        ["末2s原始峰值", f"{s['last2_raw_peak_N']:.3f} N"],
        ["最长连续周期带内跨度", f"{s['longest_in_band_s']:.2f} s"],
    ],
)
p(
    "本轮工程检查设为：原始峰值≤15N、末2s周期带内率≥99%、均力RMSE≤0.25N、前三轴各自5s RMSE≤0.1°。这些是本项目用于比较的目标，不是工业标准或设备认证。表中的通过表示四项同时满足，完成标记只表示仿真正常结束。"
)
p(
    "接触后的短促原始力脉冲仍可能超过5±0.5N。平面高度、摩擦、求解器和接触几何变化都可能放大这些脉冲；没有将刚性求解器的接触脉冲等同于真实六维力传感器输出。尤其低摩擦条件下，即使周期均力合格，原始力仍可能频繁越界。更长时间、多姿态、多随机扰动及远程网络和实体试验仍需要补充。"
)
p(
    "新增记录区分了原始合力、周期均力、反馈EMA和目标力；因此可以检查控制器是否因为滤波延迟而反复接触。重复运行用于检查确定性和实现差异，不用于宣称统计显著性。"
)

if (R / "network_results.json").exists():
    N = json.loads((R / "network_results.json").read_text())
    h("补充：真实本机 UDP 跨进程通信", 3)
    table(
        ["工况", "收到包数", "收包轮询延迟P95/ms", "生成到读取P95/ms", "保持采样数"],
        [
            [
                {"normal": "正常", "delay": "发送端延迟40ms", "outage": "发送端停发1s"}[
                    k
                ],
                v["packet_count"],
                f"{v['receive_poll_latency_p95_ms']:.2f}",
                f"{v['generation_to_receive_p95_ms']:.2f}",
                v["hold_samples"],
            ]
            for k, v in N.items()
        ],
    )
    p(
        "发送与接收运行在独立进程，实际通过127.0.0.1 UDP传递未来动作块；发送20Hz、接收目标100Hz。延迟40ms和停发1s均由发送应用注入，动作来自解析五次轨迹，不运行VLA，也不驱动物理仿真或实体机器人。接收端按时间戳取动作，数据年龄超过0.5s时保持输出。"
    )
    p(
        "收包延迟包含100Hz轮询和操作系统调度，不能当作纯网络传输耗时或RTT。同机共享单调时钟；跨主机时钟同步、重排、丢包恢复和硬实时尚未验证。测试在本轮仿真负载下完成，仅证明独立进程之间的实际通信和超时逻辑。相关代码在 scripts/network.py。"
    )

h("7. 复现及代码目录")
code(
    "panda_contact_optimization/\n  README.md\n  PROTOCOL.json\n  OPEN_SOURCE_REVIEW.md\n  scripts/\n    isaac-python\n    run.py\n    initial_run.py         初始七组对照的实现\n    suite.py\n    validation_suite.py\n    refine_suite.py\n    final_suite.py           最终方案与录屏\n    analyze.py\n    verify.py\n    figures.py\n    video.py                同步仿真画面与测量曲线\n    network.py              本机UDP跨进程接口\n    test_metrics.py         固定分箱与脉冲保留检查\n    report.py\n  figures/\n  results.json\n  verification.json\n  selection.json\n  REPORT.md\n  runs/<条件>/              本地生成，提交包不含原始CSV和视频"
)
code(
    "experiments/panda_contact_optimization/scripts/isaac-python \\\n  experiments/panda_contact_optimization/scripts/run.py \\\n  --inner pid --gravity --gravity-compensation --geometry gripper \\\n  --contact-gate --freeze-on-contact --ik-seed differential \\\n  --force-feedback ema --force-ramp 1 --approach-profile brake \\\n  --brake-distance .015 --minimum-approach-speed .001 \\\n  --force-velocity-limit .003 --solver-iterations 4 --duration 32 \\\n  --out experiments/panda_contact_optimization/runs/new_run"
)
p(
    f"原始数据保存在 experiments/panda_contact_optimization/runs。已核对{len(V)}组运行的采样、仿真时钟、有限值与逆解步长。源码包包含本轮代码、实验参数、指标JSON、报告MD和配图，不包含软件环境、模型权重、官方机器人资产或视频。已有实验目录及旧报告保持原样。最终方案的额外命令参数见 selection.json 和 scripts/final_suite.py。"
)
pic("comparison_keyframe.png", "新录制对照的画面与同步力曲线，数值均来自对应运行日志。")
h("8. 后续工作")
p(
    "后续先扩大接触姿态和未知高度范围，再测试斜面、切向运动、失去接触后恢复以及长时间保持。若接入学习策略，接触控制和轨迹执行接口应保持独立，先用已知动作验证时标、限幅与超时，再评估模型；本轮没有把解析轨迹解释为VLA输出。远程设备网络和实体闭环尚未完成，本轮结果也不依赖IMU硬件。"
)
summary = {
    "full_runs": len(A),
    "simulated_duration_s": sum(v["duration_s"] for v in A.values()),
    "physics_samples": sum(v["physics_rows"] for v in V.values()),
    "udp_cases": 3,
    "hardware_runs": 0,
}
(R / "study_summary.json").write_text(json.dumps(summary, indent=2))
text = "\n".join(md)
(R / "REPORT.md").write_text(text)
css = """@page{size:A4;margin:18mm 16mm}body{font:10pt "Noto Sans CJK SC","Noto Sans CJK JP",sans-serif;color:#243142;line-height:1.65}h1{font-size:22pt;color:#193b5b}h2{font-size:15pt;margin-top:22px;break-after:avoid}p{white-space:pre-line}table{border-collapse:collapse;width:100%;font-size:8pt;margin:12px 0}th,td{border:1px solid #ccd6e0;padding:5px;overflow-wrap:anywhere}th{background:#e9eff6}tr{break-inside:avoid}figure{margin:14px 0;break-inside:avoid}img{width:100%}figcaption{font-size:8pt;color:#516175}pre{font:8pt monospace;white-space:pre-wrap;overflow-wrap:anywhere;background:#f0f3f7;padding:10px}"""
(R / "REPORT.html").write_text(
    '<!doctype html><html lang="zh"><meta charset="utf-8"><style>'
    + css
    + "</style><body>"
    + "".join(doc)
    + "</body></html>"
)
subprocess.run(
    [
        "google-chrome",
        "--headless",
        "--no-sandbox",
        "--disable-gpu",
        "--no-pdf-header-footer",
        f'--print-to-pdf={R / "REPORT.pdf"}',
        (R / "REPORT.html").as_uri(),
    ],
    check=True,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
)
for n in [
    "REPORT.md",
    "REPORT.html",
    "REPORT.pdf",
    "results.json",
    "verification.json",
    "PROTOCOL.json",
    "selection.json",
]:
    if (R / n).exists():
        shutil.copy2(R / n, D / n)
shutil.copytree(R / "figures", D / "figures", dirs_exist_ok=True)
shutil.copy2(R / "REPORT.pdf", D / "Panda带重力接触控制优化报告.pdf")
if (R / "comparison.mp4").exists():
    shutil.copy2(R / "comparison.mp4", D / "Panda接触控制同步对比.mp4")
zip_path = D / "Panda带重力接触控制优化源码.zip"
paths = [
    p
    for p in (R / "scripts").iterdir()
    if p.is_file()
    and p.suffix in [".py", ".sh"]
    or p.is_file()
    and p.name == "isaac-python"
]
paths += list((R / "figures").glob("*.png"))
paths += [
    R / n
    for n in [
        "README.md",
        "PROTOCOL.json",
        "OPEN_SOURCE_REVIEW.md",
        "results.json",
        "verification.json",
        "selection.json",
        "network_results.json",
        "pair_checks.json",
        "study_summary.json",
        "source_hash_corrections.json",
        "REPORT.md",
        "requirements-analysis.txt",
    ]
    if (R / n).exists()
]
with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
    for path in sorted(paths):
        z.write(path, Path("panda_contact_optimization") / path.relative_to(R))
manifest = {
    p.name: hashlib.sha256(p.read_bytes()).hexdigest()
    for p in D.iterdir()
    if p.is_file() and p.name != "文件清单.json"
}
(D / "文件清单.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
print("Report and source archive:", D)
