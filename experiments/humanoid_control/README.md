# G1三维站立、单脚支撑与准静态步行

三维浮动基座（六自由度未锁定）、29个关节力矩执行器。使用固定版本的MuJoCo Menagerie G1模型，保留原始质量、惯量、关节限位和碰撞几何。原位置执行器转换为受限motor；不存在基座电机、mocap轨迹或外部支撑力。

当前是慢速准静态步行：每步约7秒，主要验证横向重心转移、真实单脚承重、交替离地和落脚。不是动态高速行走，也不是已部署到硬件的控制器。

## 独立运行

```bash
cd humanoid_control
python3.11 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python scripts/tests.py
.venv/bin/python scripts/suite.py
.venv/bin/python scripts/analyze.py
.venv/bin/python scripts/verify.py
.venv/bin/python scripts/stress.py
.venv/bin/python -c "import sys; sys.path.insert(0,'scripts'); import verify; verify.main('stress_manifest.json','stress_verification.json')"
```

核心测试和仿真使用CPU，不下载模型权重，不需要机器人或IMU。模型网格、许可证与原始描述随源码包提供，不依赖本地其他工程目录。tests.py为9项自包含检查，不需要实验原始数组。

suite.py会重建派生模型，冻结代码和模型版本，运行去重的52个条件；结果按源码指纹和配置分别保存。stress.py另运行8个强扰动条件，控制器不再调整。verify.py需要上述命令生成的原始数组，不能在刚解压且尚未仿真时直接验证历史成绩。已有结果目录不会被静默覆盖；中断的running状态也不当成完成。

工作区复用.venvs/humanoid-control，原lipm等环境不改；scripts/python为工作区启动器。迁移后直接使用新环境Python运行，或用HUMANOID_PYTHON指定解释器。

## 可选图表与视频

```bash
.venv/bin/python -m pip install -r requirements-visual.txt
.venv/bin/python scripts/figures.py
.venv/bin/python scripts/record.py
.venv/bin/python scripts/report.py
```

图表依赖已经完成的正式/强扰动运行。录屏需要OpenGL/EGL和Noto Sans CJK字体；report.py导出PDF另外需要Chrome。现有交付直接提供PDF和MP4，无需重新导出。

record.py真正重跑物理控制并从两个相机录制，将qpos、qvel、ctrl、CoM、足位和接触力与正式回合逐项比较；录制不增加独立实验数量。

## 代码与输出

- model.py：派生力矩模型及初始站姿，仅在初始化设置qpos。
- controller.py：35维广义加速度、29维力矩、每支撑足6维扳手的全身QP。
- qp.py：同一H/g/A/l/u对应Clarabel或OSQP，不改控制目标。
- run.py：连续参考、实际物理执行、按物理量验收；支撑集合按名义时间表。
- metrics.py：实际承重、离地、落脚与末态判据，不以“不倒”代替完成。
- suite.py / stress.py：固定条件与去重；无推力时忽略推扰时间，整数与浮点同值也去重。
- verify.py：独立检查四元数积分、正运动学、接触力、执行器与保存轨迹。
- backend_probe.py：同一记录状态下生成完全相同的QP，比较两个后端，不执行动作。
- storage.py：原子状态与每秒轨迹分块；突发退出后保留最后完整块，不支持模拟器断点恢复。
- tests.py：自由基座、力矩通道、原机械参数、空间雅可比、接触虚功、参考连续性、承重判据和后端转换检查。

runs/<源码指纹>/<配置编号>/内包含result.json、trace.npz、controller.npz、chunks/。trace按1ms保存具名字段，controller按控制时刻记录目标、残差、迭代与实际调用耗时。contact_ctrl/contact_external为采集接触力时的上一步输入；ctrl为本次新下发力矩，二者不能混用。

源码ZIP包含代码、完整模型资源、配置、结果汇总、报告源文和关键图，不包含虚拟环境或大规模原始数组；重跑后可生成这些数组。上游模型许可和哈希位于third_party/menagerie/unitree_g1/LICENSE与research/model_source.json。

## 范围

状态直接取自模拟器，不是IMU状态估计。接触扳手通过MuJoCo独立求解，QP预测力不直接施加；实际载荷用于验收，不参与触地事件改期。当前未实现动态步态、地形、自碰撞规避、真实电机带宽或硬件测试。原始模型保留自碰撞，强扰动中可以触发，不能假定只有足地接触。
