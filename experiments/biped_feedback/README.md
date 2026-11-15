# 双足连续行走与ZMP力矩闭环

在既有lipm_extension平面双足上，把CoM加速度与ZMP接触力矩目标接入同一个逆动力学QP，加入C2双支撑负载过渡、接触核对、力矩变化率限制，以及带采样时钟的水平CoM延迟反馈。原工程和结果不覆盖。

本轮94个唯一条件，79个通过；连续20步的两组测试均完成。压力中心跟踪改善，但部分扰动下质心误差或恢复时间变差；80ms延迟下多数条件未通过。完整结果见REPORT.md，原始条件逐项见metrics.csv，不能把条件网格通过比例当作真实部署成功概率。

## 独立环境

```bash
cd biped_feedback
python3.10 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python scripts/tests.py
.venv/bin/python scripts/suite.py
.venv/bin/python scripts/analyze.py
.venv/bin/python scripts/verify.py
```

核心测试与物理仿真使用CPU，不需要GPU模型或机器人。Python3.11也经过解压后测试。源码包包含模型、代码、配置与结果汇总；原始轨迹由suite.py重新生成。未运行完整矩阵前不要直接运行verify.py；报告中的验收结果是已有运行记录，不能代替重新验证。

图表与录屏另需：

```bash
.venv/bin/python -m pip install -r requirements-visual.txt
.venv/bin/python scripts/figures.py
.venv/bin/python scripts/record.py
```

录屏需要OpenGL/EGL，字体使用Noto Sans CJK。record.py比较录制与正式实验轨迹，视频逐帧解码核对。录制重跑不增加实验数量。PDF导出另需Chrome和markdown-it-py；已有PDF可直接阅读，无需安装导出工具。

工作区可复用experiments/biped_feedback/scripts/python启动器；迁移后可直接用新环境Python运行，或通过LIPM_PYTHON指定解释器。

## 目录

- scripts/biped.py：计划、六电机全身QP、物理执行；只在初始化设置qpos，之后只发力矩。
- scripts/feedback.py：带源时间戳的CoM队列、常速度预测、实际接触扳手及压力中心。
- scripts/suite.py：冻结条件和源码快照；结果按源码、完整配置分别编号，已有结果核对版本后复用。
- scripts/analyze.py、verify.py、tests.py：指标、独立轨迹检查、7项接口测试。
- scripts/figures.py、record.py、report.py：图表、实际仿真录屏与报告/打包。
- models/biped.xml：9自由度平面模型，3个无驱动基座自由度和6个电机。
- research/sources.json、RESEARCH.md：参考项目、许可、维护状态与取舍。
- manifest.json、results.json、summary.json、metrics.csv：所有条件与结果，包括失败。
- runs/<源码指纹>/<配置编号>/：本地原始trace.npz和audit.npz；ZIP不包含这些大数组。

## 输出字段与边界

trace.npz的data列为：t[0]，CoM x/z[1:3]，基座pitch[3]，两脚x/z[4:8]，实际法向力[8:10]，施加力矩[10:16]，QP动力学残差[16]，qpos[17:26]，qvel[26:35]，目标足位[35:39]，预测足底扳手[39:45]。

audit.npz按控制时刻保存具名字段：当前和观测CoM、目标加速度、原始/限幅ZMP、实际/预测扳手、qpos/qvel、读取接触力时的旧ctrl、支撑集合、负载比例、采样年龄和本次新力矩。实际CoP是本次新力矩生效前的物理测量；QP预测CoP是新指令对应的预测，二者不能当作同一个数。

支撑切换仍有名义时间表，但会检查应承重的脚是否实际产生法向力，超过40ms仍未接触则终止；没有实现接触触发的在线步时重规划。延迟只加在水平CoM外环，关节编码器、模型动力学和接触测量仍理想；不是全身所有传感器都有同样延迟。模型侧向、滚转、偏航受约束，没有开展三维行走或硬件测试。
