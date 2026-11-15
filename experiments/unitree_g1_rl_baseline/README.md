# Unitree G1 RL Sim2Sim 基线

官方 `unitree_rl_gym` 预训策略的 MuJoCo Sim2Sim 复现（BSD-3），与自编准静态 WBC **并列**，不是互相否定。

## 两条路线

| 路线 | 已完成 | 定位 |
|---|---|---|
| 自编 WBC | 全身动力学约束、力矩控制、单脚承重、慢速交替迈步与失败分析 | 准静态控制原型（约 7 s/phase 是设计选择；缺动态参考/接触事件/状态估计） |
| 官方 RL（本目录） | 预训 `motion.pt` Sim2Sim + 指令矩阵 | 动态行走策略复现；**12-DoF**，与 29-DoF WBC 不可直接消融 |

## 矩阵 v1（启发式未倒）

详见 `results/matrix_v1/MATRIX.md`。要点：

- `fwd_0p5`：前进 **9.23 m**，横向漂移 **-1.41 m**（直线跟踪有明显偏差，不能只报前进）
- 漂移随前进速度增大（0.3→0.8 m/s 时 lat 约 -0.91 → -2.62 m）
- 指标字段为 **浮动基座** 位姿/速度，不是全身质心
- 跌倒判据：基座高度 &lt; 0.4 m **或** |roll|/|pitch| &gt; 45°（仍非完整稳定性验收）

## 复现

```bash
cd /home/yd/Documents/VLA/experiments/unitree_g1_rl_baseline
MUJOCO_GL=egl /home/yd/Documents/VLA/.venvs/openpi-libero/bin/python scripts/run_baseline_matrix.py
```

单次：`scripts/run_g1_mujoco_headless.py --cmd 0.5 0 0 --duration 20 --out results/...`

包装相对官方 deploy 的差异见 `SOURCE.md`（计时、终止、指标；观测/PD/策略循环沿用官方）。

## 接触与停止复核（2026-11-15）

新增独立验证见 [阶段报告](studies/command_validation_v1/REPORT.md) 与 [PDF](studies/command_validation_v1/REPORT.pdf)。12种固定工况共245秒仿真，官方权重与部署控制不变。包含实际脚底受力、接触点速度、全身质心和分块状态记录；13段视频已全帧解码核对。

优先观看：

- [WBC与官方RL同播放时长对照](studies/command_validation_v1/wbc_vs_official_rl_first20s.mp4)：模型与任务不同，不作为公平算法排名。
- [前进0.5米/秒](studies/command_validation_v1/cases/forward_05/rollout_20fps.mp4)：20秒前进9.23米，同时横移−1.41米、航向偏转−15.63度。
- [归零后动作](studies/command_validation_v1/cases/walk_stop/rollout_20fps.mp4)：12秒归零后仍持续踏步；净位移减慢，未达到预设瞬时停止条件。

复核代码在`validation/`，接口测试在`tests/test_official_interface.py`。原`results/`未覆盖。复现实验使用新tag：

```bash
cd /home/yd/Documents/VLA
.venvs/openpi-libero/bin/python experiments/unitree_g1_rl_baseline/validation/run_study.py --tag YOUR_NEW_TAG
```

模型任务应使用systemd用户服务设置内存上限（本轮MemoryHigh=3G、MemoryMax=4G）。完整参数与源码快照在该阶段目录的PROTOCOL.json与source/。策略重新加载后再运行下一工况，不能复用循环网络隐状态。

## GitHub副本运行说明

模型权重不随仓库提交。先运行`python experiments/unitree_g1_rl_baseline/scripts/fetch_policy.py`下载固定官方145KB策略并验证SHA256，再运行接口测试。依赖Python3.11、torch2.7.1、mujoco3.2.3、numpy1.26.4、PyYAML、Pillow、imageio和matplotlib；版本来自本机已验证环境，不含训练栈。

完整物理轨迹分块未上传，汇总统计、协议、源码快照、图片与报告保留。视频在仓库`reports/bytedance_engineering/videos/ch09/`。新实验生成自己的studies目录；历史快照不可恢复为完整原始轨迹。README中的工作站解释器路径需替换为你的环境。
