# Panda 轨迹跟踪与柔顺接触实验

使用 Isaac Sim 5.1.0 和官方 Franka Panda 模型，完成前三关节5秒五次轨迹、10ms控制更新、固定平面接触力读取，以及5N、M=1/B=40导纳对照。

截至2026-10-26，累计34组全长物理试验、599秒、599000条1ms力样本，另有1组保护提前终止和54组纯接口回放。全长记录包含失败对照，不表示全部控制方案合格。正式结果见 REPORT.md、REPORT.pdf 和 results.json。无需实体机械臂、IMU或Jetson。

## 控制和场景范围

- 原任务使用普通关节PID，两种主条件使用相同参数；质量矩阵补偿PID单独作为扩展。
- Panda手部球形探针，半径25mm；固定水平面，原夹爪及其他连杆不参与碰撞。
- 所有条件关闭重力，当前结果不代表完整碰撞模型或实机性能。
- 物理每次只推进1ms，渲染单独调用，不通过混合step切换改变物理时长。
- 原始1kHz接触力完整保存，100Hz反馈使用上一周期10个样本均值。

## 运行

目录保持在项目的 `experiments/panda_contact_isaac` 下。已有环境入口为 `.venvs/isaacsim51`；可用 `ISAACSIM_ENV` 指定另一套Isaac Sim 5.1 Python环境。

```bash
experiments/panda_contact_isaac/scripts/isaac-python \
  experiments/panda_contact_isaac/scripts/run.py \
  --inner pid --mode admittance \
  --out experiments/panda_contact_isaac/runs/new_run
```

`--contact-gate` 在检测到接触后才启用导纳；配合 `--force-ramp .5` 可逐步增加目标力。默认主对照不启用这两个选项。

`scripts/pid_suite.sh`生成普通PID主实验、接触门控与慢速门控四组。`scripts/suite.sh`及`scripts/finish.sh`生成补偿PID的九组对照。`scripts/revision_suite.sh`生成七组模拟策略执行接口物理测试。运行脚本检查COMPLETE标记，不以应用进程退出码单独判断成功。

分析使用项目已有 `.venvs/qwen-scene`：

```bash
.venvs/qwen-scene/bin/python experiments/panda_contact_isaac/scripts/analyze.py
.venvs/qwen-scene/bin/python experiments/panda_contact_isaac/scripts/verify.py
.venvs/qwen-scene/bin/python experiments/panda_contact_isaac/scripts/analyze_execution.py
.venvs/qwen-scene/bin/python experiments/panda_contact_isaac/scripts/sweep_stream.py
```

生成报告还需要pypdf及Google Chrome。分析依赖列于requirements-analysis.txt。Isaac环境与分析环境分开，避免改动现有ROS2或系统Python。首次获取Franka USD需要访问NVIDIA资源服务器，scene.usda保存的是对官方模型的引用及本实验覆盖层。

## 文件

- `runs/<条件>/trace.csv`：100Hz目标与实际关节角、末端高度、力反馈、导纳位移。
- `physics_force.csv`：1kHz原始法向力。
- `joint_tracking.csv`：前三轴目标、实际及误差，单位rad。
- `time_audit.json`：引擎时钟总时长检查。
- `metadata.json`、`scene.usda`：条件与场景。
- `simulation.mp4`：两种主条件的仿真录屏，叠加时间和力读数。
- `verification.json`：时长、样本数、自由段零力、五次参考和成对初始轨迹的一致性检查。

`runs/pilot*`、`invalid_timebase_v1`、`interrupted_checks`为调试和排除记录，不参与正式统计。正式条件名称在analyze.py中明确列出。

接触力接口的dt必须传入真实物理步长。参考：[NVIDIA Isaac Sim 5.1 API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.api/docs/index.html)。

## 策略执行接口范围

`action_stream.py`输出Panda关节绝对目标，角度单位rad。七组执行测试使用解析五次轨迹生成动作块，没有运行VLA网络。窗口回放54组不包含机器人动力学。

默认块更新200ms、送达延迟120ms、40步动作窗口、动作间隔10ms。限幅针对命令速度0.35rad/s和加速度1rad/s²；源动作年龄超过0.5s时，超时保持条件冻结一次读取的实测姿态，再受限恢复。它不等于碰撞保护或认证安全控制。

ZIP保留报告MD及其配图，便于与PDF对照；不含CSV、视频、权重或环境。参考LeRobot异步推理设计：https://huggingface.co/docs/lerobot/async 。

## 恒力稳定性改进

先核对libfranka、ros2_controllers与FZI控制器的许可证、维护状态、依赖和相关issue，见OPEN_SOURCE_REVIEW.md及open_source_review.json。没有引入新ROS2或硬件依赖，没有复制上游控制代码。

按顺序运行stability_suite.sh（冻结/滤波/周期）、stability_ik_suite.sh（逆解初值与容差）、stability_differential_suite.sh（连续微分逆解）、stability_refine_suite.sh（反馈及周期复测和跳变保护）、stability_validation_suite.sh（重复与高度±1mm）。历史上1μm容差导致的失稳发生在加入跳变保护之前；当前代码会提前拦截，脚本对此检查FAILED.json，而非要求正常完成。

所选组合：

```bash
experiments/panda_contact_isaac/scripts/isaac-python \
  experiments/panda_contact_isaac/scripts/run.py \
  --inner pid --mode admittance --contact-gate \
  --freeze-on-contact --ik-seed differential --force-feedback ema \
  --out experiments/panda_contact_isaac/runs/new_stability_run
```

导纳M=1/B=40、控制周期10ms；增量逆解正则0.01，每轴每轮增量不超过0.003rad；指数反馈α=0.2。跳变保护默认0.05rad，并检查有限值。评价始终使用未做指数滤波的1ms力记录，先取固定10ms均值，再检查5±0.5N连续0.5秒。

```bash
.venvs/qwen-scene/bin/python experiments/panda_contact_isaac/scripts/analyze_stability.py
.venvs/qwen-scene/bin/python experiments/panda_contact_isaac/scripts/verify_stability.py
```

所选组合及重复、平面±1mm扰动都通过上述有限窗口标准。但末两秒仍有6%—18%的周期均力越界，瞬时力也有尖峰，不代表整个接触过程恒力或工业部署完成。刚度、摩擦和重力变化仍需后续系统验证。
