# Panda 带重力接触控制优化

针对上次报告中重力关闭、夹爪碰撞简化及初次接触冲击偏大的问题，分别验证重力补偿、原装夹爪接触、目标力缓升与接近减速。全部运行于固定基座 Panda / Isaac Sim 5.1，不连接实体机械臂，也不需要 IMU。

## 控制与评价

- 物理步长 1ms，主要控制周期 10ms；额外测试 5/20ms。
- 前三轴 5s 五次轨迹；随后沿世界 -Z 接近固定水平面。
- 普通关节 PID + PhysX 重力补偿；保留质量矩阵补偿作为单独对照。
- 导纳 M=1kg、B=40N·s/m。接触后冻结名义位置，连续微分逆解生成目标关节角。
- 目标力用三次平滑函数在 1s 内由 0 升到 5N。
- 接近减速使用已知仿真平面的名义高度与初始夹爪尺寸估算间隙。高度扰动不告知减速器，因此这是带有几何先验的控制测试，不是视觉定位。
- 接触力为手部、左指、右指对平面接触矩阵的世界 Z 分量之和，不包含连杆上其他接触。
- 反馈 EMA 与评价分离：结果以原始 1kHz 力和固定 10ms 分箱均力计算。
- 材质为 PhysX 柔顺接触的等效参数，不代表测量过的真实材料。摩擦只有法向压接工况，不能代替切向滑动试验。

## 目录

```text
scripts/
  run.py                 物理场景、补偿 PID、导纳、接触阶段和日志
  isaac-python           已有独立 Isaac 环境入口
  suite.py               基础对照
  validation_suite.py    高度、摩擦、刚度、周期和补偿方式对照
  analyze.py             固定信号口径的指标
  verify.py              采样、时钟、有限值和逆解步长检查
  figures.py             数据曲线
  report.py              报告与提交文件
runs/<实验名>/
  metadata.json          参数；后续运行附碰撞体、传感器和源码哈希
  source.py              对应运行源码快照（新增记录后）
  scene.usda             引用官方资源的场景覆盖层
  trace.csv              原生控制周期的关节和末端状态
  physics_force.csv      1ms 原始合力
  time_audit.json        仿真时钟核对
  COMPLETE               正常完成；不代表控制指标合格
figures/
PROTOCOL.json            指标与比较规则
OPEN_SOURCE_REVIEW.md    开源参考与使用边界
results.json
verification.json
REPORT.md
REPORT.pdf
```

## 运行

工作区根目录：

```bash
.venvs/qwen-scene/bin/python experiments/panda_contact_optimization/scripts/suite.py
.venvs/qwen-scene/bin/python experiments/panda_contact_optimization/scripts/validation_suite.py
.venvs/qwen-scene/bin/python experiments/panda_contact_optimization/scripts/analyze.py
.venvs/qwen-scene/bin/python experiments/panda_contact_optimization/scripts/verify.py
.venvs/qwen-scene/bin/python experiments/panda_contact_optimization/scripts/figures.py
```

单独执行带重力、刚性夹爪实验：

```bash
experiments/panda_contact_optimization/scripts/isaac-python \
  experiments/panda_contact_optimization/scripts/run.py \
  --inner pid --gravity --gravity-compensation --geometry gripper \
  --contact-gate --freeze-on-contact --ik-seed differential \
  --force-feedback ema --force-ramp 1 --approach-profile brake \
  --brake-distance .015 --minimum-approach-speed .001 \
  --force-velocity-limit .003 --solver-iterations 4 --duration 32 \
  --out experiments/panda_contact_optimization/runs/new_run
```

启动器默认使用工作区 `.venvs/isaacsim51`，通过 `ISAACSIM_ENV` 可指定其他已有 Isaac 环境；不改系统 Python 或 ROS2。第一次载入 Panda 资源需要网络或已缓存资产。已完成的输出目录不允许覆盖。

原始数据与录屏保存在本机；提交源码包包含脚本、参数、指标和报告配图，不打包环境、官方资产或原始录屏。

最终方案参数见 selection.json。源码包解压后请放在工作区 experiments/ 下；若使用其他位置，设置 ISAACSIM_ENV 指向已有 Isaac Sim 5.1 的 Python 环境。额外的本机 UDP 跨进程测试在 scripts/network.py；它消费解析动作，不发送机器人指令，也不代表远程设备或硬实时测试。

基础对照的早期实现另保留在 scripts/initial_run.py，scripts/suite.py 使用该文件重现最初七组条件；后续验证使用 scripts/run.py。两者的间隙计算和记录内容有差别，报告中保留初测与统一实现的区别。请勿对已存在的原始运行目录直接执行旧入口；suite.py 会自动跳过已完成目录。
