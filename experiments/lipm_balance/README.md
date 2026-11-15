# 倒立摆步态与平衡控制实验

本项目围绕月度计划的倒立摆、步态规划、扰动恢复和频率/延迟分析展开。使用独立PC环境，不需要IMU、Jetson或实体机器人。

分为两层，不能混作完整双足行走：

1. 二维LIPM：12个支撑阶段、11次理想换脚，名义前进1.98m；比较PD、LQR、受约束落脚调整和因果延迟预测。
2. MuJoCo：有限脚掌、踝关节和摆杆质量模型，具有真实刚体接触，只验证站立受扰恢复，不执行步态。

## 环境

独立Python环境：`.venvs/lipm-balance`。Python3.10、NumPy1.26.4、SciPy1.15.3、MuJoCo2.3.7。其他版本见 requirements.lock.txt；未改已有ROS、LeRobot、act-sim或系统Python。

在新机器上使用支持venv的Python3.10创建独立环境，再安装锁定依赖。MuJoCo录屏需要可用EGL/OpenGL；纯数值试验不需要显卡。源码包解压到工作区的 `experiments/lipm_balance/`；脚本入口默认使用工作区 `.venvs/lipm-balance/bin/python`，也可通过 `LIPM_PYTHON` 指定其他解释器。

```bash
python3.10 -m venv .venvs/lipm-balance
.venvs/lipm-balance/bin/python -m pip install -r experiments/lipm_balance/requirements.lock.txt
```

## 执行

从工作区根目录：

```bash
experiments/lipm_balance/scripts/python experiments/lipm_balance/scripts/tests.py
experiments/lipm_balance/scripts/python experiments/lipm_balance/scripts/suite.py
experiments/lipm_balance/scripts/python experiments/lipm_balance/scripts/physical_suite.py
experiments/lipm_balance/scripts/python experiments/lipm_balance/scripts/analyze.py
experiments/lipm_balance/scripts/python experiments/lipm_balance/scripts/verify.py
```

录屏和报告：

```bash
experiments/lipm_balance/scripts/python experiments/lipm_balance/scripts/record.py
experiments/lipm_balance/scripts/python experiments/lipm_balance/scripts/video.py
experiments/lipm_balance/scripts/python experiments/lipm_balance/scripts/report.py
```

PDF导出另需本机Google Chrome。脚本直接运行本地仿真，不建立网络控制连接，不发送实体机器人指令。suite.py 使用6个工作进程，physical_suite.py 使用4个；限制BLAS线程以避免嵌套线程过量。进程并行不代表硬实时运行。

## 目录

```text
scripts/
  python                 独立解释器入口
  core.py                LIPM、参考、LQR、落脚调整与因果预测
  suite.py               主网格与独立验证条件
  physical.py            有限脚掌非线性站立
  physical_suite.py      接触条件、延迟与控制周期批量对照
  analyze.py             分组统计、延迟谱半径与图表
  verify.py              动力学重建、约束、时钟与接触力检查
  tests.py               数学与因果性回归
  record.py              与正式条件匹配的站立录屏重跑
  video.py               保存状态动画与真实相机帧对比
  report.py              PDF和源码包
  pilot.py               初步无噪声机制对照，不混入正式统计
models/ankle.xml          原创平面脚掌-踝关节-摆杆模型
configs/protocol.json    单位、约束和评价定义
manifest.json            LIPM条件与源码指纹
physical_manifest.json   站立条件与源码/模型指纹
results.json             全部LIPM结果，含提前终止
physical_results.json    站立原始指标
physical_evaluation.json 额外足部稳定筛查
summary.json
physical_summary.json
verification.json
figures/
REPORT.md
REPORT.pdf
runs/<源码版本>/<配置编号>/
  source.py              对应版本快照，位于版本目录
  result.json            参数与结果
  trace.npz              控制周期记录；站立为物理周期记录
  physics.npz            LIPM逐物理步状态、输入、支撑与路线平移
recordings/              站立重复录屏，单独保存，不作为新增独立试验
archive/                 先导评价及实现版本，不混入正式汇总
```

源码包不包含runs、archive、录屏、权重或Python环境；包含运行所需脚本、模型、锁定依赖、参数、汇总JSON、报告MD和配图。运行后会生成原始记录。配置编号按内容哈希生成；源码改变会使用新版本目录，已有同版本条件可复用。

## 解释边界

- LIPM质心高度恒定，脚部交换为瞬时理想事件；没有摆腿动力学、双支撑过渡或全身逆运动学。
- 落脚目标每轴调整速率有限，但这不是对真实摆腿速度和关节力矩的证明。
- 地面摩擦检查基于地面反力，不把作用于质心的外推力误算为地面摩擦需求。
- 预测器只读取过去测量和已经发出的CoP输入，不读取当前真值或未来推力。
- 路线平移和相对原路线误差分开报告；移动参考不能隐藏实际绕行。
- 主网格全部用1ms物理状态评价，控制频率改变不会改变误差采样口径。半步长检查另列。
- 失败终止后不填充静止轨迹；动画显示终止状态时会明确标记。
- 3个噪声种子的比例是该组结果，不是通用成功概率。数值闭环和固定脚线性谱半径也不能替代实体验证。
- MuJoCo的PD/LQR作用于踝关节力矩；LIPM的输入是CoP，两层控制器不是同一套增益。
