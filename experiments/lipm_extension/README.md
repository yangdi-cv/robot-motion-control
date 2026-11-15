# 步态执行与反馈时序增强

上一版 `lipm_balance` 保持原样。本目录把后续问题拆成三个独立层次：

1. 固定传感器采样率、延迟抖动、乱序、丢帧、有色噪声与偏置漂移。
2. 双支撑CoP过渡、连续摆脚、受约束落脚位置与时刻选择。
3. 有髋/膝/踝、浮动基座和真实脚掌接触的平面双足，用逆动力学QP输出关节力矩。

三层分别验证，尚未将传感器收包模块接入双足全身控制器。多关节模型是平面矢状面模型，不等于完整三维人形机器人。

## 环境与运行

复用隔离的 `.venvs/lipm-balance`，仅新增OSQP及其依赖，没有替换原有NumPy/SciPy/MuJoCo版本，没有修改系统Python、ROS或厂家工程。完整锁定版本见 requirements.lock.txt。

从工作区根目录运行：

```bash
experiments/lipm_extension/scripts/python experiments/lipm_extension/scripts/tests.py
experiments/lipm_extension/scripts/python experiments/lipm_extension/scripts/suite.py network
experiments/lipm_extension/scripts/python experiments/lipm_extension/scripts/suite.py gait
experiments/lipm_extension/scripts/python experiments/lipm_extension/scripts/biped_suite.py
experiments/lipm_extension/scripts/python experiments/lipm_extension/scripts/analyze.py
experiments/lipm_extension/scripts/python experiments/lipm_extension/scripts/verify.py
```

新机器可用Python3.10创建独立环境并安装 requirements.lock.txt；通过 `LIPM_PYTHON` 指定解释器。所有运行仅为本地仿真，不建立实体机器人连接。录屏需要EGL/OpenGL，报告PDF导出需要Google Chrome。

## 代码与数据

```text
scripts/
  engine.py             原LIPM引擎的独立副本，加观察器接口
  network.py            带时间戳测量、队列、乱序和过期处理
  trajectory.py         C2摆脚轨迹、连续速度/加速度界
  gait.py               DCM、双支撑、时间与位置候选
  biped.py              平面双足IK初态、逆动力学QP与物理执行
  suite.py              反馈与规划条件矩阵
  biped_suite.py        不同物理条件；不把无随机性重跑当独立样本
  analyze.py            指标与图表
  verify.py             时钟、因果性、动力学、运动约束与执行检查
  tests.py              回归
  record.py             独立录屏重跑
  video.py              仿真相机帧与状态曲线
  report.py             报告、PDF和源码包
models/biped.xml         自建平面双足模型
configs/protocol.json
*_manifest.json         条件、源码版本及原始目录
*_results.json          全部结果，含拒绝、过期停止与失稳
figures/
runs/<类别_源码指纹>/<配置编号>/
  result.json
  physics.npz           LIPM/规划物理步记录
  observer.npz          控制更新读取的时间戳与队列统计
  trace.npz             控制/关节/接触记录
recordings/             重复录屏，不加算独立验证数量
archive/                先导实现版本，不混入正式统计
```

## 重要边界

- 25Hz传感器对应每40ms一帧；提高控制发布率不会生成新测量。
- 数据年龄超限时停止模型推进，记录为受控终止，不声称验证了真实机器人的安全停止。
- 简化模型的负载分配是可行性构造，非测得的地面反力；实际双足的接触力由MuJoCo独立产生。
- 时间候选为有限枚举，且每阶段只提交一次时刻选择，避免持续改期；可行性要求连续摆脚速度与加速度均不过界。
- 全身QP中的接触扳手不施加到模拟器；模拟器只接收六个电机力矩，基座三个自由度没有执行器。
- 双足成功须同时满足完整运行、8次实测落脚、实际前进量和末端足位，不能用“不倒”代替“走完”。
- 双足接触集合按计划给定，状态观测理想化，侧向自由度受限；未完成三维平衡、复杂地面或真实传感闭环。
- 源码包不含原始数组、录像、环境或其他项目文件；运行会重新生成数据。
