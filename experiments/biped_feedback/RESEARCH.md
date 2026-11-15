# 实现参考与取舍

2026-11-13先核对现有平面双足代码，再检索同类实现。仓库维护状态、许可证及查询时HEAD记录在research/sources.json。没有下载新模型，也没有将外部控制代码直接复制进项目。

| 参考 | 状态与许可证 | 采用的设计 | 本轮不引入的依赖或限制 |
|---|---|---|---|
| [Rhoban/PlaCo](https://github.com/Rhoban/placo) | 未归档，MIT | 步态、支撑阶段与任务求解分层；明确ZMP和运动学任务接口 | Pinocchio/C++及完整人形接口与当前MuJoCo2.3.7模型不同，不整体迁移 |
| [stack-of-tasks/TSID](https://github.com/stack-of-tasks/tsid) | 未归档，BSD-2-Clause | 浮动基座逆动力学、接触力约束、关节力矩输出 | 引入另一套动力学和求解接口会使旧结果不便配对；继续使用已验证的OSQP |
| [Caron LIPM walking controller](https://github.com/stephane-caron/lipm_walking_controller) | 已归档，BSD-2-Clause，仅设计参考 | ZMP/DCM规划、稳定器与接触扳手分配是不同层次，不能把ZMP当作踝关节力矩 | mc_rtc、HRP-4模型及归档状态不适合直接作为当前运行依赖 |
| [MuJoCo 2.3.7 API](https://mujoco.readthedocs.io/en/2.3.7/APIreference/APIfunctions.html#mj-contactforce) | 使用本地已有版本 | 接触局部坐标转世界坐标、真实接触扳手及雅可比核验 | 软接触允许小量穿透，QP刚性接触力与实际求解力不必完全相同 |

本轮直接复用的是本项目lipm_extension中的六电机模型、逆动力学QP和连续摆脚轨迹，独立保存到新目录，旧版本保持不变。模型为矢状面平面双足，不是三维人形；没有通过基座执行器、mocap、逐帧修改qpos或外施接触力来完成行走。

## 映射与符号

世界坐标x向前、z向上，力矩按世界y轴右手正方向。每只脚的接触扳手[Fx,Fz,My]以足底site为参考点。令足底位置为(x_i,z_i)，地面合成压力中心：

`p = sum(x_i*Fz_i - z_i*Fx_i - My_i) / sum(Fz_i)`。

这里My是接触力矩，不是踝电机力矩。线性倒立摆在恒高、质心角动量变化为0的近似下，有`a_x = g/h*(c_x-p)`。新控制器只算一次外环加速度：`a_cmd = a_ff + 80*(c_ref-c_obs) + 18*(v_ref-v_obs)`，再用同一值生成CoM任务与ZMP目标；由QP的完整动力学约束映射到六电机力矩。躯干与摆脚各自的任务增益具有不同物理意义，不强行取成同一数值。

实际多关节机器人不满足严格恒高、零角动量，故把ZMP力矩平衡作为软目标而非不允许偏差的硬等式。独立报告实际与预测CoP、质心误差和力矩变化，不用QP预测力替代MuJoCo真实接触测量。
