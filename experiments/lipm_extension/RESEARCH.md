# 后续实现参考与取舍

本轮在原有模型实验基础上，分别验证状态反馈时序、时空落脚规划以及带关节动力学执行。未直接复制以下项目的控制代码。

- [Khadiv等：Walking Control Based on Step Timing Adaptation](https://arxiv.org/abs/1704.01271)：下一步时刻与位置联合选择。本轮用有限候选枚举与连续摆脚可行性筛查，不称为复现其完整优化器，也没有声称全局最优。
- [CMU：Reactive balance control in walking based on a bipedal linear inverted pendulum model](https://publications.ri.cmu.edu/reactive-balance-control-walking-based-bipedal-linear-inverted-pendulum-model)：说明双支撑对平衡策略的影响。本轮简化模型采用CoP过渡与可行负载分配；真实多关节接触另由MuJoCo检验，不能把两者视为同一个双摆模型。
- [OSQP官方Python接口](https://osqp.org/docs/interfaces/python.html)、[求解器文档](https://osqp.org/docs/solver/)：Apache-2.0。采用标准凸QP接口求解全身加速度、关节力矩与接触扳手，固定版本1.0.4及迭代参数，避免依赖执行时间触发不同的rho更新。
- [MuJoCo 2.3.7编程说明](https://mujoco.readthedocs.io/en/2.3.7/programming.html)：使用真实质量矩阵、偏置力、质心与足底雅可比。mj_step后关节位置已更新，但部分运动学缓存尚对应旧状态，有限差分Jdot前需同步；本轮新增回归检查。
- [Caron LIPM控制器](https://github.com/stephane-caron/lipm_walking_controller)：BSD-2-Clause，已归档；保留规划与稳定器分层的设计参考，不增加其mc_rtc依赖。

传感时序部分是仿真时间内的离散收包模型，不运行真实网络，不把1kHz物理状态当成25Hz传感器的新测量。状态预测只使用已接受的过去测量与已发CoP输入。
