# 接触控制优化的实现参考

本轮沿用 Isaac Sim 5.1，不升级原有 ROS2、LeRobot 或系统 Python。上游项目用于核对设计和接口，没有将真实机器人控制器直接移植进仿真。

| 参考 | 许可证或来源 | 借鉴内容 | 使用边界 |
|---|---|---|---|
| [Franka libfranka](https://github.com/frankarobotics/libfranka) | Apache-2.0 | 重力模型、力反馈、平滑目标力和运动偏离检查 | 官方力控制示例要求开始前已经接触；并不解决自由空间接近问题。真实 Franka 控制接口的内部补偿行为也不能直接套用到 PhysX |
| [ros2_controllers admittance_controller](https://github.com/ros-controls/ros2_controllers/tree/master/admittance_controller) | Apache-2.0 | 力反馈、运动学映射与控制接口分离 | 本轮不安装 Rolling/Jazzy 控制器，避免影响已有 Humble 环境 |
| [NVIDIA 关节稳定性指南](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.2/dev_guide/guides/articulation_stability_guide.html) | 官方文档 | 重力前馈、离散步长、求解迭代与硬约束耦合 | 调整求解器会改变数值接触，需要独立对照，不能全部归因于算法 |
| [PhysX 材质定义](https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/latest/physxschema/class_physx_schema_physx_material_a_p_i.html) | 官方 API 文档 | stiffness>0 启用柔顺接触；damping、摩擦和恢复系数 | 柔软材质降低冲击是环境变化，不能与刚性场景混算算法改善比例 |
| [Isaac Sim 5.1 Core API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.api/docs/index.html) | 官方版本文档及本地源码 | PhysX 重力补偿向量、接触力矩阵与材质绑定 | 本地旧 API 调用实际转发到 get_gravity_compensation_forces；本轮固定基座。浮动基座的公开问题不能当作本实验根因 |

查询日期：2026-11-12。新增控制逻辑为本项目实现，不复制以上项目代码。采用 1ms 原始力与固定 10ms 分箱均值评价，控制端 EMA 不替代评价信号。
