# 倒立摆、步态和接触控制实现参考

查阅日期：2026-11-12。本项目依据月度计划的倒立摆步态与稳定性方向自主展开。

| 来源 | 许可/状态 | 本轮采用的设计 | 未直接引入的内容 |
|---|---|---|---|
| [Caron lipm_walking_controller](https://github.com/stephane-caron/lipm_walking_controller) | BSD-2-Clause；页面显示2026-10-25已归档 | 将步态规划、质心反馈、接触稳定器分层，明确模型与全身控制边界 | mc_rtc、HRP-4机器人模型与真实硬件控制链，不作为新增运行依赖 |
| [rdesarz biped-walking-controller](https://github.com/rdesarz/biped-walking-controller) | MIT；含CI、测试及预瞻/IK模块 | 分开表示支撑脚、质心轨迹和摆脚；先验证简化模型 | Pinocchio、PyBullet、OSQP等整套依赖；该项目自己也区分纯IK与动力学仿真 |
| [Capture point](https://scaron.info/robotics/capture-point.html) | 作者技术文档 | ξ=c+v/ω、DCM预测和受约束落脚调整 | 不把瞬时捕获点直接当作有摆动时间与落脚范围限制时的必然可行解 |
| [pymanoid稳定器示例](https://scaron.info/doc/pymanoid/examples.html) | 原项目示例/文档 | 以明确推扰测试DCM反馈，区分输入约束和理想响应 | 不依赖旧OpenRAVE环境 |
| [MuJoCo 2.3.7 XML参考](https://mujoco.readthedocs.io/en/2.3.7/XMLreference.html) | 官方版本文档；MuJoCo Apache-2.0 | 自建有限脚掌、踝关节、摆杆与集中质量的平面非线性接触模型 | 不将单脚站立实验解释为完整双足行走 |

控制与实验代码为本项目实现，没有直接复制上述项目源码。不新增ROS、Jetson或实体设备依赖。
