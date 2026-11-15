# 接触控制开源实现对照

核对日期：2026-10-25—26。仓库更新时间和许可证通过GitHub API核对，提交号保存在open_source_review.json。

| 项目 | 许可证与维护 | 设计与依赖 | 适用范围及本轮取舍 |
|---|---|---|---|
| [libfranka](https://github.com/frankarobotics/libfranka) | Apache-2.0；2026-09-07推送 | C++/Eigen、真实Franka FCI；力控示例使用PI和目标渐增 | 官方示例要求起始已接触，不提供完整接近阶段；借鉴分阶段与平滑目标，不复制硬件接口 |
| [ros2_controllers/admittance_controller](https://github.com/ros-controls/ros2_controllers/tree/master/admittance_controller) | Apache-2.0；2026-10-19推送 | ros2_control、力传感器接口、kinematics_interface；可串联轨迹控制、配置力滤波 | 当前查看的是Rolling接口，不能直接安装替代Humble工程；借鉴测量、运动学、内环分层和坐标系检查 |
| [FZI cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2) | BSD-3-Clause；最近推送2024-10-30 | ROS2-control；运动、力与柔顺控制组合，基于前向动力学设计 | 更新时间较早，不作为当前优先依赖；用于比较控制链思路 |

## 已核对的限制和问题

- [libfranka力控示例](https://frankarobotics.github.io/libfranka/0.16.1/force_control_8cpp-example.html)的接触初态假设，与自动接近桌面不同，不能直接照搬运行流程。
- [ros2_controllers #2670](https://github.com/ros-controls/ros2_controllers/issues/2670)报告非激活状态缓存的参考会在激活时执行，报告环境为Jazzy，核对时issue仍开放。这提醒切换时要明确参考初始化，不代表本Isaac实现存在同一个缺陷。
- [libfranka #241](https://github.com/frankarobotics/libfranka/issues/241)记录控制器切换时UDP读取超时，核对时已关闭，涉及真实FCI/ROS2网络。它与本地PhysX力波动不同，不用来解释当前波动。
- [ROS2控制器文档](https://control.ros.org/rolling/doc/ros2_controllers/admittance_controller/doc/userdoc.html)要求明确测力坐标系和运动学插件；滤波系数与循环周期相关，不能把默认系数当作通用最优值。

## 本轮实施

不引入新的ROS2或硬件库，保留已运行的Isaac Sim 5.1工程。先对照接触后冻结名义位置、末次力采样/周期均值/指数滤波、5/10/20ms控制周期。所有条件仍保存1ms原始接触力，统一用10ms时间箱评估原有5±0.5N连续0.5秒标准，不用滤波后的信号代替原始力评价。

同时检查Isaac官方Lula逆运动学API：实际关节状态与上次命令都可作为warm_start，位置容差可显式配置。对照改变初值与容差，验证逆解是否引入参考跳变。这里使用现有SDK公开接口，不复制其他项目源码。

微分逆运动学使用Isaac公开雅可比与正运动学接口，采用阻尼伪逆、小步积分与关节限位；Lula整组求解保留为对照，并增加输出有限值和参考跳变保护。它是本工程的局部适配，不等于移植并验证了上游完整控制器。
