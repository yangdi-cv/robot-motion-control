# ROS2机器人仿真控制平台搭建与Franka Panda多关节轨迹规划验证报告

日期：2026-10-13（Australia/Sydney）  
任务对应：字节 PTA Task1  
主结果：`runs/run_20261013_003`（完整实验、分析及录屏均完成）

## 1. 问题与本次完成内容

在已有ROS2 Humble、MoveIt2/Panda工作空间的基础上，新增独立Isaac Sim 5.1.0环境。通过真实ROS2 Bridge向物理仿真Panda发送关节目标、接收状态，在前三个关节上执行5秒五次多项式轨迹，并记录目标、实际值、控制指令及误差。本报告来自实际仿真与ROS消息，不用生成数据代替反馈，也不把之前的网页demo或真机动作回放代替指定实验。

本次没有向SO101真机发送命令，没有覆盖`/home/yd/ros2_ws`原有代码。

## 2. 环境与隔离

| 项目 | 实测配置 |
| --- | --- |
| 系统 | Ubuntu 22.04.5 LTS，内核6.8.0-138-generic |
| GPU | NVIDIA GeForce RTX 4090，24564 MiB |
| NVIDIA驱动 | 580.178.04 |
| ROS | ROS2 Humble，系统安装`/opt/ros/humble` |
| 外部ROS节点 | 系统Python 3.10 |
| 仿真 | Isaac Sim 5.1.0.0，独立Python 3.11.13 |
| 仿真环境路径 | `/home/yd/Documents/VLA/.venvs/isaacsim51` |
| ROS中间件 | Fast DDS / rmw_fastrtps_cpp |
| 通信隔离 | ROS_DOMAIN_ID=51，localhost，专用话题前缀 |
| physics dt | 0.010 s |

Isaac进程清除系统ROS Python路径，使用其内置Python 3.11版Humble库；外部控制节点单独source系统Humble。没有替换系统Python或修改已有机械臂/视觉环境。独立环境依赖检查通过，版本锁定记录见[requirements.lock.txt](setup_logs/requirements.lock.txt)，安装过程见[isaac-install.log](setup_logs/isaac-install.log)。

实际命令输出的Ubuntu/GPU/软件版本截图：

![环境配置截图](setup_logs/ubuntu_environment.png)

Isaac场景截图：

![Panda仿真场景](runs/run_20261013_003/isaac_environment.png)

## 3. 模型、关节与通信

使用NVIDIA提供的FrankaPanda `franka.usd`。7个旋转关节为panda_joint1至panda_joint7，另外有两个夹爪关节。关节名称在实际articulation加载后核对，范围从USD RevoluteJoint属性读取：

| 关节 | 下限（度） | 上限（度） |
| --- | --- | --- |
| panda_joint1 | -166.003 | 166.003 |
| panda_joint2 | -101.001 | 101.001 |
| panda_joint3 | -166.003 | 166.003 |
| panda_joint4 | -176.001 | -3.999 |
| panda_joint5 | -166.003 | 166.003 |
| panda_joint6 | -1.003 | 215.002 |
| panda_joint7 | -166.003 | 166.003 |

初始9维位置为`[0, -0.785398, 0, -2.356194, 0, 1.570796, 0.785398, 0.04, 0.04]`；旋转关节使用rad，夹爪使用m。本次只改变前三个关节，其余臂关节保持目标不变。标准末端框架记录为`/World/Panda/panda_hand`；本实验进行关节空间验证，不计算末端位姿误差，也未另行验收完整ROS TF树。

实际模型路径、关节范围、初始位姿与场景参数见[scene_metadata.json](runs/run_20261013_003/scene_metadata.json)。[panda_scene.usda](runs/run_20261013_003/panda_scene.usda)保存场景与OmniGraph；机器人资产仍引用官方在线USD，复现需要能访问资产或已有缓存。

```mermaid
flowchart LR
    A[ROS2五次轨迹节点] --> B[/panda_task1/joint_command]
    B --> C[Bridge Subscriber]
    C --> D[Articulation Controller与PhysX]
    D --> E[Bridge JointState Publisher]
    E --> F[/panda_task1/joint_states]
    F --> A
```

OmniGraph同时发布仿真时钟`/clock`。控制节点按反馈帧的仿真时间计算参考，每个10ms仿真反馈周期发送一次位置目标；发送的是位置控制命令，不把轨迹速度与加速度混作另一种控制模式。命令话题有1个控制发布端和1个Bridge订阅端，状态话题有1个Bridge发布端和1个记录订阅端，原始端点与QoS证据见[ros_graph.txt](runs/run_20261013_003/ros_graph.txt)。

## 4. 五次轨迹与记录方法

令`u=t/T`，`T=5 s`，插值函数：

`q(t)=q0+(qf-q0)(10u³-15u⁴+6u⁵)`。

起止速度、加速度均为零，已用数值检查验证边界条件。设置：

| 关节 | 初始目标（rad） | 终点目标（rad） |
| --- | --- | --- |
| panda_joint1 | 0 | 0.4 |
| panda_joint2 | -0.785398 | -0.5 |
| panda_joint3 | 0 | -0.3 |

记录501个样本，包含t=0与t=5端点。每条记录保存仿真状态时间、轨迹相对时间、主机接收/发布monotonic时间、7维参考/实际位置、解析参考速度与位置误差。实际发送目标与参考位置相同。

误差定义为`e(t_state)=q_reference(t_state)-q_actual(t_state)`，没有后移参考或补偿滞后来降低误差。最大误差为`max(|e|)`，RMSE为`sqrt(mean(e²))`。位置信号包含驱动响应与通信/采样链路的影响，本实验未独立辨识各部分延迟。

## 5. 结果

| 关节 | 最大误差rad | RMSE rad | 最大误差度 | RMSE度 | 终点误差rad |
| --- | --- | --- | --- | --- | --- |
| panda_joint1 | 0.034990 | 0.022361 | 2.0048 | 1.2812 | 0.001800 |
| panda_joint2 | 0.023757 | 0.014928 | 1.3612 | 0.8553 | -0.000300 |
| panda_joint3 | 0.025935 | 0.016486 | 1.4859 | 0.9446 | -0.000400 |

时间与频率：

- 实际记录仿真时长：5.000000 s。
- 仿真采样周期中位数：10.000000 ms；最大：10.000000 ms。
- 对应主机接收窗口：5.041746 s；平均反馈频率：99.172 Hz。
- 主机接收间隔中位数：9.912 ms，P95：12.662 ms。
- 该窗口实时因子：0.9917。

因此，10ms/100Hz指仿真时间基准；实际主机调度有抖动，不宣称严格硬实时100Hz。

![目标与实际轨迹](runs/run_20261013_003/trajectory_tracking.png)

![跟踪误差](runs/run_20261013_003/tracking_error.png)

参考与实际曲线整体同向且最终到达设置目标，但运动阶段有可见滞后，最大误差约1.36–2.00度。终点误差明显小于中途峰值。当前为默认驱动及位置目标链路的基准，没有进行增益调优、力控或速度前馈对照；不能据此宣称最优精度或真实硬件性能。

## 6. 视频与数据附件

- [实际Isaac窗口运行录屏](runs/run_20261013_003/simulation_recording.mp4)：179帧，已重新解码验证，时间戳单调，约8.93秒视频；包括实验前后静止片段。
- [录屏中间帧](runs/run_20261013_003/video_midpoint_verified.png)。
- [原始跟踪与指令数据](runs/run_20261013_003/tracking.csv)。
- [指标数据表](runs/run_20261013_003/metrics.csv)及[完整指标JSON](runs/run_20261013_003/metrics.json)。
- [仿真步进时间记录](runs/run_20261013_003/simulation_timing.csv)。
- [控制节点日志](runs/run_20261013_003/controller.log)、[Isaac日志](runs/run_20261013_003/isaac.log)、[命令及退出状态](runs/run_20261013_003/commands.json)。

上述视频链接是交付包内相对链接；发送时应同时附上ZIP或把该MP4上传到导师可访问的位置。本次没有代发消息或上传外部服务。

## 7. 复现与排错痕迹

在该工作站，从项目根目录运行：

```bash
cd /home/yd/Documents/VLA
/usr/bin/python3 experiments/panda_task1/scripts/run_experiment.py --out experiments/panda_task1/runs/new_run
```

`new_run`必须是尚不存在的目录，避免覆盖证据。脚本启动Isaac、等待场景与桥就绪、捕获真实窗口、启动ROS节点、保存端点证据、分析并生成完成标记。所有进程在结束后清理。详见[README](README.md)。

本次保留三个运行目录：run001首次轨迹/分析通过，ROS setup在nounset环境报错后修复，原失败日志仍保留；录屏先发现现有FFmpeg缺少X11输入支持。run002反馈记录通过，但录屏PTS量化造成非单调，保留异常。改用真实X11窗口像素与高精度视频时间基准，经独立编解码检查后run003完整完成。本报告主表使用run003，不把失败媒体当作完整交付。

## 8. 后续工作与任务边界

Task1已具备环境、模型/Bridge、指定轨迹、数据表、误差图、截图与本地录屏的核心交付。后续应开展驱动增益/命令频率对照，明确反馈延迟与驱动滞后的贡献，再衔接IMU滤波任务。

尚未完成导纳控制、LQR稳定行走、IMU/编码器融合、ACT/VLA训练；这些不属于本次已完成成果。

参考：[Isaac 5.1 Python安装](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_python.html)、[ROS2安装/内置库](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)、[ROS2关节控制](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html)。
