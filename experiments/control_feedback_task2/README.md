# Panda控制改进与IMU反馈分析

这是字节Task2的阶段工作。包含真实Isaac/ROS仿真控制实验、公开TUM-VI真实IMU分析、ROS2数据回放与后续Jetson采集入口。尚无本轮Jetson或自有板卡采集结果。

## 目录

- scripts/isaac_scene.py、controller.py、run_control.py：Isaac5.1 Panda、实际ROS2 Bridge和外部控制节点。
- config/control_suite.json：三档Kp和有/无速度前馈，共24阶段。
- config/control_validation.json：选择参数后用不同幅度验证，共16阶段。
- config/control_frequency.json：物理周期5ms/200Hz，50/100/200Hz外环对照，共24阶段。
- config/control_frequency_norender.json：同样24阶段，关闭逐帧渲染并采用绝对时刻定时。
- scripts/analyze_control.py、analyze_control_frequency.py：误差、超调、保持抖动和频率分析。
- scripts/download_public_imu.py、download_motion.py：从TUM官方服务器按HTTP范围提取所需真实IMU数据。
- scripts/analyze_imu.py：静止/运动各30秒六轴5点移动平均、统计图、约602秒静止数据的频谱与Allan曲线。
- scripts/feedback_node.py、run_feedback.py、analyze_feedback.py：三个独立ROS进程，200/25Hz输入、50/100/200Hz输出。
- scripts/imu_serial_node.py：以后接设备时使用的只读采集入口；本轮仅检查语法和参数入口。
- scripts/filter_csv.py：通用六轴CSV分析，可直接读取上述真实采集程序的输出。
- scripts/check_software.py：已知正弦幅相、缺失序号、重复测量检查。
- vendor/YbImuSerialLib.py：原IMU厂家协议示例，不是个人实现。

## 环境

Isaac5.1独立环境 `.venvs/isaacsim51`，ROS2 Humble使用系统Python。分析使用NumPy、SciPy、Matplotlib；下载使用requests。ROS域71，仿真主题 `/control_task2/`，IMU回放主题 `/task2/`，原始及滤波IMU为 `/imu/data_raw` 和 `/imu/data`。启动前确认同一ROS域没有别的IMU发布者；不要同时启动原网页的IMU串口读取服务。

Python包装器按原项目层级定位环境，源码包需保持 `experiments/control_feedback_task2` 路径。新电脑需单独安装Isaac、ROS与依赖；不是零配置安装包。

## 复现

在项目根目录执行，所有新的输出目录应未存在：

```bash
python3 experiments/control_feedback_task2/scripts/download_public_imu.py
python3 experiments/control_feedback_task2/scripts/analyze_imu.py
python3 experiments/control_feedback_task2/scripts/run_control.py --out experiments/control_feedback_task2/runs/new_control
python3 experiments/control_feedback_task2/scripts/analyze_control.py experiments/control_feedback_task2/runs/new_control --select
```

run_feedback.py默认读取公开motion_30s.csv，可用 `--input` 指定其他输入，`--out` 指定新的输出目录；analyze_feedback.py通过 `--out` 读取该目录。控制频率实验使用run_control.py加 `--config experiments/control_feedback_task2/config/control_frequency.json`。

明天硬件到位后，先检查串口和占用，再使用以下入口；本轮没有执行这些采集命令：

```bash
experiments/control_feedback_task2/scripts/ros-python experiments/control_feedback_task2/scripts/imu_serial_node.py \
  --port /dev/serial/by-id/当前IMU标识 --mode static --seconds 30 --out 新的静止采集目录
python3 experiments/control_feedback_task2/scripts/filter_csv.py --input 采集目录/samples.csv --out 新分析目录
```

运动段改为 `--mode motion`。程序不写传感器配置；加速度由厂家接口g转m/s²，角速度沿用rad/s，仍需目标设备单位及轴向检查。时间戳为主机收到数据的时刻，不是硬件曝光或采样时刻。

## 结果解释

驱动参数仅针对当前Panda仿真，速度前馈是向位置/速度驱动同时提供五次轨迹速度，不是逆动力学力矩前馈。关节频率实验使用模拟时钟调节外环采样，主机实测频率另记。IMU软件测试不把IMU用作Panda的关节反馈；这是两个分开的实验。

回放测试用本机单调时钟测量进程间耗时。数据年龄指最近输入样本到接收端的时间，不含5点滤波有效中心滞后，也不包含真实传感器内部、串口或Jetson延迟。25Hz试验是对公开记录降采样后的软件输入，不是实测自有板卡。

早期回放进程由SIGINT停止，部分退出码为1且有KeyboardInterrupt；正常数据已落盘，消息完整性按发送/接收CSV逐项核对。后续进程已增加正常中断处理。首次仿真启动失败保留在control001，未计入成功实验。

## 公开数据与许可

TUM VI Benchmark（Schubert等，2018）：https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset 。数据许可CC BY4.0。静止数据取dataset-calib-imu-static2.npy前120001行（约602秒），30秒分析段为60–90秒；运动数据取room1的imu0/data.csv，中间固定30秒。原始单位、请求范围和本地哈希见data/tum_vi及PROVENANCE.json。只下载所需IMU字节，未下载完整相机序列，也未声称通过完整远端文件MD5验证。

源码ZIP仅代码、配置、依赖与许可说明；原始公开数据子集、CSV和图表在材料包中，完整结果仍保留于本项目。

Panda频率实验保留逐帧渲染与无渲染两组结果；前者主机实际约153Hz而非200Hz。无渲染分析用 `analyze_control_frequency.py --run experiments/control_feedback_task2/runs/control_frequency_norender`。

## Jetson 实测更新（2026-10-20）

Jetson 系统 Python 3.10.12 与 ROS 2 Humble 已完成采集和三进程测试，现有 an-project 环境未修改。实体 IMU 约25Hz，50/100/200Hz是反馈发布频率，不是新的测量频率。新结果见 REPORT.md 第8节与 JETSON_REPORT.md。

在 Jetson 工程根目录运行，IMU 串口必须无其他程序占用，每次指定新的输出目录：

```bash
bash experiments/control_feedback_task2/scripts/ros-python experiments/control_feedback_task2/scripts/run_feedback.py --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --out experiments/control_feedback_task2/runs/new_hardware_feedback
python3 experiments/control_feedback_task2/scripts/analyze_feedback.py --out experiments/control_feedback_task2/runs/new_hardware_feedback
```

第二条分析命令需要 NumPy 和 Matplotlib；可将输出目录复制回工作站分析。程序不写传感器配置，不发送机械臂命令。Ctrl+C 可终止当前运行。所有测时进程同机，指标不包含设备内部采样时延或执行器响应。源码包不含原始记录；实验材料包包含本次实体采集的 CSV、原始串口字节、图表及会话信息。
