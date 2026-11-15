# Jetson 实体 IMU 采集与发布频率测试

本次把此前 PC 端流程迁移到 Jetson，完成实体 IMU 的静止、运动采集及三档 ROS 2 发布测试。没有启动机械臂，也没有修改传感器输出配置。

## 环境与方法

Jetson Orin Nano，Ubuntu 22.04.5、L4T R36.4.7、ROS 2 Humble。使用系统 Python 3.10.12，不修改现有 an-project Conda 环境。串口为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，115200 波特率。

采集程序解析 RAW 六轴数据，将加速度转换为 m/s²，角速度为 rad/s；每个新样本更新一次 5 点因果均值。独立进程按目标频率发布 `/imu/data`，另一个进程接收。全部测时进程位于同一 Jetson，使用同机单调时钟。姿态未参与本实验，ROS 消息将 orientation_covariance[0] 设为 -1。

## 静止与运动采集

两段各采集约 30 秒、749 组。静止首末样本估计 25.131 Hz，运动 25.052 Hz。静止加速度模长约 9.84 m/s²。运动包含三个方向的转动；三轴角速度绝对峰值约 4.948、4.412、3.129 rad/s。在线滤波值与离线重算一致。静止 gx、gy 为零，可能受设备量化或内部处理影响，不能解读为无噪声。

![静止六轴](runs/jetson_20261020/static_analysis_01/six_axes.png)

![运动六轴](runs/jetson_20261020/motion_analysis_02/six_axes.png)

## 发布频率测试

每档直接读取实体 IMU 约 30 秒，分析中间稳定区间，排除启动和停止尾段。

| 目标 Hz | 实际 Hz | 新测量 Hz | 重复比例 | 间隔标准差 ms | 消息传输 P95 ms | 最新原始样本年龄 P95 ms |
|---|---|---|---|---|---|---|
| 50 | 50.000 | 24.967 | 50.04% | 0.497 | 1.095 | 40.109 |
| 100 | 99.999 | 24.966 | 75.03% | 0.520 | 0.816 | 39.605 |
| 200 | 200.001 | 24.968 | 87.52% | 0.390 | 0.759 | 40.164 |

三档共发布并接收 10494 条反馈消息，元数据与 Imu 消息计数一致，发布至接收的序号未发现缺口。板卡协议没有在此解析设备样本序号，因此不能据此认定传感器或 USB 链路零丢样。

消息传输指 relay 发布前至 sink 回调接收的时间；样本年龄从主机解析 RAW 包开始计算。两者均不是设备采样到控制执行的完整时延，也不包含 5 点滤波的约 80 ms 群延迟。提高发布频率主要减少等待下一次发布的时间，不能补足新的传感器信息。

## 文件与后续工作

相关代码在 `scripts/imu_serial_node.py`、`scripts/feedback_node.py`、`scripts/run_feedback.py`、`scripts/filter_csv.py`、`scripts/analyze_feedback.py`；启动环境由 `scripts/ros-python` 指定。相对路径均以本报告所在目录为根。

原始数据与分析位于 `runs/jetson_20261020/`：

```text
hardware_static_20261020_01/   静止 CSV、原始串口字节、会话信息
hardware_motion_20261020_02/   运动 CSV、原始串口字节、会话信息
static_analysis_01/           六轴图、均值与标准差、滤波 CSV
motion_analysis_02/           六轴图、均值与标准差、滤波 CSV
hardware_feedback_20261020_01/
  source25_feedback50/        三进程日志、CSV、原始字节
  source25_feedback100/
  source25_feedback200/
  metrics.csv                发布频率、间隔、时延与重复比例
  feedback_rates.png
```

Jetson 同名工程目录保留原始记录；上述资料已回传工作站。此前暂停的 motion_01 不用于本次分析。后续先讨论约 25 Hz 的设备输出与 80 ms 滤波延迟是否适合目标控制任务，再决定是否调整设备配置或滤波窗口；当前未改固件参数，也未运行机械臂闭环。
