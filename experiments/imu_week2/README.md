# Week 2 自主补充：IMU滤波与反馈频率

本目录依据字节月度计划 Week 2 扩展，独立于老师具体下发的 Week 1 Task1。主报告为 `REPORT.md`，先看其中真实/仿真边界。

## 本机复现

```bash
cd /home/yd/Documents/VLA
/usr/bin/python3 experiments/imu_week2/scripts/run_sim.py --out experiments/imu_week2/runs/new_run
/usr/bin/python3 experiments/imu_week2/scripts/analyze.py experiments/imu_week2/runs/new_run
```

`new_run` 不得已存在。使用既有独立 Isaac5.1 Python3.11 环境；ROS2节点使用系统Python3.10、Humble、FastDDS、domain51与专用话题。场景headless，物理周期10ms，Panda末端link7挂载官方虚拟IMU。全过程不打开任何真实机械臂串口，不修改旧ROS2工作空间。

`config/experiment.json` 设置频率顺序、2秒保持＋5秒轨迹＋2秒保持；当前12个阶段，25/50/100Hz各4次，正反向各2次。输出全部实际关节与IMU数据、ROS日志及源码快照。按Ctrl+C停止；退出会清理子进程。

## IMU原始/滤波ROS发布

专用 `.venvs/imu-week2` 是系统Python3.10的独立venv，继承系统numpy/scipy，新增pyserial3.5。系统缺少ensurepip，使用 `--without-pip --system-site-packages` 创建，再由现有pip `--python` 安装；系统包未修改。`imu-python` 包装ROS环境。

离线回放已实际测试（不是硬件采样）：

```bash
# 终端1，输出目录均应尚未存在
experiments/imu_week2/scripts/imu-python experiments/imu_week2/scripts/imu_sink.py --out experiments/imu_week2/runs/new_sink --seconds 13
# 终端2
experiments/imu_week2/scripts/imu-python experiments/imu_week2/scripts/imu_node.py --input experiments/imu_week2/runs/run001/imu_filter_data.csv --out experiments/imu_week2/runs/new_publisher --seconds 10 --filter ema
```

发布 `/imu_week2/raw` 与 `/imu_week2/filtered` 的 sensor_msgs/Imu；独立接收节点验证序号、六轴值和实际消息接收时间。CSV回放没有姿态真值字段，orientation_covariance[0]=-1；未做姿态融合。

未来真实只读入口（本轮硬件未连接，未运行此分支）：

```bash
experiments/imu_week2/scripts/imu-python experiments/imu_week2/scripts/imu_node.py --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --out experiments/imu_week2/runs/new_physical_imu --seconds 30
```

运行前确认该端口仍为IMU、没有其他读取器占用；若权限不足需使用已授权dialout组。厂家串口驱动副本位于 `vendor/YbImuSerialLib.py`，来源 `/home/yd/Documents/IMU/demo`。仅打开115200串口接收，RAW包触发发布，g转换为m/s²；不设置频率、校准、清零或写Flash。没有真实串口时显式报错，不代入模拟数据。此分支仅静态检查，真实静止/运动采集与验证需要后续硬件参与。

## 文件与读法

- `runs/run002_balanced`：主频率与虚拟IMU实验，12阶段正反向平衡。
- `runs/run001`：9阶段先导实验，频率/方向不平衡，不作为主要跟踪精度结论；另有实际10秒ROS IMU回放收发验证。
- `frequency_metrics.csv`：每阶段频率、间隔、relay链路延迟、关节误差与拟合滞后。
- `filter_metrics.csv` / `imu_filter_data.csv`：因果滤波对照及已注明的注入噪声。
- `FILTER_VERIFICATION.json`：常值保持、阶跃前无响应及响应时间检查。
- 所有降噪数字是已知噪声模型条件下结果，不是物理IMU精度/噪声测量。ROS中继延迟不包含上游Isaac传输；轨迹拟合滞后不等于独立测得的网络延迟。
