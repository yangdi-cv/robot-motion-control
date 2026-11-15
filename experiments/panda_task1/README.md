# Panda / Isaac Sim 5.1 / ROS2 Task1 实验

这是对两周前ROS2/Panda工作的直接延续，新增指定的物理仿真轨迹基准。原 `/home/yd/ros2_ws` 未改动。主交付见 [REPORT.md](REPORT.md)。

## 一条命令复现

```bash
cd /home/yd/Documents/VLA
/usr/bin/python3 experiments/panda_task1/scripts/run_experiment.py --out experiments/panda_task1/runs/new_run
```

输出目录不能已存在。现有独立Isaac环境位于 `.venvs/isaacsim51`，需要可用的图形桌面与RTX GPU；本工作站显示服务器为X11 `:1`。录屏读取真实Isaac窗口，不捕获其他桌面区域。新机器需按版本锁与官方Isaac 5.1安装说明重新建环境，不能把本机venv当成可迁移二进制。

## 代码入口

- `config/experiment.json`：轨迹目标、9个关节、10ms仿真步长与隔离话题。
- `scripts/trajectory.py`：解析五次轨迹及其速度、加速度。
- `scripts/isaac_scene.py`：实际Panda模型、PhysX与ROS2 OmniGraph。
- `scripts/controller.py`：外部ROS2节点，按状态时间计算并发布位置目标，保存原始数据。
- `scripts/analyze.py`：按同一状态时间对齐参考，计算最大误差/RMSE并画图。
- `scripts/run_experiment.py`：自动启动、等就绪、记录、诊断、分析与清理；每条命令有日志和退出码。
- `scripts/record_window.py`：X11窗口真实像素录屏，保留真实采集时间戳。
- `scripts/isaac-python` / `ros-python`：隔离Python 3.11与系统ROS Python 3.10。

先看配置、trajectory和controller，再看图与CSV，就能理解轨迹→命令→反馈→误差全过程，无需重新经历环境安装步骤。

## 当前结果与边界

完整运行 `runs/run_20261013_003`：501个状态样本、5秒仿真轨迹、10ms仿真周期，实际约99.17Hz反馈。真实录屏179帧。前三关节最大误差2.0048/1.3612/1.4859度，RMSE1.2812/0.8553/0.9446度。

100Hz是仿真时基，不是严格硬实时保证。当前ROS节点由100Hz反馈驱动目标生成，并非力矩控制或自研底层动力学控制器。未优化驱动增益，未辨识延迟各来源。主从机械臂不会收到本实验的话题或动作。

## 工作痕迹

安装日志、依赖锁与环境截图在 `setup_logs/`。各运行有独立config、命令、日志、状态数据和分析。run001/002媒体采集失败的日志保留，run003有 `RUN_COMPLETE`。后续新实验追加目录，不覆盖原结果。
