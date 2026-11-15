from pathlib import Path
import json

T = Path(__file__).resolve().parents[1]
R = T.parents[1]
v = json.loads((T / "runs/control_validation/analysis.json").read_text())
n = json.loads((T / "runs/control_frequency_norender/analysis.json").read_text())
p = R / "PROJECT_LOG.md"
s = p.read_text()
s += """\n## 2026-10-20：字节Task2阶段与控制改进\n\n根据桌面导师反馈和Task2文档推进。Jetson与自有板卡不在本轮使用范围，未打开串口或控制真机。\n\n- 原Panda/Isaac5.1/ROS2基础上完成三档增益×有无速度参考、正反方向两次重复，共24阶段；选Kp×4、Kd×2＋速度参考。不同幅度0.6/1.2验证16阶段，前三关节平均运动RMSE 0.8643°→0.0819°，拟合滞后214.25ms→20ms。\n- 物理周期5ms，50/100/200Hz关节外环比较24阶段。逐帧渲染下主机约38/77/153Hz；关闭逐帧渲染并采用绝对时刻定时，另跑相同24阶段。模拟时间与主机时间分别记录，结果见control_frequency_norender/analysis.json。\n- 公开TUM-VI：取静止原始NPY前120001行约602秒和room1原始IMU CSV；静止60–90秒、运动固定中间30秒，各5981样本。真实数据不是自有采集，CC BY4.0，未下载完整相机数据。5点平均静止标准差降低约49.2%–52.6%，群延迟约10ms。补充PSD与最长约30秒平均时间的Allan曲线，不当作硬件标定。\n- 三独立ROS进程6组真实数据回放：200或25Hz输入×50/100/200Hz反馈，20993条消息逐项核对，无发送/接收序号缺失；标准Imu消息数一致。滤波值相对原始数据独立计算误差0。25Hz输入在50/100/200Hz输出时重复测量分别约50%/75%/87.5%。不将主动降采样算作丢帧，不将主机耗时当作硬件端到端响应。\n- 后续Jetson只读采集入口、串口字节保存、ROS Imu与通用CSV分析已准备；未实测硬件分支。旧任务报告未改。\n\n新报告 `experiments/control_feedback_task2/REPORT.md`，PDF和源码/实验材料包 `deliverables/20261020_字节Task2阶段/`。原始公开子集、配置、CSV、失败与成功日志、分析、检查结果均本地保留。下一阶段仍需Jetson、自有IMU静止/运动采集、设备单位和时间基准检查，以及硬件链路响应测量。\n"""
p.write_text(s)
p = R / "PROJECT_PLAN.md"
s = p.read_text().replace("更新日期：2026-10-15", "更新日期：2026-10-20", 1)
s += """\n## 2026-10-20后续：字节Task2硬件补充\n\n仿真控制增益/速度前馈与公开真实IMU软件分析已完成，详见control_feedback_task2报告。Jetson与自有板卡下一阶段验证；保留本轮频率、数据年龄、重复样本和滤波滞后的统计方法，不用公开数据替代自有采集。\n"""
p.write_text(s)
print("project log and plan updated")
