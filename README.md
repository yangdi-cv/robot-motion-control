# Robot Motion Control

[English engineering report (160 pages)](reports/bytedance_engineering/english/build/main_en.pdf) · [中文版报告（157页）](reports/bytedance_engineering/build/main_zh.pdf)

<!-- BEGIN INLINE DEMOS -->

### G1三维准静态四步行走

完整浮动基座、29个受限电机。四步约29秒、前进约0.210米，属于慢速准静态验证。

https://github.com/user-attachments/assets/5b7ebfc4-4973-46a6-9436-338076182817

### Panda带重力接触控制对照

相同重力、手部碰撞、材质和内环下的位置控制与优化导纳；保留同步力记录。

https://github.com/user-attachments/assets/e6d07b9c-0beb-498b-b0da-dd4f7cb8c695

[查看全部可播放视频](docs/DEMOS.md)

### 本次新增：官方G1强化学习行走

官方预训练策略复现，未训练或改权重。前进0.5米/秒时20秒前进9.23米、横移−1.41米；归零后仍踏步。与自编WBC使用不同模型，不能据此直接比较算法优劣。

### 自编准静态WBC与官方RL：原速对照

https://github.com/user-attachments/assets/7b201c9a-51cd-4f19-9111-31b51110c9c9

### 官方RL：前进0.5米/秒

https://github.com/user-attachments/assets/4c86b2b1-6092-47e1-bc0f-dea632dd1f28

### 官方RL：行走后归零（12秒后）

https://github.com/user-attachments/assets/5a57d19c-a52f-438a-baa4-4371d4aad6ee

<!-- END INLINE DEMOS -->

机器人运动控制与反馈系统：从Panda轨迹跟踪、IMU滤波与接触导纳，扩展到倒立摆平衡、平面双足连续行走和G1三维全身控制。

这是个人控制算法实现与物理仿真记录。各阶段保留模型假设、参数、失败条件和原始统计口径，不把预设条件通过数当成真实部署成功概率。

## 从哪里开始

- [完整技术工程报告（中文PDF，157页）](reports/bytedance_engineering/build/main_zh.pdf)
- [按章节阅读代码](docs/CODE_MAP.md)：8个技术篇章、29个阶段。
- [轨迹与状态反馈](experiments/control_feedback_task2/)
- [接触控制与重力优化](experiments/panda_contact_optimization/)
- [倒立摆与延迟预测](experiments/lipm_balance/)
- [连续摆腿与平面动力学](experiments/lipm_extension/)
- [统一ZMP与关节力矩接口](experiments/biped_feedback/)
- [G1三维全身控制](experiments/humanoid_control/)
- [官方G1强化学习行走复现与复核](experiments/unitree_g1_rl_baseline/)
- [官方RL全部可播放视频](docs/UNITREE_RL_DEMOS.md)

## 主要结果与边界

| 内容 | 保存结果 | 边界 |
|---|---|---|
| Panda独立运动幅度验证 | 平均误差0.864°降至0.082° | 同一模型与场景，不是硬件精度 |
| 实体IMU与Jetson | 新测量约25Hz；200Hz发布约87.52%复用 | 消息发布不增加传感器信息 |
| 优化接触条件 | 1ms原始峰值11.11N，末段均力5.007N | 步长减半峰值20.43N，仍有数值敏感性 |
| 平面二十步 | 两组完成，前进约2.323m | 六电机、平面约束、理想或限定反馈 |
| 平面反馈条件 | 94个去重条件79个通过 | 长延迟多数失败 |
| G1三维正式条件 | 52个条件51个通过 | 四步29秒、约0.210m，属于准静态 |
| 官方G1 RL复核 | 12种固定工况累计245秒，未触发终止条件 | 非随机成功率；方向漂移与停止模式仍有不足 |
| G1额外强扰动 | 8个条件2个通过 | 与正式条件分开报告 |

机器人运动结果均来自物理仿真；真实硬件记录仅限明确标注的IMU与Jetson采集。未进行企业设备代测或完整硬实时验证。动作块接口测试使用解析参考，没有运行VLA网络。

## 运行接口检查

不同物理版本采用独立环境。以下从仓库根目录执行，不会启动完整条件扫描或连接硬件。

```bash
# 三维G1：Python3.11，MuJoCo3.2.3及对应依赖
python -m pip install -r experiments/humanoid_control/requirements.txt
python scripts/check_offline.py --suite humanoid

# 平面与倒立摆：在另一个环境中使用原锁定依赖（MuJoCo2.3.7）
python -m pip install -r experiments/biped_feedback/requirements.txt
python scripts/check_offline.py --suite planar

# 接触统计函数检查，不启动Isaac
python scripts/check_offline.py --suite metrics
```

三维检查包含浮动基座、电机通道、原模型机械参数、接触力和运动学约定。平面检查包含因果测量、受限摆脚与动力学接口。测试结果保存在`repository/offline_checks_*.json`。

历史requirements.lock.txt可能包含系统ROS包，仅用于环境快照，不直接作为通用pip安装清单。完整实验入口与解释器见各阶段README。Isaac Sim5.1和ROS2 Humble单独准备，不要在同一环境混装两种MuJoCo版本。渲染另需OpenGL/EGL。批量suite是重新运行实验，不是安装检查；旧输出应保留，新实验使用新目录。

## 仓库内容

保留各阶段源码、配置、汇总结果、必要模型资源和许可证，以及报告的LaTeX、配图、英文命名视频。原始大数组、完整实验输出、虚拟环境、官方Isaac发行资产未整体上传。依赖全量轨迹的verify脚本应在生成对应轨迹后运行，不能将汇总JSON当成完整原始数据。

报告中的画面分为原截图、视频抽帧和保存状态重建，分别标注。状态重建不推进物理积分，图中接触值取自原日志，不计为新增实验。

`PROJECT_LOG.md`是整理前共享技术日志，含视觉操作方向背景；本仓库范围以章节索引和报告为准。导出文件校验见`repository/export_manifest.json`，第三方说明见[THIRD_PARTY.md](THIRD_PARTY.md)。原模型和依赖保留各自许可，未为整个仓库统一授予新的开源许可证。
