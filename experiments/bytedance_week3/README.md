# Panda接触与导纳控制

Week3的PC仿真实验：MuJoCo2.3.7动力学Panda、实际关节力矩驱动、末端球形探头对水平平面的接触。力来自mj_contactForce，不是预设信号。没有真实硬件或Jetson结果，未接入ROS调度下的接触闭环。

## 文件

- scripts/fetch_model.py：下载固定Menagerie版本，保留来源哈希和Apache2.0许可。
- scripts/contact_task.py：模型、七关节逆解、限力矩PD内环、质量阻尼/弹簧导纳。
- scripts/run_suite.py：177条预设条件，已有结果跳过；默认输出runs/contact_suite_v3。
- scripts/joint_check.py：六条重复核心条件额外记录关节参考，输出runs/joint_check_v3。
- config/suite.json：条件、虚拟质量阻尼、速度/偏移限制和力矩上限。
- assets/panda/panda_collision.xml：实际加载的简化视觉模型；原panda.xml仅保留作来源，其装饰网格未完整下载，不作为本项目加载入口。

## 运行

```bash
experiments/bytedance_week3/scripts/contact-python experiments/bytedance_week3/scripts/run_suite.py
```

也可在独立Python3.10环境安装mujoco==2.3.7、numpy==1.26.4、av后运行。渲染使用MUJOCO_GL=egl，批量无视频部分无需摄像头。下载模型需要requests。PC已有环境未替换；源包不复制Python环境。

每次8秒模拟时间、1ms步长。v3样本矩阵29列，前22列：t、法向力、目标力、实际tip_z、名义tip_z、导纳偏移、偏移速度、力矩是否裁剪、七关节位置、七关节速度。joint_check的29列再追加七关节参考。主程序也会保存额外参考列，前22列与早期记录兼容；数值控制逻辑一致。

## 解释限制

只启用探头与平面碰撞，机器人其他几何不参与碰撞。接触求解器参数2000/8000/20000并不是实物材料标定值。位置与导纳共享姿态、参考和力矩限制。噪声为每次外环更新独立的0.5N高斯噪声，延迟为额外0/20/60ms，非自有传感器测量。

代码包含必要模型和碰撞网格，保留上游许可；不含NPZ、运行日志、视频和报告图表。完整结果在本机runs和figures，报告PDF单独提供。源模型来自 https://github.com/google-deepmind/mujoco_menagerie ，固定revision见assets/panda/SOURCE.json。
