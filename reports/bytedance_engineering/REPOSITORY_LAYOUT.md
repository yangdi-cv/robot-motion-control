# 代码组织与章节阅读方式

按章节建立阅读入口，共享控制与分析模块只保留一份。当前没有迁移任何实现；原实验仍在原目录运行，28份索引位于code_index/ch02至ch08。

建议未来私有仓库结构：

```text
robot-control/
├── README.md
├── docs/
│   ├── report/                  中文LaTeX；未来增加英文入口
│   └── code_index/              与报告章节和小阶段对应
├── src/
│   ├── trajectories/            插值、连续摆脚、阶段参考
│   ├── feedback/                滤波、时间戳、延迟与数据年龄
│   ├── contact/                 导纳、接触切换与力信号处理
│   ├── dynamics/                模型接口、坐标与接触扳手
│   ├── whole_body/              任务构建、约束和QP后端
│   ├── execution/               动作块、限幅与超时
│   └── analysis/                指标和独立轨迹核验
├── experiments/
│   ├── ch02_platform/
│   ├── ch03_feedback/
│   ├── ch04_contact/
│   ├── ch05_optimization/
│   ├── ch06_lipm/
│   ├── ch07_biped/
│   └── ch08_humanoid/
├── models/                      派生模型、原始来源与许可证
├── configs/                     各冻结实验配置
├── tests/                       接口、因果性与物理量检查
├── third_party/README.md        上游模型/库来源、版本、获取方式
└── artifacts/README.md          数据、完整轨迹与视频的外部位置
```

每个实验目录保留README、对应章节、依赖入口、配置、最小运行和结果格式。不要把不同物理引擎或不同符号约定的函数仅因名字相同直接合并；先固定单位、坐标、采样时刻和接口再提取共享实现。

Panda官方USD、G1模型网格与厂家IMU解析库分别保留上游版本与许可证，不计作自编控制算法。Isaac、ROS发行环境、虚拟环境、所有原始大数组和视频不应直接塞入普通Git历史；模型运行所需的小型资产可按许可独立归档。

报告中的短路径scripts/、config/、runs/均以对应阶段原实验目录为根；code_index给出完整位置。当前没有上传GitHub，也没有将整个工程重新打包成可脱离资产独立运行的软件发行版。
