# 三维模型与求解器选择

2026-11-13。先检查本地模型和控制环境，再检索维护活跃、许可明确的模型与求解实现。没有下载策略权重或运行现成强化学习行走策略。

## 模型

采用[MuJoCo Menagerie的Unitree G1](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_g1)。原描述由Unitree公开模型衍生，G1目录为BSD-3-Clause；固定提交、全部文件SHA256和维护状态见model_source.json，原LICENSE随源码提供。

比较过H1、G1与现有自建平面模型。H1腿部关节较少，G1每腿包含髋三轴、膝、踝俯仰/滚转，更适合检验横向平衡。现有平面模型保留作历史工作，不通过在其基座加侧向弹簧冒充三维。

原G1使用位置执行器，不能直接把其ctrl当成力矩。派生模型改为29个gear=1的motor，ctrl/force上下限取原forcerange；保留质量、惯量、连杆、关节限位、原始碰撞几何、关节摩擦和六自由度freejoint。增加两个足底site用于扳手和轨迹计算。接触锥设为elliptic，与控制中的平面摩擦锥近似相对应；配置另外固定1ms积分、求解迭代和容差。派生文件与修改脚本单独保存，原文件不改。

33.341kg为模型质量，不当作某台实际G1的实测质量。原始视觉网格和简化碰撞也不能等同于完成硬件建模。原README提及的硬件迁移属于其MJX分支，不能作为本项目真机成绩。

## 控制框架

[TSID](https://github.com/stack-of-tasks/tsid)与[PlaCo](https://github.com/Rhoban/placo)用于参考任务分层、浮动基座动力学和接触约束。许可证与查询时HEAD记录在controller_references.json。继续复用本项目已验证的逆动力学结构，扩展为3D空间雅可比、SO(3)姿态误差和每脚6维接触扳手；不重新实现刚体动力学，也不整体引入另一套Pinocchio模型。

## QP后端

先沿用OSQP 1.0.4。站立、重心转移及单脚支撑可以完成，但在先导步行中出现迭代上限和数值敏感性。全局残差还混合了N、N·m和加速度，因此必须按物理量校验；不能只看一个统一数字，也不能因为模拟器会限幅就忽略控制器违反力矩界。

随后查阅[Clarabel官方QP示例](https://clarabel.org/stable/examples/py/example_qp/)和[仓库](https://github.com/oxfordcontrol/Clarabel.rs)。Clarabel 0.11.1为Apache-2.0，采用内点法，可直接处理二次目标；维护状态、许可和版本记录在solver_source.json。仅在新建humanoid-control环境中安装。

两个后端使用同一H、g、A、l、u，不切换模型或控制目标。Clarabel转换为等式零锥与不等式非负锥；OSQP保留原上下界形式。逐次检查电机力矩和物理量残差，最大迭代或不可行状态都保留为失败，不自动执行未收敛动作。对照结果应以冻结条件和同一QP输入的后端核验为准，不能以开发先导数量做成功率。

已知限制：Python每次构建问题有开销；离线P95不等于硬实时；状态和接触读数理想；支撑切换按名义时间表，尚未做在线触地重规划；当前步态为慢速准静态，未验证动态行走、地形或硬件执行器带宽。
