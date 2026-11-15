# 机器人运动控制与反馈系统：技术与工程报告

中文版v1.2，覆盖Panda平台与轨迹、状态反馈、接触控制、倒立摆、平面双足及三维G1控制。10章，8个技术篇章，29个阶段，4个附录。

## 阅读入口

- `build/main_zh.pdf`：完整中文版PDF。
- `main_zh.tex`、`preamble.tex`、`chapters/`：可编辑LaTeX源文件。
- `code_index/ch02`至`ch09`：29个阶段对应的实现与配置文件位置。
- `appendices/`：代码导航、素材索引、术语、历史运行命令。
- `REPOSITORY_LAYOUT.md`：未来私有代码仓库的组织建议。
- `videos/`：9段复制的视频，路径与文件名均为英文；PDF给出相对路径。

中文标题与正文采用Fandol宋体，白底黑字，不使用装饰性背景。英文与代码分别采用DejaVu Serif和DejaVu Sans Mono以完整显示数学及希腊符号；数学公式使用LaTeX数学字体。未附带中易SimSun字体。

## 资料范围与结论口径

从10份主要实验报告选入58个小节，包含75张详细实验表、92个图片文件，另补概述、摘要和综合结论。不以增加新实验的方式扩充报告。

导师月度计划的四个主题分别对应第2至6章；实际摆腿与三维人形为自主扩展。早期简化模型与后期修正版本分开，不能将它们叠加成总体成功率。真实硬件仅限已明确记录的IMU与Jetson采集，机械臂接触和双足运动全部来自物理仿真。

重点保留：原始峰值与均力、仿真与主机时间、发布与采样频率、解析动作与模型动作、条件网格与独立随机样本、正式与强扰动测试的区别。接触峰值的步长敏感性、80毫秒延迟失败、G1准静态速度和自接触局限均写入结论。

## 构建

```bash
cd /home/yd/Documents/VLA/reports/bytedance_engineering
./tools/build.sh
python3 tools/validate.py
python3 tools/audit_saved_results.py
```

支持已有XeLaTeX/latexmk；本机使用用户目录中的Tectonic0.17.0。字体和TeX宏包首次编译需要下载或安装。当前英文/代码字体也需系统安装DejaVu。Pandoc仅用于初次迁入材料，正常编译不需要。

`tools/assemble.py`及`tools/polish_tex.py`记录迁入和排版过程，生成后的.tex已有手工编辑，不属于正常构建步骤，重新运行会覆盖编辑。后续直接修改LaTeX源文。

## 文件对应关系与检查

- `outline.json`：章节、阶段、结论和原始代码目录。
- `source_coverage.json`、`source_fragments/`：源报告小节与内容摘要校验。
- `asset_manifest.json`：97个图片文件与22段视频的原路径、英文相对路径及SHA256。
- `RESULT_AUDIT.json`：读取已保存结果核对条件数、通过数和接触关键指标；没有重新运行控制实验，也不替代全量轨迹核验。
- `VIDEO_CHECK.json`、`BUILD_CHECK.json`：视频首末帧与PDF检查记录。

本次只整理报告、复制素材和渲染已保存状态，没有移动旧实现、修改历史结果、连接硬件或上传GitHub。报告不是模型代码或平台资产的完整软件包。后续英文版可以复用相同素材路径。

## v1.1 场景与动作画面补充

历史v1.1 PDF148页，在v1.0基础上新增29幅场景、动作序列与完成/终止对照图。包含Panda纯相机接触画面、有限脚掌站立、平面双足二十步、G1双侧单脚支撑与四步双视角，以及强扰动终止姿态。第1章增加三类场景总览；重复引用总览图不新增独立素材计数。

素材分为原截图、原视频抽帧、已保存状态重建和数学模型示意，图注分别标明。状态重建只调用运动学前向计算和渲染，不推进物理积分、不运行控制器；图中法向力读取原日志。SIMULATION_VISUALS.json与STATE_RENDER_MANIFEST.json记录帧号、时刻、裁剪和状态来源。

初版PDF及源文件保存在versions/main_zh_v1.0.pdf和versions/sources_v1.0.zip。正常构建仍只运行build.sh，媒体补充脚本也不属于日常编译步骤。

## v1.2 增补

新增第九章：宇树官方G1强化学习行走复现与12种工况复核，13段新视频在videos/ch09。原WBC结果保留，官方预训练策略单列。旧v1.1 PDF归档在versions/v1.1。

当前v1.2总报告157页、97张图片素材、22段视频。原始汇编工具仅适用于历史28阶段，重新生成章节会覆盖后续增补；现版请使用tools/build.sh直接编译。新增交付核对见BUILD_CHECK_V1_2.json。

## English edition

[Complete English PDF — 160 pages](english/build/main_en.pdf) · [LaTeX sources and build instructions](english/README.md). All chapters, appendices, tables and captions are translated. Original experiment figures and videos are shared with the Chinese edition.
