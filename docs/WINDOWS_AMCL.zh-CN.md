# Windows 键盘 AMCL 与论文地图对照

[已实跑的地图、回放表与 RViz 截图](results/localization/README.md)。

[静态/动态场景的本次实测报告](results/static-dynamic/README.md) · [录制数据重绘对照视频](results/static-dynamic/dynamic-localization-replay.mp4) · [真实键盘/RViz 验证截图](results/static-dynamic/rviz-dynamic-keyboard.png)。

## 纯静态与动态场景的一键测试

本机准备好一次 `16_record_static_dynamic.cmd` 实验后：

- 双击 `14_static_keyboard.cmd`：打开纯静态场景，使用静态驾驶录制生成的原始地图。
- 双击 `15_dynamic_keyboard.cmd`：打开有三个运动箱体的动态场景，菜单选择 `raw`、BeautyMap、DUFOMap、`rgbd-raw`、ConceptGraphs 或 HOV-SG 地图。直接按 Enter 使用 LiDAR 原始地图。
- 两个入口均自动启动 AMCL、RViz、独立 Windows 键盘终端和 bag 录制。操作按键与下表相同。橙色半透明箱体显示独立观测到的动态物体位置，只用于观察，未输入 AMCL。

静态场景没有这三个箱体，动态场景保留相同房间和固定物体。机器人通过真实轮关节和碰撞动力学移动；箱体按目标 10 Hz 的仿真时间脚本轨迹更新。它们用于控制遮挡与地图残影，不模拟人的运动决策。没有用 set_pose 移动机器人。Gazebo 障碍物控制使用 [UserCommands](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1UserCommands.html) 和 [Transport Python 请求接口](https://gazebosim.org/api/transport/13/python.html)。

自动录制入口 `16_record_static_dynamic.cmd` 默认执行：

1. 静态、动态场景各采集一次建图 A（40 仿真秒）和另一次测试 B（26 仿真秒）。A/B 驾驶速度不同；动态 B 的箱体轨迹相位另偏移 5 秒。
2. 从每个 A 中最多取 40 个 LiDAR 与 24 个 RGB-D 关键帧，实际执行作者算法并生成地图。主实验明确向所有建图方法提供同一套仿真传感器位姿；这是一组给定位姿的建图对照，不是完整 SLAM 的评测。RGB-D 原始基线与语义方法使用同一批 RGB-D 输入。绕完整圈的 8 帧试跑曾导致 ConceptGraphs 默认多视图过滤后没有对象；该失败记录保留，正式录制采用更密的 24 帧，没有放松作者过滤阈值。
3. 每张静态地图回放同一个静态 B，每张动态地图回放同一个动态 B，各启动全新的 AMCL、重复 3 次。B 的 AMCL 输入只有激光、轮速和待测地图；B 的真值仅用于评分/显示。没有重用 A 做测试。
4. 在静态干净地图上回放同一个动态 B 一次，分辨实时动态遮挡与动态建图残影的影响。
5. 生成 `summary.csv` / `summary.json`、`REPORT.zh-CN.md`、占据地图对比图、定位误差图，以及基于录制数据重绘的动态对比 MP4。视频不是 RViz 录屏，显示第一遍回放的实际轨迹、误差和观测到的箱体位置。

每次输出到 `runs/static-dynamic-.../`。`recordings/` 保留四份原始 bag、传感器输入与节点日志；`maps-static/`、`maps-dynamic/` 保留实际算法输出；`comparison-*/` 保留逐次回放数据。窗口显示各阶段进度，`03_stop.cmd` 请求结束本次任务，已经写出的数据保留。

主要指标为去掉前 5 秒初始化后的定位 RMSE，同时保留全程 RMSE/P95/最大误差、航向误差和观测延迟。正的“误差减少”表示比本家族原始地图更好，负值表示更差。3 次重复检查同一输入的可复现性与实际波动，不代表 3 条独立路径或论文正式评测；输出一致时标准差照实为 0，不表示泛化不确定性为 0。CG/HOV 没有被改造成动态激光过滤器。

为何主实验向建图提供位姿：首轮完整绕圈的轮速位姿已有约 0.675 m / 29.2° 的 RMSE，导致纯静态墙体被叠歪。轮速建图保留在 `diagnostic-wheel-maps-*`，主实验用给定位姿隔离这一混杂因素。提供的位姿通过录制消息时间戳插值；原始 bag、扫描和 RGB-D 不改变，单独保存 `provided_*_poses.npy` 与来源哈希。后续若评测完整 SLAM，应另接建图位姿估计前端。可用 `-PoseSource wheel` 显式运行轮速建图版本，不与给定位姿结果混为一组。

超出已录真值时间覆盖的建图关键帧会明确列为不可用，并由该传感器家族的所有方法一致排除，不做位姿外推。RGB-D 帧的时间戳通过原始录制消息与已保存图像的精确匹配恢复，提供位姿的传感器外参和来源哈希独立保存。

可在 PowerShell 指定方法或实验时长：

```powershell
.\launchers\scene-keyboard.ps1 -Scenario dynamic -Method dufomap
.\launchers\scene-keyboard.ps1 -Scenario static -Method beautymap
.\launchers\benchmark-localization.ps1 -MappingDuration 60 -TestDuration 40 -Repeats 5
```

原 `10` / `12` 入口同样可通过 `.\launchers\localize.ps1 -Scenario dynamic` 打开运动障碍物，默认 SDF 地图依旧明确标为参考基线。`14` / `15` 使用传感器实际生成的地图。

## 启动与操作

本机已经安装依赖。Windows 双击 `10_amcl_keyboard.cmd`，Gazebo / ROS Jazzy / AMCL 实际在 WSL2 Linux 内核中运行，RViz 经 WSLg 显示在 Windows，键盘节点在独立 Windows Terminal 中交互。以后重建仿真侧用 `00_setup.cmd`，检查用 `00_check.cmd`。算法仍在已有 Ubuntu-22.04 作者环境离线执行。

```text
键盘 → 超时停车 → Gazebo 真实轮关节/碰撞底盘
                     ├→ 轮速里程计 → odom→base TF
                     ├→ 2D 激光 + 地图 → AMCL → map→odom TF
                     ├→ 3D 激光 / RGB-D → 建图输入
                     └→ 独立模型位姿 → 仅显示与评分
```

出现 `AMCL_READY` 后键盘窗口自动打开。将输入焦点放在**键盘窗口**，使用英文输入法：

| 按键 | 操作 |
|---|---|
| 按住 `i` / `,` | 前进 / 后退 |
| 按住 `j` / `l` | 原地左转 / 右转 |
| 按住 `u` / `o` | 前进并转弯 |
| `k` / 空格 | 停车 |
| `q` / `z` | 提高 / 降低速度 |
| 键盘窗口 `Ctrl+C` | 停车，结束录制并关闭本次 RViz |

默认 0.25 m/s、0.5 rad/s；超过 0.6 秒无新键盘指令会停车，因此连续行驶请按住方向键。也可用 `03_stop.cmd` 正常结束，等待主窗口显示结果。直接杀死主终端可能来不及写 bag 元数据。

RViz：绿色是真值轨迹/箭头，蓝色是 AMCL+里程计估计，红色是实时激光。顶部文字显示 XY 偏移、航向偏移和 AMCL 更新年龄。协方差默认不显示，原始数据仍录进 bag。

真实初始位置 (6,6,0 rad)，AMCL 固定猜测 (6.25,6.15,0.12 rad)，所以开始有偏移。这个初始化误差是明示参数。正式对比不要随意用 `2D Pose Estimate` 人工消除偏差；若使用，要记录初始化操作。

`10` 默认使用 **SDF 参考地图**，明确作为基线。机器人通过轮关节移动，没有 `set_pose` 移动机器人。场景是原 Sim2Real 资产旁的测量区域，不冒充原赛道全向车辆。

移动定位入口支持 `static` 与 `dynamic`。原 `04` 动态离场入口仍属于独立的传感器架实验；新增移动底盘动态场景由 `15` 打开。

## 录制结果

每次生成 `runs/amcl-live-<时间戳>/`：

- `bag/`：SQLite ROS bag，记录时钟、2D/3D 激光、轮速、里程计、TF、AMCL、地图、指令、RGB-D、独立真值。
- `amcl_samples.csv`：逐次滤波更新的真值误差与匹配激光的观测延迟。
- `live_error.csv`：连续预测/显示误差，与滤波更新分开。
- `resources.csv`：自有 AMCL 进程树 CPU/RSS、实时因子。
- `localization_summary.json` / `localization.png`：统计与轨迹/误差图。
- `mapping_scans.npz` / `rgbd/`：轮速位姿、3D 扫描及 RGB-D、内参、光学位姿。
- `session.json` 与组件日志：配置、地图来源、哈希和启动记录。

建图关键帧最多每秒一个，并要求约 8 cm 位移或 0.08 rad 转动，最多保存 160 个 LiDAR、80 个 RGB-D 关键帧；静止等待不持续占额度。完整 bag 不受这些关键帧数量上限限制。

## 论文方法如何接入

四篇方法不能直接替换 AMCL。当前保持同一定位器，替换离线建立的地图：

| 方法 | 实际执行 | 给 AMCL 的内容 |
|---|---|---|
| LiDAR raw | 同一批扫描与轮速位姿累积 | 2D 占据地图，LiDAR 家族基线 |
| BeautyMap | 作者代码重新过滤/恢复 | 静态点云几何投影 |
| DUFOMap | 原生绑定重新处理同一扫描 | 过滤点云几何投影 |
| RGB-D raw | 同批深度/内参/光学位姿反投影 | RGB-D 家族几何基线 |
| ConceptGraphs | 新 SAM/CLIP + 作者对象关联融合 | 实际对象点云几何投影 |
| HOV-SG | 新 SAM/CLIP + 作者特征建图 | 实际特征地图点云几何投影 |

CG/HOV 的语义特征与层级关系**没有被 AMCL 使用**；该比较不验证语义重定位。HOV-SG 当前也不运行层级导航。不同传感器家族分别比较，不把 LiDAR 和 RGB-D 输入量差异当作方法优劣。

RGB-D 使用真实标定、毫米深度和光学坐标；CG 的数据配置复制后换成录制内参及 `png_depth_scale=1000`，不套用 Replica 的深度比例。`11` 默认用轮速加已知初始坐标锚点和固定外参，不使用连续真值或 AMCL 输出；`16` 的主实验使用上述给定位姿对照协议。

双击 `11_build_localization_maps.cmd`，选择**建图录制 A**。依次执行原生方法，输出 `localization-maps-.../maps/<方法>/map.yaml` 和 `maps.json`、原生日志。默认最多 40 个 LiDAR、8 个 RGB-D 关键帧。失败方法会明确记录，不能作为可用地图出现。

统一投影为 5 cm 分辨率、原点 (-1,-1)、高度带 0.50–0.80 m，占据格扩张 1 格。自由空间来自同一家族共用的原始射线，占据格来自各方法实际输出；未知区保持未知，不用 SDF 补墙。这是明确的 3D→2D 适配，遮挡和投影会影响结果。

用 `12_amcl_choose_map.cmd` 选择地图后，可以手动驾驶演示偏移。每次手动路径不同，不能直接据此下精度结论。

## 可比较的精度和实时性实验

1. 用驾驶 A 生成地图。
2. 再用 `10` 或 `12` 驾驶，保存未参与建图的测试录制 B。
3. 双击 `13_compare_localization.cmd`，选择地图集合 A、测试 B。脚本拒绝把建图录制本身用作测试。
4. 每张地图启动全新的 AMCL，1× 回放同一 `/scan`、`/wheel_odom`、`/truth`、`/clock`；不回放旧地图、TF、AMCL 或命令，避免污染。
5. 看 `comparison.csv` 和各方法的统计。正式实验至少重复 5 次，报告均值与离散程度；Jazzy AMCL 使用自身随机采样，当前没有伪造可用的随机种子参数。

默认回放不打开 RViz；录屏时用 `.\launchers\compare-localization.ps1 -Gui`。

精度：同坐标、同时间真值匹配的位置 RMSE/P95、航向 RMSE，不用轨迹对齐消除偏差。显示用的连续预测与离散滤波输出分别记录。

实时性：“激光到达本机观察节点 → 对应位姿到达”墙钟延迟、更新间隔、CPU、RSS、实时因子。观测延迟包含通信/调度，不冒充 AMCL 内部耗时。静止时 AMCL 按运动阈值减少更新是正常现象。CPU 包含本次 `ros2 run` 包装器与 AMCL 子进程，100% 是一个 CPU 核。

**当前为离线建图 + 在线 AMCL。** CG/HOV 推理不在回放时重新执行，因此不能据此声称其 GPU 计算提高或降低了在线定位实时性。在线前端竞争、地图持续更新和语义定位需要另加实验组与定位模块。

额外压力实验可用 `.\launchers\localize.ps1 -OdomLinearScale 1.03 -OdomAngularScale 1.02` 明示加入里程计比例误差；回放会恢复测试录制的同一比例。

官方依据：[AMCL](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/)、[ROS 2 键盘节点](https://github.com/ros2/teleop_twist_keyboard)、[Gazebo DiffDrive](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)。Mesa 地图着色兼容采用固定版 Sim2Real 的分离纹理绑定办法，在运行目录覆盖资源，保留系统 ROS 文件。
