# Windows 键盘 AMCL 与论文地图对照

[已实跑的地图、回放表与 RViz 截图](results/localization/README.md)。

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

当前移动定位入口使用静态环境；原 `04` 动态离场入口属于传感器架实验，尚未接入这个移动底盘。

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

RGB-D 使用真实标定、毫米深度和光学坐标；CG 的数据配置复制后换成录制内参及 `png_depth_scale=1000`，不套用 Replica 的深度比例。建图用轮速加已知初始坐标锚点和固定外参，不使用连续真值或 AMCL 输出。

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
