# 键盘避障、单雷达 costmap 与稀疏点分析总结

日期：2026-10-26。本文记录当前实现和已验证事实，不代表真实轮椅运动验收通过。

## 1. 当前状态与修改范围

本轮修复代码、配置、测试、构建产物和采样分析均保存在 `/home/whill/eye_ctrl_wheelchair`。

此前控制和感知修复仅将外部 `livox_ws`、`odin_ws` 作为驱动依赖读取；后续按用户要求统一点云 QoS 时，已修改并构建这两个驱动的点云发布/订阅，详见 [点云 QoS 调整](POINTCLOUD_QOS_20261026.md)。最早临时同步到 `/home/whill/wheelchair_shared_ws` 的键盘源码已恢复，并核对与原安装副本一致。

最后一次关闭操作已终止残留 Odin 进程并停止 ROS CLI daemon。随后检查结果：

```text
Remaining nodes: []
Remaining related processes: []
```

这表示检查当时 ROS Domain 0 无其他节点，相关驱动、转换、costmap、控制和 RViz 进程无残留；不保证之后用户新启动的程序也处于关闭状态。

## 2. 已完成的修改

### 2.1 键盘入口

原先点击 iPad GUI 的 Arm 后，键盘焦点进入 GUI，但 GUI 不转发 W/A/D。现在导航模式 GUI 通过 `/wheelchair/keyboard/gui_key` 转发带时间戳的按键，由 direction_keyboard 复用原有会话、心跳、重复键抑制、重发及回执处理。超过 250 ms 的转发消息被丢弃。

实车入口已更新为 [按住运行](HARDWARE_KEYBOARD_HOLD_TO_RUN_20261026.md)：在 iPad GUI 按住 W 启动/继续，A/D 停车时转向、行驶时选择偏好，U 掉头，C 取消偏好，空格/S 停止。最后一个运动键松开即释放摇杆并撤销任务；GUI 非停止键需要 Arm。终端只接受停止和退出，仿真事件式输入默认行为不变。

后续日志已确认 GUI 的 W 成功到达并收到回执；当时被拒绝的原因是 NOT_HEALTHY 或 LOWER_HEALTH_TIMEOUT_OR_FAULT，而不是未收到按键。

### 2.2 工作区路径

`scripts/run_hardware_keyboard.sh` 固定使用仓库内 `wheelchair_shared_ws/install`，检查控制、硬件桥接、Nav2 和 whill_controller 包路径，避免静默回退到外部旧工作区。

正常键盘入口中的测试专用 preflight 等待已移除。正常控制的启动接口仍为原命令；传感器需要先独立启动并保持运行。

### 2.3 点云车体过滤

确认原 cloud_to_scan 直接订阅原始点云，绕过已有 body_cloud_filter。已改为默认订阅：

- `/mid360_201/points_filtered`
- `/mid360_202/points_filtered`

话题可由 right_cloud_topic / left_cloud_topic 配置，无自动回退原始点云。过滤器保留原时间戳和原坐标系，转换节点继续应用 sensor→base_link TF。

为保证修改在仓库内，传感器启动所需配置和脚本复制到 `sensor_visualization/`，入口为 `scripts/start_sensors.sh`。仓库外旧启动脚本 `/home/whill/sensor_visualization/start_sensors.sh` 现已转发到仓库入口，统一使用仓库配置。

用户要求将左右过滤范围扩至各 40 cm，当前仓库配置为：

| 参数 | 范围 |
|---|---|
| 车体过滤 X | [-0.42, +0.63] m |
| 车体过滤 Y | [-0.40, +0.40] m |
| 车体过滤 Z | 不限制高度 |

cloud_to_scan 内部还保留 X=[-0.45,+0.55]、Y=[-0.30,+0.30] 的旧车体过滤，因此中间宽度内的后缘还会清除到 -0.45 m。

**已知问题：**该过滤器只判断位置，不识别物体。紧贴车身、进入上述框内的真实障碍也会被删除。规划 footprint 仍为 X=[-0.45,+0.55]、Y=[-0.30,+0.30]，与过滤框不一致。用户已报告贴身障碍未被识别，尚未通过同帧原始/过滤点云确认其消失位置，不能认为此问题已修复。

### 2.4 倒装与地面高度

实际运行 TF 已包含两台 MID360 的 roll=180°、pitch=-1°；右侧 yaw=0°、左侧 yaw=-1°。Livox 驱动外参为零，转换节点先使用 TF，再按 base_link 高度处理；不能额外翻转 Z。

用户先确认底座离地 40.3 cm，随后确认点云原点离地 35.6 cm。厂家安装图中原点 O 距底面 47 mm，水平倒装时两者关系与这两个测量值一致。

来源：[Livox 快速入门指南，印刷页 5](https://dl.djicdn.com/downloads/Livox/Mid-360/QSG/Livox_Mid-360_Quick_Start_Guide_multi.pdf)。

当前原点在 base_link 的 Z=-0.100 m，因此地面分离使用：

```text
ground_z = -0.100 - 0.356 = -0.456 m
```

已有 base_link→base_footprint TF 仍为 Z=-0.503 m。地面分离直接使用 base_link 中测得的地面参考，不依赖该旧地面网格；但 RViz 地面参考可能仍有约 4.7 cm 偏差。安装测量基准与 TF 的全面统一尚未完成。

### 2.5 地面与低矮障碍分离

原先高度切片下界约离地 25.3 cm，可能漏掉低矮物体。现在启用 ground_filter_enabled：

- 地面容差带内的点归为地面，单独发布 `/wheelchair/ground_points`。
- 高于地面容差上界、最高离地 90 cm 的回波进入障碍候选。
- 低于地面容差下界的回波也保留为阻挡候选，避免静默删除落差或异常标定证据。
- 同一角度仍取最近障碍。
- 只有地面点时发布全 inf 扫描，不伪造最大量程回波或自由空间。

当前参数分为两种：

| 参数 | 正常控制配置 | 独立单雷达测试 |
|---|---:|---:|
| ground_z | -0.456 m | -0.456 m |
| ground_tolerance | **±0.08 m** | **±0.08 m** |
| 地面 Z 区间 | [-0.536,-0.376] m | [-0.536,-0.376] m |
| obstacle_height | 0.90 m | 0.90 m |

按用户要求，正常 YAML 已同步为 8 cm，独立测试 launch 保持 8 cm。传感器脚本只启动传感器、TF、车体过滤和 RViz；地面容差在后续控制栈启动 cloud_to_scan 时生效。旧 height_min=-0.203、height_max=0.397 仅在关闭 ground_filter_enabled 时生效。

**边界：**这是固定水平地面模型，不是坡道或坑洞识别方案。地面容差会同时吞掉该高度内的真实低矮物体，不能将其当作可通行高度标准。

## 3. 正常控制与测试入口

### 正常双雷达键盘避障

终端一：

```bash
cd ~/eye_ctrl_wheelchair
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/start_sensors.sh
```

保持终端一运行，终端二：

```bash
cd ~/eye_ctrl_wheelchair
WHEELCHAIR_ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/run_hardware_keyboard.sh
```

等待 iPad 居中后 Arm，在 GUI 按住运动键运行、松键停止，按键回执在终端显示。空格/S 停止；Ctrl+C 退出控制栈。

### 独立单雷达 costmap 测试（当前推荐测试入口）

先退出已有传感器和控制程序，给另一台 MID360 断电。只保留右侧 .201：

```bash
cd ~/eye_ctrl_wheelchair
WHEELCHAIR_ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/test_single_lidar_costmap.sh 201
```

测试左侧改为 `202`。该脚本自行启动选定 MID360、Odin、安装 TF、车体过滤器和 RViz；数据/TF 就绪后启动 cloud_to_scan、odom_relay、scan_map、Nav2 costmap 和生命周期管理。

不启动键盘、规划器、执行器、twist_to_key 或 iPad 控制器。按 Ctrl+C 关闭本次测试。当前默认地面容差为 ±8 cm。

旧的 `run_hardware_keyboard.sh lidar_mode:=201` 说明已由独立测试入口取代。兼容参数仍保留单雷达禁用物理输出的保护，但不要再把它当成推荐测试流程。

## 4. odom 缺失与启动检查

一次错误日志的明确时序为：Odin 18:02:31 退出，costmap 18:02:59 启动，导致 odom 不存在。另一次是传感器只启动 .201，控制命令使用默认 both，等待不存在的 .202。

preflight 检查选定 filtered 点云、Odin 新鲜里程计和 TF。Odin 安装外参查询已与实际 OdomRelay 一致，使用最新固定安装变换；动态 odom→base_link 仍查询扫描时刻。此前按里程计精确时间查询固定安装变换导致间歇误报，该问题已修正。

preflight 现在只由独立测试入口使用，不阻塞正常键盘入口。不发布假的静态 odom 来掩盖缺失的定位数据。

曾发现外部旧工作区与仓库内同时发布 scan/planning_map。不同地图会话混用会导致门控拒收，必须避免同时运行两套感知。此前残留进程已关闭。

## 5. 稀疏点 Z 分析

### 采样定义

最新一组分析使用右侧 .201 的 filtered 点云，60 帧，转换到 base_link，水平范围为：

```text
sqrt(X² + Y²) ≤ 2.0 m
```

运行时参数已确认 ground_z=-0.456、ground_tolerance=0.08。

“稀疏候选”定义为单帧中 8 cm 球形邻域内不足 4 个点（包括自身）。这是密度规则，不是真实噪声标签。所有数量为跨帧回波数，不是独立物体数。

### 结果

2 m 内共有 459176 条回波，6242 条稀疏候选：

| 相对配置地面高度 | 稀疏候选回波数 |
|---|---:|
| 地面上下 8 cm | 1583 |
| 8～10 cm | 29 |
| 10～20 cm | 2888 |
| 20～50 cm | 1567 |
| 50～90 cm | 175 |

重建二维扫描后，仍保留 717 条稀疏候选，其中 8～10 cm 仅 15 条，其余约 98% 高于 10 cm。地面附近高度主峰约 Z=-0.4575 m，与配置地面相差约 1.5 mm。

这组数据不支持“所有稀疏障碍都源于地面阈值偏紧”的判断。单纯继续提高地面容差不能有针对性地消除剩余候选，且会降低低矮障碍检出能力。

### BEV 图与候选 A

[BEV 总览与局部放大](../artifacts/noise_radius2_20261026/bev_candidates.png)：上方为车头、左侧为轮椅左侧。灰色为全部回波，橙色为稀疏候选，红色为进入二维扫描的稀疏候选，蓝框为当前车体过滤框。

候选 A：

```text
X = -0.150 m
Y = +0.407 m
Z = -0.265 m
相对配置地面高度约 19.1 cm
```

A 在过滤框左边缘外约 7 mm。尚未通过现场实物确认它属于轮子、车体、外部物体还是噪声。60 帧叠加图可显示位置，但不能替代单帧密度判断或物体标注。

相关产物：

- [Z 与阈值分布图](../artifacts/noise_radius2_20261026/height_distribution.png)
- [统计 JSON](../artifacts/noise_radius2_20261026/report.json)
- [候选 A 坐标](../artifacts/noise_radius2_20261026/bev_candidate_a.json)
- `artifacts/noise_radius2_20261026/points.npz`、`analyze.py`、`summarize.py`、`plot_bev.py`

此前车体框外 60 cm、40 帧的分析位于 `artifacts/noise_z_20261026/`，范围和地面容差不同，不能直接与本次数字比较。

## 6. 验证情况

已完成相关阶段的验证：

- GUI 按键转发、ROS 方向事件、停止及超时保护测试。
- 倒装 TF、双传感器新鲜度、地面分离、低矮障碍、filtered 话题接入测试。
- 单雷达转换、过期数据拒绝、单雷达禁用物理输出条件验证。
- 硬件桥接测试在加入启动就绪检查后共 37 项通过；随后固定安装 TF 查询修复的 5 项启动检查测试通过。
- 独立测试 launch 参数解析与构建通过；传感器启动失败时不启动 costmap 的流程验证通过。
- 现场只读采样、运行参数读取和高度分布绘图完成。

没有据此宣称真实载人运动、全部避障场景或稀疏点识别已验收。

## 7. 尚未解决与建议下一步

1. **先标注候选点，而不是继续扩大过滤范围。** 对照 BEV 中 A 点及其他红点与现场实物位置，必要时采集原始点云、filtered 点云和图像。
2. **查贴身障碍在哪一级消失。** 原始有而 filtered 无，属于车体 mask 问题；filtered 有而 scan 无，再检查高度/地面/距离过滤；scan 有而地图无，再检查建图。
3. **统一车身尺寸。** 当前全高度矩形过滤会产生贴身盲区，且与规划 footprint 不一致；应依据真实几何改为分部件、有高度范围的自车过滤。
4. **不要把稀疏直接等同于噪声。** 细杆、物体边缘、低反射表面也可能满足密度规则。当前尚未在运行链路中加入自动离群点删除。
5. **互扰未确认。** 单雷达断电对照入口已提供，但缺少同一场景下系统的通电/断电重复对照证据。停止订阅不等于停止发射，必须给另一台断电。
6. **地面 TF 与安装基准仍需统一。** 当前 ground_z 与旧 base_footprint 不一致；需避免只改一条 TF 导致其他外参偏移。
7. **历史感知时效、地图几何阻塞不能视为全部解决。** 用户已报告 freespace 修复；本轮没有对其所有行为重新验收，也未把未知区全部改为可通行。
8. 正常配置与独立测试现统一为 8 cm；需按此设置进行低矮实物障碍与停止行为验证，尚未完成验收。

## 8. 后续：车身附近回波持续时间分析

已完成 60 秒静态点云采样和滑动窗口离线对照，详见 [车身附近稀疏候选持续时间与滑动窗口评估总结](NEAR_BODY_NOISE_PERSISTENCE_SUMMARY_20261026.md)。尚未加入在线时间过滤，也未证明这些候选是历史安全门控阻挡的来源。
