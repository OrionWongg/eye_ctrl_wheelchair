# 真实轮椅键盘避障模式：问题汇总

记录日期：2026-10-26。依据：[HARDWARE_KEYBOARD_NO_MOTION_20261026.md](HARDWARE_KEYBOARD_NO_MOTION_20261026.md) 的排查，以及当天之后三次硬件运行的日志（`~/.ros/log`，时长约 22 s、48 s、22 s）和只开感知节点的现场测量。

本文是问题清单，不是修复报告。截至成文，下列问题都还没有修复，也没有做过上车运动验证。

## 1. 总览

| 编号 | 问题 | 现象 | 是否本次不动的直接原因 |
| --- | --- | --- | --- |
| P1 | W 键没有进到控制程序 | 门控决策全部 `intent_enabled=false` | 是 |
| P2 | 车身周围存在未知栅格 | `INITIAL_HARD_BOUNDARY` | 否，但按键修好后会拦住起步 |
| P3 | 感知数据时效不够 | `MAP_STALE_OR_INCOMPLETE`、`SCAN_STALE_OR_INVALID` | 否，同上 |
| P4 | 安全门控周期被拖慢 | `PLANNING_TIMEOUT_FAULT` | 否，同上 |
| P5 | 运行的代码不在仓库工作区 | 改仓库代码不生效 | 影响所有修复 |

## 2. P1：按键没有到达控制程序

证据：

- 三次运行中，安全门控每一条决策都是 `intent_enabled=false, intent=0`，从未收到前进请求。
- `direction_keyboard` 日志只有退出时自动发送的停止（kind=5）。该程序每次按键都会记录“收到按键”，被拒绝时会记录“按键未发送”，这次两种都没有。

原因：点击 Arm 后，[controller_node.py:755](../projects/Ble2WChair/src/whill_controller/whill_controller/controller_node.py#L755) 调用 `self.root.focus_set()`，键盘焦点切到 GUI 窗口。导航模式下 GUI 收到 W/A/D 只显示“请在终端按 W/A/D”（[第 1350 行](../projects/Ble2WChair/src/whill_controller/whill_controller/controller_node.py#L1350)），不转发。用户要在 Arm 后再点回 `run_hardware_keyboard.sh` 的终端才能操作，这一步很容易漏掉。

候选修法：

- A（推荐）：GUI 在导航模式下直接接收 W/A/D/S 并发给方向管理节点，操作只在一个窗口完成。
- B：去掉 Arm 后的 `focus_set()`。改动最小，但仍要手动切回终端。

## 3. P2：车身周围的未知栅格

证据：

- 第一次运行的碰撞点 (-0.2, -0.552) 在车身右缘外约 25 cm；之后的运行碰撞点集中在车身 x≈0.54 m，即车头前缘（footprint 前缘 0.55 m），clearance 为 0。
- 现场 40 帧：车身加安全余量范围内每帧有 47～59 个未知格，前后左右都有；占据格只有 0～1 个。
- 车头正前方（车身坐标 x 0.55～0.70 m，y 0～+0.10 m）的格子，30 帧里有 30%～70% 的帧为未知。
- 合并最多 0.5 s 的帧后，仍有约 70% 的帧在起步检查区域内有未知格。
- 角度分辨率改粗只能减少、不能消除：0.5° 平均 57 个，1° 为 42 个，2° 为 32 个。

原因：切片高度为离地 0.30～0.40 m（`hardware_bridge.yaml` 中 `height_min: -0.203`、`height_max: -0.103`，base_link 坐标）。这个高度上近处常常没有物体，720 个角度 bin 中约 29% 没有回波。`scan_map` 的 `rasterize()` 要求相邻两个 bin 都有回波才把格子判为自由，所以车身周围大片格子无法证明是自由的，而未知格在避障中按障碍处理。

候选修法：

- 加宽切片高度，例如离地 0.10～0.90 m（待确认）。
- A：用 3D 点云中的地面回波证明射线经过的格子是自由的。改动最大，不需要把未知当安全。
- B：只在静止起步检查时，豁免车身外很窄一圈（如 10 cm）的未知格。改动小，但这一圈成为盲区。
- C：无回波的 bin 按“最远量程都畅通”处理。改动最小，但黑色或吸光障碍物会被漏检。

## 4. P3：感知数据时效不够

证据：

- MID360 点云到达时，时间戳已比当前时间旧约 102 ms（帧起始时间戳加 10 Hz 积分周期）。里程计只旧约 1 ms。
- 只开感知节点时，地图源年龄中位数 181 ms，90 分位 266 ms。
- 完整运行时，门控看到的 scan 年龄中位数 0.248～0.261 s，最大 0.62～0.72 s，正好压在 `narrow_control.yaml` 的 `sensor_age: 0.25` 上，所以约一半周期报过期。
- `scan_map` 用 50 ms 定时器轮询，平均多出约 25 ms 延迟；日志中还有 TF“extrapolation into the future”等待警告。

候选修法：

- `scan_map` 改为收到数据即处理，去掉轮询延迟。这一步没有安全代价，但单独做仍会偶尔超时。
- 再二选一：把约 102 ms 积分时间单独计入预算（例如 `sensor_age` 改为 0.35；规划器使用实测观测年龄，制动距离估算会随之变保守），或保持 0.25 并通过 Livox 驱动配置降低延迟。

## 5. P4：安全门控周期被拖慢

证据（完整运行日志的中位数 / 最大值）：

- 门控目标周期 50 ms，实测定时器间隔 116～129 ms / 224～285 ms。
- 门控内地图回调 42～44 ms / 84～91 ms，costmap 回调 51～52 ms / 85～88 ms。
- 机器负载约 6.3（12 核）。

原因：地图和 costmap 回调耗时与门控定时器在同一执行线程上互相阻塞，导致 `PLANNING_TIMEOUT_FAULT`。

关于换 CycloneDDS：本机未安装 `rmw_cyclonedds_cpp`（需 `sudo apt install ros-humble-rmw-cyclonedds-cpp`，并让感知和控制两边都设 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`）。本机进程间传输延迟只有毫秒级，主要延迟来自雷达积分时间和上面的回调耗时，换 DDS 预计解决不了 P3、P4。

## 6. P5：运行的代码不在仓库工作区

`run_hardware_keyboard.sh` 在仓库内 `wheelchair_shared_ws` 没有 `install/local_setup.bash` 时，会回退到 `/home/whill/wheelchair_shared_ws`，本次实际运行的就是后者。任何修复都要改那边的源码并重新构建，或者先把仓库工作区构建好。

## 7. 待决定事项

1. P1 按键：GUI 转发按键（A），还是只去掉抢焦点（B）？
2. P2 切片高度是否加宽到离地 0.10～0.90 m；未知格处理选 A、B 还是 C？
3. P3 时效：放宽 `sensor_age`，还是降低 Livox 延迟？
4. 是否现在换 CycloneDDS？

修复后需要用户在场进行上车验证。
