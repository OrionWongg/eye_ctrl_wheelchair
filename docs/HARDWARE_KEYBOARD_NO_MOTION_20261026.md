# 真实轮椅键盘避障模式无法运动：日志与代码分析

记录日期：2026-10-26。分析对象：当天 14:24:22～14:25:09（本机 Asia/Shanghai 时区）的一次硬件键盘控制运行。

本文是排查记录，不是修复完成报告。此次只读取日志和代码，没有启动硬件、发送运动指令或修改控制参数。分析时相关控制节点已退出。

## 1. 结论与证据边界

控制栈已经启动，但记录到的安全门控决策全部为停车。存在两个明确的安全阻塞：当前位置几何安全检查失败，以及感知数据反复过期/失效。另外，本次日志没有记录到成功发送的前进键盘事件，不能认为前进意图已经到达控制器。

可以确定的是“上游未放行运动”；不能仅凭这批日志确定真实障碍、自身回波、未知栅格分别占多大影响，也不能确定用户是否已点击 Arm、键盘窗口焦点是否正确、蓝牙最终是否能驱动轮椅。

## 2. 实际使用的工作区与启动链路

入口：[run_hardware_keyboard.sh](../scripts/run_hardware_keyboard.sh)。

脚本第 29 行起先选择仓库内 `wheelchair_shared_ws`；该目录没有 `install/local_setup.bash` 时，会回退到 `/home/whill/wheelchair_shared_ws`。本次 launch 日志中的可执行文件路径确认使用了后者。因此，不能仅修改仓库内同名代码就认为运行程序已经改变；后续修复需要核对实际安装路径、源码和构建产物。

关键代码：

- 脚本第 121 行：启动 `hardware.launch.py`，参数为 `input_source:=keyboard`、`interaction_mode:=directional_intent`。
- 脚本第 144 行：通过 `ros2 pkg prefix wheelchair_control` 查找并启动 `direction_keyboard`。
- [hardware.launch.py](/home/whill/wheelchair_shared_ws/src/wheelchair_hardware_bridge/launch/hardware.launch.py)：启动感知桥接、方向管理、局部代价地图、路径规划、安全门控、执行器及 iPad 控制节点。

数据链路：

```text
MID360 / Odin → cloud_to_scan、odom_relay → scan_map / 局部代价地图
                                                 ↓
终端键盘 → direction_manager → safety_gate → executor
                                                 ↓
                      twist_to_key → whill_controller → iPad → 轮椅
```

传感器由 `/home/whill/sensor_visualization/start_sensors.sh` 单独启动。

## 3. 日志文件与统计

日志目录：`/home/whill/.ros/log/`。

| 文件 | 用途 |
|---|---|
| `2026-10-26-14-24-22-048462-ubuntu-7357/launch.log` | 节点启动、可执行路径、退出过程 |
| `python3_7377_1792995863279.log` | 安全门控决策及几何诊断 |
| `python3_7375_1792995863336.log` | 执行器接收与停车原因 |
| `python3_7585_1792995865491.log` | 键盘事件 |
| `python3_7367_1792995863188.log` | 扫描建图、TF 等待 |
| `python3_7379_1792995863272.log` | iPad 控制器初始化 |
| `wheelchair_costmap_7369_1792995862181.log` | 代价地图配置和生命周期 |

安全门控日志中可解析的 JSON 决策共 563 条：

| reason | 次数 | 含义 |
|---|---:|---|
| `MAP_STALE_OR_INCOMPLETE` | 261 | 地图无效或其有效期限已过 |
| `INITIAL_HARD_BOUNDARY` | 220 | 当前车身位置不满足硬安全边界 |
| `SCAN_STALE_OR_INVALID` | 42 | 扫描过期或无效 |
| `EXECUTOR_FAULT` | 35 | 执行器反馈处于故障状态 |
| `EXECUTOR_STALE` | 4 | 执行器反馈超时 |
| `ODOM_STALE` | 1 | 里程计过期 |

这 563 条记录均为 `intent_enabled=false`、`intent=0`、`action=0`、`executed=0`、`requires_rearm=true`。这里统计的是日志记录次数，不是持续时间占比，也不是全部控制周期数。

## 4. 问题一：当前位置触发硬安全边界

日志多次出现：

```text
state: BLOCKED
reason: INITIAL_HARD_BOUNDARY
candidate.current_geometry.raw_clearance: 0.0
candidate.current_geometry.hard_padding: 0.09280598470419693
requires_rearm: true
```

实际配置中的车身轮廓为 `x ∈ [-0.45, 0.55]`、`y ∈ [-0.30, 0.30]`（米）。诊断表示车身与用于碰撞检查的障碍几何之间间隙为零，且还要求约 9.28 cm 的硬安全余量。因此此时即使有前进意图，安全门控也会阻止运动。

对应代码：

- [planner.py:337](/home/whill/wheelchair_shared_ws/src/wheelchair_control/wheelchair_control/planner.py:337)：`tick()` 先检查当前位置是否安全，不满足时调用 `trip('INITIAL_HARD_BOUNDARY')`。该检查在处理用户停止意图之前，因此没有前进输入时也可能报这个错误。
- [scan_map.py:22](/home/whill/wheelchair_shared_ws/src/wheelchair_hardware_bridge/wheelchair_hardware_bridge/scan_map.py:22)：`rasterize()` 构造局部占据地图。
- [nav2_transport.py:9](/home/whill/wheelchair_shared_ws/src/wheelchair_control/wheelchair_control/nav2_transport.py:9)：`occupied_rectangles()` 使用 `(cells < 0) | (cells >= 100)`，把未知格和占据格都转成禁止进入的多边形。

建图实现中与本问题有关的细节：

1. 只有有效的有限距离回波才能证明自由空间；缺失角度数据保留为未知。
2. 自由栅格要求四个角均通过相邻角度回波检查。
3. 车身清除也要求栅格四个角全部位于车身轮廓内；跨越车身边界的栅格可能仍未知。
4. 回波命中格最后写入占据，优先于自由空间和车身清除。
5. 地图分辨率为 0.1 m；车身边缘栅格及安全余量附近的未知格可能影响静止位置检查。

**待验证假设：**真实近距离障碍、车身自身回波、未知格贴近/覆盖车身边缘，或坐标变换误差都可能解释该现象。现有几何日志无法把这些原因区分开，不能直接认定为“自车误检”。需要保存同一时刻的扫描、地图、TF 和轮廓进行对照。

## 5. 问题二：地图与扫描有效期反复失效

对应代码：

- [narrow_control.yaml:12](/home/whill/wheelchair_shared_ws/src/wheelchair_control/config/narrow_control.yaml:12)：`sensor_age: 0.25`，`odom_age: 0.15`。
- [scan_map.py:80](/home/whill/wheelchair_shared_ws/src/wheelchair_hardware_bridge/wheelchair_hardware_bridge/scan_map.py:80)：检查扫描年龄，在扫描时间戳查询 TF；TF 暂不可用时等待后续定时回调。
- 同一文件发布地图时沿用扫描消息的时间戳，`out.complete=True`；“complete”表示局部范围内保守表达了未知区域，不表示所有空间都已观测。
- [nodes.py:112](/home/whill/wheelchair_shared_ws/src/wheelchair_control/wheelchair_control/nodes.py:112)：`on_map()` 校验坐标系、完整标志、时序和几何。
- [nodes.py:163](/home/whill/wheelchair_shared_ws/src/wheelchair_control/wheelchair_control/nodes.py:163)：地图无效或 `map_guard` 不再新鲜时产生 `MAP_STALE_OR_INCOMPLETE`。
- [protocol.py](/home/whill/wheelchair_shared_ws/src/wheelchair_control/wheelchair_control/protocol.py)：`StreamGuard.accept()` 根据源时间戳计算剩余有效期，而不是从接收时重新给足 250 ms。

因此，即使地图刚收到，原始扫描已经较旧，地图仍可能很快过期。采集、TF 等待、建图、通信和回调执行都消耗同一个时效预算。

建图日志确实出现 TF 外推到未来的警告，例如扫描请求时间 `1792995900.087953`，最新 TF 时间 `1792995900.084749`，相差约 3.2 ms。不过，这些警告只能证明存在等待，不能证明它们单独造成了全部超时；地图无效也可能来自其他校验分支。应记录源数据年龄、发布/接收间隔及 TF 可用延迟后再决定修复方式。

## 6. 问题三：没有前进键盘事件的日志证据

键盘日志只有一条：

```text
已发送键盘指令 1，等待控制器回执（kind=5）
```

对应代码：

- [direction_manager.py:6](/home/whill/wheelchair_shared_ws/src/wheelchair_control/wheelchair_control/direction_manager.py:6)：`START=1`，`STOP_EVENT=5`。
- [direction_input.py](/home/whill/wheelchair_shared_ws/src/wheelchair_control/wheelchair_control/direction_input.py)：`KeyEvents.key()` 把 `w` 映射为启动，空格/`s` 映射为停止；`submit()` 记录“已发送键盘指令”，回执回调记录接受或拒绝。
- 输入循环要求控制时钟有效且键盘进程属于终端前台进程组，才能发送非停止事件。
- `finally` 分支在退出时发送 `STOP_EVENT`；这与日志中停止事件出现在退出时的现象一致，但仅凭一条日志不能排除人为停止或输入有效性变化触发。

这次没有 `kind=1` 的发送记录，也没有启动接受/拒绝回执；安全门控记录的意图始终未启用。这支持“本次未观察到有效启动意图”，不等于证明用户没按键。

待核对：按键是否输入到脚本终端、终端是否处于前台、键盘事件是否送达方向管理器。点击 GUI 的 Arm 后，需要回到运行脚本的终端输入方向意图。

## 7. iPad 输出及其他日志如何理解

控制器日志显示加载了标定并启动：`mode=app_locked, hardware enabled, port=/dev/ble2wchair`。这只证明初始化走到了该位置，不证明已经 Arm 或真实输出成功。

对应 [controller_node.py:1194](../projects/Ble2WChair/src/whill_controller/whill_controller/controller_node.py)：`on_navigation_command()` 校验命令时间、坐标系和速度；`update_navigation_drive()` 要求 `keyboard_armed`，命令超时 0.2 秒或存在阻塞条件时解除 Arm。`toggle_keyboard_arm()` 位于该文件第 907 行。

目前上游始终停车，因此没有证据把“不动”首先归因于蓝牙、串口或 iPad；下游是否也有独立问题仍未验证。

代价地图还记录了 inflation radius 与车身内切半径的配置告警，但随后进入了激活状态。该告警值得复核，现有证据不足以认定它是本次不动的直接原因。

退出附近存在 SIGINT、进程退出错误及最后一条 `ODOM_STALE`；这些必须结合关停时序解释，不能把退出过程中的错误直接当成最初不动的根因。

## 8. 建议的后续定位与验收

1. **确认键盘链路：**观察按 `W` 后是否出现 `kind=1` 的发送日志及接受/拒绝回执。必要时记录 `/wheelchair/keyboard/direction_event`、`/wheelchair/keyboard/health` 和 `/wheelchair/intent_ack`。
2. **定位几何冲突：**同步保存 `/wheelchair/scan`、`/wheelchair/planning_map`、`/wheelchair/obstacle_map`、`/wheelchair/odom`、`/tf`、`/tf_static`，对照轮廓检查触发点是实际占据还是未知空间。结合传感器原始点云复核自车过滤和安装变换。
3. **测量时序：**统计扫描源时间年龄、地图更新间隔、TF 延迟、建图处理耗时和安全门控回调耗时，找到 250 ms 预算在哪一段耗尽。
4. **上游恢复后检查输出：**再确认 Arm 状态、执行器输出及 `/whill/controller/navigation` 命令是否被接受。

修复应针对确认后的原因，不应以关闭安全门控、将全部未知区域当作自由空间或简单放宽超时代替定位。

验收应同时满足：键盘启动有明确回执；车身静止位置的地图几何与现场一致；正常运行期间感知时效满足配置；安全门控可以在条件满足后重新使能；停车输入和数据中断仍能触发停车。本文尚未完成这些运行验证。
