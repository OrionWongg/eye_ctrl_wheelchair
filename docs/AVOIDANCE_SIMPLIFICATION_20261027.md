# 避障简化、iPad 接口修复与连续转向（2026-10-27）

适用启动方式：

```bash
script -q -e -f -c 'WHEELCHAIR_ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/run_hardware_keyboard.sh' \
  "logs/hardware_keyboard_$(date +%Y%m%d_%H%M%S).log"
```

本次改动均未在实车上完成验证，首次上车请空载、低速、在开阔处测试。

## 1. iPad 控制器拒绝转向指令

**现象**：A/D 转向时 iPad 控制器解除 Arm，轮椅不动。

**原因**：`planner.py` 的 `Config.velocity()` 为转向加入了 ICC 弧线模型，
即前移速度 v = `icc_x` × w（`icc_x` = 0.138 m）。左右转速为 0.8 rad/s 时，转向指令变成
(0.110, ±0.8)。`whill_controller` 的 `on_navigation_command` 拒绝线速度、角速度同时超过 0.05
的指令，并在拒绝时解除 Arm。

**修复**：`twist_to_key.py` 在 |w| 超过阈值时把 `linear.x` 置 0 再转发，iPad 收到的仍是 A/D 键。
弧线模型只保留给安全门做碰撞预测，以及给仿真使用。实车按下 A/D 时会自然走出这段弧线。
测试见 `test_twist_to_key.py`。

## 2. 避障约束简化

**停车保护区域** = 车体轮廓 + `margin`（0.06 m），这是唯一的外扩。

| 删除项 | 原作用 |
|---|---|
| `model_error`、`step` 采样误差 | 再外扩约 0.04 m |
| `free_space_clearance`、`comfort_distance` | 路径搜索再外扩 0.08 m |
| `passage.py`、`entries.py`、`corridor_assist` 模式 | 通道识别、入口搜索、居中与航向限制 |
| 发给 Nav2 的走廊约束带、`PATH_TRACK_DEVIATION` | 路径横向偏差限制 |
| 碰撞最近点诊断、预测采样/包络/终点可视化 | 诊断 |

**预测只计算刹车距离**：取"当前速度"和"本次动作的指令速度"中较大的一个，
按 `linear_brake` / `angular_brake` 刹到零，检查这段扫掠区域是否碰撞。
不再叠加反应时间、租约、起步延迟和采样误差。
微转时按脉冲时间内能达到的速度计算；行驶中的滚动检查按当前速度计算。

C++ 路径规划器（discrete、smac）同样只用 margin 外扩。

**行为变化**
- 行驶中按 A/D：原地偏转 `open_area_heading_bias` 后继续直行，不再沿通道找入口。
- 前方空间不够刹停时，原因码为 `NAV_NO_FORWARD_SPACE`（原为 `PASSAGE_END_OR_BRANCH`）。
- launch 参数 `mode` 默认改为 `stop_only`，可选 `bounded_assist`。
- 参数改名（数值不变）：`corridor_correction_angle` → `path_correction_angle`，
  `corridor_heading_enter` → `path_heading_deadband`，
  `corridor_min_improvement` → `turn_min_improvement`，`entry_scan_period` → `goal_check_period`。

**RViz**（`sensors.rviz`）只保留：Nav2 footprint、Local costmap、最终硬安全边界、
三个感知过滤框、目标箭头、规划路径和实际轨迹。

## 3. 刹车实测与参数

当前 `linear_brake` = 0.5 m/s²，按 v²/(2a) 计算：

| 起始速度 | 模型预测 | 实测 |
|---|---:|---:|
| 0.39～0.44 m/s | 15.2～19.4 cm | 8.9～13.0 cm |
| 0.52～0.56 m/s | 27.0～31.4 cm | 15.0～19.3 cm |

模型比实测多估约 6～11 cm。但实测从"开始减速"算起，不含 STOP 经 BLE、iPad 链路下发的延迟，
而预测中已删去反应时间。这部分多估正好用来抵消一部分延迟，因此暂不调高 `linear_brake`。
下一步应测量"STOP 发出 → 速度开始下降"的延迟，再决定余量是否足够。

**风险**：0.534 m/s 下约 0.5 s 的链路延迟相当于约 0.27 m，大于 margin 与多估距离之和。
正面接近障碍时存在接触风险。

## 4. 日志中两类拦截的修复（`hardware_keyboard_20261027_172526.log`）

- **INVALID_TURN_ADJUST_PATH**：规划器返回的路线是
  [对齐微转, 直行, 对齐微转, 转向]，其中两段对齐微转都小于 0.003 rad。原校验只允许在直行前有一段对齐微转。
  现在 `valid_path` 会丢弃任意位置小于 0.01 rad 的转向再校验；
  真正的反向转向仍会被拒绝。
- **STOPPING_ENVELOPE_RISK**：输出已经是 STOP、车仍在惯性转动时，复查会取消整个任务。
  现在改为只记录诊断 `momentum_brake_risk`，每一次运动授权仍会先做刹车检查。

## 5. 连续转向

`Config.turn_mode`：`fixed`（原有的定角转向）或 `continuous`。
`hardware.launch.py` 在 `input_source:=keyboard` 时自动使用 `continuous`。

- 停车时按住 A/D：一直以 0.8 rad/s 原地转向，没有目标角度、超时和分段，松键即停。
- 每个周期都检查满速转向的刹车扫掠；如果会碰撞，输出 `NAV_TURN_WAIT_CLEARANCE` 并停下，
  不锁存，空间够了自动继续。
- 行驶中按 A/D、按 U 掉头、眼控输入：保持原有逻辑。
- 松键后受链路延迟影响，预计还会多转约 10°～20°。

## 6. 其他

- `run_hardware_keyboard.sh` 删除了启动时最长 30 s 的 `tf2_echo` 等待。
  里程计就绪前，安全门本身就会输出 STOP。
- `icc_x`：左右实测分别为 0.050 m 和 0.159 m，0.138 m 是折中值。
  C++ 路径规划器和路径跟踪仍按原地转建模。

## 7. 测试

`wheelchair_shared_ws/tests` 与 `wheelchair_hardware_bridge/test`：220 通过、33 失败。

33 个失败在本次改动前就已存在，来自以下改动：
- 几何阻挡不再锁存；
- 删除规划超时故障；
- 取消 ASSIST_BOUND 停车；
- `task_manager`、`direction_manager` 中的预算删除；
- `icc_x` 导致 2 个"转向后位置不变"的测试失败。

`test_filtered_cloud_input` 偶发失败，重跑即通过。

删除了只测已删除功能的测试：`test_passage.py`、`test_passage_rectangles.py`、
`test_manual_micro_turn.py`、`test_junction_regression.py`，
共用的测试辅助函数移至 `tests/nav_fixtures.py`。
