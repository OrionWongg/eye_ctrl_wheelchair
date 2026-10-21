# 窄通道改进：实现说明与验证记录

日期：2026-10-13。对应 [改进计划](WHEELCHAIR_NARROW_PASSAGE_IMPROVEMENT_PLAN.md)。

## 使用方式

旧的 `stop_only` 和 `bounded_assist` 保留。新行为使用独立的 **`corridor_assist`**，启动时自动选择 `narrow_control.yaml`；默认启动方式仍是 `stop_only`。

先关闭旧仿真，再开两个终端：

```bash
# 终端 1：办公室＋新窄通道辅助
~/wheelchair_shared_ws/scripts/run_sim.sh \
  input_source:=keyboard mode:=corridor_assist scene:=clearpath_office

# 终端 2
~/wheelchair_shared_ws/scripts/keyboard.sh
```

W 前进，A/D 明确转向，空格/S 停车。锁定后停稳按 R，再选择方向。新模式在清晰直通道内允许持续 FORWARD 期间多次有限对正；不会因为旧辅助次数用完而必然停止。

单独试标准场景，可以把场景换成 `passage_080`、`passage_090`、`passage_100`、`passage_110`、`door_090`。数字代表物理净宽（厘米），不是保证所有偏角都可通过。

可选诊断视图：

```bash
~/wheelchair_shared_ws/scripts/visualize_safety.sh
```

RViz 中绿色为仿真实体轮廓，红色为硬安全外廓，黄色为舒适范围，蓝色为候选扫掠，紫色为预测停止轮廓，红色线段为首次违规处最近点，青色为识别出的局部通行带。决策序号与 `/wheelchair/decision_trace` 对应；每次更新清除旧图层，停止发布后标记过期。

保留眼动入口，新模式也可用于相同公共控制链路：

```bash
cd /home/whill/projects/EYETLE
WHEELCHAIR_MODE=corridor_assist ./start_eye_turtle.sh --gazebo --ritnet --eyes both \
  --calibration-file /home/whill/projects/EYETLE/state/ritnet-calibration.json
```

本轮通行验收使用脚本意图，不把它表述为真人眼动驾驶验收。

## 1. 复现与几何预算

原办公室日志、障碍图、参数和卡点位置保存在 [narrow-reproduction](../wheelchair_shared_ws/artifacts/narrow-reproduction)。原接近位置约 `(2.8325,0.9422)`，朝向约 21.43°；历史停止位置约 `(2.9348,0.9737)`。历史原因包括辅助次数/时间限制，不能把它们统一归为障碍碰撞。

最终配置**没有降低原 6 cm 硬间距和 2.5 cm 模型误差**。移除的是已通过 SDF collision 外廓核对的“规划轮廓左右各比仿真实体多 5 cm”这一层。开发期间较小硬间距的实验留在 `artifacts/narrow-preliminary`，不作为最终配置的验收证据。

| 项目 | 原配置 | 新窄道配置 | 来源与处理 |
|---|---:|---:|---|
| 仿真实体宽 | 0.60 m | 0.60 m | 未缩小 collision 几何，脚踏/扶手/车轮仍保留 |
| 规划轮廓宽 | 0.70 m | 0.60 m | 新配置与已核对的仿真实体一致；不作为实机尺寸 |
| 硬间距 `margin` | 0.06 m | 0.06 m | 保留既有硬预算，没有将其任意解释成可删除的舒适项 |
| 综合模型误差 `model_error` | 0.025 m | 0.025 m | 保留既有仿真预算；没有伪造定位/跟踪各项实测置信区间 |
| 采样误差 | 约 0.00793 m | 约 0.00781 m | 按轮廓半径、线/角速度、25 ms 步长计算；速度更大时相应增加 |
| 舒适距离 | 未独立表达 | 0.15 m | 只参与偏好/评分，不决定恢复资格，不替代硬约束 |
| 时间预算 | 原有配置 | 不变 | 保留观测年龄、150 ms 动作 TTL、通信/周期延迟和制动 |

由此，完全对正的理想平行通道硬可行宽度从约 **0.886 m** 变成约 **0.786 m**，并非取消安全间距。斜入、偏心、门框突出和制动仍可能使动作不可执行。

轮廓建模、定位及控制跟踪误差目前仍由原有 25 mm 综合项覆盖建模假设，没有完成各项独立实测上界；真实机器人必须重新标定。地图量化的保守扩张已体现在地图本身，没有反向扣除或删掉门框。

只读地图审计比较了原 collision 网格投影和现有地图。在办公室 `x=4.0/4.5, y≈1.0` 的截面，物理投影净宽约 1.3535 m，地图净宽约 1.20 m。其他没有可靠双边界的截面记录为空，不推断为自由空间。详见 [几何审计](../wheelchair_shared_ws/artifacts/narrow-geometry-audit.json)。

本轮**保留原办公室栅格地图**，没有声称已经实现精确静态多边形替换或门框局部分辨率升级。

## 2. 转向能力、预测与执行

固定档位不变：前进 0.372 m/s、左转 +0.389 rad/s、右转 −0.403 rad/s，另有 STOP。没有增加倒车、混合 v/w 或高频开停调速。

真实 Gazebo 动作测量覆盖左右两侧、0.10/0.20/0.30 秒持续参数，记录实际停止角及预测包络，结果在 [微转向测量](../wheelchair_shared_ws/artifacts/result-micro_turn.json)。0.10 秒档的该次测量约 0.74～0.77°，对应保守预测包络约 4.22°；这不是实机的最小可靠转角认证，也不是每次运行都恰好转同一个角度。

实现要点：

- 搜索多个持续时间，默认 0.10/0.20/0.30 秒；超过单次角度或碰撞包络的候选直接拒绝。
- **主动停车的名义终点用于评估纠偏收益，授权失效后的最坏停车包络用于硬安全检查。** 不再把最坏过冲终点当成用户希望到达的朝向。
- 转向计时以执行器 `RUNNING` 反馈为起点，与能力测量一致；不等到较晚的 odom 速度阈值才开始计时。
- 执行器仍必须经历停稳、启动、固定动作和制动。STOP 可在计划时间前打断，重复同方向消息只续期。
- 每周期按实际 odom 重新检查。实际转角、预计制动角、用户初始方向边界和动作时长都限制继续转动。
- 连续两次修正没有产生最低有效转角，会将该修正标为不可执行，不无限重复“准备转动→停车”。若用户请求的前进仍通过全部硬约束，可延后修正并继续前进；方向权限已耗尽时不再重试该自动转向。前进也不安全则报 `PASSAGE_ACTION_UNEXECUTABLE` 停车。

默认 6°单次候选包络限制下，不是所有持续时间都可执行；较长候选可能被过滤。完整故障/延迟组合的可靠性统计还没有做完。

## 3. 通道识别与连续居中

首版使用已知静态地图，支持与地图轴对齐的直通道及有限厚度门洞。它不是任意方向通道拟合器，不处理自动转 90°弯或自动选分叉。

控制过程：

1. 在当前用户前进方向附近检查局部左右边界，估计通道方向、中心、净宽、入口和可信终点。
2. 建立固定的通道轴和中心参考；不会每周期随车体偏转重置参考。
3. 根据朝向与横向误差提前选择对正，而不只等到直接前进被拒绝。
4. 搜索转向、前进以及必要的转回平行方向候选，检查中间全部扫掠与停车过程；只发当前动作，停稳后用实际状态重新规划。
5. 同一清晰通道内允许多次有限修正。真实末端、双侧交叉或无法确认的变化需要停车，不能自己选路。短单侧开口只有在对侧墙连续、原主通道两侧边界重新出现时，才保留原轴直行。

识别到通道、建立辅助过程时，把首次观察与复杂候选搜索分开处理，避免两项重工作堆在同一个 50 ms 周期里。未识别到通道的开放区域可以边行驶边观察，不能仅因尚未找到通道就要求停车。不会为了周期性重新观察而不断中断执行器的启动。需要扩展有限观察范围时先停稳，保留原轴/中心进行扩展；地图几何改变则撤销授权。

如果尚未识别通道，保留普通用户前进的安全检查，不凭空发转向；已进入跟随的通道失去可信条件时则停止。

当前权限参数：

| 参数 | 新模式默认值 |
|---|---:|
| 相对本次用户前进方向的最大偏转 | 25° |
| 单次修正候选的最大预测转角 | 6° |
| 通道中心参考下的横向范围 | ±0.25 m，另须满足更严格的车体硬约束 |
| 每次转向修正超时 | 2 s |
| 朝向纠偏进入阈值 | 0.8°，并有反向选择滞回 |
| 最低有意义改善量 | 0.4° |

这些是独立新模式的仿真参数。旧 `bounded_assist` 的 15°/0.25 m/2 s 和重新确认规则没有被替换。

## 4. 门洞辅助

使用 `APPROACH → ALIGN/TURN → SETTLE → TRAVERSE → EXIT` 的局部过程，根据偏差可多次返回对正。有限门洞还要求侧面具有门墙特征，避免把所有短边界都直接视为可穿越门洞。

- 在门前对正，不能仅检查穿门终点。
- 进入前检查剩余完整穿越距离及最后停车，出口被堵时拒绝进入。
- 对零角速度、前向单调的直线路径，使用起终车体轮廓凸包作连续扫掠检查；仍使用同一套加速、授权、延迟、制动和误差预算。这样减少长过门预测的重复几何运算。
- 存在很小的 odom 角速度残差时，使用全部采样车体轮廓的保守凸包，不把反馈强行置零；其他运动继续使用带误差膨胀的采样预测。没有为了速度而漏掉中间门框。
- 出门后结束当前局部辅助并停车等待用户确认，不继续自动寻找下一条路线。
- STOP、失效和明确 LEFT/RIGHT 立即取消剩余局部计划；不排队执行整套穿门动作。

需要倒车、超出方向权限、无法安全微转或出口不能停车的情况，允许明确拒绝。

## 5. 诊断、恢复和计算预算

恢复资格按**硬安全几何**判断，舒适距离不足不锁死；已进入硬边界时仍停住，不实现未经授权的硬边界内脱困。

每条 `decision_trace` 包含决策序号、会话、模式、实际状态、输入年龄、`compute_ms`、执行器原因、硬预算、当前候选和通道参考。失败候选包含首次失败时刻、姿态、障碍/边界标识、最近点、原始间距、各硬余量及预计停止位置。

每周期清空旧候选诊断；RViz 图层也随决策替换，不把历史安全候选显示成当前放行结果。

规划和可选可视化构造超过 **50 ms** 时，本周期改发 `PLANNING_TIMEOUT_FAULT` 的 STOP 并锁定，不续发那条过时的非零动作。动作 TTL、传感器年龄和底盘看门狗没有放宽。开发中确实捕捉到过规划超预算并停车；它不能算通行成功。

这是超预算安全处理，不是所有系统负载下的硬实时保证。

## 6. 验证与可复现命令

最终实际 Gazebo 结果汇总在 [narrow-final-matrix.json](../wheelchair_shared_ws/artifacts/narrow-final-matrix.json)，每个案例另有 `narrow-final-*.json/.log`。几何和权限单元测试位于 `tests/test_passage.py`，超预算停车测试位于 `tests/test_ros_health.py`。

```bash
source ~/wheelchair_shared_ws/scripts/env.sh
ROS_DOMAIN_ID=97 IGN_PARTITION=wheelchair_validation ROS_LOCALHOST_ONLY=1 \
  /usr/bin/python3 ~/wheelchair_shared_ws/scripts/validate_narrow_matrix.py
```

需要只复测某几项，可加 `--cases passage090 door090 office_original`。运行时不要同时开启另一套 Gazebo，以免改变负载条件。

验收分别记录：实际前进距离、修正次数、最小实测间距、硬余量、接触、拒绝原因和最大计算耗时。不可行测试必须因明确的几何/权限原因停止，不能拿数据超时当成几何拒绝。

本轮不是计划中全部笛卡尔组合的全面验收：仍缺任意方向/复杂形状门洞、全部墙厚和突出物组合、完整延迟/丢帧/制动扰动矩阵、用户操作次数和完成率的统计对照。实时雷达/点云地图、动态行人、落差和真实底盘也不在已完成范围内。

## 7. 主要代码

- [passage.py](../wheelchair_shared_ws/src/wheelchair_control/wheelchair_control/passage.py)：通道/门洞识别和辅助过程。
- [planner.py](../wheelchair_shared_ws/src/wheelchair_control/wheelchair_control/planner.py)：硬安全预测、首次失败诊断、直线连续扫掠。
- [nodes.py](../wheelchair_shared_ws/src/wheelchair_control/wheelchair_control/nodes.py)：执行阶段反馈、时效检查和计算超时停车。
- [narrow_control.yaml](../wheelchair_shared_ws/src/wheelchair_control/config/narrow_control.yaml)：独立的新模式参数。
- [visualization.py](../wheelchair_shared_ws/src/wheelchair_control/wheelchair_control/visualization.py)：决策对应的 RViz 叠加。
- [audit_narrow_geometry.py](../wheelchair_shared_ws/scripts/audit_narrow_geometry.py)：只读几何/地图净宽审计。

## 最终验收摘要

115 项测试通过；以下为最终参数下的实际 Gazebo 单次案例，不是统计完成率。所有案例机器人接触数为 0。

| 案例 | 结果 | 前进距离 | 最大规划耗时 |
|---|---|---:|---:|
| passage080 | 通过 | 2.134 m | 12.0 ms |
| passage090 | 通过 | 2.115 m | 12.9 ms |
| passage100 | 通过 | 2.123 m | 16.5 ms |
| passage110 | 通过 | 2.122 m | 13.6 ms |
| door090 | 通过 | 1.938 m | 29.5 ms |
| hard_reject | INITIAL_HARD_BOUNDARY | — | 2.7 ms |
| exit_reject | PASSAGE_BLOCKED_COLLISION_OR_UNKNOWN | — | 8.6 ms |
| office_original | PASSAGE_END_OR_BRANCH | 1.640 m | 20.9 ms |
| legacy_b1 | 通过 | — | 见日志 |
| legacy_assist | 通过 | — | 见日志 |


## 开放区域顿挫修复

后续用户日志出现62次 PASSAGE_WAIT_OBSERVATION_STOP，与 USER_FORWARD_NO_PASSAGE 反复交替。根因是把“尚未识别通道”也当成“必须停稳才能观察”，刚起步就又下发STOP。

修复后，开放区域每周期在新鲜地图上验证并续发安全FORWARD；只有已跟踪通道的可信范围需要扩展时保留停稳要求。速度、制动、硬余量和超时参数未修改。

真实Gazebo --scenario smooth验证：5.5秒测试窗口前进1.772m（含起步），巡航约0.372m/s，起步后无多余STOP；持续前进接近障碍后正常PASSAGE_UNCERTAIN_BLOCKED并停稳，碰撞0。结果 artifacts/result-smooth.json，原日志快照 artifacts/stutter-original-trace.json。


## 侧门与人工选路

用户持续FORWARD表示沿当前方向走，不因旁边有门就自动拐入。单侧墙短缺口在对侧连续、原通道同轴同宽恢复、中心线未被障碍占据时可连接；默认识别上限1.5m。两侧同时缺失、真正尽头、较长或无法确认的缺口不自动连接。每段仍通过完整车体/制动检查，硬间距未变。

明确LEFT/RIGHT立即取消旧通道辅助，并保存离开的通道轴。之后FORWARD沿用户转出的新方向检查，不自动吸附回旧通道；必要时可先STOP再FORWARD，停转不会抹掉选路意图。若新朝向对应另一个明确门洞，可启动该门洞辅助；若仍斜对门框或出口不足，拒绝危险动作，不猜测一个目标点强行导航。

真实Gazebo side_choice测试：持续FORWARD沿主走廊经过侧门约3.03m；随后在门口由脚本明确请求RIGHT、STOP、FORWARD，进入侧门约1.45m并在出口停稳，没有转回旧走廊，碰撞0。实际办公室截图位置(4.5528,1.0372)也已验证可前进2.10m通过侧门缺口，碰撞0，硬余量仍约9.28cm。见 artifacts/result-side_choice.json 和 artifacts/result-office-side-opening.json。
