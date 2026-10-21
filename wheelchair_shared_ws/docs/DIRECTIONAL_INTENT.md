# 方向意图模式（键盘先行）

> 当前 Nav2 路径控制改造尚未完成整体验收，已按用户要求停止继续调试。最新状态、已通过场景与未解决问题见 [Nav2 改造总结](../../projects/WHEELCHAIR_NAV2_DISCRETE_PATH_CONTROL_IMPLEMENTATION.md)。下文既有通过结果属于对应历史版本，不能作为当前改造版本的全量通过证据。


该模式使用 `interaction_mode:=directional_intent`，仅用于当前准确地图的 Gazebo 仿真。旧 `gaze_joystick`、`intent_confirm` 不改变按键语义。眼动适配尚未接入，新模式启动时会拒绝 `input_source:=eye`。

## 运行

首次修改接口后需构建并重启相关消费者：

```bash
cd ~/wheelchair_shared_ws
source scripts/env.sh
colcon build --symlink-install --base-paths src
```

分别在三个终端运行：

```bash
# 左侧入口演示；省略 scene 参数时沿用办公室地图
~/wheelchair_shared_ws/scripts/run_direction_sim.sh scene:=intent_left
~/wheelchair_shared_ws/scripts/direction_keyboard.sh
~/wheelchair_shared_ws/scripts/direction_status.sh
```

W 启动直行；行驶时 A/D 提交一次左/右方向意图，U 掉头后继续巡航。停车时 A/D 分别执行一次约 20° 原地转向，U 原地掉头 180°，完成后保持停车，只有新 W 才启动直行。C 取消运行中的侧向偏好并停车建立当前方向巡航，空格/S 停止，X/Ctrl+C 停止并退出。W 在运行中的转入任务中保持原任务；在停车转向任务中则先完成停车握手，再启动直行。故障后仍需健康恢复和新的运动事件，无须按 R。

普通终端无 key-up，以 `--quiet-interval 0.6` 秒的静默间隔辨认新按键手势。连续自动重复不连发；活动同侧事件由管理器再次幂等去重。该方法不能可靠区分比静默间隔更慢的系统自动重复与真实松开再按；可按终端重复设置增大间隔。键盘健康采样反映当前前台终端状态，长期无按键不会终止健康巡航；输入进程退出/超时仍触发停车。

## 接口与隔离

| 话题 | 类型 / 职责 |
|---|---|
| `/wheelchair/keyboard/direction_event` | `DirectionEvent`：START_OR_CONTINUE=1、PREFER_LEFT=2、PREFER_RIGHT=3、CANCEL_PREFERENCE=4、STOP=5 |
| `/wheelchair/keyboard/health` | 复用 `InputHealth` v1，源 session、递增序号、采集年龄、有效性与短 TTL；不刷新事件预算 |
| `/wheelchair/intent_ack` | 复用 `IntentAck` v1，同 ID/同内容重发返回同一 ACK；ACK 不要求用户确认 |
| `/wheelchair/navigation_status` | `NavigationIntentStatus`：任务/事件、阶段、偏好、入口 ID、预算、目标、实际动作与原因 |
| `/wheelchair/emergency_stop` | 独立优先停车；清空待处理事件和底层任务 |
| `/wheelchair/decision_trace` | 每周期几何预测、导航状态、odom、compute_ms 和停止原因 |

`DirectionEvent` 使用原始时间戳及 event_id 重试，不能用重试刷新 TTL。源从启动配置选择，消息使用 volatile、小队列。管理器是新模式 `/wheelchair/intent` 的唯一生产者；旧 mux、确认任务管理器和眼动直接动作发布路径均不启动。

内部 `ControlIntent.task_kind` 3/4/5 分别表示巡航/左侧偏好/右侧偏好，command 恒为 FORWARD，**不是直接执行 FORWARD 的授权绕行**：安全层解释任务后仍自行输出经检查的 STOP/FORWARD/LEFT/RIGHT。`task_elapsed` 保留事件原始年龄，任务重建不能刷新搜索时间。旧 task_kind 1/2 保留原固定参考权限与行为。

## 规划与预算

`entries.py` 从正交直走廊的矩形墙段间提取开口，使用世界坐标端点的几何哈希作为稳定入口身份，不引用场景名称或固定门位置。按前向距离、几何 ID 选择最近可达入口。可达性预检覆盖接近、旋转、穿门及出口制动余量；执行每周期继续使用原完整车身碰撞与真实速度制动预测。

`navigation.py` 执行 CRUISE / SEEK_ENTRY / APPROACH_ENTRY / ALIGN_ENTRY / TRAVERSE / EXIT；开阔区用独立 OPEN_HEADING。锁定后不更换入口，失败停车。整车后沿通过外侧墙面并完成对正后清除偏好，沿新方向继续。唯一直行延续可保持，通道末端/未识别分叉保守停车。转入位置基于实际停稳位姿重新核验。

新模式不再在 10 秒/3 米后结束正常巡航。参数集中在 `src/wheelchair_control/config/directional.yaml`：搜索 8 秒或实际路程 3 米、前向局部范围 3 米、开阔单次偏置 20°、入口任务 20 秒、无进度检查 3 秒。健康心跳、W、相同事件和重规划不重置预算。

当前入口轨迹是保守的直线接近—固定档位转向—直线穿门基元组合，复用通道微转向修正；不能处理任意曲线走廊、任意多边形入口、全局绕路或倒车。该组合不能安全执行的入口会被跳过（锁定前）或停车（锁定后）。短时动作授权、执行器看门狗、安全余量和旧模式角度权限均保留。90°入口转向使用独立局部任务权限：终点容差仍为 2°，额外 5°预测范围同样经过完整几何和制动检查；不扩大旧确认模式的 25°权限。

## 验证与后续眼动

```bash
cd ~/wheelchair_shared_ws
source scripts/env.sh
/usr/bin/python3 -m pytest tests/test_directional.py -q
/usr/bin/python3 scripts/validate_directional.py --case all
```

Gazebo 验证使用隔离的 ROS domain 94 与独立 Gazebo partition，每个场景写入 `artifacts/directional/<case>/`，包含物理接触、实际轨迹、阶段、入口身份、停止原因和计算耗时。仅清理脚本创建的进程组。本轮最终 22 个 Gazebo 场景通过，物理接触 0；完整回归 294 项通过。详见 [测试记录](../TEST_RESULTS.md) 和 [实际轨迹图](../artifacts/directional/trajectories.png)，后续运行以实际生成的 JSON 为准。

眼动接入待办：沿用本事件接口及导航算法；稳定侧视 dwell 生成一次事件，持续侧视不连发；回正只重新准备触发；相反侧稳定注视替换；双眨眼仅启动，单眨眼不恢复；长闭眼 STOP；源时间和健康与手势分离。需同步修改 core/node/resume、配置、合成链路测试，再做真人误事件评估。

### 停车转向协议

内部 `task_kind` 7/8/9 分别表示停车左转、右转、掉头，使用同一 Nav2 路径服务和安全跟踪器。此类任务禁止绕行，必须通过原地旋转的完整碰撞检查。`NAV_TURN_COMPLETED` 结束授权并保持 STOP；任务心跳、同一事件重发和 STOP 前的旧事件均不能重新启动。运行中的任务 3/4/5/6 含义不变。

Humble 仿真时钟使用专用 ROS executor 更新，控制与传感器回调仍串行；不推算时钟、不扩大动作 TTL。

### 2026-10-16：原地受阻时的位置调整试验

原地转向可以执行时行为不变；空间不足时才尝试经现有规划器检查的“前方短距离直行＋原地转向”组合。停车任务完成后仍停车，无安全组合则终止并在键盘显示需要协助。详见 [TURN_ADJUSTMENT.md](TURN_ADJUSTMENT.md)。这不是此前已撤回的通用脱困/额外盲区限制。

## 走廊出口连续巡航（2026-10-16）

`config/directional.yaml` 新增 `nav_corridor_end_behavior: continue_straight`。
方向模式下，W 授权的巡航沿当前走廊轴线越过墙体末端，不再仅因两侧墙体
结束而触发 `PASSAGE_END_OR_BRANCH`。车尾及硬余量完全离开后解除走廊约束。
路径服务的完整车体检查、代价地图未知区域拒绝、执行时碰撞与制动检查仍生效。
该选项也允许沿当前方向直穿路口，不自动选择左/右分支；前方有障碍时仍可能停车。

需要旧的路口停车确认行为时，改为 `nav_corridor_end_behavior: stop`。
修改后须退出并重新启动整套仿真；节点不支持在线修改安全配置。
此参数属于方向巡航上层逻辑，与选择原离散规划器或 Smac 无关。
原 `gaze_joystick` / `intent_confirm` 通道控制逻辑不使用此参数。

回归入口：`scripts/validate_office_directional.py --case corridor_exit`，从默认出生点
经左向意图进入办公室走廊，验证越过 x=9.30 附近原停车点并抵达 x>10.6，随后验证 STOP。
