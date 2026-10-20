# 方向意图共享驾驶与避障控制实施记录

> 当前 Nav2 路径控制改造尚未完成整体验收，已按用户要求停止继续调试。最新状态、已通过场景与未解决问题见 [Nav2 改造总结](WHEELCHAIR_NAV2_DISCRETE_PATH_CONTROL_IMPLEMENTATION.md)。下文既有通过结果属于对应历史版本，不能作为当前改造版本的全量通过证据。


更新日期：2026-10-14。当前依据为 [方向意图驱动的共享驾驶：键盘先行、眼动后接实施方案](WHEELCHAIR_DIRECTIONAL_INTENT_KEYBOARD_FIRST_PLAN.md)。范围为软件与 Gazebo，不连接真实轮椅、不修改 ESP32 固件。

**状态：已新增 `directional_intent` 的键盘输入、事件管理、地图入口识别及转入控制代码；键盘仿真验收结果见下方实施记录。眼动适配尚未接入。** 文末保留的 `intent_confirm` 实施记录、运行命令和实验结果仅对应旧版，不能作为新需求的完成证据。新需求与旧记录冲突时，以新需求为准。

## 本轮代码实施（2026-10-14）

工作空间：`/home/whill/wheelchair_shared_ws`。新增独立模式 `interaction_mode:=directional_intent`，保留旧模式；五个 ROS 包已重新构建。

- `direction_input.py`：一次方向事件与独立健康心跳，终端静默间隔去重，STOP/退出独立停车路径。
- `direction_manager.py`：启动授权、同方向/W 幂等、事件重放防护、反向替换及 C 的停稳握手；故障或 STOP 后需新 W。
- `entries.py`：正交直走廊的墙体开口提取、稳定几何 ID、最近可达入口选择、整车接近/旋转/穿门扫掠和出口制动空间预检。
- `navigation.py`：搜索、接近、对正、穿门、退出及新方向巡航；开阔单次 20°偏置；锁定入口失败停车，不换门。
- `DirectionEvent.msg`、`NavigationIntentStatus.msg`：独立事件和被动状态；新配置 `directional.yaml` 保留 8 秒/3 米搜索预算及 20 秒入口任务上限。
- 接入 `planner.py`、`nodes.py`、`task_node.py`、launch 和执行器既有输出链路。复用原固定档位、完整轮廓、动作 TTL 和看门狗。

新入口转向将**最终对正容差（2°）**与**局部预测允许的额外角度范围（5°）**分开，额外范围同样参与完整几何/制动检查，避免沿用旧控制器时临近 90°反复短脉冲停车。旧模式的 25°授权不变，车身安全余量不变。

```bash
# 仿真、键盘及状态各用一个终端
~/wheelchair_shared_ws/scripts/run_direction_sim.sh scene:=intent_left
~/wheelchair_shared_ws/scripts/direction_keyboard.sh
~/wheelchair_shared_ws/scripts/direction_status.sh
```

W 启动，A/D 表达入口偏好，空格/S 停止，C 取消偏好，X 退出；不再使用 Enter 逐次确认或 R 恢复。省略场景参数时沿用办公室地图。完整说明见 [新模式 README](../wheelchair_shared_ws/docs/DIRECTIONAL_INTENT.md)。

当前支持保守的直线接近—定点旋转—穿门基元组合及既有通道微修正；不支持任意曲线走廊、全局绕路或倒车。不满足该轨迹组合的入口在锁定前跳过，锁定后失败停车。眼动输入接入会在 launch 中明确拒绝，后续按需求第 10 节接入同一事件协议。

验证脚本为 `tests/test_directional.py` 和 `scripts/validate_directional.py`，生成的 Gazebo 证据见 [场景结果](../wheelchair_shared_ws/artifacts/directional/result.json)。最终验证为 **294 项单元/接口回归通过，22 个 Gazebo 场景通过，物理接触 0**。开发中发现并修正了转向补偿超时及实际停稳复核重复叠加位置容差的问题。详见 [测试记录](../wheelchair_shared_ws/TEST_RESULTS.md) 和 [实际轨迹图](../wheelchair_shared_ws/artifacts/directional/trajectories.png)。

## 1. 新版避障控制目标

用户只表达启动、停止和左/右方向偏好；系统自动决定入口、转向位置及安全动作。先用键盘验证整个规划链路，后续眼动接入同一事件接口，不维护第二套入口选择算法。

一次左侧事件应产生以下行为：

```text
在前方有限范围内寻找最近可达左侧入口
  → 锁定入口身份
  → 沿当前通道接近
  → 对正并转入
  → 整车通过门洞并完成对正
  → 消耗本次事件，沿新通道/朝向安全巡航
```

不能将左侧事件转换成当前位置正左方目标点或立即下发 LEFT，也不能等直行将要碰撞才寻找入口。沿旧走廊接近已选入口属于执行侧向意图。目标失效时停车，不自动换门；通过后不得继续使用本次偏好选择第二个分支。

仿真假设几何、感知与 odom 准确，但固定档位、惯性、通信延迟和制动仍参与规划。保留 STOP/FORWARD/LEFT/RIGHT 动作集合，不增加倒车、横移或任意连续速度控制。

## 2. 交互及授权规则变更

已新增独立 `directional_intent` 模式，保留旧 `gaze_joystick` 和 `intent_confirm` 的原有语义用于对照。启动参数为 `interaction_mode:=directional_intent`；运行脚本见上方实施记录。

| 输入 | 新模式行为 |
|---|---|
| W / START_OR_CONTINUE | 停止后启动；运行中保持现有路线、侧向任务和预算 |
| A / PREFER_LEFT | 运行时提交一次左侧事件；停止时不启动车辆 |
| D / PREFER_RIGHT | 同上，右侧 |
| C / CANCEL_PREFERENCE | 开发测试用：取消侧向任务，安全停稳后建立当前方向巡航；首版眼动不映射 |
| 空格、S / STOP | 优先清除全部任务、偏好及待执行段，并停车 |
| X、Ctrl+C / EXIT | 发布 STOP 后退出，执行端看门狗作为后备 |

不再要求方向候选提示、Enter 或双眨眼逐次确认。回正只准备下一次眼动触发，不取消转入、不重建前方目标。双眨眼可保留为后续眼动模式的明确启动动作，单眨眼不能恢复旧任务。

同方向活动/待处理事件幂等，不能重选入口、刷新预算或重复偏转。普通终端没有 key-up，需配置无输入间隔进行再次触发，并明确其限制；任务完成本身不能让持续按住的按键重新提交。眼动同侧再次触发需先稳定回正，持续侧视不能连发。相反方向稳定事件可替换当前任务，先撤销旧计划并完成必要停车，再从当前可行空间重新解释。

STOP、失效或故障后清除旧授权，只有健康恢复之后的新 W/启动手势才能启动。故障期间的启动请求不能永久排队。软件自动完成 disabled STOP → ready 内部握手，不要求用户再按 R；心跳、重复事件、传感器恢复不能充当启动动作。

## 3. 通路解释与目标锁定

### 3.1 巡航及搜索

- 无侧向任务时，走廊中沿当前通道居中前进，开阔区域保持锁定朝向；不因出现侧门自动进入。
- 路口有唯一明确直行延续时可继续；死胡同、无唯一延续的分叉或授权方向内无安全动作时停车，不自行选左右。
- 左/右事件搜索当前行进方向前方、对应侧、有限距离内的入口。按沿当前通道的前向距离选择最近可达入口，距离相近时按稳定几何 ID 排序。
- 可达性同时要求入口几何可通行、完整转入动作可执行、入口内有足够已知安全空间。锁定前可跳过不可达入口，不选需要回退的身后入口。
- 暂无入口时沿原安全路线前进并保留短期偏好；搜索时间或累计路程任一耗尽即清除偏好。只有巡航仍安全才继续，过期不能强迫前进。

### 3.2 锁定与完成

选中后固定入口 ID、所在侧、几何范围和参考方向。重规划只能改善该入口的局部轨迹，不能逐帧更换目标。入口丢失、路径变得不可执行或任务超时均停车，不能自动重开搜索预算。

局部目标位姿固定在一致的 odom/map 坐标系中，依据入口宽度、墙厚、扫掠范围和底盘能力生成接近、对正、穿门及出口参考。不能每周期随车头重建目标，也不能仅用车体中心抵达门中心判断完成。须确认整车通过门洞且已对正，才能消耗事件并建立新巡航参考。入口后空间不足、方向不明或马上需要再次选路时停车。

### 3.3 开阔区域

无走廊/入口约束时，一次侧向事件解释为相对当前参考方向的一次有限朝向偏置，仿真初值 ±20°；完成后沿新方向继续。不能选择身后目标、持续累计转角，或在“开阔偏转”和“进入侧门”之间逐帧切换。该行为不沿用旧 `ADJUST_HEADING` 的“15°转完即停、不自动前进”语义。

## 4. 避障规划与执行状态机

```text
STOPPED → CRUISE → SEEK_ENTRY → APPROACH_ENTRY → ALIGN_ENTRY
                                                  ↓
CRUISE ← EXIT ← TRAVERSE ←──────────────────────────┘

SEEK_ENTRY 预算耗尽 → 清除偏好 → 安全 CRUISE 或停车
开阔侧向事件 → 独立有限朝向偏置子任务 → 新方向 CRUISE
任意状态 STOP/故障 → 清除授权与待执行段 → STOPPED/BLOCKED
```

控制链路为：方向事件与独立健康心跳 → 意图管理 → 通路解释与任务状态机 → 局部目标位姿和固定档位运动基元规划 → 完整轮廓、延迟及制动安全检查 → 执行器。

避障层必须满足：

1. 使用现有固定档位动力学及真实执行反馈，搜索实际支持的动作、持续时间和必要停车转换，不把连续速度结果简单量化成按键。
2. 检查整个车身轮廓、旋转扫掠和完整轨迹，包含动作中断、授权过期及最终制动后的停车包络；门中心无碰撞不足以证明可通过。
3. 原地旋转空间不足时可规划入口外的提前调整；若当前动作集合内仍不可执行，则正确停车，不能加入倒车或减小安全余量强行通过。
4. 评分考虑目标进度、安全间距、旋转/切换次数和相对旧轨迹的偏离。有安全且有效的旧轨迹时，不为微小评分改善反复换计划。
5. 中间参考点不是强制停车点，同方向段连续执行；需要旋转时按底盘约束停稳再切换。位姿容差与最小可控动作及真实制动相匹配，防止来回修正。
6. 局部绕行限制在当前目标及其局部可达空间内，不演变成整栋建筑的自动寻路。安全检查每周期生效，不能以规划正在计算为由延长动作授权。

## 5. 新模式参数与预算

以下初值已统一置于 `src/wheelchair_control/config/directional.yaml`，由安全规划层读取。

| 参数 | 初值 | 计时/计程规则 |
|---|---:|---|
| intent_search_distance | 3.0 m | 从事件产生点起，沿实际巡航累计的最大搜索路程 |
| intent_search_timeout | 8.0 s | 从事件产生起算的无入口偏好保留上限 |
| entry_lookahead | 3.0 m | 当前局部环境中的前向入口搜索范围 |
| open_area_heading_bias | 20° | 一次开阔区域事件的有限朝向偏置 |
| maneuver_timeout | 20 s | 入口锁定后的独立任务执行上限 |
| progress_timeout | 3 s | 排除合理停车转换阶段后的持续无有效进度检查初值 |

心跳、重复事件、重规划和临时停顿均不能清零预算。锁定入口结束搜索阶段，进入独立任务预算。正常巡航取消旧版每 10 s/3 m 结束并重新确认的规则，但仍要求连续健康心跳、真实执行反馈和每周期安全检查。

保留现有动作短 TTL、独立底盘看门狗、延迟/制动模型与安全余量。不能将搜索的 3 m 与旧任务总授权的 3 m 混为同一约束，不能通过放宽动作 TTL 实现持续巡航。

## 6. 消息职责和代码改造落点

新增 `DirectionEvent` 或明确等价的新适配协议，字段包括 header、source_session_id、event_id、kind、ttl_ms。事件与底层 LEFT/RIGHT 动作分离。相同事件重发返回同一 ACK，启动/方向事件必须新鲜，过期、乱序或旧 session 不产生动作。STOP 走独立优先取消路径，不等待候选或规划完成。

`InputHealth` 独立发布源 session、序号、时间、valid 和 TTL，只证明链路健康，不提交事件或延长任务预算。`NavigationIntentStatus` 记录 event_id、task_id、偏好、入口 ID、阶段、剩余预算、局部目标、实际动作及原因，仅供被动显示/诊断。

输入源在启动时明确选择 keyboard 或 eye，不自动切换。进程重启、会话切换和时钟异常清除旧授权并默认停止。运动消息使用 volatile、小队列，不由历史持久消息恢复运动。

需求职责与代码落点如下。键盘阶段已通过新增独立模块接入，具体完成项见上方实施记录；眼动行仍为待办：

| 位置（相对 `wheelchair_shared_ws`） | 必须调整的职责 |
|---|---|
| `src/wheelchair_interfaces/msg/` | 新增方向事件、导航状态及必要协议字段，明确与旧确认协议的区别 |
| `src/wheelchair_control/wheelchair_control/inputs.py`、`task_input.py` | 新模式键盘一次事件与独立心跳；保留旧模式按键语义；STOP 清空待发送事件 |
| `src/wheelchair_control/wheelchair_control/task_manager.py`、`task_node.py` | 分离巡航授权、搜索预算与入口任务预算；管理事件幂等、替换、锁定及内部恢复握手 |
| `src/wheelchair_control/wheelchair_control/passage.py` | 新增侧向开口识别和稳定几何身份；新区分唯一直行延续与需新意图的分叉；调整过门后行为 |
| `src/wheelchair_control/wheelchair_control/task_scope.py`、`planner.py` | 引入局部目标位姿及转入子任务；旧固定参考角度范围不能直接套到完整转入任务，也不能全局删除原权限检查 |
| `src/wheelchair_control/wheelchair_control/nodes.py`、`executor.py` | 接入新目标层的诊断/反馈，保留最终碰撞检查、实际停稳判断、短 TTL 和独立执行时限 |
| `src/wheelchair_control/config/`、`launch/shared.launch.py`、`scripts/`、`README.md` | 独立模式和集中参数，交付可运行的一键仿真、键盘及状态查看命令 |
| `src/eye_shared_control/eye_shared_control/core.py`、`node.py`、`resume.py` 及确认模式适配层 | 键盘验收后再接眼动：运动会话与方向触发分离；稳定侧视一次提交、回正不取消、单眨眼不启动 |

后续眼动的 dwell=250 ms、enter=0.25、leave=0.15、长闭眼约 650 ms 可作为实验初值。健康短眨眼/扫视不取消已提交任务；invalid、源帧过旧和超时必须停车，定时心跳不能掩盖旧帧。配置、合成输入和状态机测试同步调整，真人误事件率需另行测量。

## 7. 实施与验收要求

实施顺序：接口与键盘意图管理 → 入口识别/锁定 → 转入轨迹及连续巡航 → 键盘集成验收 → 眼动适配。入口从准确仿真地图提取，不写死办公室门的位置或按键触发坐标；至少覆盖不同位置、宽度及多个入口。

必须覆盖需求第 12 节的全部键盘场景：提前左转、入口附近左转、两个同侧入口、最近入口不可达、无入口预算过期、身后入口排除、开阔单次偏转、重复 A、A 后 D、转入期间 W、任意阶段 STOP、过门后继续、分叉无新意图、节点退出/超时。任务/规划/输入/执行器分别中断时均验证最终停车。

每次记录事件和任务 ID、解释类型、锁定入口、状态转移、参考位姿、实际动作、compute_ms、轨迹、停止原因和最低安全间距。对照旧模式统计入口选择、通过率、接触次数、完成时间、动作切换/停车次数和人为操作次数；不能只以“没有碰撞”证明成功，也不能把正确拒绝不可达场景当作通行失败。

交付时补齐代码、参数、可复现运行命令、测试脚本和结果、眼动改造清单及实际完成状态。**以下旧版测试结果不覆盖上述新验收，不能直接沿用为新模式通过。**

## 8. 旧版 `intent_confirm` 实施记录（历史基线）

以下保留自 [上一版实施方案](WHEELCHAIR_INTENT_SHARED_CONTROL_IMPLEMENTATION_PLAN.md)，描述旧版已实现行为。逐次确认、15°转完停车、10 s/3 m 总任务预算、出口重新选方向等规定仅适用于旧模式，不适用于上述新模式。

### 旧版已实现行为

- `intent_confirm` 与旧 `gaze_joystick` 分离。前者不再将视线直接写成运动动作：原始预览即时反馈，250 ms 稳定后可确认，双眨眼冻结并提交同一候选；单眨眼无启动/恢复权限。确认后候选被消耗，第三次眨眼不能复用。
- 首次闭眼冻结候选 ID、版本、参数、上下文和原有效期；新稳定选择、上下文变化、失效、长闭眼、期限耗尽都撤销未提交事务。已提交事件原 ID/原时间戳重试，不创建新事件伪装续期。
- `CONTINUE` 锁定确认时参考，仅首次识别可用通道；复用居中、门前对正和过门。开阔区保持原朝向，不在途中任意附着新侧通道；分叉与通道终点停车。
- `ADJUST_HEADING` 先撤销旧任务并输出禁用 STOP，等待新的实际停稳反馈后锁定绝对目标。使用 odom、提前制动及已验证的 0.10/0.20/0.30 s 固定档位脉冲；续包不能延长执行器单次上限。实际姿态及完整预测轨迹都检查原角度权限，达到目标容差且停稳才完成，不自动前进。
- 默认有限转角 15°、完成容差 2°、外层角度权限 25°。25°是权限上限，不是所有 25°请求都保证可执行；靠近硬边界无法执行最小动作时明确阻挡。目标未达到而时间预算用尽，会报 BLOCKED，不能误报 COMPLETED。
- 任务默认 10 s/累计实际路径 3 m，按任一上限结束；预留最坏固定档位制动距离、观测年龄、管理器与动作授权过期时间和检查周期，因而通常提前开始停车。重规划、子动作切换、健康心跳与同一事件重发都不刷新预算。
- CANCEL、源 invalid/超时、下层故障、时钟暂停/回跳、会话变化均撤销授权与待处理事件。替换最多保留一个最新确认，过期不能偷偷延长等待。停稳及健康恢复后仍需新确认。

### 旧版消息、接线与参数

新增 CandidateStatus、IntentEvent、IntentAck、InputHealth、TaskStatus，任务消息协议版本为 1；新增 EyeSample 供带时间戳检测源使用。详情见 [字段协议](../wheelchair_shared_ws/docs/INTENT_PROTOCOL_V1.md)。

新模式首版要求 corridor_assist，stop_only/bounded_assist 仍用于 gaze_joystick 对照；不支持的组合在启动前拒绝。

新模式只有任务管理器向 `/wheelchair/intent` 写 ControlIntent 适配流；legacy mux 不启动，眼动节点也不发布旧 eye_intent。选择输入源后仅订阅对应 `/wheelchair/{eye|keyboard}/{candidate,event,health}`。task_id 包含管理器、输入源会话和事件号；旧任务反馈不能完成新任务。

ControlIntent 增加任务标识、类型和固定参考/目标；ControlStatus 增加 task_id；DiscreteAction 的 max_run_ms 由执行器强制约束。升级需重新构建消息、控制、眼动包，并同步重启消费者。系统仍只有 STOP/FORWARD/LEFT/RIGHT 固定速度输出，安全余量与短期动作看门狗没有放宽。

| 参数 | 默认值 | 位置与含义 |
|---|---:|---|
| dwell / enter / leave | 0.25 s / 0.25 / 0.15 | eye.yaml，预览稳定与方向滞回 |
| long_close | 0.65 s | eye.yaml，停止识别阈值，非瞬时急停 |
| candidate_ttl / event_ttl | 1.5 s / 1.5 s | 冻结/提交后原期限不因重试延长 |
| relative_yaw | 15° | eye.yaml，候选有限转角 |
| task_max_time / task_max_distance | 10 s / 3 m | launch 参数；tasks.yaml 对应 max_time/max_distance |
| task_heading_tolerance / task_heading_limit | 2° / 25° | narrow_control.yaml，目标容差与硬权限 |
| action_ttl | 0.15 s | 原动作短授权；不是任务生命期 |

任务管理器读取同一控制配置中的速度、制动和延迟参数；不能为了加快表现放宽 TTL 或删掉停稳等待。运行中不允许直接更改交互模式，停车确认停稳后重启切换。

### 旧版启动（不是新方向意图模式）

```bash
# 新模式：键盘也是候选选择与确认，不是旧动作键盘
~/wheelchair_shared_ws/scripts/run_sim.sh interaction_mode:=intent_confirm input_source:=keyboard
~/wheelchair_shared_ws/scripts/intent_keyboard.sh
# W/A/D 选择，Enter 确认，空格/S 取消，Q 退出。
```

```bash
# 新眼动模式，真实摄像头输入，输出仅至 Gazebo
cd ~/projects/EYETLE
WHEELCHAIR_INTERACTION_MODE=intent_confirm WHEELCHAIR_MODE=corridor_assist \
./start_eye_turtle.sh --gazebo --ritnet --eyes both \
  --calibration-file ~/projects/EYETLE/state/ritnet-calibration.json
```

```bash
# 旧交互对照：单眨眼恢复规则仅属于此模式
WHEELCHAIR_INTERACTION_MODE=gaze_joystick WHEELCHAIR_MODE=corridor_assist \
./start_eye_turtle.sh --gazebo --ritnet --eyes both \
  --calibration-file ~/projects/EYETLE/state/ritnet-calibration.json
```

RITnet 新模式显示预览、实际稳定候选及角度、ACK、任务状态、实际动作、原因。候选是固定三方向语义，未实现注视现实物体或三维选门。

### 旧版自动验收与证据

完整回归与最终结果以 [实验记录](../wheelchair_shared_ws/TEST_RESULTS.md) 为准。可复现命令：

```bash
cd ~/wheelchair_shared_ws
source scripts/env.sh
ROS_DOMAIN_ID=96 IGN_PARTITION=wheelchair_intent_task_validation ROS_LOCALHOST_ONLY=1 \
  /usr/bin/python3 scripts/validate_intent_tasks.py --case all
ROS_DOMAIN_ID=98 IGN_PARTITION=wheelchair_intent_eye_validation ROS_LOCALHOST_ONLY=1 \
  /usr/bin/python3 scripts/validate_intent_eye.py
```

任务案例逐个启动隔离 Gazebo：预算、有限转向、替换、取消、健康失效、四进程中断、分叉/堵塞、重放、时钟暂停、偏角居中、正常过门、管理器重启。通过既要求实际动作/距离，也要求正确拒绝与停车；没有把“未移动无碰撞”当通行成功。

- [17 个任务案例](../wheelchair_shared_ws/artifacts/intent-tasks/result.json)：含默认预算前进约 2.76 m、有限 15°目标（实际在 ±2°容差内）、5°偏角走廊对正、门洞通行、分叉/堵塞拒绝。
- [合成眼动链路](../wheelchair_shared_ws/artifacts/intent-eye/result.json)：未确认无运动、无候选/单眨眼不启动、有限转向、健康扫视保持任务、长闭眼和 invalid 取消。
- [时序分解](../wheelchair_shared_ws/artifacts/intent-eye/latency.json)：接收→预览、稳定→确认、确认→ACK、ACK→实际运动、停车转换、手势→识别→STOP→停稳分别输出 P50/P95/max 和样本数。是这些合成试次的观测值，不是人群统计。
- [真实双摄传输](../wheelchair_shared_ws/artifacts/intent-camera/result.json)：151 条带源时间戳消息，双眼标定加载；这次有效双眼样本为 0、运动事件为 0，只证明传输与失效链路，不证明真人成功驾驶。

旧 Float32MultiArray 只能按接收时间判断源年龄；新的 EyeSample 使用最旧相机读帧完成时间映射到 SYSTEM 时间，保留推理/排队年龄及源序号。它不是硬件曝光时间。实际相机采集/标定核心未修改。

### 旧版尚需真人配合的验收

真实佩戴后的双眨眼误提交率、漏确认率、阅读/观察干扰、主观负担与旧模式对比尚未完成。不得使用算法自己的事件作为真值；[人工标签与实验模板](../wheelchair_shared_ws/docs/INTENT_EXPERIMENTS.md) 已提供。若误触发不能接受，应评估独立确认输入，不能据现有水平信号宣称理解真实目的地。

所有空间安全检查仍依赖已知仿真几何；没有实时行人障碍融合、复杂全局导航或自动倒车。以上不构成实机安全验收。


### 既有运行环境独立化记录

收尾期间旧 TurtleBot 工作空间被移入回收站，发现原轮椅脚本仍依赖其 Gazebo 运行库。已仅复制运行库到 wheelchair_shared_ws/vendor/runtime，迁移CLI配置、重建ScanPoints插件和ROS包，启动不再引用旧目录；旧项目保持删除状态。预编译runtime用COLCON_IGNORE排除源包发现。迁移后17个独立分区任务案例、6个眼动案例、278项测试及双摄传输重新通过。验证脚本会等到自己进程组退出，避免Gazebo孤儿服务干扰后续案例。
