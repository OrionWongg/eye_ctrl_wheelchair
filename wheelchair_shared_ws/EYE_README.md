# 当前试用入口：眼动方向输入

> 2026-10-14：RITnet 默认改用三维眼球估计 + 单点正前方参考；首次按 S 确认中心，旧三点方案需显式传 `--gaze-method legacy`。流程见 [/home/whill/projects/EYETLE/GAZE3D.md](/home/whill/projects/EYETLE/GAZE3D.md)。现有轮椅交互状态机不变。


运行 `~/projects/EYETLE/start_eye_wheelchair.sh`，默认采用 `directional_intent`，接入当前 Nav2 避障链路。完整启动、手势、标定、停车及测试范围见 [使用说明](docs/EYE_DIRECTIONAL.md)。以下 `intent_confirm` 与 `gaze_joystick` 为仍可显式选择的旧接口说明，默认组合入口以本段为准。

# 眼动意图输入：确认模式与旧遥杆模式

`eye_shared_control` 实现 `projects/EYE_WHEELCHAIR_SHARED_CONTROL.md` 的 Agent A 部分。只处理检测后的水平/闭眼/有效性信号，不修改 EYETLE 检测、标定及海龟演示，不直接发布 Twist 或连接轮椅。

## 新模式 intent_confirm（2026-10-14）

单独眼动节点的配置默认保留 `gaze_joystick`；`start_eye_turtle.sh --gazebo` 组合入口默认 `intent_confirm` 并使用带源时间戳消息，根 launch 也可显式选择 `interaction_mode:=intent_confirm`。两种模式启动时固定，运行中改参数不切换授权。新模式通过独立 `ConfirmMachine` 工作，不调用旧 `ResumeGesture`。以下旧模式章节的“单眨眼恢复”“持续侧视持续转向”仅适用于 `gaze_joystick`。

单独启动眼动节点（先启动采用新模式且选 `eye` 来源的任务管理器，勿重复启动眼动节点）：

```bash
ros2 run eye_shared_control eye_intent --ros-args \
  --params-file ~/wheelchair_shared_ws/src/eye_shared_control/config/eye.yaml \
  -p interaction_mode:=intent_confirm -p eye_sample_mode:=legacy
```

候选为正中“继续当前方向（有限任务）”、左/右“有限转角”，默认 15°。250 ms 稳定停留后可确认，双眨眼提交，单眨眼不启动、不恢复。无稳定候选时不默认前进。第一次闭眼冻结候选 ID、版本、上下文、角度、有效期；闭眼瞳孔波动不改变对象。双眨眼间隔超时、上下文变化、重新稳定选择另一方向、长闭眼和失效会清除尚未提交的事务。提交立即消耗候选，第三次眨眼不形成重复事件；健康扫视不修改正在执行的任务。

新模式输出 `/wheelchair/eye/candidate`、`/wheelchair/eye/event`、`/wheelchair/eye/health`；**不创建旧 `/wheelchair/eye_intent` 发布者**。订阅 `/wheelchair/task_status` 和 `/wheelchair/intent_ack`，接口见 [协议 v1](docs/INTENT_PROTOCOL_V1.md)。候选稳定与可确认只是输入授权条件，不承诺可执行。任务上下文必须新鲜；长闭眼约 650 ms、invalid、输入或任务反馈超时触发 CANCEL。恢复健康不会恢复旧任务。

事件每 100 ms 可重发，保持原消息 stamp/event_id/参数；到 ACK 或原截止时刻结束。等待停稳可能消耗 TTL。冻结候选在等待 ACK 时继续公告供任务管理器重校验，剩余 TTL 每次递减，心跳不延长期限；ACK 后撤回。默认候选 TTL 1.5 s，事件 TTL 上限 1.5 s（实际受冻结候选剩余有效期限制），ACK 等待上限 1.5 s。ACK=ACCEPTED 仅表示授权接收，实际运行、停车和完成读取 TaskStatus。

`eye_sample_mode:=legacy` 接收原 `/eye_tracking/signals`，只能按**接收时间**判旧。重复收到同样值可能是健康稳定注视，因此不能由数值判断重放。`eye_sample_mode:=stamped` 则只接收 `/eye_tracking/samples` (`EyeSample`)，要求源 SYSTEM/WALL 时间戳、UUID 会话与单调递增 sequence；重复/乱序序号不刷新健康，源会话切换撤销旧事务。源时间戳不得使用仿真 `/clock`。事件/任务 ROS 时间戳使用 `/clock`；本地健康、手势和重试由单调时钟计时，仿真时间暂停或跳变使旧授权失效。

反馈 `/wheelchair/eye_feedback` 与 `/eye_control/feedback` 为 JSON，包含候选、稳定/已消费、ACK、健康、当前任务/动作及原因。`/wheelchair/eye_latency_events` 发布 JSON 记录：统一 `stage`、`monotonic_s`、`source_session_id`、`event_id`、`candidate_id`、`task_id`，以 `received → preview`（同一 sample_sequence）、`stable → confirmation`（同一 candidate_id）、`confirmation → ack` 记录延迟；`stop_gesture → stop_recognized` 使用同一 CANCEL event_id，分别记录闭眼起始和达到长闭眼阈值时刻。带源时间戳输入额外产生 `source_sample`，按 `received_monotonic - source_wall_age` 映射到本机单调时钟，仅适用于源与节点共享本机系统时钟的测量；旧输入绝不产生此阶段。停止发布和实际停稳由任务管理器以同一取消事件 ID 输出，便于串联。旧输入只记录接收后延迟，不称为相机端到端延迟；实际车轮运动/停稳延迟应结合任务管理器与 odom 记录。记录可用于 P50/P95/最大值统计，不把状态机假时间测试当作物理测量。

新模式合成单测：

```bash
cd ~/wheelchair_shared_ws
PYTHONPATH=src/eye_shared_control python3 -m pytest -q src/eye_shared_control/test
```

已覆盖方向不自动启动、单/双/第三次眨眼、冻结期间噪声、上下文撤回、过期/重试/旧 ACK、长闭眼/invalid/超时停车及恢复必须新确认。真实摄像头自然扫视、阅读、观察场景的误提交率和漏确认率仍需人工标注实验；本实现未自动打开摄像头或连接实机。

## 旧模式 gaze_joystick

## 启动

先按工作空间 README 构建并启动 Gazebo、安全规划和意图仲裁，选择 `eye` 输入源。两个输入源同时运行时也只允许仲裁器选中的源控制。

```bash
source /opt/ros/humble/setup.bash
source ~/wheelchair_shared_ws/install/setup.bash
export ROS_DOMAIN_ID=64 ROS_LOCALHOST_ONLY=1
ros2 launch eye_shared_control eye.launch.py
```

此 launch 默认 `use_sim_time=true`，与 Gazebo `/clock` 一致；若主 launch 已启动眼动节点，不要重复运行。墙钟系统可显式传 `use_sim_time:=false`。节点内部输入、状态和暂停看门狗使用单调时钟，不会随仿真暂停而停止检测。仿真时间暂停超过 300 ms 或跳变会撤销授权，恢复后需重新确认。

合成眼动输入（另一个相同 ROS 环境的终端）：

```bash
ros2 run eye_shared_control mock_eye --direction forward --duration 8 --arm-after 2
```

该工具先持续发送有效开眼数据，在第 2 秒生成双眨眼，再保持指定注视方向；只授权一次，正常有效输入下持续运行，无5秒限时。`--direction left|right` 模拟持续侧向注视；`--no-arm` 验证未授权注视；`--invalid-after 4` 和 `--long-close-after 4` 验证停止。结束时发 invalid，异常退出由源超时保护。

查看用户反馈：

```bash
ros2 topic echo /wheelchair/eye_feedback
ros2 topic echo /wheelchair/eye_intent
```

`eye_feedback` 为可查看的 JSON：状态、方向、enabled、valid、原因及 resume_phase/resume_progress；暂未实现语音。

## 接入真实检测器（用户自行启动摄像头）

新系统默认域为 64，原 EYETLE 脚本默认域为 58。仅设置 ROS_DOMAIN_ID 不够，因为旧脚本会从 EYE_ROS_DOMAIN_ID 重新赋值：

```bash
cd /home/whill/projects/EYETLE
EYE_ROS_DOMAIN_ID=64 ./start_ritnet.sh
```

以上入口只启用检测/预览及 ROS 信号输出，不启动旧海龟控制器。真实信号接入时先停止 mock_eye，避免多个发布者交错产生伪眨眼。需完成原项目标定，真实误触发率仍需实验量化。本次实现/测试没有自动打开相机或连接真实轮椅。

## 接口和状态

- 输入 `/eye_tracking/signals`：Float32MultiArray，严格三个有限值 `[horizontal, eye_closed, valid]`；水平范围 -1 到 1，flags 为 0/1，左负右正。depth=1、volatile、best effort，兼容可靠检测发布者。
- 输入 `/wheelchair/control_status`：ControlStatus；depth=1、volatile，拒绝同会话乱序/重复序号和已退出的旧会话；按 ROS 消息时间及单调到达时间检查陈旧。
- 输出 `/wheelchair/eye_intent`：ControlIntent；20 Hz，TTL 250 ms，独立 UUID 会话、递增序号、机器人本体方向。仲裁器选源后转发至 `/wheelchair/intent`。
- 输出 `/wheelchair/eye_feedback` 及 `/eye_control/feedback`：String JSON。RITnet 订阅公共反馈；旧模式显示实际状态和眨眼恢复提示，新模式分别显示原始预览、实际候选（有限角度）、稳定性、ACK、任务状态、实际动作与原因，过期反馈显示 UNKNOWN。

新增 `PAUSED`：长闭眼后保持控制模式但输出 disabled STOP。睁眼不自动恢复；先睁眼，再完成一次60～400ms的短眨眼。失效/闭眼/底层阻断取消未完成恢复动作，重启进程不保留控制授权。

初始 `DISARMED`；信号无效或超时进入 `INVALID`；安全层要求停止或状态超时进入 `BLOCKED`。所有这些状态都输出 disabled STOP。恢复有效输入只回到待确认状态，不能自动 ACTIVE。

双眨眼可进入 `ACTIVE`；未启用时单眨眼不启动；已启用且暂停时单眨眼恢复，ACTIVE 内双眨眼不改变模式，正常行驶无5秒限时。进入 ACTIVE 后方向也需稳定停留；正中 FORWARD，左侧 LEFT，右侧 RIGHT，没有后退、组合动作或固定转向脉冲。

安全层阻挡时立即清除授权；先发送 disabled STOP，安全层健康且停稳后回报 ready_to_rearm=true。此时 requires_rearm 仍可为 true，已有控制会话时用户必须完成一轮新的单次短眨眼恢复动作；从未启用的会话仍需首次双眨眼。反复的 ready 确认不会清掉这次新授权；新的阻挡、状态会话变化和过期均会撤销授权。

参数文件：`src/eye_shared_control/config/eye.yaml`。

| 参数 | 初值 | 含义 |
|---|---:|---|
| dwell | 0.25 s | 方向连续稳定停留 |
| enter / leave | 0.25 / 0.15 | 左右进入、退出滞回 |
| blink_min / blink_max | 0.06 / 0.40 s | 可识别的单次短眨眼闭合时长 |
| double_gap | 0.45 s | 两次短眨眼结束点之间的最大间隔 |
| long_close | 0.65 s | 长闭眼进入暂停并停止输出运动 |
| source_timeout | 0.25 s | 最新检测数据到达年龄上限 |
| status_timeout | 0.35 s | 安全状态到达年龄上限 |

短闭眼期间保留最近确认方向；闭眼不会算作方向停留；明确 invalid 不享受眨眼宽限。停止路径不经过方向滤波。

旧检测消息没有采集时间和序号，因此只能知道 ROS 回调到达时间，不能证明摄像头数据本身新鲜，也无法可靠区分多个源。depth=1 限制积压，但不替代未来带源时间戳/序号的检测接口。眼动包装输出使用 ROS 时间，不把源到达时间冒充采集时间。60～400 ms/650 ms 等均为仿真交互初值，不是经过真实轮椅验收的安全参数。

## 测试

```bash
PYTHONPATH=~/wheelchair_shared_ws/src/eye_shared_control \
  python3 -m pytest -q ~/wheelchair_shared_ws/src/eye_shared_control/test
```

测试以合成事件覆盖双眨眼、单眨眼、方向停留/滞回、持续左右和回正、短/长闭眼、失效/源超时、持续运动、暂停/单次短眨眼恢复、安全恢复握手和状态会话/重放。ROS/Gazebo 联调验证记录见工作空间 README。
# 新默认入口说明（2026-10-14）

`start_eye_turtle.sh --gazebo` 的组合入口现在默认 `intent_confirm`：先看方向选候选，再双眨眼确认一个有限任务；单眨眼不恢复，回正不自动前进，长闭眼取消。原有单眨眼恢复只属于显式选择的 `WHEELCHAIR_INTERACTION_MODE=gaze_joystick`。新模式完整行为和验证记录见 [实施记录](../projects/WHEELCHAIR_INTENT_SHARED_CONTROL_IMPLEMENTATION.md)。
