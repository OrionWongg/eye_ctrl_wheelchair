# 意图确认输入与实验记录

仅软件与仿真；以下是复现和采集方法，不代表已完成真实眼动可用性实验。

## 同协议键盘/脚本源

使用 `intent_confirm` 模式，输入源选 `keyboard`，硬件关闭。该工具只发布
`/wheelchair/keyboard/{candidate,event,health}`，订阅 task_status/intent_ack。
不启动旧键盘/眼动 ControlIntent 生产者。先构建并启动仿真任务管理、安全门与执行器。

```bash
cd /home/whill/wheelchair_shared_ws
./scripts/intent_keyboard.sh
# 以下各条是独立实验，不能同时运行多个输入源
./scripts/intent_keyboard.sh --submit continue --seconds 12 --log /tmp/continue-input.jsonl
./scripts/intent_keyboard.sh --submit left --seconds 8 --log /tmp/left-input.jsonl
./scripts/intent_keyboard.sh --submit right --seconds 8 --log /tmp/right-input.jsonl
```

入口为 `ros2 run wheelchair_control task_input`；脚本加载工作区环境并默认 ROS domain 64、use_sim_time=true。
键盘 W/A/D 只选择继续/左转 15°/右转 15°；Enter 在 1.5 秒内确认一次。
Space 或 S 立即 CANCEL；Q、Ctrl-C、正常退出也取消。确认后不得重用旧候选。
终端显示任务 ID、任务状态/动作中文提示、原因、候选以及确认接受/拒绝。
状态码：0 IDLE、1 ACCEPTED、2 RUNNING、3 STOPPING、4 COMPLETED、5 BLOCKED、6 FAULT；
动作码沿用共享接口 STOP/FORWARD/LEFT/RIGHT。接受不等于已经运动。

键盘健康每 50 ms 采样终端是否仍为前台进程组；默认最后操作后 15 秒失效。
这不是摄像头检测健康，也不能检测图形窗口是否被遮挡。脚本源只在显式指定的有限生命期内健康。
失效立即取消，health TTL 250 ms；进程被强杀/终端断开时由下层健康超时停车。
输入循环不能读取/发布时不会有后台线程替它维持健康。

候选必须有最近 500 ms 的管理器状态和可用上下文。上下文变化清除候选与重试。
提交事件 TTL 1500 ms，至多每 100 ms 重发同一 ROS 消息（原 ID、原 stamp），
最长 1500 ms，且不超过选择时确定的候选 1500 ms 有效期；等待期间候选广告使用剩余 TTL，
ACK 或任一期限后撤回，不因重发延长事件或候选期限。
管理器等待停稳时 can_accept=false 不撤回已提交事务。任何新任务仍以最终安全检查为准。
管理器会话切换后拒绝被替代会话回放；ACK 仅消费当前会话、合法接受/拒绝状态、
最近 500 ms 且不早于原提交的时间戳。ROS 时钟停止推进超过 250 ms、回跳或
相对单调时钟突增超过 500 ms 时取消旧候选和提交，恢复后需要新选择/确认。
每个健康 sample_sequence 对应当前循环实际检查终端前台/脚本生命期，
不是对旧键盘事件或旧眼动检测的无限刷新。

## 时序报告

JSONL 每行一个对象，字段：`stage`、`monotonic_s`、`source_session_id`，以及阶段关联键
`event_id` / `candidate_id` / `sample_sequence`；可加 `task_id`、`clock_domain`。
所有合并日志必须来自同一单调时钟域，或先完成明确的时钟同步。

阶段约定：`source_sample`、`received`、`preview`、`stable`、`confirmation`、`ack`、
`motion_started`、`stop_wait_started`、`stop_wait_ended`、`stop_gesture`、`stop_recognized`、
`stop_published`、`stopped`。停止链各阶段必须关联同一停止事件 ID。
键盘自身输出 stable/confirmation/ack/cancel/task_status；`cancel` 不能冒充实际停稳。
只有实际 odom 越过预先声明速度阈值才记录 motion_started/stopped。

```bash
python3 scripts/intent_latency_report.py /tmp/continue-input.jsonl /tmp/manager.jsonl > /tmp/latency.json
```

工具逐阶段输出 P50/P95/最大观测值（毫秒）、样本数、未配对数量及时间逆序数量。
重试按 session + 关联 ID 去重。缺少阶段返回 null，不能作为零延迟。
没有采集时间戳的旧 Float32MultiArray 只有 received→preview，必须标记
`reception_timed=true`，不可报告相机端到端延迟。手势检测自身阈值、ACK 等待和机械停稳
分别统计；最大观测值不等于最坏情况保证。

## 人工标签与可用性实验模板

每个参与者以随机或交叉顺序体验 gaze_joystick 和 intent_confirm；相同场景/任务范围、
速度、安全参数。记录自然扫视、阅读标识、观察路人、正常眨眼、明确控制及主动停止。
事先定义成功判据、事件匹配时间窗、停止速度阈值和最小间距计算方式；保留无控制意图片段。
人工标注者依据参与者明确指示/回放标注，不能把算法输出当真值。

建议人工标注 CSV：

```csv
trial_id,participant_id,mode,scene,start_s,end_s,human_intent,expected_direction,confirmation_attempt,expected_stop,annotator,notes
```

结果表（尚未采集，保持空白）：

| 指标 | gaze_joystick | intent_confirm | 样本数/分母与计算规则 |
|---|---|---|---|
| 每分钟误提交次数 | 待采集 | 待采集 | 无人工控制意图片段中的提交数/观察分钟 |
| 漏确认率 | 待采集 | 待采集 | 未成功匹配的明确尝试/总明确尝试 |
| 确认时间 P50/P95/max | 待采集 | 待采集 | 人工动作起点至确认完成 |
| 每段路用户操作次数 | 待采集 | 待采集 | 同一任务定义与长度 |
| 任务完成率 | 待采集 | 待采集 | 完成/尝试，正确拒绝另列 |
| 停止延迟 P50/P95/max | 待采集 | 待采集 | 手势起点至实际停稳，附各阶段 |
| 碰撞数与最小间距 | 待采集 | 待采集 | 实际运动轨迹与物理模型 |
| 主观操作负担 | 待采集 | 待采集 | 事先约定量表，保留原始评分 |

每个仿真实验保存启动参数、场景/初始位姿、输入日志、管理器日志、odom、碰撞反馈和退出原因。
同时提供有实际移动且成功完成、明确被安全规则正确拒绝的案例；不得仅以未移动无碰撞宣称成功。
本工具及合成单元测试不提供人类误提交率或实机安全结论。
