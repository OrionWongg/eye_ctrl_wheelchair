# 融合眼动 → 轮椅（小乌龟交互逻辑）

2026-10-19 更新。替代此前“每次方向都眨眼确认”的实现。

## 启动

退出旧仿真与占用相机的检测器，然后运行：

```bash
cd ~/projects/EYETLE
WHEELCHAIR_SCENE=willow_office ./start_eye_wheelchair.sh
# 需要重新标定时追加 --recalibrate
```

默认启动 `start_fusion.sh --publish-ros`，ROS domain 64、本机通信。`WHEELCHAIR_ROS_DOMAIN_ID` 统一覆盖。Ctrl+C 关闭整套程序。修改代码后需重启。旧 `--ritnet` 模式保留原交互，不属于本文。

## 操作

- 首次双眨眼启动前进：两个完整短眨眼事件结束间隔 120～900 ms。
- ACTIVE 时，看左／右超过 0.55 阈值直接发方向事件，无需眨眼，无需候选停留确认。
- 持续看同一侧不重复触发。回到中心释放区（绝对值小于 0.25）后才能触发下一次。
- 输入端保留 1 秒转向脉冲门控：回中不提前结束当前输入脉冲，脉冲期间不触发另一个方向。
- 短闭眼、方向无效、丢眼、陈旧样本立即暂停；长闭眼清除待用手势授权。
- 已经启动后，一次新短眨眼恢复前进；只恢复有效视线不会自动行驶。
- 启动／恢复后默认 0.5 秒内过滤左右视线（`post_blink_gaze_delay`）；接受新眨眼也重新开启缓冲。缓冲内的视线不排队，结束后必须收到采集时间晚于缓冲截止的新样本才允许转向。先看前方、随后看左右即可，无需看屏幕或再次眨眼。该参数可在启动前调整，范围 0～2 秒。闭眼、丢失、超时、急停及导航阻塞仍即时处理。这只过滤睁眼初期的转向，不能解决前进受阻后的重复恢复问题；0.5 秒尚需佩戴测试验证。
- 急停或检测器换会话撤销启动授权，需要重新双眨眼。

**映射边界：** 小乌龟输出连续 Twist 并能边走边转；轮椅仍通过 DirectionEvent、方向管理器、规划器、安全门和执行器行动。输入脉冲只提交一次左右偏好，不直接规定车轮转向 1 秒，也不在 1 秒后取消尚未完成的导航转弯。回中表示允许下一次输入，不取消当前已提交的导航任务。规划受阻会暂停并要求新眨眼恢复。实际转弯轨迹、时长、速度和小乌龟不同。

## 接口

- `/eye_tracking/fusion_samples`：String，沿用 EYETLE schema=1。
- `/wheelchair/eye/direction_event`：启动／恢复发 START_OR_CONTINUE，左右视线发 PREFER_LEFT／PREFER_RIGHT，暂停发 STOP。
- `/wheelchair/eye/health`：原始帧新鲜度和跟踪健康；连续短闭眼通过 STOP 暂停，不再同时视为设备故障。
- `/wheelchair/emergency_stop`：Empty，撤销眼动启动授权。
- `/eye_control/feedback`：state 为 DISARMED / ACTIVE / PAUSED，target 为 wheelchair，另带 blink_count、event_decision、reason、ack 和 task。

时间：检测器同机系统时间映射到本地单调时间；方向事件使用仿真 ROS 时间。运动帧龄和健康上限仍为 250 ms；手势到达静默 watchdog 单独采用 400 ms，避免处理延迟把新眨眼误判为停车前事件。已完成事件授权最多保留 250 ms，以容许短暂方向恢复及导航健康反馈回传；过期不自动授权。会话、样本序号和眨眼事件 ID 去重，旧事件不能跨停车屏障恢复。ROS 时钟暂停／跳变或导航断联仍停车。

实现：`eye_shared_control/fusion.py` 的独立启动／暂停状态机，`confirm_node.py` 复用事件传输、ACK、健康与导航状态。融合入口不再使用稳定候选加眨眼确认。`EYE_DRIVE` 日志记录状态、停车原因、眨眼 ID、方向事件 ID、事件判定和视线值。

## 验证范围

测试覆盖双眨眼启动、无需眨眼的左右触发、持续侧视去重、回中释放、输入脉冲门控、短闭眼停车及单眨眼恢复、导航健康反馈延迟、旧事件屏障、超时、急停与真实 ROS JSON 到方向事件的传输。实际佩戴在 Gazebo 中的驾驶仍需现场验证。

本次结果：轮椅侧完整 Python 回归 390 项通过；EYETLE 101 项通过；`eye_shared_control` 构建成功。日志分别为 `/tmp/wheelchair-turtle-logic-tests.log`、`/tmp/eyetle-turtle-logic-tests.log` 和 `/tmp/wheelchair-turtle-logic-build.log`。

眨眼无响应的现场分析和修复见 [2026-10-16 诊断](EYE_BLINK_DIAGNOSIS_20261016.md)。此次轮椅回归 394 项、眼动回归 101 项通过。
