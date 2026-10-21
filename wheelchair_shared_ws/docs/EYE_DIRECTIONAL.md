# EYETLE 眼动方向输入 + Nav2 避障

当前眼动入口与方向键盘共用方向管理器、Nav2 路径服务、安全门和执行器。EYETLE 只提供检测信号，不发送车轮速度。原 `intent_confirm` 和 `gaze_joystick` 仍可显式选择；轮椅组合入口默认改为 `directional_intent`。

## 一键试用

先关闭原来同 ROS domain 64 的轮椅仿真和其他眼动检测进程，然后在图形桌面终端运行：

```bash
cd ~/projects/EYETLE
./start_eye_wheelchair.sh
```

轮椅入口直接调用现有 EYETLE 启动脚本，**不再强制双目、不指定估计方法或标定文件**。当前 EYETLE 默认是单目 3D 眼动；相机、建模与中心参考均由 EYETLE 自己管理。显式传入的检测参数原样转交。眼动代码不由轮椅侧修改。

已有环境变量若指定了旧模式，可显式启动：

```bash
WHEELCHAIR_INTERACTION_MODE=directional_intent ./start_eye_wheelchair.sh
```

需要路径、轮廓和停车原因视图时另开终端：

```bash
~/wheelchair_shared_ws/scripts/navigation_rviz.sh
```

## 操作

| 输入 | 效果 |
|---|---|
| 看正中并稳定至少 250 ms，再双眨眼 | W：启动/继续沿当前方向行驶 |
| 看左/右并稳定至少 250 ms，再双眨眼 | A/D：停车时原地转约 20° 后保持停车；行驶时提交方向/入口偏好 |
| 长闭眼约 650 ms | 停车，撤销当前任务 |
| 丢眼、无效检测、检测数据超时 | 停车；恢复检测不自动恢复旧任务 |
| 单次短眨眼、普通扫视 | 不提交新的运动指令 |

先看预览中的 **candidate** 和 **stable: YES** 再确认；Preview 是实时视线，尚未稳定时可能与候选不同。看回正中本身不会改变正在执行的任务。左右意图执行后是否进入房间仍受原有门洞/车体安全距离限制。

当前检测器只有水平视线与闭眼信息，因此本接口没有额外猜测“掉头”手势。不要把连续眨眼当作 U。原键盘模式的 U 掉头功能保留。

预览显示 `START / CONTINUE (W)`、`LEFT INTENT (A)`、`RIGHT INTENT (D)`、导航阶段、实际动作、ACK 和停车原因。`ACK: ACCEPTED` 是授权接收，实际运动看 action 和仿真。若显示 INPUT INVALID，查看现有 EYETLE 窗口提示。最新版单目 3D 流程为先转眼建模，待 3D_READY 后看正前方按 S 确认中心；具体操作以 [EYETLE GAZE3D.md](../../projects/EYETLE/GAZE3D.md) 为准。轮椅侧只接受现有输出的有效性标志，不放宽建模门限，也不改动中心文件。

退出 RITnet 窗口（Esc）或在启动终端 Ctrl+C 会关闭本次启动的检测和仿真。紧急停车可另开 `~/wheelchair_shared_ws/scripts/keyboard.sh`，按空格/S；eye 模式不会接受键盘 W/A/D 的运动控制。

## 接口

```text
EYETLE RITnet /eye_tracking/samples (EyeSample，系统时间+会话+序号)
  → eye_intent (directional_intent，稳定候选+双眨眼确认)
  → /wheelchair/eye/direction_event + /wheelchair/eye/health
  → direction_manager (input_source=eye)
  → /wheelchair/intent
  → Nav2 路径服务 + safety_gate
  → /wheelchair/action_safe → executor → Gazebo
```

`/wheelchair/eye/candidate` 仅供候选预览；方向管理器接收方向事件。反馈使用 `/wheelchair/navigation_status`、`/wheelchair/intent_ack` 和 `/eye_control/feedback`。重发保留原事件序号和时间戳；旧源/重复样本不能刷新健康。源时间戳用系统时间，导航事件使用 ROS 仿真时间，二者不混用。

输入源在启动时选择，不能靠同时启动两个发布者混合驾驶。切回键盘：退出本次组合入口，再运行 `scripts/run_direction_sim.sh scene:=clearpath_office` 和 `scripts/direction_keyboard.sh`。

## 历史验证范围（双目默认已取消）

2026-10-14：真实相机与 TensorRT 短测 90 帧，已有标定成功加载，约 23.3 FPS，帧读取后年龄 P95 约 91 ms。该测试没有执行驾驶，也不证明佩戴后的注视/眨眼准确率。

隔离 domain 93 的带时间戳合成眼动经过真实 ROS/Gazebo/Nav2 链路：左右转向、启动、长闭眼/无效数据/断流停车、恢复不自动运行、未选中的键盘运动输入被忽略，以及办公室障碍前停车均通过。两组用例无碰撞、计算超时或 ACTION_STALE，最大控制计算分别约 35.2 / 29.9 ms。

证据：`artifacts/eye-direction-20261014/verified/{gestures,obstacle}/`、`camera.json`、测试及构建日志。真实佩戴下的误触发率、漏检测率和舒适性由现场试用确认。

完整代码回归：355 项通过；另有 1 项组合入口测试验证旧 EYETLE domain 58 环境不会覆盖轮椅默认 domain 64。相关包已构建。

双目默认入口复测：两路相机及两份既有标定加载成功，完成 90 帧双目采集（约 11.5 FPS）。首次启动有一次左相机读失败，重试通过；复测左眼分类为 UNKNOWN，试用时需佩戴并确认双眼有效后再确认动作。日志为 `binocular-camera*.json/log`。

## 最新单目接入约定

按用户最新要求，仅维护轮椅侧接口。`run_eye_sim.py` 不注入 `--eyes`、`--gaze-method`、`--center-file` 或 `--calibration-file`，沿用 EYETLE 当前默认值，只请求其已有 `--publish-stamped` 输出并统一 ROS domain。现有 `EyeSample` 的水平值、闭眼、有效性、会话、序号和源时间戳保持原含义。上文双摄硬件结果为历史记录，不是本轮单目验收结果。
