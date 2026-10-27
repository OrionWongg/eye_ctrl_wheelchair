# 眼动／键盘 → iPad → 轮椅接入总结（2026-10-22）

## 16:52 日志后续排查：保持仿真眼动模块，仅调整 iPad 后端

用户要求保持已验证的仿真眼动检测不变。本轮未修改 EYETLE 检测、引导、
标定、启动器及 `eye_shared_control` 共享代码；对检测目录、共享状态机及原标定
共37个文件记录修改前后 SHA256，全部一致。
另与 `/home/whill/projects/EYETLE` 的原文件逐字节核对，`ritnet_blink.py`、
`purest_backend.py`、`blink_events.py`、`fusion_control.py`、`dual_eye_tracker.py`
完全相同；`fusion.py`、`confirm.py`、`core.py`、`confirm_node.py` 也与
`/home/whill/wheelchair_shared_ws` 原仿真目录相同。
此前的摄像头路径迁移、只读标定加载及 iPad UI 接入仍保留，不能据此宣称
整个启动器与原目录逐字节相同。

分析 `20261022-165221-e5654efa-9eff-43b8-a184-f3eccc5fdf8f.jsonl`：
1508条帧记录中识别 READY 1333条；控制器 ACTIVE 235条、INVALID_INPUT
543条、FRAME_AGE_TIMEOUT 499条。首次可见 ACTIVE 约3.265秒后暂停。
相邻样本发布时上一采集帧已超过250ms的情况有213处，发布端时序也仍存在缺口。

将1615条原始样本按发布端记录时间，连同50ms间隔的看门狗更新分别送入
仿真的 `FusionInput` 和 iPad 的 `EyeDrive`：3147次更新中，启停、暂停原因、
方向、转向重触发、眨眼事件及启用状态完全相同。该受控回放假定导航后端就绪并
立即接受事件，不含 Gazebo、真实 DDS 接收延迟、人工启用/停止时序或真实运动。
两边均观察到14次 INVALID_INPUT 和2次 FRAME_AGE_TIMEOUT 导致的 ACTIVE 退出；
不能把这些数值直接与界面反馈帧数比较。

本轮仅在 iPad 后端进一步把眼动模式的 Tk/ROS 轮询间隔改为5ms（键盘仍20ms），
保留每轮最多8次及5ms预算；时间戳比较使用与仿真相同的系统时间。
`IPAD_EYE_DRIVE` 状态变化日志新增样本序号、到达帧龄、到达间隔、静默时间和
检查时源帧龄，反馈中也包含 `input_timing`。关闭时取消轮询任务。
未改变250ms阈值、闭眼停车、双眼一致性、眨眼规则或任何标定数值。

7项离线测试通过，包括新增仿真/iPad手势一致性测试、真实DDS持续输入、
启动/转向/断流释放/恢复/停止及键盘拖动。未发送真实轮椅运动指令；
新接收端日志需下次运行后结合实际串口动作再判断剩余暂停原因。

## 16:41 日志后续排查：ROS 回调处理不足

最新分析的 trace 为 `20261022-164134-8e7a4580-c1fc-44bc-b740-4d8c8528dea0.jsonl`。
1211条帧记录中，识别 READY 1138条、EYES_DISAGREE 仅3条；控制器反馈
FRAME_AGE_TIMEOUT 855条。11段可见 ACTIVE 均以 FRAME_AGE_TIMEOUT 结束。
这些是反馈采样统计，不代表独立故障次数或精确串口按住时长。

控制器原先每20毫秒只调用一次 `rclpy.spin_once`，一次最多执行一个回调；
运动、反馈、状态定时器共31Hz，与约20Hz眼动消息竞争处理机会。
真实 DDS + 离线串口复现：连续20Hz、源帧龄120ms的有效样本仍触发超时。
新增回归测试在旧实现上失败。现改为持久 SingleThreadedExecutor，
在 Tk 所在线程每轮最多处理8次、时间预算5ms，再执行眼动看门狗。
时间预算在回调之间检查，不会中断正在执行的回调。
统一界面模式也跳过后台隐藏控制窗口的20Hz文本和轨迹重绘，减少同线程工作。
250ms源帧超时、闭眼停车、双眼一致性及两份标定保持原值。
最终6项离线测试通过，覆盖真实 DDS 持续输入、双眨眼启动、方向拖动、
断流释放、单眨眼恢复、停止按钮与键盘拖动。测试中的单帧发送等待 DDS
实际收到再进行后续动作，避免用固定50ms假设消息已送达。

发布端也存在真实时序余量不足：最新日志中52处“下一样本发布时刻减去上一样本
采集时刻”超过250ms。因此此修复解决已复现的回调调度问题，不能视为所有超时
已经消除，也未执行真实轮椅运动验收。摄像头预览已由用户确认图像正常。

## 后续代码更新：键盘自动拖动

以下原始总结记录的是修改前状态。后续排查确认：键盘方向键原先只移动指针，
必须先按 F 才会保持左键；眼动模式才会自动发送按下。
现在 `scripts/ipad_controller.sh` 默认开启 `keyboard_auto_drag`：APP LOCKED
模式按方向键自动从中心按下并拖动，长按保持目标，松键释放并回中；F / Enter
在该自动模式下禁用。可用 `-p keyboard_auto_drag:=false` 恢复原操作。
标定及眼动控制逻辑保持原值。用法与验证边界见 [EYE_IPAD.md](EYE_IPAD.md)。

另在仓库外发现 `/home/whill/esp32-firmware-draft-unflashed/main/main.c`，
草稿中 `[` 设置按钮位，移动使用保存的按钮位，`]` 清零；目录明确标记未烧录，
不能用于认定当前 ESP32 的实际实现。本次没有烧录固件或发送真实运动命令。

## 当前结论

软件已接通眼动消息到 iPad 指针控制，眼动引导和 iPad 控制操作已合并到一个窗口。
用户确认 ESP32 到 iPad 的连接链路是通的，指针能够按照期望轨迹运动。

**当前未解决的问题：指针运动时，WHILL APP 的滑杆没有跟随变化；用手指拖动则会变化。**
用户已将 nimble 按钮操作改为“轻点”，现象仍未解决。
因此，不能将“指针能移动”或“主机已发送按住指令”当作 APP 接收到有效拖动、
或者真实轮椅已可由眼动可靠控制的证据。

本次总结不更改程序、标定或 iPad 设置，不执行真实运动测试。

## 启动和界面

仓库：`/home/whill/eye_ctrl_wheelchair`。

```bash
cd /home/whill/eye_ctrl_wheelchair
./scripts/project.sh eye-ipad             # 统一眼动界面，真实串口
./scripts/project.sh eye-ipad --dry-run   # 统一界面，离线串口
./scripts/project.sh eye --ipad          # 与 eye-ipad 等效
./scripts/project.sh eye                 # 原眼动仿真入口
```

键盘控制单独启动：

```bash
./scripts/ipad_controller.sh -p eye_control_enabled:=false
```

眼动和键盘不能同时占用 ESP32 串口。默认设备为 `/dev/ble2wchair`，本次检查指向
`/dev/ttyACM0`；默认 ROS domain 为 64。

统一界面包含眼动引导、启用眼动、停止／取消启用、BLE／归位状态、iPad 标定文件名
和中心步数。串口控制器后台运行并隐藏自身窗口，关闭统一界面会停止两个进程。
键盘模式仍有独立窗口：点击 Arm Keyboard Control；F 切换持续按住／释放左键，
Enter 发送一次点击，W/A/S/D 选择标定方向，松开方向键回中，Esc 释放并退出。

## 控制链路与仿真的关系

```text
相机 → RITnet + PuReST → 原子眼动样本 /eye_tracking/fusion_samples
                       → 共享手势状态机 → 标定目标 → USB串口 → ESP32 BLE HID
                       → iPad指针／按钮事件 → WHILL APP → 轮椅
```

仿真和 iPad 模式共用 `run_eye_sim.py → start_fusion.sh → fusion_tracker.py`，
共用融合检测、引导界面和眼动标定；iPad 使用共享 FusionInput 手势逻辑的直接输出适配器。
双眨眼启动，向左右看触发一秒转向，回中后允许再次触发；暂停后单眨眼恢复；
启动／恢复后有 0.5 秒视线缓冲。闭眼、无效输入、源帧超过 250 ms 时停车。

**完整运动控制链并非只换了输出设备：iPad 后端是直接遥控，未接入仿真的 Nav2
避障，也没有实测轮速／里程反馈。** 一秒方向输入不能保证真实轮椅执行特定转角。

## 固定的两份标定：不得混淆

### iPad 指针／摇杆标定

最终统一文件：

`projects/Ble2WChair/src/whill_controller/config/ipad_calibration_20261009.json`

它是本次对话最初键盘启动命令实际使用的
`~/.ros/whill_controller/calibration_logs/latest_calibration.json` 的完整快照，
生成时间为 2026-10-09 11:05:07。两者在本次核对时 SHA256 完全一致：

```text
d515e4c0c0aacaab961e4d5af1f4e94e9345acc84a232ccb18c5548bb2dd64dd
```

| 目标 | 相对右下边界的步坐标 |
| --- | --- |
| 中心 | (-16, -19) |
| 前进 | (-16, -27) |
| 左转 | (-24, -19) |
| 右转 | (-8, -19) |

中心归位保留文件原始步序，累计左16、上19；不是把原步序重新排序成两个直线段。
眼动和键盘默认使用同一快照，不自动跟随 `latest` 的后续变化。
`IPAD_CALIBRATION_FILE` 或显式 ROS 参数可以覆盖默认值，排查时以控制器输出的实际路径和哈希为准。

过程中曾把用户所说的“键盘那份”误解为原仓库 `default_calibration.json`
（中心左25、上14），导致切换错误。最终已恢复到上述当时键盘实际使用的快照。
旧 default 文件保留，但不是当前入口的默认文件。

### 眼动识别标定

直接读取原仿真文件：

`/home/whill/projects/EYETLE/state/fusion-calibration.json`

日期为 2026-10-16，SHA256：

```text
d71d47c064e3742136dc8aed5151c03184418da7794a66d61a7c2e6fb40d87c3
```

只读加载，启动仅确认正中，不重新采集／保存左右方向标定。
旧 USB hub 路径为 `4.4.1/4.4.2`，当前为 `4.1.1/4.1.2`，已添加这组已知路径迁移兼容。
左右顺序、旋转、尺寸及模型签名仍须匹配。左眼默认旋转180°，右眼0°。

本仓库 `projects/EYETLE/state/fusion-calibration.json` 曾在10月22日14:35重新生成，
其数值不同，**当前默认入口不使用它**。`FUSION_CALIBRATION` 日志明确记录实际路径、
loaded、saved_only、binding_remapped 和文件哈希。

## 日志证据

日志目录：`projects/EYETLE/model-results/fusion-traces/`。以下统计是帧记录数量，
同一个控制状态会出现在多帧中，不能把状态计数理解为独立动作或事件次数。

| 日志起始时间 | 发现 |
| --- | --- |
| 14:32 | 使用 missing-left-camera 占位路径，启动失败；实际是 USB 路径变化 |
| 14:33 | 886 个原子样本全部 calibrated=false |
| 14:35 | 1259 个样本，960 个 calibrated=true；14:35:26 写入了新的眼动标定 |
| 14:40 | 原仿真标定已成功加载；识别22次眨眼，但589帧反馈为 DISARMED / IPAD_NOT_READY |
| 14:42 | 原仿真标定成功加载；13个不同眨眼事件，已出现 ACTIVE、双眨眼启动和单眨眼恢复 |

本次最新可见 trace：
`20261022-144244-94be4d41-36d0-403a-bde8-dabd9604497b.jsonl`。
其中控制器状态为 DISARMED 338帧、ACTIVE 45帧、PAUSED 636帧；
FRAME_AGE_TIMEOUT 399帧、INVALID_INPUT 395帧；事件判定 ACCEPTED 588帧、
AUTHORIZATION_EXPIRED 216帧、REJECT_BEFORE_STOP 4帧。
这说明后续运行中眨眼并非一直被屏蔽，但输入时效／有效性导致的频繁暂停值得后续单独排查。
这些日志不能证明对应串口动作被 APP 当作拖动执行。

早期 IPAD_NOT_READY 合并了多个条件，无法从旧日志判断到底是未启用、未连接还是未归位。
现在反馈细分为 IPAD_NOT_ARMED、IPAD_BLE_NOT_CONNECTED、IPAD_HOMING_NOT_READY 等，
终端在条件变化时打印 Eye input gate，统一窗口显示具体条件。

## 指针移动但 APP 滑杆不动

### 已确认／用户反馈

- ESP32 到 iPad 的连接链路是通的，指针能按照期望轨迹移动。
- 用手指拖动 WHILL APP 滑杆时，滑杆会变化。
- nimble 的按钮操作改成“轻点”后，指针控制滑杆仍不生效。
- 没有证据表明修改眼动标定能解决此问题，当前不应继续切换标定文件。

### 当前主机实现

程序在标定中心发送 `[`（左键持续按下），然后发送方向移动字节；
停止时发送 `]`（释放），再回中心。换方向时先释放、回中心，再重新按住拖动。
键盘 Enter 对应串口回车（单次点击），F 对应 `[`／`]`。

协议预期移动期间持续保持 HID 左键状态。但仓库没有完整 ESP32 固件源码，
尚未抓取实际 BLE HID 报告，因此还不能证明每个移动报告都保持了按下位。
主机 `/whill/controller/command` 是指令审计，不是 iPad 或 APP 的执行回执。
`left_button_held` 同样是主机维护的状态，不是 iPad 返回的触摸接触状态。

### 已讨论的 iPad 设置及其局限

此前建议检查辅助触控、nimble 按钮映射、执行触控手势，以及停留控制／拖移锁定。
用户明确确认的是“改为轻点仍无效”；其余开关状态和当前 iPadOS 版本尚未完整记录。
不能把全部建议当成用户已经完成的设置，也不能承诺某个开关必然使 APP 接受拖动。

Apple 的设备按钮与拖移说明可参考：
[将指针设备与辅助触控搭配使用](https://support.apple.com/zh-cn/111775)。

### 下一步应验证什么（未执行）

1. 记录 iPadOS、WHILL APP 版本和当前辅助触控设置，保留当前标定与指针速度。
2. 在不会驱动轮椅的普通系统滑块上验证同一套“按住—移动—释放”。
   若也失败，检查 HID 按钮状态及触控手势映射；若成功而 WHILL 失败，重点检查 APP 的触控兼容性和按下位置。
3. 记录主机 command 审计及 ESP32 实际收到的字节，核对是否确实出现 `[`、移动、`]`。
4. 检查／抓取 ESP32 HID 报告：按下后各移动报告是否一直保留左键位，释放报告是否清零。
   “点击识别成功”与“拖动期间持续按住成功”是两项独立验证。
5. 若连续按住报告无误，比较普通鼠标、nimble 与手指在同一滑杆上的行为，
   检查中心是否命中滑杆可拖动区域，以及按下到首个移动之间的时序。
6. 独立排查眼动 FRAME_AGE_TIMEOUT／INVALID_INPUT 导致的快速停止，避免把输入中断误判为 iPad 拖动故障。

当前不应直接判定是 ESP32 固件坏了、iPad 设置错误或 WHILL APP 不支持鼠标；这些都仍需证据。

## 本次已完成的验证与边界

最近一次相关检查：15项控制器检查、8项启动器测试、16项眼动标定／摄像头测试，合计39项通过。
控制器集成测试覆盖真实 ROS DDS、统一窗口按钮、隐藏后台窗口、陈旧启用消息拒绝、
双眨眼启动、标定目标、断流释放、单眨眼恢复、停止和固定标定哈希；另做了窗口布局检查。

这些测试使用离线串口，**不等于 BLE 持续按住、WHILL APP 拖动和真实轮椅运动已验收**。
没有修改或重新烧录 ESP32 固件。本次工作仍未完成真实轮椅闭环验收。
主机进程失效时的固件独立超时释放、BLE断链释放、实际停车效果需要现场验证。

操作说明及测试命令见 [EYE_IPAD.md](EYE_IPAD.md)。
