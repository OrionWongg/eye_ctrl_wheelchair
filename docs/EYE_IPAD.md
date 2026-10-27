# 眼动与 iPad 控制

当前实机现象：指针可移动，但 APP 滑杆不跟随；改成“轻点”仍无效。
完整过程与待验证项见 [2026-10-22 接入总结](EYE_IPAD_INTEGRATION_SUMMARY_20261022.md)。

## 启动

```bash
./scripts/project.sh eye-ipad            # 真实串口
./scripts/project.sh eye-ipad --dry-run  # 离线，不打开串口
```

也可使用 `./scripts/project.sh eye --ipad`。
仿真和真实入口共用 `run_eye_sim.py → start_fusion.sh → fusion_tracker.py`，
iPad 模式把启用、停止、BLE／归位状态和眼动引导合并到同一个窗口。
后台串口控制器隐藏自身窗口。关闭界面会停止两个进程。
默认 ROS domain 64，可通过 `WHEELCHAIR_ROS_DOMAIN_ID` 修改。

完成眼动正中确认、BLE 连接和 iPad 指针归位后，在界面下方点“启用眼动”。
双眨眼启动前进，向左右看触发一秒转向，回中后允许再次转向。
闭眼、输入失效或源帧超过 250 ms 时释放鼠标；暂停后单眨眼恢复。
“停止 / 取消启用”立即撤销启用；再次启用后需重新双眨眼。
启用按钮在反馈中断、蓝牙未连接或未归位时禁用；过期启用消息不能触发启用。

键盘模式仍使用独立键盘窗口：

```bash
./scripts/ipad_controller.sh -p eye_control_enabled:=false
```

此入口现在默认启用 `keyboard_auto_drag:=true`。等待归位完成，点击
**Arm Keyboard Control**，按住 W/A/S/D 自动在中心按下左键并拖到标定目标；
持续按键保持目标，松开所有方向键发送释放并回中。相反方向抵消时也释放回中。
APP LOCKED 自动拖动模式下 F / Enter 禁用，避免手动按钮状态与自动拖动冲突。
Disarm / Esc 释放按钮。O 切换到 FREE SCREEN 后仍为手动 F 按住／释放。
如需复现原来的手动按钮操作，加 `-p keyboard_auto_drag:=false`。

例如 W 的主机串口序列为 `[` → `wwwwwwww` → 保持 → 松键后 `]` → `ssssssss`。
不周期重复位移，也不反复点击；ESP32 必须在移动 HID 报告中保留左键按下位。
此修改仅确认主机自动发送拖动序列，不代表 iPad / WHILL APP 已接受拖动。
先在不会驱动轮椅的系统滑块上验证。若此前 F + W 也失败，仍需排查实际固件
按钮报告、iPad 触控映射及按下位置。

两种模式不要同时运行，它们使用同一串口 `/dev/ble2wchair`。

## 10 月 9 日旧代码对照

若自动拖动仍失败，可用 `./scripts/ipad_keyboard_20261009.sh` 直接运行本机保留的旧源码，
按 Arm → O → 等归位 → F → 方向键操作。旧版松方向键仍保持按下，第二次 F 才释放。
核对依据、指令序列和限制见 [历史实现对比](IPAD_KEYBOARD_20261009_COMPARISON.md)。

## 固定使用的两份标定

**iPad 摇杆标定**：统一使用
`projects/Ble2WChair/src/whill_controller/config/ipad_calibration_20261009.json`。
这是本次对话最初键盘命令实际加载的 `latest_calibration.json` 的完整快照，
SHA256 为 `d515e4c0c0aacaab961e4d5af1f4e94e9345acc84a232ccb18c5548bb2dd64dd`。
中心相对右下边界为 `(-16,-19)`；前进 `(-16,-27)`、左转 `(-24,-19)`、右转 `(-8,-19)`。
保留中心原始步序，不改写标定。眼动、键盘及仓库硬件启动脚本默认都指向此快照，
不自动跟随 `latest`，也不切回旧 `default_calibration.json`（左25／上14）。
窗口显示文件名与中心步数，控制器日志打印完整路径和 SHA256。
如需明确指定另一份文件，可设置 `IPAD_CALIBRATION_FILE`。

**眼动识别标定**：直接读取原仿真文件
`/home/whill/projects/EYETLE/state/fusion-calibration.json`（2026-10-16）。
只读使用，启动仅确认正中，不采集新的左右标定。
只兼容已知 `4.4.1/4.4.2 → 4.1.1/4.1.2` USB hub 路径变化，
左右顺序、旋转、图像尺寸和模型签名仍需匹配。
`FUSION_CALIBRATION` 日志报告文件路径、loaded、saved_only 和 SHA256。
`EYE_FUSION_CALIBRATION_FILE` 可指定另一份已有眼动文件；它与 iPad 标定完全独立。

本机复用了 `/home/whill/projects/EYETLE` 的虚拟环境、ROS install、PuReST 库和
TensorRT 引擎；运行的算法和界面为本仓库源码。本机资源链接不随 Git 分发。

## 接口与能力范围

`/eye_tracking/fusion_samples`（schema 1 JSON）→共享手势状态机→标定目标→ESP32。
`/eye_control/feedback` 提供启用、阻挡原因、BLE、归位、标定文件及按住状态。
`/whill/controller/ui` 接收统一界面的带时间戳 arm/stop；仅统一界面模式启用。
`/whill/controller/input` 在眼动模式仅接受 stop。
`/whill/controller/command` 审计串口指令，离线时 hardware_enabled=false。

新方向先释放、回中心，再按住拖动；停止先释放再回中心。
输入源重启撤销启动状态，重复样本不会延长看门狗。
此版本是直接遥控，没有接入仿真 Nav2 避障，也没有真实轮速反馈。
主机看门狗依赖进程存活，实机仍需验证固件断链释放和实际停车行为。

## 验证

```bash
cd projects/Ble2WChair/src/whill_controller
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD:../../../../wheelchair_shared_ws/src/eye_shared_control:$PYTHONPATH"
ROS_DOMAIN_ID=165 ROS_LOCALHOST_ONLY=1 xvfb-run -a /usr/bin/python3 -m pytest test -q
```

集成测试使用真实 DDS、统一窗口按钮和隐藏串口控制器，验证启用／停止、
陈旧启用消息拒绝、双眨眼前进、方向目标、断流释放、恢复和标定一致性。
全部使用离线串口；不驱动 iPad 或轮椅。
