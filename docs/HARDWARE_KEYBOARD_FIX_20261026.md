# 键盘焦点修复（2026-10-26）

启动方式（先单独启动传感器，ROS_DOMAIN_ID 保持一致）：

```bash
cd /home/whill/eye_ctrl_wheelchair
WHEELCHAIR_ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/run_hardware_keyboard.sh
```

等待 iPad APP 居中后点击 Arm Keyboard Control，可直接在该窗口按 W/A/D/U/C、空格/S；终端输入仍可用。W 启动/继续，A/D 停车时转向一次、行驶时选择方向，U 掉头，C 取消偏好，空格/S 停止。终端显示按键来源、事件编号和接受/拒绝原因。

GUI 通过 `/wheelchair/keyboard/gui_key` 发送带 ROS 时间戳的按键。direction_keyboard 丢弃年龄超过 250 ms 或时间无效的消息，复用原有会话、心跳、事件去重、重发和回执；GUI 按键不直接驱动 iPad。非停止按键要求 GUI 已 Arm；解除 Arm 和关闭窗口发送停止。终端进程仍需运行并处于终端前台进程组。

脚本固定使用本仓库 `wheelchair_shared_ws/install`，检查控制、桥接、导航和 iPad 包路径，禁止静默回退外部工作区。本次源码、测试和有效构建产物均在仓库内；先前临时同步至外部工作区的 direction_input.py 已恢复。Nav2 依赖复制到仓库 vendor 并修正其 CMake 中的旧绝对路径。

无硬件验证：按键消息时效和去重、真实 ROS GUI topic → PTY 键盘进程 → DirectionEvent、方向管理回归共 28 项通过；iPad GUI 按键转发及导航输出/停车/超时/重复消息测试 1 项通过。测试使用独立 ROS domain 和 hardware_enabled=false。只解析硬件 launch 参数，没有启动硬件控制栈。

硬件所需 6 个共享工作区包与 whill_controller 已构建。仿真专用 wheelchair_scan_viz 缺少 ignition-gui6 库，未构建，不在硬件启动链路中。

本次只修复按键入口和运行路径。原排查文档中的地图几何冲突、感知超时和规划周期问题仍可能拒绝运动；真实运动需现场验收。
