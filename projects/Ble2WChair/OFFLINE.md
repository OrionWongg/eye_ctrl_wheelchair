# Orin 无硬件运行与验收

路径：`/home/whill/projects/Ble2WChair`。已在 Ubuntu 22.04 / ROS 2 Humble / ARM64 构建。

## 一键运行

```bash
cd ~/projects/Ble2WChair
./run_offline.sh
# 直接进入 APP 模式：
./run_offline.sh whill_mode:=true
```

此入口强制 `hardware_enabled:=false`，不会打开真实串口。窗口打开后点击 **Arm Keyboard Control**，再使用 W/A/S/D；APP 模式下 F 切换按住状态，按住时方向键才更新估计里程计。Esc 退出。

需要查看 ROS 数据时，在另一个终端执行：

```bash
source ~/projects/Ble2WChair/whill_ros_env.bash
ros2 topic echo /whill/controller/status
ros2 topic echo /whill/controller/odometry
```

环境脚本已改为根据仓库所在位置加载工作空间，不再引用旧电脑的 `/home/zyy/whill_control_ws`。默认使用 ROS domain 57、仅本机发现。

## 可重复的无硬件测试

```bash
source ~/projects/Ble2WChair/whill_ros_env.bash
# 独立测试域，避免与正在运行的演示混用。
export ROS_DOMAIN_ID=157
xvfb-run -a python3 ~/projects/Ble2WChair/tools/smoke_pipeline.py \
  --output ~/pipeline-validation/ble-dry-run
xvfb-run -a python3 ~/projects/Ble2WChair/tools/smoke_pipeline.py \
  --mock-serial --output ~/pipeline-validation/ble-mock-serial
```

第一项测试实际 Tk 窗口事件、ROS 输入与命令、按住状态限制、里程计、轨迹、按键时长、重置和退出，完全不开串口。
第二项自动创建 `/dev/pts/*` 虚拟串口，模拟 ESP32 的 `BLE_CONNECTED=0/1` 回复，验证 PySerial 实际收发、连接前禁止归位、断连停止跟踪、重连重新归位和退出时释放按钮。不会选择 `/dev/ttyACM*` 或 `/dev/ttyUSB*`。

原仓库测试：

```bash
cd ~/projects/Ble2WChair
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select whill_controller
colcon test-result --verbose
```

本次原有 8 项测试通过；无硬件运行与虚拟串口流程也已通过。结果和命令字节记录在 `~/pipeline-validation/`。

## 验证范围

验证的是电脑端 GUI → ROS → 命令/虚拟串口，以及状态和估计里程计。仓库不包含完整 ESP32 固件，真实 BLE 无线链路、iPad 指针、轮椅和实体标定需要连接硬件后另行验证。两个项目均未设置开机自启。
