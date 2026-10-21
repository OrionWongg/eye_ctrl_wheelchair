> 统一项目入口与当前状态见 [根目录 README](../../README.md)。默认融合手势以 [2026-10-19 接入总结](../../projects/EYE_WHEELCHAIR_INTEGRATION_SUMMARY_20261019.md) 为准；下文保留历史模式说明。

# Eye Turtle Control

> **2026-10-16：默认海龟入口为 RITnet + PuReST 二维融合。** 算法、ROS 接口、数据格式和最新修复见 [技术总览](docs/EYE_CONTROL_ARCHITECTURE_AND_INTERFACES.md)，操作见 [当前使用总结](CURRENT_EYE_CONTROL_SUMMARY.md)。下文三维方案为历史/对照入口。


**轮椅眼动 + 避障试用：关闭旧轮椅仿真后运行 `./start_eye_wheelchair.sh`。默认使用 RITnet 双目接口和三维中心参考（每只眼均需有效参考），接入当前 directional_intent/Nav2 链路；看正中/左/右稳定后双眨眼确认，长闭眼或丢眼停车。详见 [眼动方向接口使用说明](../../wheelchair_shared_ws/docs/EYE_DIRECTIONAL.md)。**

**当前眼动方案、启动与标定状态总结：[CURRENT_EYE_CONTROL_SUMMARY.md](CURRENT_EYE_CONTROL_SUMMARY.md)。历史项目总结：[PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)。**

**历史三维对照入口：RITnet 预训练模型 + TensorRT。运行 `./start_ritnet.sh`，或 `./start_eye_turtle.sh --ritnet` 控制海龟。默认三维模式先扫视建模，再看正前方按 S 确认中心，自动保存并在后续启动加载有效参考。见 [GAZE3D.md](GAZE3D.md)。**

**仅检查左眼相机画面（不运行模型）：`./start_left_eye.sh`（已修正当前安装的倒置画面）。**

**旧版双摄像头阈值／模板入口（保留）：预览运行 `./start_dual_eye.sh`；双眨眼控制海龟运行 `./start_eye_turtle.sh --dual`，见 [DUAL_CAMERA.md](DUAL_CAMERA.md)。**

**Orin 无硬件启动与验证：见 [OFFLINE.md](OFFLINE.md)。**

ROS 2 Humble package for controlling turtlesim using eye signals.

The ROS 2 package is located at `src/eye_turtle_control`, following the standard colcon workspace layout.

The unused built-in turtlesim color sensor is remapped to a hidden topic, so `/turtle1/color_sensor` is not exposed in the normal topic list.

Inspect topic publishers/subscribers and distinguish local, remote, and stale local endpoints:

```bash
./tools/check_ros_topics.sh /cubemap/front/image
./tools/check_ros_topics.sh                 # inspect every visible topic
```

The launch file starts turtlesim and the controller only. Use the MediaPipe test script as the sole camera/signal publisher.

For V4L2 devices, select an explicit node when needed: `camera_device:=/dev/video2`. Use `v4l2-ctl --list-devices` and `v4l2-ctl -d /dev/videoN --list-formats-ext` to find the color stream.

## MediaPipe test tracker

The more accurate FaceMesh/iris test is independent from the Haar node:

```bash
python3 -m pip install -r requirements-eye-tracking.txt
python3 tools/eye_tracking_test.py --camera 0 --publish-ros
```

Start turtlesim and the controller separately when using this script:

```bash
ros2 run turtlesim turtlesim_node
ros2 run eye_turtle_control eye_turtle_controller
```

Do not run more than one eye tracker at the same time. After startup, complete two short blinks to arm forward motion; center gaze alone intentionally keeps the turtle stopped.

The test window shows iris points and `A/D/W/S` direction labels. Press `Esc` to exit.

## One-command startup

```bash
chmod +x start_eye_turtle.sh
./start_eye_turtle.sh       # default camera 0
./start_eye_turtle.sh 2     # camera 2
```

The script starts turtlesim, the controller, and the MediaPipe tracker. Press `Esc` in the tracking window or `Ctrl+C` in the terminal to stop all processes.

To try actual keyboard events instead of direct Twist control:

```bash
python3 -m pip install pynput
CONTROL_MODE=keyboard ./start_eye_turtle.sh 0
```

Keep the turtlesim window focused. Return to the default robust mode with `CONTROL_MODE=twist` (or omit it).

## Input

Publish `std_msgs/msg/Float32MultiArray` on `/eye_tracking/signals` with three values:

`[horizontal, eye_closed, valid]`

`horizontal` is normalized to `[-1, 1]` (left negative, right positive). `eye_closed` and `valid` are `0` or `1`.

## Gestures

- Two short blinks while stopped: forward immediately after the second blink.
- Closed eyes held for 650 ms: stop immediately when the long blink is detected.
- Look left/right while moving: turn left/right. Center gaze drives straight.
- Missing/invalid input for 400 ms: stop.
- Healthy motion has no five-second cap. Long closure pauses the session; look center, then to a side for 250 ms, then center for 700 ms to resume.
- Input loss cancels the resume gesture; recovery alone never restarts motion. The RITnet preview shows controller state and resume progress.
- While moving, a left/right gaze produces a 1-second turn pulse; recenter your gaze before another turn.
- Near a turtlesim boundary, the controller stops and recenters the turtle at `(5.54, 5.54)` automatically.

Emergency stop from another terminal: `ros2 topic pub --once /eye_turtle/stop std_msgs/msg/Bool "{data: true}"`.

The default `control_mode` is direct ROS2 Twist control. To try real `W/A/S/D` key events, install `pynput` and start the controller with `--ros-args -p control_mode:=keyboard`; keep the turtlesim window focused.

## Run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch eye_turtle_control eye_turtle.launch.py
```


## Gazebo 轮椅避障仿真

```bash
./start_eye_turtle.sh --gazebo --ritnet --eyes both \
  --calibration-file /home/whill/projects/EYETLE/state/ritnet-calibration.json
```

需先关闭旧轮椅仿真。该入口使用 `/home/whill/wheelchair_shared_ws` 的 Gazebo 办公室、眼动意图适配器和公共避障控制，默认 ROS domain 64；不加 `--gazebo` 保留原 turtlesim 行为。Ctrl+C 或退出 RITnet 会关闭本次启动的整套仿真。详细操作见 wheelchair_shared_ws/README.md 的“EYETLE RITnet 双目眼动一键启动”。

### 轮椅候选确认反馈

轮椅使用 `intent_confirm` 时，RITnet 预览读取 `/eye_control/feedback` 的新结构，显示三行英文提示：原始 `Preview`、实际候选及 `stable`，当前 `Task` / 实际 `action` / `ACK`，以及拒绝、阻挡或停车原因。左右候选明确显示有限角度（默认 `15.0 deg`），正中为 `CONTINUE (limited task)`；这不是物体选择或三维注视识别。原始预览与仍稳定的旧候选分别显示，避免把扫视误显示为已确认选择。`ACK: ACCEPTED` 仅表示授权被接收，实际动作看 `action`。反馈过期显示 UNKNOWN，不继续显示旧动作。旧 `gaze_joystick` 的恢复提示保留。

这些显示不修改检测、标定或眼动分类参数；OpenCV 内置字体使用英文以避免中文乱码。带源时间戳输出可用 `--publish-stamped`，并通过 `EYE_WHEELCHAIR_SETUP` 显式加载轮椅工作空间；时间戳表示源端最旧相机读帧完成时刻，不是硬件曝光时刻。
