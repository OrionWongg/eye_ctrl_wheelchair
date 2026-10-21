# 双目近眼摄像头

本页为旧版阈值／模板方案。当前推荐预训练模型入口，见 [RITNET.md](RITNET.md)。

只连接左眼时，运行 `./start_left_eye.sh` 打开单眼预览（不启动海龟、不发布控制信号）。当前已确认左眼在 USB 4.4.1、设备 `/dev/video2`，佩戴画面倒置；左眼默认旋转 180°，修正同时作用于预览、选区、标定及检测。右眼默认不旋转。

可用 `--left-rotation 0/90/180/270` 和 `--right-rotation 0/90/180/270` 指定顺时针旋转角度；额外水平镜像使用 `--left-mirror` 或 `--right-mirror`。更改方向后需要重新标定。

启动两个摄像头的预览、瞳孔检测及 ROS 发布：

```bash
cd /home/whill/projects/EYETLE
./start_dual_eye.sh
```

使用独立的 OpenCV 暗瞳孔检测入口，适配单眼近拍。原来的 `start_eye_turtle.sh` 和 MediaPipe 人脸入口保留。

当前两个 HBVCAM 的采集节点是 `/dev/video2`（USB 4.4.1）和 `/dev/video0`（USB 4.4.2）；另外两个节点不是独立摄像头。默认按 `/dev/v4l/by-path/*-video-index0` 排序取前两台，以 USB 接口区分相同序列号设备。更换接口或接入其他摄像头后请显式指定：

```bash
./start_dual_eye.sh --left-camera /dev/video0 --right-camera /dev/video2
```

画面 CAM 1/2 分别对应传入的 left/right 参数，程序无法自动判断实际佩戴侧。默认请求 MJPG 640×480、30 FPS，启动日志输出实际协商参数。两路软件采集，不是硬件同步。

1. 佩戴后调整摄像头，使每路清晰拍到一只眼睛。资料标注默认物距 30 cm 至无穷远（可调）；近眼安装时需实际确认对焦，程序不能修复光学失焦。
2. 按 `R`，依次框选两路眼睛区域，按 Enter 确认。尽量排除眉毛、镜框和大块阴影。绿色椭圆是候选瞳孔，须肉眼确认标记正确。
3. 看左并保持至少一秒，按 `1`；看正中保持至少一秒，按 `2`；看右保持至少一秒，按 `3`。每个标定点需连续 20 帧检出两眼瞳孔。左右标定也适配镜像安装。移动设备或重新选区后重新标定。
4. 按 `B` 后立即闭上双眼并保持约 4 秒。程序等待 2 秒后采集 20 帧闭眼模板；重新睁眼后检查提示是否保存成功，否则重试。标定过程中不会输出有效控制信号。
5. `calibrated=True` 且 `valid=1` 时，`gaze` 输出左负右正的归一化水平位置。按 Esc 退出。

ROS 话题沿用 `/eye_tracking/signals`，类型 `std_msgs/msg/Float32MultiArray`，内容 `[horizontal, eye_closed, valid]`。沿用 `eye_ros_env.bash` 的 ROS domain（默认 58）。未选区域、未标定、未完成睁闭眼标定时输出无效。已标定后，闭眼由各眼图像与闭眼模板的匹配确认，双眼同时闭眼才输出 `eye_closed=1`，此时无需检出瞳孔。两眼不同步或无法分类时最多沿用上一状态 100 ms，随后输出无效；退出发送无效信号。

启动双摄像头、海龟与控制器：

```bash
./start_eye_turtle.sh --dual
```

不要同时运行 `start_dual_eye.sh` 和上述命令。完成全部标定并重新睁眼后：

- 双眨眼：前进。单次闭眼至少 60 ms，小于 650 ms；两次睁眼事件在 900 ms 窗口内。第二次眨眼结束即前进，不再等待第三次眨眼。
- 持续闭眼达到 650 ms：停止。
- 单次眨眼不启动；单眼眨动不作为双眼闭合；不明图像持续超过 100 ms 时取消待识别手势并停止。双路启动模式将眨眼冷却时间设为 120 ms，支持较快的连续眨眼。

睁眼模板取自 `1/2/3` 三个标定点；`B` 自动采集闭眼模板。睁闭眼图像差异不足或闭眼采样不稳定时会拒绝标定。每次重新选区域、修改方向标定，都需要重新做闭眼标定。模板仅保存在本次进程内。

这是需要个人标定的图像模板方法，尚未经过真实佩戴眨眼数据验证；光照变化、镜头位移或与闭眼外观相似的遮挡可能导致漏检或误检。应确认窗口中 `OPEN/CLOSED/UNKNOWN` 与实际一致。摄像头实际帧率低时，极短眨眼可能无法采到足够帧。

无窗口硬件检查（无需眼睛或标定）：

```bash
./start_dual_eye.sh --no-display --max-frames 120 --report /tmp/eyetle-dual-report.json
source eye_ros_env.bash
.venv/bin/python -m unittest discover -s test -p 'test_dual*.py'
```
