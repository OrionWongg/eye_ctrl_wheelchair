# Orin 无摄像头运行与验收

路径：`/home/whill/projects/EYETLE`。已在 ROS 2 Humble / ARM64 构建。

## 一键运行合成手势演示

```bash
cd ~/projects/EYETLE
./start_eye_turtle.sh --demo
# 连续演示：
./start_eye_turtle.sh --demo --loop
```

这会打开真实 turtlesim，使用明确标记为 SYNTHETIC INPUT 的眼动信号演示：双眨眼前进、左右转、长闭眼停止。无需摄像头，也不会启动 MediaPipe。单次演示结束或 Ctrl+C 后，仅清理本次启动的进程，不再使用全局 pkill。

查看话题需使用相同环境：

```bash
source ~/projects/EYETLE/eye_ros_env.bash
ros2 topic echo /eye_tracking/signals
ros2 topic echo /turtle1/cmd_vel
```

默认 ROS domain 58、仅本机发现。可通过 EYE_ROS_DOMAIN_ID / EYE_ROS_LOCALHOST_ONLY 覆盖。

## 真正运行 MediaPipe 的离线视频流程

```bash
cd ~/projects/EYETLE
./start_eye_turtle.sh --video test/fixtures/portrait.avi
# 无视频窗口、处理 100 帧：
./start_eye_turtle.sh --video test/fixtures/portrait.avi --no-display --max-frames 100
```

此流程执行实际 FaceMesh / iris 推理，再发布眼动信号到控制器和 turtlesim。
测试视频由 MediaPipe 官方公开人像测试图片生成，来源及 SHA256 见 `test/fixtures/provenance.json`。它是静止人脸，只验证推理、校准、ROS 传输和保持停止；实际前进和左右转向及眨眼状态机由独立合成手势测试验证，不能把静止图片当作真实眨眼识别精度验证。

重建测试视频：`python3 tools/make_offline_video.py`。
图片来源：https://storage.googleapis.com/mediapipe-assets/portrait.jpg

## Python 环境

独立环境：`~/projects/EYETLE/.venv`，使用 `--system-site-packages` 访问系统 ROS。
原固定 MediaPipe 0.10.21 不提供 Python 3.10 ARM64 wheel，现按架构选择 0.10.18；x86 仍使用 0.10.21。
依据：[MediaPipe 0.10.18 官方发布文件](https://pypi.org/project/mediapipe/0.10.18/#files)。

```bash
cd ~/projects/EYETLE
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-eye-tracking.txt
.venv/bin/python -m pip check
```

已验证的本地依赖版本保存在 `requirements-orin-lock.txt`。MediaPipe 使用 CPU / XNNPACK 推理；虚拟环境包含 OpenCV 4.11，系统原有 OpenCV 4.12 CUDA 保留，且已分别验证纯 ROS 环境与默认 CUDA OpenCV 环境下的视频流程。
模型启动时可能打印 SVE 能力探测和 feedback tensor 警告；验收中模型实际完成了 100 帧推理，并非用模拟推理替代。

## 功能修复

- 双眨眼完成即前进，后续眨眼不改变行进方向；已移除后退功能。
- 缺失字段、NaN / 非有限值和无效输入均停止，不保留旧的待确认眨眼序列。
- 校准完成前不发布有效控制信号。
- 支持视频输入、无窗口运行、帧数限制与 JSON 推理报告。
- 启动脚本使用本次进程组清理，修复退出时的 ROS context 异常。

## 可重复验证

```bash
source ~/projects/EYETLE/eye_ros_env.bash
xvfb-run -a python3 ~/projects/EYETLE/tools/smoke_pipeline.py \
  --output ~/pipeline-validation/eye-after
xvfb-run -a python3 ~/projects/EYETLE/tools/smoke_video_pipeline.py \
  --output ~/pipeline-validation/eye-video
```

ROS / turtlesim 检查覆盖前进、第三次眨眼保持前进、左右转、转弯脉冲、长闭眼停止、无效/NaN/畸形输入停止、信号超时、紧急停止、运动时限和边界回正。
视频链路实测 100/100 帧检测到人脸，校准后收到 71 条有效眼动信号，静止注视没有误触发运动。
结果位于 `~/pipeline-validation/eye-after/`、`eye-video/` 和 `eye-video-user-env/`。

## 接入摄像头时

```bash
./start_eye_turtle.sh 0
```

真实摄像头曝光、用户视线校准、实际眨眼识别准确率仍需届时验证。当前默认 Twist → turtlesim 流程已验收；可选 pynput 键盘后端未作为本次核心验收路径。未给项目配置开机自启。
