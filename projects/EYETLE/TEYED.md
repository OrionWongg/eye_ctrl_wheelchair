# TEyeD 双眼方向 + RITnet 眨眼

2026-10-16 更新。TEyeD 对照入口使用 TEyeD 方向 + RITnet 睁闭眼，MEYE 只作显式选择的对照。目标是省去每次启动扫视、三维拟合和按 S 的步骤；仍需启动时看正前方完成一次自动中心确认，不宣称跨摄像头免标定或抗镜架移位已经验证。

## 使用

```bash
cd /home/whill/projects/EYETLE
./start_eye_turtle.sh --teyed 2>&1 | tee model-results/teyed-live.log
```

无参数海龟启动已切到 [RITnet + PuReST 融合](FUSION.md)；本方案请显式使用 `--teyed`。启动后戴稳眼镜，看正前方、保持双眼睁开约 2 秒；预览显示采集进度。模型输出和开眼状态连续稳定时自动完成，无需键盘确认。进度清零表示样本中出现闭眼、未知状态、延迟或方向波动，不应通过放宽阈值掩盖。

中心就绪后双眨眼启用，长闭眼约 650 ms 暂停；暂停后先睁眼，再短眨眼一次恢复。跟踪失效停车。按 C 停车并重新自动采集中心，Esc 退出。中心仅存在于当前运行中，不读取或改写旧三维中心文件。重新佩戴或镜架明显移位后，需要停止并重新采集中心。

仅预览（不发布 ROS 信号）：

```bash
.venv/bin/python tools/teyed_tracker.py --eyes both
```

`./start_teyed.sh` 发布眼动信号，但不启动海龟控制器。
旧方案仍可使用 `./start_eye_turtle.sh --ritnet --eyes both`。
轮椅 `--gazebo` 暂时保留原入口；新检测器没有接入轮椅的 stamped 协议。

## 实现

- TEyeD：每眼独立推理，完整纠正方向后的眼图缩放到 96×96 RGB，按作者推理脚本归一化到 [-1,1]，输出视线向量的二维分量。不是屏幕坐标或角度。
- RITnet：复用原有 TensorRT 全图定位、320×240 裁剪与双眼 batch 分割。可靠瞳孔为睁眼；最近 1.5 秒内曾睁眼、眼区分割面积显著塌缩、周围图像稳定且分割置信度足够时判闭眼。其余为 unknown，不把 unknown 直接当闭眼。裁剪区域变化时重建外观参考。只调用 `EyeState`，不创建 `EyeGeometry`，不读取三维中心。
- 中心：连续至少 2 秒、至少 20 对新鲜睁眼样本，两个分量的窗口极差均 ≤ 0.06。取中位数。稳定不等于正前方，用户必须遵循启动提示。
- 控制：左右眼分别减去中心 x，乘方向符号，差值 ≤ 0.15 时取均值，除以 0.25 后截断到 [-1,1]。这些是初始工程参数，不是现场优化结果。
- 时间：摄像头读取完成时间差 <80 ms，处理后帧龄 <250 ms。没有硬件曝光同步。
- 双眼均闭合才形成闭眼状态；开闭不同步最多宽限 100 ms。闭眼时不要求 TEyeD 给出有效方向，但必须有最近 1.2 秒内的有效睁眼方向和已经确认的中心。输入未知、过期或方向冲突会停车并取消手势历史。
- 中心完成后冻结，不会把持续看侧方学成正中。图像移位的可靠检测和跨用户泛化仍需验证；这不是自动抗滑移方案。

左右方向符号默认 -1；若实测方向相反，可用 `--left-gaze-sign 1` / `--right-gaze-sign 1`。必须在海龟中验证左右方向，不能把来源数据集的坐标约定直接视为当前安装的真值。

## 模型和构建

- TEyeD 来源：https://github.com/jtlicardo/teyed-gaze-regression
- MEYE 来源：https://github.com/fabiocarrara/meye
- 原始权重 URL 和 SHA256：`models/teyed/sources.json`。
- 转换：`tools/build_teyed.sh`，使用独立 `.venv-model`，依赖固定在 `requirements-teyed-export.txt`；运行使用现有 `.venv` 的 TensorRT，不需要 TensorFlow。
- Keras(torch backend) → ONNX → TensorRT FP16，构建引擎需在目标 Jetson 执行。转换脚本校验 Keras/ONNX 输出。

## 2026-10-16 组合方案验证

56 项自动化测试通过。新增测试覆盖 RITnet 分割到闭眼状态、裁剪变化清除参考、全图坐标反馈、遮挡拒绝。控制器集成测试用合成分割输出经过真实 RITnet 闭眼规则、TEyeD 双眼融合和海龟回调，验证自动中心、双眨眼启动、长闭眼暂停、单眨眼恢复。

实际双摄像头 150 帧短测未发布 ROS 信号，见 `model-results/teyed-ritnet-smoke.json`。双眼方向+分割+融合中位数约 67.8 ms，P95 111.4 ms；本次画面未满足连续稳定睁眼条件，没有完成中心，也没有有效控制输出。该短测只证明组合模型能在本机运行，不证明佩戴时启动成功或眨眼准确率。

运行时窗口标题为 `EYETLE TEyeD + RITNET`，启动日志显示 `TEYED direction + RITNET closure`。`TEYED_INPUT` 新增 `blink_method` 与两眼 `blink_details`（瞳孔置信度、面积、分割置信度、ROI），不再用 MEYE 概率解释 RITnet 的闭眼判断。

## 历史 TEyeD + MEYE 验证与边界

已检查自动中心的稳定性、帧过期、断流、非有限数、闭眼输入拒绝、中心不随转眼漂移、双眼冲突、独立闭眼保持，以及真实海龟控制器的启动/暂停/恢复回调。

本次全部 52 项 unittest 测试通过。

`model-results/teyed-runtime-validation.json`：25 组输入的 TensorRT/ONNX 数值比对；最大绝对差约 TEyeD 0.00303、MEYE 0.000048。数值一致不代表识别准确。

`model-results/teyed-camera-smoke.json`：双摄像头 80 帧不发布信号短测，双眼四次网络推理及融合耗时中位数约 22.7 ms、P95 28.1 ms；短测没有完成中心确认，因此不能据此宣称实际控制验证通过。该耗时不包含摄像头曝光/USB延迟，也不等于显示帧率。

随后 `model-results/teyed-camera-extended.json` 的 300 帧不发布信号检查中，自动中心已完成，产生 41 帧有效输入；双眼推理及融合中位数约 24.8 ms、P95 28.3 ms。没有有效闭眼帧；未经用户动作标签核对，不能推断眨眼准确率、方向准确率或正前方中心正确性。

尚待实际佩戴验证：正前方自动中心成功率、左右方向符号和区分度、自然眨眼/长闭眼/遮挡的混淆、不同用户与镜架移位。不能沿用原 RITnet 的准确性或性能结论。

离线双眼回放（必须为相同 FPS、已对齐的视频，禁止发布控制）：

```bash
.venv/bin/python tools/teyed_tracker.py --left-video left.mp4 --right-video right.mp4 --no-display --report model-results/teyed-replay.json
```

## MEYE 对照模式的闭眼阈值（不用于 RITnet）

MEYE 对照模式的闭眼概率阈值从 0.8 降为 0.35，睁眼阈值仍为 0.2，眼睛存在置信度仍需达到 0.8。可通过 `./start_eye_turtle.sh --teyed --blink-method meye --blink-threshold 0.35` 指定；阈值必须大于 0.2 且不超过 1。日志会记录实际阈值。修改后需要重启检测器。

对调整时最新日志的 257 条中心就绪状态记录重新分类：0.8 和 0.35 都未产生双眼同时闭合。日志只记录状态变化及心跳，不能用它准确回放完整眨眼持续时间，阈值下降尚不能宣称解决启动问题。

新默认方案不需要 `--blink-threshold`。这个参数只接受在 `--blink-method meye` 下使用，避免误以为改概率门限会影响 RITnet 的分割规则。RITnet 引擎复用 `models/ritnet/` 中已有文件；如需重建，执行 `tools/build_ritnet.sh`。
