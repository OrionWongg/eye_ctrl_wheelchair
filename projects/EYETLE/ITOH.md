# Itoh / Orlosky / Świrski 原算法试验入口（已暂停推荐）

2026-10-16：用户反馈效果不佳，当前已回到 TEyeD + RITnet。本文件保留试验记录。来源：[YutaItoh/3D-Eye-Tracker](https://github.com/YutaItoh/3D-Eye-Tracker)，固定提交 `a8e2180577308596b6ac8078558df1e430789191`。本次使用原仓库 C++ 算法，不使用同名 Python 近似实现，也不使用 pye3d 或 TEyeD 计算三维方向。

## 先预览

```bash
cd /home/whill/projects/EYETLE
./start_itoh.sh 2>&1 | tee model-results/itoh-live.log
```

默认双眼，按 USB 路径选摄像头；左眼旋转 180°、右眼 0°。默认预览不发布 ROS。

1. 戴稳眼镜，缓慢看左、右、上、下，尽量覆盖多个注视位置。
2. 每眼收集 30 个空间分散的瞳孔椭圆，原算法建立眼球模型；重复盯住同一点不能完成建模。
3. 等左右眼显示 `ITOH_READY`，再看正前方，连续稳定约 2 秒自动确认中心。
4. R：重建两眼模型并清空中心。C：只重新采集中心。Esc：退出。

显示瞳孔椭圆、眼球中心投影到瞳孔的箭头，以及模型状态和采样进度。原算法也需要建模；不能因换了模型就称为免标定。

## 海龟试验

```bash
./start_eye_turtle.sh --itoh 2>&1 | tee model-results/itoh-turtle.log
```

三维方向通过检查并完成中心确认后，仍是双眨眼启用、长闭眼暂停、有效睁眼后单眨眼恢复。眨眼采用已有 RITnet 分割规则；缺失瞳孔不直接当闭眼。原有双眼新鲜度、一致性、输入失效停车等检查保留。

`./start_eye_turtle.sh --teyed` 保留此前 TEyeD + RITnet；无参数启动现为 [RITnet + PuReST 融合](FUSION.md)。轮椅入口没有切换。

## 原算法和 Linux 适配范围

- 二维：原 `PupilFitter::pupilAreaFitRR`，暗区搜索、阈值/边缘与椭圆拟合。原实现固定 640×480，包装层拒绝其他纠正方向后的尺寸。
- 三维：原 `EyeModelUpdater` 的空间采样、`unproject_observations()`、`initialise_model()`，以及逐帧 `unproject_single_observation()` 和 `initialise_single_observation()`。
- 可靠性：沿用原演示的椭圆重投影相似度 > 0.8，并检查有限值和基本几何条件；该相似度不是实际准确率。
- 原演示不调用 Ceres/SPII 的可选迭代精修函数。本构建从固定版本中提取实际调用的函数，省去未使用的优化器依赖，没有声称启用了这些精修功能。生成代码位于 `build/itoh/`。
- 修复 GCC 不允许的临时 Mat 非 const 引用、模板参数同名、缺失头文件和旧 OpenCV 常量；修复采样网格分配数超过初始化数的问题。原始 checkout 不改，变更集中在 `tools/itoh/build.py`。
- 退化圆锥会让原代码的断言终止进程，现改为异常并返回无效输入；新观测入模前先检查能否正常反投影，避免奇异椭圆污染整批初始化样本。没有放宽原可靠性阈值。
- C++ 库通过 ctypes 接入现有 Python 双摄像头/ROS 界面，每眼独立建模。初始化样本保留独立图像内存；C++ 异常返回错误状态，不输出有效运动输入。

## 参数与限制

默认 `--itoh-focal-px 476.701437` 由相机资料视场角推算，不是实测内参；原仓库使用其设备的内参，不能直接套在当前相机上。本次未增加畸变校正。眼球半径在原算法中归一化为 12 mm，因此输出长度也依赖此假设。

方向使用三维单位法向量的 x、y 分量，分别减去自动中心后融合，不是屏幕像素。左右方向符号暂沿用现有 -1 配置，现场需确认是否与实际左/右相符；可用 `--left-gaze-sign`、`--right-gaze-sign` 调整。

模型拟合后不会自动检测镜架滑移；重新佩戴或结果明显失真时按 R。原算法不是专门的眨眼模型，也不能自动解决此前双眼闭眼时序问题。

日志 `ITOH_INPUT` 包含每眼瞳孔椭圆、模型采样数、三维法向量、眼球中心、重投影相似度、中心参考与最终输入有效性。`SWEEP_TO_BUILD` 表示样本不足，`NO_PUPIL` 表示无可靠二维检测，`UNRELIABLE_3D_RESET_IF_PERSISTENT` 表示已有模型但当前三维结果不可靠。

## 构建与测试

本机依赖现有 g++、OpenCV 4、Eigen3、Boost 头文件，无需 sudo 或更改系统库：

```bash
./tools/build_itoh.sh
.venv/bin/python -m unittest discover -s test -p test_itoh.py -v
```

测试用已知三维眼球和法向量生成投影椭圆，检查原算法重建的方向；同时检查固定注视不能凑够空间样本、重置和空白图像拒绝。合成几何通过不等于实际佩戴准确率。

本次全部 59 项测试通过，原生算法的 3 项测试均执行、未跳过。结果见 `model-results/itoh-tests.log` 和 `itoh-all-tests.log`。

双摄像头 120 帧短测没有发布 ROS，组合处理耗时中位数约 67.9 ms、P95 109.8 ms。结束时两眼空间样本分别为 24/30、12/30，未完成三维模型和自动中心；没有有效控制输出。见 `model-results/itoh-camera-smoke.json`，这只能说明程序和硬件链路跑通，不能说明现场方向或眨眼准确率已验证。

## 首次有效三维结果时的日志闪退修复

现场 `itoh-live.log` 曾出现 `TypeError: Object of type bool_ is not JSON serializable`。原包装层仅在三维结果有效时返回 NumPy 布尔值，导致周期日志及退出报告序列化异常。现已统一转换成 Python bool；合成眼球测试在建模前后逐次验证完整元数据可写入严格 JSON。修复无需重编 C++，重启入口即生效。验证记录：`model-results/itoh-json-fix-tests.log`。
