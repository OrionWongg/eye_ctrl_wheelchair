# 本机恢复记录（2026-10-20）

## 来源

- 移动硬盘输入：`/media/orion/Elements SE/whill_bk`。
- 成功读取的原文件整合到单一仓库 `~/whill`；第一次原样读取副本和 rsync 错误在 `~/whill-recovery`。
- EYETLE 基础仓库：https://github.com/Yongying-Zhu/EYETLE ，提交 `eca49f294fb8af37eacbf741766654ff51aec665`。只补入缺失文件，没有覆盖成功恢复的修改版控制器。冲突清单为 `reports/eyetle-upstream-import.json`。
- RITnet：https://github.com/AayushKrChaudhary/RITnet ，提交 `6431c57ce7bf0eda935fb6178b926ae9440b50bf`。模型与许可证保留在 `projects/EYETLE/models/ritnet`。ONNX SHA256 `3648e6aed9db8e72702b43d88ef240e0eb048b0afc19247a2099c140be9dfbec`；上游权重 SHA256 与备份元数据一致。
- PuReST：https://github.com/openPupil/PyPupilEXT ，固定提交 `a2999de1dee62243e3be764312556be2e4986068`。原始代码与许可证位于 `projects/EYETLE/third_party/PyPupilEXT`，来源文件另存 SHA256。
- Willow 场景：https://github.com/osrf/gazebo_models ，固定提交 `8163eb4b5e7e21985c6591d1c0bfb56468c0093f`，CC-BY-3.0，作者／归属信息见 `wheelchair_shared_ws/vendor/willowgarage/LICENSE` 与 `SOURCE.json`。

## 恢复清单

`reports/source-import.json` 是直接扫描原硬盘的记录；共 2079 个可读文件已存在于本机副本，139 个全零文件、41 个不可读取文件、7 个不可遍历目录（排除了历史运行环境、构建目录及缓存）。不可遍历目录的内部文件数量未知。所有损坏项保留相对路径和错误，未将全零内容作为有效代码导入。

源硬盘 NTFS 内核报告 MFT 序列号不一致，ntfscat 只读尝试也报告 MFT/MFTMirr 不一致。没有执行磁盘修复或改写源数据。

仿真原始 `world.py` / `scenes.py` / `simulation.launch.py`、轮椅几何、ROS 接口、Willow 原始网格成功恢复。`import_willow_office.py` 从原始网格重新生成约 58.63 × 45.67 米场景，13348 个源三角形、12587 个碰撞三角形、905 个地图矩形。

原控制包核心文件全零，原眼动工具目录无法遍历。因此 `runtime/` 为依据文档重建的可运行实现，不能标注为完整原版恢复。原 Nav2 共享控制、动态 ROI、TensorRT 批量优化和中文 Tk 引导尚未恢复；当前用英文 OpenCV 引导和 CPU ONNX 推理，前方激光近障停车。iPad 接口仅保留可恢复部分，未做真实硬件验证。

## 本机环境与验证

Ubuntu 24.04.4 / x86_64 / ROS 2 Jazzy / Gazebo Harmonic；RTX 5070 Laptop，当前 ONNX 用 CPU。ROS 安装包和构建环境在本机重新安装，没有使用备份 ARM 二进制。只有 `wheelchair_interfaces`、`wheelchair_sim` 被构建。

两只 USB 相机 `/dev/video2`、`/dev/video4` 各成功采集 12 张 640×480 图像。正式双眼模型链路诊断 30 组图像，30 组新鲜，最旧输入帧龄中位数约 98.56 ms；详见 `reports/eye-camera-diagnostic.json`。未有人执行标定动作，诊断按设计全部运动无效。

原生 PuReST 合成椭圆中心误差小于 1 像素，缺失 ROI 时不检测，见 `reports/purest-native.json`。

仿真 IO 验证：905 个地图障碍、720 束激光、base_scan 坐标、odom/base_link、960×540 第一人称图像。见 `reports/simulation-io.json`。实际位移、转向与看门狗停车见 `reports/simulation-motion.json`。

合成眼动事件通过真实 ROS topic 驱动物理仿真：首次眨眼不动，第二次眨眼后前进 0.439 米，左视后转向 0.415 弧度，失效输入停车，视线恢复不自动重启，停车漂移为零。见 `reports/eye-sim-transport.json`。这些合成事件不评价摄像头识别准确率。

`reports/local-tests.txt`：105 项通过（包含恢复的眼动适配器测试和新运行状态机测试）。没有把旧文档中的 394／101 项测试当成本机结果。

下一步现场验证是佩戴相机后确认左右画面与旋转、完成引导标定、逐一检查双眨眼／左右视线／闭眼停车。当前尚未完成真人驾驶验收。

## 上传

当前是单个 Git 仓库，没有嵌套 Git 或子模块；构建结果、虚拟环境、个人标定、运行日志和旧 ARM 运行时不进入版本控制。尚未绑定远端。准备 GitHub 空仓库后：

```bash
git remote add origin <你的仓库地址>
git push -u origin main
```

仓库包含多种上游许可证，第三方原声明保留；没有用统一开源许可证替代 PuReST 的非商业／学术条款。
