# 依赖与构建

当前代码针对 Ubuntu 22.04、ROS 2 Humble、Jetson Orin aarch64。TensorRT 和可视化插件含平台依赖，尚未验证 x86 的完整部署。

## ROS 与 Python

安装 ROS 2 Humble、colcon、rosdep，按各工作空间 package.xml 安装系统依赖。眼动环境使用系统 ROS Python 包：

```bash
cd projects/EYETLE
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements-eye-tracking.txt
```

其他算法的依赖见本目录的 `requirements-*.txt`。Orin 锁定版本记录于 `requirements-orin-lock.txt`。共享控制运行依赖见 `wheelchair_shared_ws/requirements-runtime.txt`。测试需 pytest、numpy，以及测试所涉及的眼动依赖。

## 模型和 PuReST

在 `projects/EYETLE` 中运行已有构建脚本：

```bash
./tools/build_ritnet.sh
python3 tools/purest/build.py
```

RITnet 脚本固定上游提交，下载模型并生成 ONNX / TensorRT engine；需要兼容的 CUDA / TensorRT 和模型导出环境。PuReST 需要 g++、pkg-config、OpenCV 开发库。其他方案见 RITNET.md、ITOH.md、TEYED.md。第三方源代码和模型有独立许可，包括非商业研究限制，见原始许可与构建脚本。

模型文件、原始相机视频、个人 `state/` 标定不入库；新设备应重新标定。已有本机部署可以自行复制相应模型，但 TensorRT engine 不保证跨设备兼容。

## Gazebo / Nav2

`wheelchair_shared_ws/scripts/env.sh` 默认读取 `vendor/runtime` 的工作空间内 Gazebo Fortress 运行库，C++ 可视化插件也使用该布局。此运行库原先从本机已有安装迁移，项目没有完整的一键重建脚本；仅安装系统 Gazebo 并不能替代该目录布局。

- 原机器的运行库在 `/home/whill/wheelchair_shared_ws/vendor/runtime`，版本清单已保留在 `vendor/runtime-packages.json`。
- 可以通过 `WHEELCHAIR_GAZEBO_RUNTIME` 指向兼容运行库；构建时同步传入 `--cmake-args -DGAZEBO_RUNTIME=/绝对路径`。
- 若复制运行库到新位置，需要修改其中 Ignition CLI YAML 的安装路径并重新构建插件，见 `vendor/RUNTIME_README.md`。
- Nav2 和 Smac 局部安装可分别使用 `scripts/install_nav2_runtime.py` 与 `scripts/install_smac_runtime.py`，这些脚本会联网下载并验证软件包。现有版本清单保留在 `vendor/nav2/manifest.json`、`vendor/smac/manifest.json`。

本次整理未重新下载依赖或重新构建完整仿真；新机器部署仍需完成这些步骤。场景运行所需模型已位于 `src/wheelchair_sim/models`，不需要上传下载缓存中的完整上游仓库。

## 回归测试环境

先构建工作空间并 source `wheelchair_shared_ws/install/local_setup.bash`，测试需要生成的 `wheelchair_interfaces` 消息类型。三维眼动测试还依赖 `requirements-gaze3d.txt` 中的 pye3d。可通过 `TEST_PYTHON=/path/to/python ./scripts/project.sh test -q` 指定装好依赖的解释器。
