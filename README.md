# 眼动控制轮椅 / Eye-controlled Wheelchair

统一管理眼动识别、轮椅共享控制与仿真、BLE 底盘接口的 ROS 2 项目。主要开发环境为 **Ubuntu 22.04 / ROS 2 Humble / Jetson Orin（aarch64）/ Gazebo Fortress**。


## 项目结构

```text
projects/
  EYETLE/                 双目眼动、RITnet + PuReST 融合、标定及 ROS 输出
  Ble2WChair/             BLE 底盘接口、硬件及离线说明
  *.md                    历史方案、接口说明和变更记录
wheelchair_shared_ws/
  src/                    控制器、接口消息、眼动适配、Nav2、仿真和可视化
  scripts/                仿真入口、依赖安装、诊断与验证脚本
  tests/                  共享控制回归测试
  docs/                   协议、场景与实验说明
  vendor/                 外部依赖版本清单（不包含安装后的运行库）
scripts/project.sh        统一命令入口
docs/                     整理说明、依赖与来源清单
```

保留三个子工作空间的布局和独立构建方式，因此跨项目相对路径仍然有效。仓库可放在任意目录，不需要覆盖原来的 `~/projects` 或 `~/wheelchair_shared_ws`。

## 当前控制链路

摄像头 → RITnet + PuReST → 原子眼动样本 → 眼动适配器 → 方向管理 → Nav2 / 安全门 → 离散执行器与看门狗 → Gazebo 轮椅。

默认融合入口：双眨眼启动；运行中看左右提交转向意图；暂停后单眨眼恢复；闭眼或输入失效停车。启动／恢复后有 0.5 秒视线缓冲。最新行为与已知问题以 [2026-10-19 接入总结](projects/EYE_WHEELCHAIR_INTEGRATION_SUMMARY_20261019.md) 为准，旧文档中的确认手势属于历史模式。

目前眼动共享控制交付是仿真闭环；BLE 硬件代码一并归档，不代表已完成眼动到真实轮椅的整体验收。墙角受阻恢复等未解决问题见上述总结。

## 安装与构建

先阅读 [依赖与构建说明](docs/SETUP.md)。仓库只包含源码、必要场景资源和依赖元数据；克隆后需要配置 ROS、Gazebo 运行库、Python 环境及眼动模型，不能直接启动。

依赖准备完成后，分别构建三个工作空间：

```bash
source /opt/ros/humble/setup.bash
(cd projects/EYETLE && colcon build --symlink-install)
(cd projects/Ble2WChair && colcon build --symlink-install)
(cd wheelchair_shared_ws && source scripts/env.sh && colcon build --symlink-install)
```

## 统一入口

从仓库根目录运行：

```bash
./scripts/project.sh sim       # 键盘方向意图仿真
./scripts/project.sh keyboard  # 第二个终端中的键盘控制
./scripts/project.sh eye       # 相机眼动 + 轮椅仿真
./scripts/project.sh test -q   # Python 回归测试，不启动真实底盘
```

可用 `EYETLE_WORKSPACE`、`WHEELCHAIR_WORKSPACE` 覆盖工作空间路径。眼动入口默认统一 ROS domain 为 64。更多参数见各子项目说明。

## 文档导航

- [眼动算法及接口](projects/EYETLE/docs/EYE_CONTROL_ARCHITECTURE_AND_INTERFACES.md)
- [共享控制说明](wheelchair_shared_ws/README.md)
- [BLE 硬件接线与运行](projects/Ble2WChair/HARDWARE.md)
- [整理范围、来源与许可](docs/IMPORT.md)
- [本次验证结果](docs/VALIDATION.md)

历史记录中的旧机器绝对路径和历史测试结果保留作追溯；当前入口使用仓库相对路径。各组件和第三方资源保留原许可，本仓库不额外授予统一许可。
