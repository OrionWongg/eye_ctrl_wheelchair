# 本次整理验证（2026-10-21）

## 结果

- 顶层共享控制 `tests/` + EYETLE `test/`：**410 passed, 5 skipped**。
- 眼动适配器、仿真几何及 BLE motion_model / calibration_lock / serial_connection：**112 passed**。
- 总计 **522 passed, 5 skipped**；未运行 BLE 风格检查。
- 新导入源码中 159 个 Python 文件通过 AST 解析，33 个 shell 文件通过 `bash -n`。
- 新导入文本未检出所扫描的 GitHub Token、AWS Access Key ID 或私钥头模式（模式扫描并非完整安全审计）。

## 测试环境

使用原机器 `/home/whill/projects/EYETLE/.venv/bin/python`，source ROS Humble 及原工作空间的 `install/local_setup.bash`，复用已生成的 ROS 消息。PYTHONPATH 优先指向新仓库源码；没有将旧 build/install 复制到仓库。

```bash
source /opt/ros/humble/setup.bash
source /home/whill/wheelchair_shared_ws/install/local_setup.bash
TEST_PYTHON=/home/whill/projects/EYETLE/.venv/bin/python ./scripts/project.sh test -q
```

首次使用系统 Python 时，三维眼动测试因缺少 pye3d 失败；切换到已有完整眼动环境后通过。5 个用例由现有测试根据可选依赖/资产条件跳过，未将跳过计作通过。

这次没有重新构建 ROS/C++ 包、启动完整 Gazebo、连接真实底盘或进行真人佩戴测试。原始文档/第三方网格含已有行末空格或 CRLF，本次保留其内容；新增顶层文档和入口通过 diff whitespace 检查。

## 删除旧版本后的检查

已删除 Jazzy 目录及文档入口，Humble 运行源码未修改；本次只检查删除范围与文档链接，未重复运行上述测试。
