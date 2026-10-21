# 现有办公室场景验证与眼动接入状态

日期：2026-10-14。

结论：**当前新版方向意图算法尚未通过办公室场景验收。** 此结论补充此前 30 个专项场景通过的结果，不能用专项通过代替办公室通过。

## 验证条件

使用本机现有 `clearpath_office` 模型，在 Gazebo 中以默认原点出生位置测试，无传送、无新增障碍、无场景坐标写入控制算法。沿用当前生产参数，包括 50 ms 安全计算保护。控制源码及配置与上次验收快照逐文件哈希比对一致，办公室模型哈希验证未变。

办公室使用现有导入的物理碰撞网格和对应仿真地图（1068 个矩形），并非新建的简化走廊。本次是无界面的实际 Gazebo 运行；仅覆盖默认起点附近的行驶及路线，不代表全楼覆盖，也不是实机/未知地图感知验收。

## 实际结果

共 5 类操作、7 次试验，3 次通过、4 次失败。所有试验接触数为 0，7 次 Costmap 均正常退出。

| 操作 | 结果 | 实际表现 |
|---|---|---|
| 前进 2 m、停止、恢复 | 两次均失败 | 前进约 2.17 / 2.14 m；行驶/恢复阶段出现 `PLANNING_TIMEOUT_FAULT`，安全计算峰值 56.00 / 55.75 ms |
| 左方向意图后短程前进 | 通过 | 起点为开阔区，约 +19.5° 调整后继续前进；未验证进入门口 |
| 右方向意图后短程前进 | 通过 | 约 -20.7° 调整后继续前进；未验证进入门口 |
| 掉头并继续前进 | 首次失败、复测通过 | 首次转到约 24.8° 时出现执行器 `ACTION_STALE`，上层显示 `LOWER_HEALTH_TIMEOUT_OR_FAULT`；复测完成约 179.7° 掉头 |
| 左方向调整后较长行驶 | 失败 | 实际累计行驶约 2.57 m，停在约 (2.44, 0.81)；未达到设定的调整后 3 m 进度，出现超时/下层健康故障，计算峰值 50.63 ms |

无接触说明这些试次的保护停车有效，不等于任务完成。掉头一成一败，不能标为稳定通过。

完整报告：[validation_summary.json](../artifacts/nav2-office/validation_summary.json)。
轨迹图：[trajectories.png](../artifacts/nav2-office/trajectories.png)。
逐周期日志分别位于 `artifacts/nav2-office/`、`artifacts/nav2-office-route/`、`artifacts/nav2-office-repeat/`。

当前需要继续定位的是大地图运行中的控制计算耗时和执行命令时效性。应对几何识别、地图回调和控制发布间隔做性能分析；此次验证尚未确证各自的耗时占比，未修改控制算法、地图或放宽保护阈值。

## 启动及复现

交互验证，在两个终端分别运行：

```bash
~/wheelchair_shared_ws/scripts/run_direction_sim.sh scene:=clearpath_office
~/wheelchair_shared_ws/scripts/direction_keyboard.sh
```

自动化验证：

```bash
cd ~/wheelchair_shared_ws
source scripts/env.sh
/usr/bin/python3 scripts/validate_office_directional.py --case all --output artifacts/nav2-office-new-run
```

脚本现含上述五类操作。保存的新报告不会覆盖本次基线试验。

## 眼动接入状态

**架构可以接入；当前新版尚未接通，不能直接把 `input_source` 改为 `eye` 使用。**

- 已有 RITnet 及 `EyeSample` 输入、旧版眼动控制和带时间戳/序号的健康校验逻辑。
- 新版管理器已经按输入来源订阅 `/wheelchair/eye/direction_event`（`DirectionEvent`）和 `/wheelchair/eye/health`（`InputHealth`）。
- 现有 `EyeIntentNode` 仅实现 `gaze_joystick`、`intent_confirm`；启动文件明确拒绝 `directional_intent + eye`。
- 所需工作是增加方向事件适配：稳定左/右注视生成一次方向偏好，明确的手势产生启动/停车/掉头事件，并处理防抖、重发去重、丢眼、陈旧帧、会话重启与健康超时。方向事件交给同一规划和路径跟踪链路，不直接变成持续转动底盘指令。
- START/STOP 与掉头手势的具体映射应沿用已确认的交互约定或另行明确；当前数据字段仅包含水平注视、闭眼和有效性，不应假称已有上下视线掉头功能。
- 完成适配后仍需在同一办公室地图做眼动端到端回归。此次验证没有启动摄像头或修改眼动交互。
