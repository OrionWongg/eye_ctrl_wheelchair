# 办公室路口停车修复

日期：2026-10-14。对应 [停车诊断](WHEELCHAIR_OFFICE_JUNCTION_STOP_DIAGNOSIS.md)。

## 代码修改

- `ComputeDiscretePath` 请求新增 `corridor_origin`，表示 odom 坐标系中的走廊中心线原点和方向；响应新增实际使用的 `costmap_stamp`。导航端把 `passage.origin` 加横向中心偏移后传入。C++ 对起点、目标、直连与搜索轨迹统一使用此轴，不再借用机器人起始朝向。横向范围仍为 0.25 m。
- 明确区分 `START_OUTSIDE_CORRIDOR`、`GOAL_OUTSIDE_CORRIDOR`、`START_COLLISION`、`GOAL_COLLISION`、`LOCKED_ROUTE_BLOCKED`、`SEARCH_TIMEOUT` 和 `SEARCH_EXPANSION_LIMIT`。请求日志包含起终点、走廊约束、任务/请求编号和地图时间戳。
- 同一导航周期复用通道识别结果；追加目标检查按已有的 0.25 s 识别周期限频。候选目标的停车可行性检查保留相同碰撞几何和动力学，但不生成无用的首次碰撞位置诊断；当前执行动作仍保留详细诊断。
- Costmap 的占据区间提取改为 NumPy 批处理；占据掩码、分辨率与原点均不变时复用碰撞矩形。未知栅格仍视为障碍。
- 完整障碍地图先用无填充歧义的原生序列化内容比较，变化后才展开顶点并验证、重建几何。保留地图会话、时效、完整性和变化停车检查。
- RViz 标记改为动作发布后以 5 Hz 生成。控制日志新增阶段耗时、上一完整回调耗时、地图回调耗时、timer 间隔与动作发布时刻；执行器新增 `/wheelchair/executor_timing`，包含动作编号、源时间戳、接收时刻/间隔、年龄和拒绝原因。

没有修改车体、硬余量、制动模型、50 ms 计算保护或 150 ms 动作 TTL。

## 复现与验证方法

所有新仿真使用隔离 ROS domain 94 和独立 Gazebo partition；原办公室地图与默认出生点不变，不传送车辆。原 domain 64 的用户实例没有重启。

1. `scripts/validate_junction_request.py` 在 domain 96 启动实际 C++ 路径服务，用空地图单独验证约束协议。文档起终点配旧轴返回 `GOAL_OUTSIDE_CORRIDOR`，配中心线 `(0, 1.05, 0)` 返回 `PLANNED`；额外验证起点/目标越界和占据地图上的碰撞拒绝。这不替代真实办公室碰撞验证。
2. 新增 `junction_resume` 仿真：默认出生点启动、左向调整、斜向到达 x>2.4 m、STOP 停稳、重新启动、通过 x>3.8 m，再 STOP。开始任务后的计算超时和 `ACTION_STALE` 均判失败。
3. 单元回归覆盖旋转坐标系、占据缓存失效、随机栅格压缩等价性、地图心跳复用及变化验证、关闭候选诊断时的碰撞结果等价性。

## 开发期间发现

- 首次优化后仍复现 51.05 ms 超时：通道识别约 9.81 ms，导航步骤约 38.04 ms。进一步减少失败候选的重复诊断及重试后，同路线降至约 29.51 ms。
- 一轮虽然走过目标位置，但收发日志发现 `ACTION_STALE`，未视作稳定通过。该次动作接收间隔约 107 ms、源时间戳年龄约 68 ms，剩余租约不足。后续增加回调计时，发现重复地图处理在行驶中可达约 35 ms，随后优化地图回调。
- 开发中曾发现 CDR 空字符串的未定义填充字节使序列化比较误报地图变化，已用固定对齐字段消除，并增加重复地图回归测试。
- `left_route`（开放区域左向调整后、不重新授权地连续前进 3 m）仍在 `PASSAGE_END_OR_BRANCH` 停止。这属于开放方向巡航的既有终点/通道接管策略，本次未取消该保护，也不宣称整个办公室任意方向导航全部验收通过。

完整日志、构建结果、阶段性失败与最终结果保存在 `wheelchair_shared_ws/artifacts/office-junction-fix-20261014/`。

## 最终仿真结果

最终构建：3 个相关包成功。完整 Python 测试：**328 passed**（19.17 s）。实际 C++ 服务的 5 项约束/碰撞请求验证通过。

| 用例 | 结果 | 最大控制计算 | 碰撞接触 | 任务开始后计算超时 / ACTION_STALE |
|---|---|---:|---:|---:|
| junction-v4 / junction_resume | PASS | 33.82 ms | 0 | 0 |
| junction-v5 / junction_resume | PASS | 30.78 ms | 0 | 0 |
| junction-v6 / junction_resume | PASS | 30.60 ms | 0 | 0 |
| cruise / cruise_stop_resume | PASS | 21.62 ms | 0 | 0 |

三次路口恢复均超过 x=3.8 m 后响应 STOP。模型文件哈希检查全部不变。仿真几何计算的最小车体间距分别约 0.224、0.184、0.223 m；这些是仿真结果，不是实车测量。逐项汇总见 `artifacts/office-junction-fix-20261014/verification.json`。

## 使用

代码与接口已在本工作空间构建。由于服务接口发生变化，使用新版本时需整体重启相关仿真/导航节点，不能仅热更新服务一端。保留原启动参数即可；不需要放宽任何安全阈值。
