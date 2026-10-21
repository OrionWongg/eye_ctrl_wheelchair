# 办公室导航 RViz2 诊断

仿真已运行时，在新终端执行：

```bash
~/wheelchair_shared_ws/scripts/navigation_rviz.sh
```

无需重新启动仿真。脚本默认使用 ROS domain 64，与 `run_direction_sim.sh` 和 `direction_keyboard.sh` 一致；自定义域时三个终端都使用相同的 `WHEELCHAIR_ROS_DOMAIN_ID`：

```bash
WHEELCHAIR_ROS_DOMAIN_ID=64 ~/wheelchair_shared_ws/scripts/navigation_rviz.sh
```

未启动仿真时，先运行：

```bash
~/wheelchair_shared_ws/scripts/run_direction_sim.sh scene:=clearpath_office
```

RViz 仅用于观察。关闭 RViz 或按 Ctrl+C 只关闭 RViz 及诊断节点，不关闭原仿真，不发布底盘命令。

## 显示内容

| 图层/颜色 | 含义 |
|---|---|
| Office planning map | 当前办公室规划地图 |
| Local costmap (inflation) | Nav2 局部代价地图及膨胀区；颜色表示代价，不等于所有有颜色的位置都绝对不可通行 |
| 青色点 | 当前二维雷达 |
| 绿色轮廓 | 轮椅本体、规划轮廓与 Nav2 发布轮廓 |
| 红色外轮廓 | 硬安全余量；颜色本身不表示已经碰撞 |
| 黄色外轮廓 | 舒适余量 |
| 绿色线 | 最近收到的规划路径，文字同时标出接收时间 |
| 白色线 | 诊断节点启动后记录的实际轨迹，最多保留 3000 个点 |
| 半透明蓝色轮廓 | 当前候选动作预测扫掠轮廓 |
| 紫色轮廓 | 安全候选的预计停车位置；不安全候选则是预测终点 |
| 粗红色短线 | 当前诊断提供的车体与障碍物违规点连线；并非所有故障都有碰撞点 |
| 橙色箭头 | 当前导航目标及目标朝向 |
| 轮椅旁文字 | 状态、停车原因、执行器原因、是否需要重新启动、当前计算耗时、最近故障峰值、导航阶段、规划耗时及观测年龄 |

默认视角跟随 `base_link`，固定坐标系为 `odom`。滚轮缩放，中键平移；Views 面板可切换 `Office overview` 查看整个办公室。

## 卡住时看什么

1. 查看文字中的 `reason` 和 `executor`，不要只凭 Gazebo 外观判断碰撞。
2. `PLANNING_TIMEOUT_FAULT` 表示安全控制计算超过 50 ms；`last fault ... peak` 保留本诊断节点观察到的峰值，停车后的低耗时不会覆盖它。
3. `ACTION_STALE` 表示执行器收到的动作指令过期。`LOWER_HEALTH_TIMEOUT_OR_FAULT` 是上层汇总，需同时看执行器原因和数据年龄。
4. `PATH_*`、`TASK_HEADING_*` 等原因配合局部代价地图、轮廓和预测扫掠检查。即使本体能放入通道，安全余量及转向/制动包络也可能不能通过。
5. `READY_TO_REARM` 表示先前故障后已具备重新启动条件，不表示机器人正在继续执行。
6. `STALE TRACE` 表示超过 0.5 秒未收到新的控制诊断。显示的是旧数据，不是当前实时状态。

## 迟打开可视化的限制

`/wheelchair/local_plan` 原发布者是 volatile QoS。若卡住后才打开可视化，可能收不到之前生成的路径；界面会显示 `last plan: not received`。诊断节点会保留它启动后收到的最近路径。建议先打开 RViz，再开始一次测试。已有轮廓、实时原因、地图、雷达仍可在运行中直接查看。

最近故障记录与实际轨迹也只覆盖诊断节点启动之后；控制节点会话变化时清空历史。最近路径始终标注接收年龄，不保证仍是当前任务的有效路径。

## 实现与验证

- `navigation_debug` 独立订阅 `/wheelchair/decision_trace`、`/wheelchair/odom`、`/wheelchair/local_plan`，只发布 `/wheelchair/debug/*` 可视化话题。
- RViz 不订阅 `/wheelchair/safety_markers`，避免触发安全控制循环内的原绘图逻辑；此次未修改控制算法或安全阈值。
- 构建通过。在独立 domain 96 启动原办公室仿真验证，未发送行驶指令；地图、局部代价地图、雷达、轮廓和诊断文字成功显示，RViz 与诊断节点均正常退出。
- 使用此前办公室的真实 56 ms 故障 trace 验证了故障峰值保留及过期标记。
- 结果位于 `artifacts/rviz-navigation/result.json`、`stale_fault_check.json`，RViz 实际截图为 `artifacts/rviz-navigation/rviz.png`。

主要文件：`config/navigation_debug.rviz`、`launch/navigation_debug.launch.py`、`wheelchair_control/navigation_debug.py`。
