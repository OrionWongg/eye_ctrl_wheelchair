# Smac State Lattice · Humble 新接口试验总结

日期：2026-10-16。

已经新建可切换的 Smac 实验接口并完成一轮验证。**原离散规划器仍是默认后端。** 独立栅格地图上的原生 Smac 绕障通过，但 Gazebo 实际局部代价地图的绕障用例尚未通过，因此当前接口可用于试验，尚未达到全面替换默认规划器的验收条件。

## 1. 新接口

| 接口 | 作用 |
|---|---|
| `/wheelchair/smac/compute_discrete_path` | 新规划服务，复用 `wheelchair_interfaces/srv/ComputeDiscretePath` 格式 |
| `/wheelchair/smac/path_planner_health` | 独立健康话题，不干扰原规划器的健康流 |
| `/wheelchair/smac/raw_plan` | 原生 Smac 路径，仅作观察 |
| `/wheelchair/local_plan` | 现有客户端发布的、经过转换与复核的执行路径 |

使用官方未修改的 **Nav2 Humble 1.1.20 `nav2_smac_planner/SmacPlannerLattice`** 插件，通过插件接口加载。运行库单独安装到 `wheelchair_shared_ws/vendor/smac/`，校验了 Open Robotics 签名索引及软件包 SHA256；没有替换系统 ROS 或原 `vendor/nav2`。接口依据见 [Humble 官方 API](https://api.nav2.org/nav2-humble/html/classnav2__smac__planner_1_1SmacPlannerLattice.html)。

新节点不发布底盘速度或离散动作，只提供路径。现有输入、任务授权、完整车体预测、制动检查、执行器及 STOP 链路继续生效。

## 2. 哪些请求实际经过 Smac

接口明确区分三种成功结果：

| 响应 `reason` | 实际执行内容 |
|---|---|
| `SMAC_DIRECT_ROUTE_VALIDATED` | 直接直行/原地转向即可到达，完成精确几何检查，无需图搜索 |
| `SMAC_LOCKED_ROUTE_VALIDATED` | 请求禁止绕行，按原指定路点验证直行/原地转向组合，不让 Smac 改换路线 |
| `SMAC_LATTICE_PLANNED` | 直达不可行，实际调用原生 Smac State Lattice 搜索，再转换并复核路径 |

停车 A/D/U、锁定门洞路线及前移调整候选中的部分用例属于直接或锁定路线验证。**这些用例通过，不等于已经验证了 Smac 的真实绕障能力。** 独立接口的障碍绕行用例另外要求收到原生 Smac 路径，且响应必须为 `SMAC_LATTICE_PLANNED`。

Smac 失败后不会悄悄改用旧 DWB 搜索，而是返回失败，让现有任务逻辑停车或处理已授权的位置调整。

## 3. 为什么需要适配

当前轮椅执行器支持固定速度前进、原地左/右旋转和停止。Smac 默认基元中的连续曲线不能直接作为当前底盘动作，因此本接口按 [Humble 基元格式](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/lattice_primitives/README.md) 生成自定义控制集：

- 5 cm 栅格、16 个离散朝向、64 个前进/原地旋转基元。
- 关闭倒车扩展、路径平滑和连续曲线解析捷径。
- 将原始路径作为路点参考，构造可执行的直行与原地转向段；起终点精确对齐原请求。
- 参考点最多作相邻一个栅格的有限调整，所有输出段均以细分位姿重新检查完整车体多边形、障碍栅格及请求范围。
- 执行时仍由原安全层检查当前动作和完整停车轨迹。

JSON 中的 `turning_radius=0.2` 是当前搜索启发式元数据，不代表车体或安全转向包络缩小；控制集中仍有原地旋转动作。

## 4. 参数与余量

配置文件：`src/wheelchair_nav2/config/smac_lattice.yaml`。

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `Lattice.max_planning_time` | 0.25 s | 原生 Smac 搜索预算 |
| `search_time_limit` | 0.35 s | 新服务搜索与转换总预算，超时路径不返回为成功 |
| `Lattice.max_iterations` | 50000 | 搜索迭代上限 |
| `Lattice.rotation_penalty` | 1.0 | 适配只有直行/原地旋转的控制集 |
| `Lattice.allow_reverse_expansion` | false | 不生成倒车扩展 |
| `Lattice.smooth_path` | false | 不平滑成连续曲线 |
| `Lattice.analytic_expansion_max_length` | 0.0 | 禁用连续曲线捷径 |
| `smac_search_sampling_margin` | 0.05 m | 仅用于新接口的搜索采样补偿 |
| `free_space_clearance` | 0.08 m | 沿用原配置的绕行跟踪余量，避免实际提前停车后在转弯点受阻 |

原 `margin=0.06 m`、`model_error=0.025 m`、车体轮廓和固定速度均未改变。普通硬余量仍约每侧 9.28 cm。自由空间绕行按原跟踪余量保留额外空间，再为 Smac 搜索加采样补偿；直接/锁定路线仍使用原硬余量。搜索轮廓使用保守外包多边形以降低计算开销。

这些量使实验后端在部分窄处更保守。没有通过缩小车体、降低硬余量或放宽 50 ms 控制保护来强行通过测试。另优化了安全预测中完全相同连续位姿的重复几何查询，保留时间采样、判据与最终轨迹检查。

修改配置后应重启实验节点或仿真，不依赖在线修改来调整几何预算。

## 5. 验证结果

| 验证 | 结果 |
|---|---|
| 完整单元/接口回归 | **368 passed** |
| 控制包与规划包构建 | 通过 |
| 独立新服务验收 | **11/11 通过**，包括预期的安全拒绝 |
| 新后端 Gazebo 功能用例 | **6/7 通过，1 项未通过** |
| 原默认后端对照 | **2/2 通过** |

独立接口验收覆盖直行、原地掉头、非栅格端点/朝向、锁定路线、真实 Smac 障碍绕行，以及过期请求、长度超限、锁定路线被挡、未知区域、分辨率不匹配、地图过期等拒绝情况。成功返回的路径另用现有 Python 几何实现交叉验证。最终独立地图 Smac 绕障计算约 **281 ms**。

新后端的六项通过用例：

- 办公室巡航、停止和继续。
- 办公室停车左转、右转和 180° 掉头。
- 受阻后前移再掉头。
- 找不到安全调整组合时保持停车。

未通过的用例是 `path_obstacle` 场景的实际绕障：返回 **`PATH_NO_EXECUTABLE_PATH`**，原生插件日志报告未找到有效路径，轮椅保持停车，碰撞为零。该结果不被计为“绕障成功”。独立测试与 Gazebo 使用的栅格地图和目标不同，独立接口通过不能替代真实仿真通过。

默认后端另验证了办公室原地掉头及前移调整后掉头，均通过；原离散 C++ 规划器源码与接入前一致，默认环境也不加载可选 Smac 运行库。

所有最终仿真用例的物理接触计数均为零。功能验收与关闭过程分开记录：新 Smac 进程已修复插件卸载顺序问题并正常退出；部分原有 Python 节点在 SIGINT 关闭期间仍出现 rclpy 回调转换异常，原后端对照中也出现了该现象，本轮没有宣称整套退出流程已完全无异常。

## 6. 如何试用

先关闭原整套仿真，终端 1：

```bash
~/wheelchair_shared_ws/scripts/run_smac_sim.sh scene:=clearpath_office gui:=true
```

终端 2：

```bash
~/wheelchair_shared_ws/scripts/direction_keyboard.sh
```

键位保持原来的 W/A/D/U、空格/S、X。可在第三个终端运行 `scripts/direction_status.sh` 查看阶段和拒绝原因。

恢复原规划器：停止实验仿真，重新运行原命令即可，无需回滚代码：

```bash
~/wheelchair_shared_ws/scripts/run_direction_sim.sh scene:=clearpath_office gui:=true
```

如果只希望旁路调用新接口、不接管控制，在原仿真运行时另开终端执行：

```bash
~/wheelchair_shared_ws/scripts/smac_interface.sh
```

旁路模式只增加独立服务和健康话题，原安全控制器继续使用原接口。

## 7. 结论与保留问题

本轮完成了新接口、可选启动入口、离散运动适配和实际试验，**当前保留为实验后端，不切换默认**。真实局部代价地图下的绕障仍是明确未通过项，需要进一步对照限时搜索、基元覆盖、代价地图和可执行路径转换，不能据此宣称 Smac 已优于现有规划器。

当前依然是完整仿真障碍地图支持的系统，没有实车硬件验收。Smac 的原始 `raw_plan` 不可直接下发到底盘。

详细操作与接口说明：[SMAC_STATE_LATTICE_HUMBLE.md](../wheelchair_shared_ws/docs/SMAC_STATE_LATTICE_HUMBLE.md)。

最终证据及源码哈希：`wheelchair_shared_ws/artifacts/smac-lattice-20261016/verification.json`。其中 `accepted-interface-final/`、`accepted-simulation/`、`default-uturn/`、`default-adjust/` 为最终验证；`probe-*` 等阶段记录保留，不能混作最终通过结果。

## 8. 走廊出口停滞修复补记（2026-10-16）

原 `run_direction_sim.sh` 现场状态为 `BLOCKED / PASSAGE_END_OR_BRANCH`，
停车位姿约 `(9.30005, 1.08833, 0.00293)`。原因在 `NavigationScope.cruise_goal()`：
将走廊两侧墙体末端减去车体半径和余量设为巡航硬终点。原生 Smac 搜索未参与此判定。

新增方向巡航参数 `nav_corridor_end_behavior`，`directional.yaml` 设置为
`continue_straight`；保持原走廊轴线驶出，车尾及余量完全离开后解除走廊约束。
值 `stop` 恢复旧停车确认策略。完整车体、制动、代价地图和执行器检查保留。
沿原方向直穿路口也属于该选项的行为。配置与代码修改需要重启仿真。

办公室原始地图与现场位姿的离线回归复现了旧逻辑无目标；新逻辑生成
`(11.30005, 1.05, 0)` 的前向目标。Python 完整回归 302 项通过，随后新增了
现场坐标回归。Gazebo 独立复验受到运行时健康超时影响，尚未确认整段实景通过；
记录在 `artifacts/corridor-exit-20261016/`，不能作为实景成功证据。
