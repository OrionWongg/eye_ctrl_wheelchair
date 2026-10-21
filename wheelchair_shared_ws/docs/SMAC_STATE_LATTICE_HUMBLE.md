# Smac State Lattice · Humble 实验接口

本接口在当前“原地转向受阻后搜索前方调整位置”的版本上独立增加。原 `run_direction_sim.sh` 默认仍使用原离散规划器；只有显式选择 `path_planner:=smac_lattice` 才切换到实验后端。

## 实现范围

- 使用官方未修改的 **Nav2 Humble 1.1.20 `nav2_smac_planner/SmacPlannerLattice`** 插件。
- 新建 `/wheelchair/smac/compute_discrete_path` 服务，沿用现有 `wheelchair_interfaces/srv/ComputeDiscretePath` 请求/响应格式，避免改动输入、任务授权和执行器协议。
- 新服务只有规划、路径和健康输出，没有底盘速度或动作发布器。
- 普通可绕行请求先检查能否直接用直线/原地转向到达；直达失败才调用原生 Smac 搜索。`allow_detour=false` 的锁定入口、停车原地转向以及“前进＋转向”候选，按请求的直线/原地转向顺序作完整几何验证，不允许 Smac 改换路线。响应明确区分 `SMAC_LATTICE_PLANNED`（实际 Smac 搜索）、`SMAC_DIRECT_ROUTE_VALIDATED`（直达检查）和 `SMAC_LOCKED_ROUTE_VALIDATED`（锁定路线检查）。
- 不会在 Smac 搜索失败后悄悄改用旧 DWB 搜索；返回失败交由现有任务逻辑处理。

## 运动模型与安全接口

轮椅当前执行器只支持固定速度的前进、左/右原地旋转和停止。实验接口使用自定义的 **16 个方向、64 个直行/原地旋转基元**，分辨率 5 cm；不启用倒车、平滑或连续曲线解析捷径。JSON 由 `scripts/generate_wheelchair_lattice.py` 可重复生成。

Smac 的原始路径不是底盘授权。接口把它作为路径点参考，转换成现有执行器支持的直线前进与原地旋转组合，精确连接请求的起点和终点。转换时允许路径参考点在相邻一个栅格内作有限调整；每一段按细分位姿重新计算完整车体多边形与障碍栅格的几何距离，未知格也不可通过。转换失败就拒绝。

这里保留了两层不同的余量：

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `margin` | 0.06 m | 原安全层的硬距离，未修改 |
| `model_error` | 0.025 m | 原模型误差，未修改 |
| `step` | 0.025 s | 原预测采样补偿，未修改 |
| `free_space_clearance` | 0.08 m | 沿用原规划器的绕行跟踪余量，防止实际提前停车后转不动 |
| `smac_search_sampling_margin` | 0.05 m | 只加在 Smac 搜索轮廓上，补偿栅格/角度采样误差 |

原硬外扩仍约每侧 9.28 cm。Smac 搜索轮廓额外留 5 cm，并使用外包八边形近似降低栅格轮廓计算开销；自由空间绕行的输出转换保留原有 8 cm 跟踪余量（起终点不满足该软余量时退回硬余量）；直达/锁定路线和现有安全控制层使用原硬余量，执行阶段继续复核完整停车轨迹。这会使实验后端在部分窄处更保守，不能将它理解为原车辆尺寸或全局安全参数已经改变。

最终路径继续经过原 `PathFollower`、安全门控和执行器。固定速度、50 ms 控制计算保护、动作 TTL、输入健康检查与 STOP 机制均保留。服务搜索在单独的 C++ 进程中进行，其默认总预算为 0.35 s，原生 Smac 搜索预算为 0.25 s，不占用安全控制器的 50 ms 计算周期。

搜索代价中的 `rotation_penalty` 设为 1.0，以适配仅靠原地旋转换向的基元集；不以降低实际执行速度实现转向。

## 接口

| 名称 | 类型/作用 |
|---|---|
| `/wheelchair/smac/compute_discrete_path` | `wheelchair_interfaces/srv/ComputeDiscretePath`；实验规划服务 |
| `/wheelchair/smac/path_planner_health` | `wheelchair_interfaces/msg/InputHealth`；独立健康流 |
| `/wheelchair/smac/raw_plan` | `nav_msgs/msg/Path`；原生 Smac 输出，仅用于观察 |
| `/wheelchair/local_plan` | 现有客户端发布的已转换执行路径 |

请求使用 `odom` 坐标系，保留 `task_id`、`request_id`、目标序列、路径长度上限、走廊约束和 `allow_detour`。地图必须新鲜、分辨率为 0.05 m。过期请求、过期/不匹配地图、未知空间、无法执行的路径和越出请求预算的路径均被拒绝。

原生插件没有通过 `createPlan` 暴露展开节点数，因此响应中的 `expansions` 暂为 0，不代表实际没有搜索；`compute_ms` 是接口实测总计算时间。

## 启动与切换

先关闭当前整套仿真，再在终端 1 启动实验后端：

```bash
~/wheelchair_shared_ws/scripts/run_smac_sim.sh scene:=clearpath_office gui:=true
```

终端 2 使用原键盘：

```bash
~/wheelchair_shared_ws/scripts/direction_keyboard.sh
```

可另开终端查看状态：

```bash
~/wheelchair_shared_ws/scripts/direction_status.sh
```

恢复原规划器只需停止实验仿真，使用原启动命令：

```bash
~/wheelchair_shared_ws/scripts/run_direction_sim.sh scene:=clearpath_office gui:=true
```

如只想把新服务作为旁路接口观察，保持原仿真运行，另开终端运行：

```bash
~/wheelchair_shared_ws/scripts/smac_interface.sh
```

旁路服务使用独立服务和健康话题，不会自动接管现有安全控制器。

## 安装与文件

可选运行库位于 `vendor/smac/`，由 `scripts/install_smac_runtime.py` 下载并校验 Open Robotics 签名索引及 deb 的 SHA256。当前安装了 Humble Smac 1.1.20 与对应 ROS OMPL 运行库，没有写入系统 `/opt/ros`，也没有替换 `vendor/nav2`。

只有 `smac_env.sh` 和实验启动脚本加入该运行库路径。若迁移到缺少该目录的同类环境，可执行：

```bash
cd ~/wheelchair_shared_ws
python3 scripts/install_smac_runtime.py
source scripts/env.sh
colcon build --symlink-install --base-paths src --packages-select wheelchair_nav2 wheelchair_control
```

主要实现文件：

- `src/wheelchair_nav2/src/smac_lattice_interface.cpp`
- `src/wheelchair_nav2/config/smac_lattice.yaml`
- `src/wheelchair_nav2/config/wheelchair_lattice.json`
- `scripts/run_smac_sim.sh`、`scripts/smac_interface.sh`、`scripts/smac_env.sh`
- `scripts/validate_smac_interface.py`、`scripts/validate_smac_sim.py`

## 验证与边界

原始证据保存在 `artifacts/smac-lattice-20261016/`，最终结果以汇总文件为准。独立接口验证包含真实插件直线、掉头、绕障、精确端点，以及锁定路线、未知区域、过期请求/地图等拒绝情况；返回路径另外使用现有 Python 几何实现交叉核验。

当前接口仍使用现有完整仿真障碍地图与 Nav2 局部代价地图，尚无实车验收。Smac 的栅格与角度量化、额外搜索余量和离散路径转换都可能使某些本来几何可达的窄路径被拒绝。原生 `unsmoothed_plan` 或 `raw_plan` 不可直接当成当前底盘可执行的速度轨迹。

快照包装器复用已经生成的代价地图，不重新启动一套传感器/膨胀层。插件可能提示快照没有 InflationLayer 加速器；接口仍检查完整轮廓，并非关闭碰撞检查。

## 上游依据

- [Humble SmacPlannerLattice 接口](https://api.nav2.org/nav2-humble/html/classnav2__smac__planner_1_1SmacPlannerLattice.html)
- [Humble 运动基元格式与差速原地旋转说明](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/lattice_primitives/README.md)
- [固定版本 1.1.20 的实现](https://github.com/ros-navigation/navigation2/tree/1.1.20/nav2_smac_planner)

## 本轮最终结果

完整回归 **368 passed**，两包构建成功，独立服务 **11/11** 验收通过。新后端 Gazebo 功能用例 **6/7** 通过；`path_obstacle` 实际绕障返回 `PATH_NO_EXECUTABLE_PATH`，尚未通过。原默认后端两项对照通过。因此接口保持实验可选状态，默认仍为原规划器。

部分已有 Python 节点在 SIGINT 关闭期间出现回调转换异常，原后端对照也复现；新 Smac 插件进程的独立卸载崩溃已修复。功能通过与关闭异常分开记录。

完整总结：[WHEELCHAIR_SMAC_STATE_LATTICE_HUMBLE_SUMMARY.md](../../projects/WHEELCHAIR_SMAC_STATE_LATTICE_HUMBLE_SUMMARY.md)。最终证据：`artifacts/smac-lattice-20261016/verification.json`。
