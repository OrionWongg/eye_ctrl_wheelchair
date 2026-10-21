# 紧凑通行仿真参数

此配置是用于原办公室及其他 Gazebo 地图的通用参数实验，不修改地图，不缩小真实车体轮廓。默认配置不变。

先停车并退出原仿真，再启动新的仿真实例：

```bash
~/wheelchair_shared_ws/scripts/run_direction_tight_sim.sh scene:=clearpath_office
```

其他终端照常启动：

```bash
~/wheelchair_shared_ws/scripts/direction_keyboard.sh
~/wheelchair_shared_ws/scripts/navigation_rviz.sh
```

| 参数 | 默认 | 紧凑仿真 | 作用 |
|---|---:|---:|---|
| `inflation_radius` | 0.60 m | 0.40 m | 缩小软膨胀代价的影响范围 |
| `cost_scaling_factor` | 3.0 | 10.0 | 代价随障碍距离增长衰减得更快 |
| `free_space_clearance` | 0.08 m | 0.02 m | 减少开阔区绕行搜索额外预留的跟踪空间 |
| `margin` | 0.06 m | 0.02 m | 降低显式硬安全余量；C++ 搜索和 Python 安全门同步使用 |
| `comfort_distance` | 0.15 m | 0.08 m | 舒适距离及可视化轮廓 |

仍使用 1.00 × 0.60 m 真实规划轮廓、0.025 m 模型误差及原运动采样误差；固定档位、加减速、制动模型、数据过期保护、50 ms 控制计算保护不变。

总静态轮廓余量 = `margin + model_error + sampling_error`，从每侧约 0.0928 m 变为 0.0528 m，而不是变成每侧 2 cm。对齐直行的理论最小宽度约从 0.7856 m 变为 0.7056 m；地图栅格、原办公室导入时的保守膨胀及实际跟踪误差还会占用空间，不能据此保证 71 cm 的真实门洞可通过。

1 m 长、0.6 m 宽的车体偏转 20° 时，横向投影约为 `0.6*cos(20°) + 1.0*sin(20°) = 0.906 m`，再加紧凑配置两侧余量，总计约 1.011 m。因此窄入口应在有空间的位置先对正，参数不能使真实旋转包络消失。

## 为什么不能仅凭膨胀色块判断

本项目 C++ 搜索使用完整 footprint，并拒绝碰撞代价 `>=254`；其余膨胀代价影响路径评分。Python 的 Nav2 地图转换仅把未知和致命障碍变为硬障碍。彩色区域重叠不等于完整轮廓一定不能通过。内切半径内的高代价颜色也不会随增大衰减系数而消失，不应为缩小颜色而虚报轮椅尺寸。

若实际原因是 `PLANNING_TIMEOUT_FAULT` 或执行器 `ACTION_STALE`，本配置不会修复计算耗时或消息时序。请看 RViz 的完整 `reason`、`executor`、`last fault`；`last plan` 时间很久的绿色轨迹是历史路径，不代表当前仍有效。`READY_TO_REARM` 表示恢复条件满足，但尚需明确重新启动。

## 文件与验证范围

- 控制配置：`src/wheelchair_control/config/narrow_control_tight_sim.yaml`。
- Nav2 配置：`src/wheelchair_nav2/config/nav2_tight_sim.yaml`。
- 启动文件新增可选 `nav2_config` 参数；原启动命令仍使用原配置。
- 已验证配置构建、参数实际加载与静态几何判定。0.76 m 合成直通道在紧凑配置下静态轮廓可放入，偏转 20° 时仍拒绝；这仅是参数作用检查，不是办公室通行验收。
- 仅用于仿真实验，尚未完成该配置的办公室行驶回归，也未实机标定。结果位于 `artifacts/tight-sim-profile/result.json`。

恢复默认：退出紧凑配置的仿真后，重新执行 `run_direction_sim.sh scene:=clearpath_office`。
