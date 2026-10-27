# 避障栈清理总结

## 目标

实现底盘键盘控制 + 自主避障（不撞墙）。保留与按键/眼动输入相关的安全门（松键停车、收不到信息不启动），保留几何碰撞检查（真正的避障），删除项目自加的时间/预算/session 限制，让 nav2 自带机制承担更多责任。

## 改动文件

```
geometry.py                  +4   空边界崩溃修复
nodes.py                     +/-16 时效门控删除 + ttl 修正
planner.py                   +/-35 时间预算限制删除 + latch 自动解锁
task_manager.py              +/-35 任务时间/距离预算删除
direction_manager.py         +/-9  健康门删除
turn_adjustment.py           +/-4  路径容差放宽
scan_map.py                  +7   自打回波过滤
hardware_bridge.yaml         +/-9  地图分辨率/尺寸调整
test_hardware_perception.py  +/-2  mock 对齐生产配置
```

## 修复的 Bug

### 1. 空边界崩溃（geometry.py）

**症状**：safety_gate 启动时 `clearance()` 抛 `ValueError: min() arg is an empty sequence`，连带整个 launch 崩溃。

**根因**：还没收到任何地图时 `self.boundary` 为空，`min(edges(boundary))` 对空序列取 min 崩溃。

**修复**：`clearance()` 和 `collision_diagnostic()` 入口加 `if not boundary: return -1.` 守卫，空边界视为阻塞。

### 2. 激光自打回波误判为障碍（scan_map.py）

**症状**：轮椅启动位置 clearance=0，所有运动被 `INITIAL_HARD_BOUNDARY` 拦下。

**根因**：`rasterize()` 的 `corner_body` 只把四角完全在 BODY 矩形内的格子标 free，车体边缘格子不满足条件。激光打到机器人自身后部结构的回波落在边缘格子上，被写成 occupied(100)，`clear_blind_band` 只清 UNKNOWN(-1) 救不回来。

**修复**：在回波点写入占用格之前，把落在 BODY 矩形内的点过滤掉。

### 3. ObstacleMap 包被 StreamGuard 误判为过期（nodes.py）

**症状**：`MAP_STALE_SOURCE` 反复出现，轮椅每个 tick 都被 trip。

**根因**：`on_map` 建 Packet 时 ttl 用 `sensor_age=0.25s`，但实车扫描帧年龄稳定在 280-480ms，包到达时 stamp 年龄已超 ttl。

**修复**：ttl 改用 `map_age=2.0s`，让地图包按地图自身时效验证。

### 4. turn_adjust 路径容差过紧（turn_adjustment.py）

**症状**：左转/右转时 `INVALID_TURN_ADJUST_PATH` 与 `TURN_ADJUST_SEARCH` 死循环，轮椅卡住不动。

**根因**：`valid_path()` 用 0.005 rad（0.3°）检查路径起始 yaw 与当前 odom yaw 一致性，实车 odom 抖动 0.02-0.04 rad 远超此容差，每次都被拒。位置容差 0.005m 同样过紧。

**修复**：yaw 容差放宽到 0.05 rad（约 3°），位置容差放宽到 0.02m。

## 删除的自加限制

### 实时性限制

| 限制 | 原行为 | 新行为 |
|---|---|---|
| `PLANNING_TIMEOUT_FAULT` | 单 tick compute_ms 超 50ms 强停 | 改为警告日志，不停车 |

### 时间/预算限制

| 限制 | 原行为 | 新行为 |
|---|---|---|
| `ASSIST_TIME_LIMIT` | 绕行超 2s 强停 | 移除 |
| `ASSIST_STOP_TIMEOUT` | 绕行准备超 2s 强停 | 移除 |
| `MANUAL_TURN_TIMEOUT` | 转向超 2s 强停 | 移除 |
| `ASSIST_ALREADY_USED` | 绕行只允许一次 | 移除，每次持键可重试 |
| `TASK_TIME_LIMIT` | 单任务超 10s 截断 | 移除 |
| `TASK_DISTANCE_LIMIT` | 单任务超 3m 截断 | 移除 |
| `ASSIST_BOUND` | 偏移超角度/侧向上限锁定 | 改为重置参考点继续 |

### 几何决策限制

| 限制 | 原行为 | 新行为 |
|---|---|---|
| `NO_USEFUL_SAFE_ASSIST` | 两侧都不通时特殊码 | 统一报 `BLOCKED_FORWARD` |
| `AMBIGUOUS_SIDE_CHOOSE_DIRECTION` | 两侧得分接近停车 | 优先选用户上次转向方向 |

### Session 完整性限制

| 限制 | 原行为 | 新行为 |
|---|---|---|
| `INTENT_SESSION_CHANGED` | session 切换 → FAULT 锁定 | 静默清状态继续 |
| `MAP_SESSION_CHANGED` | 同上 | 同上 |
| `EXECUTOR_SESSION_CHANGED` | 同上 | 同上 |
| `PLANNER_SESSION_CHANGED` | 同上 | 改为 IDLE 停车，不锁定 |

### 几何阻塞 latch 行为

| 原行为 | 新行为 |
|---|---|
| 障碍物阻塞 → 锁定，需手动按 R 解锁 | 位置清空自动解锁；只有硬故障（sensor/executor/clock）才 latch |

## 删除的传感器时效门控

之前在 nodes.py tick() 里自行添加的时效门，本次全部删除：

- `ODOM_STALE`（odom_age=0.15s）
- `SCAN_STALE_OR_INVALID`（sensor_age=0.25s）
- `MAP_STALE_OR_INCOMPLETE`（map_age=2.0s）
- `NAV2_MAP_OR_PLANNER_STALE`（costmap_timeout=0.5s）
- `INTENT_STALE`（StreamGuard 0.25s）
- `EXECUTOR_STALE`（StreamGuard 0.25s）
- ExecutorNode `tick()` 里的 odom 年龄健康判断

task_manager.py / direction_manager.py 里对应的健康租约门也删除：

- `INPUT_HEALTH_TIMEOUT`
- `LOWER_HEALTH_TIMEOUT_OR_FAULT`
- `NOT_HEALTHY`
- `SESSION_MISMATCH`

## 保留的安全门

### 几何碰撞（避障核心，全部保留）

- `INITIAL_HARD_BOUNDARY`：静止时 footprint clearance≤padding
- `MAP_BOUNDARY_PROXIMITY`：同上但对象是边界
- `STOPPING_ENVELOPE_RISK`：制动包络与障碍相交
- `BLOCKED_FORWARD`：前向路径不安全
- `BLOCKED_TURN`：转向空间不足
- `COLLISION_OR_UNKNOWN`：滚动预测撞障

### 输入安全（按键/眼动相关，全部保留）

- 松键 → `USER_STOP`：键盘松开自动停车
- `STATUS_STALE`：safety_gate 状态超时
- `STATUS_SESSION_OR_TIMEOUT`：safety_node 重启
- `CLOCK_PAUSED` / `CLOCK_JUMP`：ROS 时钟异常
- `MUX_RESTART_NEEDS_DISABLED_STOP`：首次必须收到 disabled-STOP
- `REARM_REJECTED`：安全层未确认运动请求

### 消息合法性（全部保留）

- `REPLAY_OR_OUT_OF_ORDER`：事件/消息重放
- `INVALID_PACKET`：包格式错误
- `TF_FAULT`：scan→base_link TF 查询失败

### 硬件状态（全部保留）

- `EXECUTOR_FAULT`：executor 自报故障
- `ACTION_STALE`：executor 状态超时
- `ODOM_FAULT`：odom 数据无效
- `EXECUTOR_INVALID`：executor 消息 session 为空
- `INVALID_ODOM` / `ODOM_DISCONTINUITY`：odom 跳变

### nav2 交互（全部保留）

- `PASSAGE_MAP_CHANGED`：corridor_assist 模式下静态地图变化
- `PATH_NO_EXECUTABLE_PATH`：nav2 路径服务找不到路径
- `PATH_SERVICE_TIMEOUT`：nav2 路径服务调用超时

## 性能调优

### scan_map 地图参数（hardware_bridge.yaml）

| 参数 | 旧值 | 新值 | 效果 |
|---|---|---|---|
| `resolution` | 0.05 m | 0.10 m | 格数 40000 → 3600 |
| `size` | 10.0 m | 6.0 m | 障碍多边形数量降约 10 倍 |

**安全性论证**：footprint padding（0.09m）大于格半宽（0.05m），碰撞检查仍保守。6m 半径提供充足前视距离（轮椅宽 0.9m，最大速度 0.4m/s，制动距离约 0.3m）。

**收益**：`PLANNING_TIMEOUT_FAULT` 消失（原 obstacles 阶段 40ms+ 超时），RViz 不再因 spin 线程阻塞卡死。

## 部署步骤

```bash
cd /home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws
colcon build --packages-select wheelchair_control wheelchair_hardware_bridge
source install/setup.bash
```

## 测试

```bash
python3 -m pytest src/wheelchair_control/ src/wheelchair_hardware_bridge/test/ -x -q
# 41 passed
```

## 设计原则

1. **避障靠几何**：所有停车决策基于 footprint clearance 与障碍物的几何关系，不基于时间预算。
2. **输入靠租约**：键盘/眼动输入的时效由输入侧 StreamGuard + ClockGuard 保证，safety_gate 不重复检查。
3. **故障分级**：硬故障（sensor/executor/clock）需要手动 rearm；几何阻塞自动恢复。
4. **nav2 为主**：路径规划、costmap 更新交给 nav2，safety_gate 只做几何碰撞预测，不重复实现时效门。
