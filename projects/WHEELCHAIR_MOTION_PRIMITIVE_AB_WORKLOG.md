# 运动原语 A/B 接口改造 · 工作记录

分支 `eye-ipad-fusion`，2026-10-28。状态：**未完成，可编译，未做端到端验证**。

本轮起点是一个具体假设：现有 discrete 搜索用固定动作步长加较粗状态去重，可能漏掉合适的转弯位置。目标是让离散搜索与自定义运动原语的 Smac Lattice 在相同地图、相同目标下可比。

开工后从真机数据得到一个更根本的结论：**两侧的转向原语前提都是错的**。详见第 6 节。

---

## 1. 一句话状态

接口层已经参数化、两后端参数契约已统一、编译通过、无测试回归；但**真正的 A/B 对比还跑不起来**，并且**转向原语与真车运动不符**这一项被新发现的证据推翻了原假设的前提。

---

## 2. 已完成并验证

### 2.1 discrete 搜索参数化

`src/wheelchair_nav2/src/discrete_path_planner.cpp`

- 状态去重桶从硬编码 `0.10 m / 5°` 改为参数 `state_cell`、`state_heading_bins`（`key()`，约 L138）。这是"粗去重会不会漏掉转弯位"这个问题的直接旋钮。
- 新增 `search_sampling_margin`：`valid()` 增加可选轮廓参数，搜索展开用 `search_footprint_`，连接器与最终路径仍用硬 `margin`。原先两侧余量不可比的问题就此消除。
- 默认值保持 0.10 / 72 bins，与工作区原有未提交状态一致，**discrete 行为未变**。

### 2.2 Smac 接口对齐

`src/wheelchair_nav2/src/smac_lattice_interface.cpp`

- 余量参数名 `smac_search_sampling_margin` → `search_sampling_margin`，两个后端共用同一个键。
- 成功返回的 reason 统一为 `PLANNED`，删除不再使用的 `used_smac_`。
- 此前缺失的走廊检查曾补上，后按要求整体删除（见 3.1）。

### 2.3 配置文件

`nav2.yaml`、`nav2_tight_sim.yaml`、`nav2_hardware.yaml`、`smac_lattice.yaml` 同步新增/统一 `state_cell`、`state_heading_bins`、`search_sampling_margin`。

### 2.4 启动参数

`shared.launch.py` 与 `hardware.launch.py` 均新增 `state_cell`、`state_heading_bins`、`search_sampling_margin`、`search_time_limit` 四个 launch 参数，并用同一个 `search_params` 字典喂给两个 executable。

切换用法：

```bash
scripts/run_direction_sim.sh path_planner:=discrete
scripts/run_direction_sim.sh path_planner:=smac_lattice state_cell:=0.05
```

### 2.5 走廊检查移除（按要求）

实测 `nav2_transport.py` 每次请求都发 `corridor_half_width = 0`，该约束在运行时从未生效；全仓库只有 `validate_junction_request.py` 设 0.25。

已清理：两个 planner 的走廊判定与 `origin`/`band` 参数、`band_`/`origin_` 成员、`nav2_transport.py` 的两个赋值、`validate_junction_request.py` 的三条走廊用例。

### 2.6 验证结果

- `colcon build --packages-select wheelchair_nav2` 通过（24.2s / 0.29s 两次）。
- 全量测试 **33 failed / 202 passed**；用 `git stash` 回到 HEAD 复跑，**逐文件失败计数完全一致**（task_planning 9、task_manager 7、turn_adjustment 5、stationary_turn 4、discrete_control 2、corridor_assist 2、ros_health 1、path_tracking 1、eye_resume_integration 1、directional 1）。即：**没有引入新回归，但这 33 个失败是既有问题，本轮未处理**。

---

## 3. 没做好的

### 3.1 [我自己引入] 硬件 launch 参数覆盖顺序错误

`hardware.launch.py` 中我最初写成：

```python
parameters=[config, timing_cfg, nav2_config, search_params, smac_lattice.yaml, {...}]
```

`smac_lattice.yaml` 在 `search_params` **之后**，会覆盖掉四个 A/B 参数——硬件上 `state_cell:=` / `search_sampling_margin:=` 等全部静默失效。`shared.launch.py` 是对的（`search_params` 在后）。

**修正已写入文件，但未经编译或运行验证。** 复现时要先确认这一点。

### 3.2 A/B 对比实际上跑不起来

原始目标的"对比"还不是可执行状态，缺三样：

1. **航向数不匹配。** `wheelchair_lattice.json` 是 16 航向（22.5°），discrete 现在是 72 桶（5°）。Smac 在角度上比 discrete 粗一倍多，此时比出的差异会混入量化误差。`scripts/generate_wheelchair_lattice.py` 尚未参数化航向数。
2. **没有对比脚本。** 同一份 costmap、同一批起终点、分别发两后端、汇总成可比 JSON 的工具不存在。
3. **`expansions` 在 Smac 侧恒为 0。** 已核实 vendored 的 `nav2_smac_planner::SearchInfo` 不含展开计数，`createPlan()` 也不返回。两侧搜索代价无法直接对比，只能比 `compute_ms` 与路径本身。

### 3.3 静默改掉了 Smac 的两个既有默认值

- `search_sampling_margin`：0.05 → 0.0（parity 设置）。
- `search_time_limit`：`smac_lattice.yaml` 写 0.35，但 launch 默认 0.25 且排在后面，**实际会以 0.25 运行**。

两处都是为了让两后端可比，但它们确实改变了既有 Smac 行为，且没有实跑验证。这是需要明确决策的点，不该默默接受。

### 3.4 `.srv` 遗留死字段

`ComputeDiscretePath.srv` 的 `corridor_origin`、`corridor_half_width` 现在无人读取。因为先前定了不改 `.srv`，暂时保留。清理需要重编译 `wheelchair_interfaces` 并同步消费方。

### 3.5 文档同步滞后

`docs/SMAC_STATE_LATTICE_HUMBLE.md` 仍描述 `smac_search_sampling_margin`、三种 `SMAC_*` reason，以及走廊行为。已失效，未更新。

### 3.6 勘察只完成 3/5

5 个只读 agent 中 3 个返回了结论（执行/跟踪、安全预测、实测数据），2 个在停止前只输出开场白、**没有产出**（discrete 搜索、Smac lattice）。所以第 4 节的 lattice 侧结论来自我直接读代码，而非该 agent。

---

## 4. 尚未开始

- lattice 生成器航向数参数化（3.2 第 1 条）
- A/B 对比脚本（3.2 第 2 条）
- 端到端实跑，任一后端
- 文档更新（3.5）

---

## 5. 勘察得到的既有缺陷（非本轮引入，未修）

这些是我读代码/看数据确认的：

- `segments()` 遇到任何弧线直接 `raise ValueError`（`path_tracking.py:34`），`navigation.py:66` 把它转成 BLOCKED。**只替换原语 JSON 不产生效果，甚至直接不可执行。** 文档 `SMAC_STATE_LATTICE_HUMBLE.md:129` 也警告过这一点。
- `PathTurn` 只积分 yaw，从不读 x/y；转向完成判定纯角度。
- `State` 只有 `v`、`w`；`nodes.py:102` 直接丢弃 odom 的 `twist.linear.y`。
- `Config.velocity` 的 `icc_x = 0.138` 是纯前向偏移（`planner.py:74,87-92`），且 `planner.py:77-78` 拒绝非正值，无法表达后轴负偏移。
- 真机 `twist_to_key.py:20-25` 显式把转向时的 `linear.x` 清零，仿真里的弧与真车实际走的弧不是一回事。
- `startup_delay` 0.12 s 对比实测起动延迟 0.287~0.434 s，0.10/0.20 s 的短脉冲可能在可观测运动前就结束。

---

## 6. 关键发现：真机转向不是原地旋转

数据：`artifacts/motion_id_20261028_151804/`（44 段：前进 16、左转 17、右转 11）。

体坐标速度对角速度过原点最小二乘：

| 方向 | vx/ω | vy/ω | 对应旋转中心 (base_link) | \|C\| |
|---|---:|---:|---|---:|
| 左转 | 0.0764 | 0.2786 | (-0.2786, +0.0764) | 0.2888 m |
| 右转 | 0.0755 | 0.2744 | (-0.2744, +0.0755) | 0.2846 m |

即绕 base_link **后方约 0.28 m、左侧约 0.08 m** 的一点做刚体旋转；左右两向差 5 mm 以内。用该中心回推全部 28 个转向位移：中位绝对误差 0.004 / 0.003 m，最大 0.018 / 0.010 m，角度范围 5°~97°。模型与数据吻合得相当好。

**后果**：按 `nav_position_tolerance = 0.06 m` 计，转 90° 累积偏移约 0.45 m，转 180° 约 0.899 m，是容差的 7~15 倍。而偏移产生在转向阶段（无人检查位置），要到下一段直行才以 `PATH_ENDPOINT_LATERAL_ERROR` 暴露——那时未经碰撞校验的那段已经走完了。

**但这份数据不能直接当安全参数**（README 自己列了限制，我已复核）：

- **无 stop-to-stop 转向样本**：完整静止起止的干净子集为前进 4、左转 0、右转 0。规划原语必须是起停到起停的，目前所有转向时序值都是从运动中起点外推的。
- **前进最高速未测到**：从未出现平台段，`peak = 0.339*hold - 0.058`（n=15，rms 0.029）。最长一次正好落在斜坡上。把 0.64 m/s 当 v_max 是编出来的上限。
- **松键到停稳样本过少**：左转 n=3、右转 n=2，无法给出停车距离上界。
- **减速度离散度约 2 倍**（左 1.838~3.606，右 1.932~3.390）。用中位数会低估制动距离最多约 40%。
- 转向确实会饱和，峰值约 1.0 rad/s，斜坡约 0.5 s——这一条比前进可靠。

方向已经清楚，但顺序必须是：**先核对 TF / base_link 与实际轴距，确认旋转中心，再改原语**。只换 JSON 无效；换成弧线原语还必须同时改 `segments()`、`PathTurn`、`State`/`integrator`、`brake_distance`、`turn_adjustment.valid_path`，以及 `tests/test_path_tracking.py:85-87`（该测试目前断言弧线必须抛错）。

---

## 7. 复现与继续

```bash
cd ~/eye_ctrl_wheelchair/wheelchair_shared_ws
source scripts/smac_env.sh          # 需要 vendored Smac 运行库
colcon build --symlink-install --base-paths src --packages-select wheelchair_nav2

# 测试（必须 source local_setup，仅 source env.sh 不够）
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 -m pytest tests -q
```

建议的下一步顺序：

1. 验证 `hardware.launch.py` 参数顺序修正（3.1）。
2. 决策 3.3 的两个默认值。
3. lattice 生成器端航向数参数化 + 对比脚本（3.2）。
4. 核对旋转中心（第 6 节），再决定原语是否引入弧线。

第 4 步之前，任何"A/B 对比结论"都只反映仿真里那个不存在的原地旋转模型。

## 8. 优先修复本轮回归（2026-10-28）

- 恢复 Smac 搜索余量 0.05 m（C++ 默认与 YAML 同步），总预算保留 0.35 s。
- shared / hardware launch 按 path_planner 选择默认值：discrete 仍为 0.0 m / 0.25 s，Smac 为 0.05 m / 0.35 s；显式 launch 参数继续覆盖 YAML。硬件参数顺序已检查，search_params 在 smac_lattice.yaml 之后。
- 同步 Smac 文档中的参数名、PLANNED reason、已移除的走廊检查，并将旧验收结果明确标为历史记录。
- 验证：使用 ROS LaunchContext 实际求值两个 launch、两种后端的默认值，全部通过；wheelchair_nav2、wheelchair_control、wheelchair_hardware_bridge 三包构建通过。
- 未做端到端仿真或真机验证；环境脚本提示缺少 Gazebo runtime。第 3.2 节未完成的 A/B 工具和第 5/6 节运动模型问题不属于此次默认值回归修复，仍待后续处理。
