> 统一项目入口与当前状态见 [根目录 README](../README.md)。默认融合手势以 [2026-10-19 接入总结](../projects/EYE_WHEELCHAIR_INTEGRATION_SUMMARY_20261019.md) 为准；下文保留历史模式说明。

> EYETLE 眼动方向接口现已接入同一 Nav2 避障链路：`~/projects/EYETLE/start_eye_wheelchair.sh`。见 [试用说明](docs/EYE_DIRECTIONAL.md)。下文旧眼动遥杆示例需显式选择 `WHEELCHAIR_INTERACTION_MODE=gaze_joystick`。

> 2026-10-14：RITnet 默认改用三维眼球估计 + 单点正前方参考；首次按 S 确认中心，旧三点方案需显式传 `--gaze-method legacy`。流程见 [/home/whill/projects/EYETLE/GAZE3D.md](/home/whill/projects/EYETLE/GAZE3D.md)。现有轮椅交互状态机不变。


# 眼动 / 键盘轮椅共享控制（ROS 2 Humble + Gazebo Fortress）

> 当前 Nav2 路径控制改造尚未完成整体验收，已按用户要求停止继续调试。最新状态、已通过场景与未解决问题见 [Nav2 改造总结](../projects/WHEELCHAIR_NAV2_DISCRETE_PATH_CONTROL_IMPLEMENTATION.md)。下文既有通过结果属于对应历史版本，不能作为当前改造版本的全量通过证据。


## 方向意图：键盘先行模式

新增 `interaction_mode:=directional_intent`。W 启动；行驶中 A/D 选择方向或前方最近可达同侧入口，U 掉头后继续巡航；空格/S 停止。停车时 A/D 原地转向一次（默认 20°），U 原地掉头，完成后保持停车，新 W 才直行。W 不打断运行中的入口转入任务，无须候选确认或 R。

```bash
~/wheelchair_shared_ws/scripts/run_direction_sim.sh scene:=intent_left
# 第二个终端
~/wheelchair_shared_ws/scripts/direction_keyboard.sh
# 第三个终端：开发诊断
~/wheelchair_shared_ws/scripts/direction_status.sh
```

完整接口、参数、构建、验证与支持范围见 [方向意图模式](docs/DIRECTIONAL_INTENT.md)。本阶段仅键盘接入，眼动适配待完成；下文原有确认式模式的操作规则仅属于旧模式。


## 2026-10-14：确认意图后执行有限任务

已实现 [眼动意图共享驾驶方案](../projects/WHEELCHAIR_INTENT_SHARED_CONTROL_IMPLEMENTATION_PLAN.md)。
`interaction_mode:=intent_confirm` 中，视线只选择候选，双眨眼提交一次确认；默认前进任务最多 10 秒/累计 3 米，左右为有限 15°转向，转完保持停车，长闭眼取消。单眨眼、回正和健康心跳都不能启动或恢复任务。候选可确认不代表保证可执行。

`interaction_mode` 决定输入交互。新交互首版要求 `mode:=corridor_assist`（默认自动选择）；其他辅助策略保留在旧 `gaze_joystick` 中对照，冲突配置会在启动前明确拒绝。运行时修改交互配置会被拒绝，须停车、确认停稳后重启。新的相机组合入口默认使用 `intent_confirm`；单独 `run_sim.sh` 不指定交互模式时保持旧键盘兼容。

键盘体验同一任务接口：

```bash
# 终端 1
~/wheelchair_shared_ws/scripts/run_sim.sh interaction_mode:=intent_confirm input_source:=keyboard scene:=clearpath_office
# 终端 2
~/wheelchair_shared_ws/scripts/intent_keyboard.sh
```

W/A/D 选择候选，Enter 确认，空格/S 取消。任务替换由系统完成停车握手，不需要额外按 R。

真实眼动输入控制仿真：

```bash
cd ~/projects/EYETLE
WHEELCHAIR_INTERACTION_MODE=intent_confirm WHEELCHAIR_MODE=corridor_assist \
./start_eye_turtle.sh --gazebo --ritnet --eyes both \
  --calibration-file ~/projects/EYETLE/state/ritnet-calibration.json
```

该入口自动启用带源读帧时间戳的消息；`WHEELCHAIR_INTERACTION_MODE=gaze_joystick` 可选择旧交互。检测/标定算法未重写，没有实机运动输出。

实现、验收和限制见 [实施记录](../projects/WHEELCHAIR_INTENT_SHARED_CONTROL_IMPLEMENTATION.md)，字段和兼容规则见 [协议 v1](docs/INTENT_PROTOCOL_V1.md)，复现与人工标注模板见 [实验说明](docs/INTENT_EXPERIMENTS.md)。以下原有直接键盘/眼动操作说明适用于旧交互模式。

[项目工作总结](../projects/WHEELCHAIR_SHARED_CONTROL_WORK_SUMMARY.md)：交付内容、问题修复、验证结果和后续限制。

[窄通道改进实现与启动](../projects/WHEELCHAIR_NARROW_PASSAGE_IMPLEMENTATION.md)：新 `corridor_assist` 模式、独立配置、诊断视图和验证范围。

[当前避障逻辑概述](../projects/WHEELCHAIR_OBSTACLE_AVOIDANCE_OVERVIEW.md)：控制链路、两种模式、停车恢复及已知限制。

按 `/home/whill/projects/EYE_WHEELCHAIR_SHARED_CONTROL.md` 实现。工作空间独立于旧 TurtleBot 和 EYETLE 演示；未修改眼动识别、标定、BLE 控制或 ESP32 固件。

**当前交付为软件与真实 Gazebo 仿真闭环，不连接真实轮椅。** 底盘为有轮子、惯量和完整碰撞体的通用轮椅占位模型，不是厂家验证 CAD。前进/左转/右转固定档位，不能任意缩小 v/w，没有自动倒车，也不组合前进与转向。

## 启动和操作

默认使用开源 **Clearpath Office** 办公室，模型平面跨度约 **44.01 × 25.45 m**；这是包围范围，不是可用地板面积。模型与贴图来自 [Clearpath Robotics](https://github.com/clearpathrobotics/cpr_gazebo/tree/509afbeec942c6aad376a5a76a24330f1c56dc17/cpr_office_gazebo)，BSD-3-Clause 许可，原许可已随资产保留。

Gazebo 启动时自动显示蓝色激光扫描点，无需手动添加插件。当前为 360° 单线激光：720 个采样方向、12 m 量程，点位于约 0.45 m 的传感器扫描平面，不能覆盖整个办公楼。视图默认俯瞰全景，可用滚轮放大、中键旋转、左键拖拽平移。点显示不参与底盘指令链路。

本机已构建，Gazebo Fortress 运行库由本工作空间的 `vendor/runtime` 持有，不再依赖 TurtleBot 工作空间；路径可用 `WHEELCHAIR_GAZEBO_RUNTIME` 覆盖。

### 键盘 + 默认 stop_only

终端 1：

```bash
~/wheelchair_shared_ws/scripts/run_sim.sh
```

终端 2：

```bash
~/wheelchair_shared_ws/scripts/keyboard.sh
```

直接按 **W/A/D** 请求前进/左转/右转，正常操作无需 R，也没有键盘 5 秒授权限制。动作会保持到切换方向、**空格/S 停车**、退出或安全层介入；X 退出，没有后退键。

只有遇到障碍或发生故障后才锁定停车，并显示原因。此时一直按住 W 不会自动恢复；等待停稳和传感器正常，按 **R 恢复**，再按方向键。R 本身不执行旧动作。主动按空格/S 停车后，下一次可直接选择方向，不需要 R。仿真未就绪时方向键不排队，准备好后重新按键。

底层仍使用相同的意图仲裁、安全检查和短期有效消息；首次按方向键自动完成正常启用流程，停止、过期、进程退出和碰撞保护保持有效。眼动首次双眨眼启用；正常行驶已取消 5 秒上限，暂停后用单次短眨眼恢复。

用户反馈：

```bash
~/wheelchair_shared_ws/scripts/status.sh
```

显示请求、执行、介入原因、requires_rearm 与 ready。当前仅有终端/ROS 状态反馈，未实现语音。

### 眼动接口

先关闭之前的仿真，再运行：

```bash
~/wheelchair_shared_ws/scripts/run_sim.sh input_source:=eye
```

眼动包装节点由主 launch 启动，不需要另开旧海龟控制器。另一个终端可用合成眼动：

```bash
source ~/wheelchair_shared_ws/scripts/env.sh
export ROS_DOMAIN_ID=64 ROS_LOCALHOST_ONLY=1
ros2 run eye_shared_control mock_eye --direction forward --duration 10 --arm-after 3
```

工具在指定时间只生成一次双眨眼，仿真此时必须已启动并健康。若双眨眼时仍未 ready，将保持停止；可重新运行工具作新的确认。实际按时间驱动的联调验收脚本会等待 ready 后再生成眨眼。

连接已有真实检测器（用户自行启动，不连接真实轮椅）：

```bash
cd ~/projects/EYETLE
EYE_ROS_DOMAIN_ID=64 ./start_ritnet.sh
```

先关闭 mock_eye，避免同时多个源。原检测器默认域 58，必须按上面设置 `EYE_ROS_DOMAIN_ID=64`；单独设置 ROS_DOMAIN_ID 会被它的环境脚本覆盖。保留原 `/eye_tracking/signals` 的 `[horizontal, eye_closed, valid]` 接口及原检测/标定功能。

双眨眼授权，正中前进，左/右持续注视持续请求相应原地转向；方向稳定 250 ms 后切换，进入/退出阈值 0.25/0.15。短眨眼保留已确认方向；长闭眼 650 ms 进入 PAUSED；invalid 和源超时立即停车。恢复有效输入后仍保持停止，通过一次新的60～400ms短眨眼恢复，正常行驶无5秒限时。详细说明见 [EYE_README.md](EYE_README.md)。

### 切换有限辅助和场景

```bash
~/wheelchair_shared_ws/scripts/run_sim.sh mode:=bounded_assist scene:=slight_offset
```

场景参数：

| scene | 场景 |
| --- | --- |
| clearpath_office | **默认**：开源大办公室，多房间、走廊和办公家具 |
| complex_indoor | 10×8 m 的本地室内测试场景，带桌椅和门洞 |
| straight_wall | 正前方墙，原停车测试场景 |
| rotation_wall | 贴墙转向，检查脚踏/完整车体扫掠 |
| narrow | 窄通道，允许安全直行，拒绝危险旋转 |
| slight_offset | 轻微偏置低障碍，有限侧偏实验 |
| both_blocked | 前方及两侧均被阻挡 |
| dynamic | 实际运动的障碍物，地图预留整个运动走廊 |

`gui:=false` 可不打开 GUI，但 GPU 激光仍使用本机现有图形环境。一次只运行一套相同 domain/partition 的仿真。默认 ROS_DOMAIN_ID=64、ROS_LOCALHOST_ONLY=1、IGN_PARTITION=wheelchair_shared_demo；与旧 demo 的域隔离。启动脚本显式采用 `WHEELCHAIR_ROS_DOMAIN_ID`（默认 64），覆盖终端遗留的 ROS_DOMAIN_ID；自定义时所有新工程脚本及眼动检测器应使用相同域。

## 新的控制逻辑

```text
/eye_tracking/signals → 眼动状态机 → /wheelchair/eye_intent ┐
                                                         ├→ 显式选源仲裁 → /wheelchair/intent
键盘状态机                     → /wheelchair/keyboard_intent ┘

/wheelchair/intent + 实际 odom + 仿真障碍图 + scan/TF 健康
                          ↓
                离散动作规划 + 最终安全门
                          ↓ /wheelchair/action_safe
                    离散执行器
                          ↓ 固定档位或零
                    仿真底盘看门狗 → Gazebo
```

只有选中的输入源可以下达意图，不自动切换到另一个源。键盘空格的紧急 STOP 事件即使在眼动模式也生效。切换源需重启节点，默认禁止运动。仲裁器重启后必须先看到禁用 STOP，不能直接转发旧运动心跳。

共享接口定义在 `wheelchair_interfaces`：ControlIntent、DiscreteAction、ControlStatus、ExecutionStatus、ObstacleMap。动作枚举统一 STOP=0、FORWARD=1、LEFT=2、RIGHT=3。

### B1：固定动作安全门

`stop_only` 是默认对照组：仅输出用户动作或 STOP。前进目标 0.372 m/s，左转目标 +0.389 rad/s，右转目标 -0.403 rad/s；这些来自方案的预测初值，不是实机测量结果。执行器输出集合只包含 `(0,0)`、`(.372,0)`、`(0,.389)`、`(0,-.403)`，物理加减速由 Gazebo 底盘模型执行。

安全检查使用配置中的凸多边形轮廓，默认 x∈[-0.45,0.55]、y∈[-0.35,0.35] m，覆盖仿真脚踏、扶手和轮子。预测从 Gazebo 实际位姿/速度开始，覆盖：

1. 实测观测年龄和传输/控制延迟内的当前执行动作与动量；无实际年龄时采用配置最大年龄预算。
2. 不兼容方向的先刹车过程、启动延迟和短期动作授权。
3. 最后的传输延迟与完整线/角制动过程。

每 25 ms 采样完整轮廓，按最大平移速度与 `外接半径×角速度` 对半步间隔进行膨胀，另加几何间隙和模型误差余量。障碍物碰撞或越过已知边界是硬拒绝条件；不能通过较高前进评分抵消。

STOP 是停车指令，不是假设瞬间静止。无安全动作时发零并锁定；如果障碍突然进入当前动量的停车包络，会报告 `STOPPING_ENVELOPE_RISK`，不宣称物理上必然不碰撞。

### B2：有限辅助

只有 FORWARD 可以选择侧偏；LEFT/RIGHT 仍只放行原方向或 STOP。搜索 `STOP→LEFT/RIGHT→STOP→FORWARD` 的候选，先通过完整碰撞/未知空间/停止余量/累计偏转检查，再按进度、方向偏差、间隙和切换成本评分。左右差异不足时停车请人选择；选侧后保持该侧，不周期左右摇摆。

当前动量的停车段单独校验；随后最多预测 3 秒的转向/前进/停车序列。只发布当前动作授权，逐周期重新检查，绝不把完整计划排入执行器队列。用户 STOP、新方向、失效或故障都会取消旧候选。

约束：累计朝向偏离 ≤15°、横移 ≤0.25 m；一次自动动作授权最长 2 秒。**2 秒从第一次自动转向阶段开始计时，前置停稳等待另有 2 秒超时；实际制动时间仍纳入碰撞预算。** 转后沿偏转方向前进也计入这 2 秒；结束后停车并等待用户重新确认或明确选择方向。每次用户确认的运动授权最多一次自动侧偏；新的禁用 STOP 确认流程完成、用户再次授权后，重新获得时间/次数预算。持续 FORWARD 心跳和单独 STOP 不能刷新预算。安全 STOP/重新授权不会重置累计角度和横移参考，只有用户明确 LEFT/RIGHT 后再 FORWARD 才建立新参考。

此模式用于轻微偏置，不承诺绕过所有障碍或完成整条通道。为满足角度/时间边界，它可能完成小转向和短前进后就停车。

### 恢复、顺序和执行器

- 启动、阻挡、故障、过期、重放、会话变化：输出 STOP，requires_rearm=true。
- 输入先撤销授权，发送 `enabled=false, command=STOP`。
- 传感器健康、当前轮廓有空间、实际反馈停稳后，ready_to_rearm=true。
- 阻挡/故障后，需要底层允许后以新的单次短眨眼恢复动作或键盘 R 确认，再选择方向。持续旧 FORWARD 不会自动解除停车。键盘正常启动与主动停车后的再次操作由新方向键直接确认，内部仍完成禁用 STOP→ready→启用。
- 同方向的新序号消息只续期，不重复释放/启动。新方向清掉旧待执行动作，输出 STOP，实际 odom 低于阈值且稳定后才允许启动新方向。迟迟不停稳会 FAULT。
- 命令 volatile/depth=1；源时间、UUID 会话、严格递增序号及单调接收期限共同检查。仿真暂停/时间跳变使旧授权失效。

意图 20 Hz / TTL≤250 ms；动作 20 Hz / TTL≤150 ms。执行器丢失授权发零；执行器进程死亡时，独立仿真底盘看门狗在 200 ms 内发零。真正停稳还需要物理制动过程。

## 感知范围与实机边界

本阶段按方案允许的第一种路径，使用**已知仿真障碍图**闭环：场景 SDF 和障碍多边形由同一份几何描述生成，包含车体保护高度内的低障碍，显式排除地面和机器人自身。Clearpath Office 从原始网格提取 z=0.04～1.30 m 的碰撞表面，并投影为 5 cm 栅格、保守膨胀一格后合并成 1068 个矩形。独立平面承担地面接触，当前轮椅最高物理部分低于 1.30 m；此高度适配不适合更高的机器人或有人乘坐的实机。办公室地图以 10 Hz 发布；其他场景仍为 20 Hz。详细来源和适配见 `src/wheelchair_sim/models/clearpath_office/ADAPTATION.md`。

大图使用缓存和保守的局部候选筛选；筛选范围覆盖动量、完整预测/制动距离及车体余量。雷达和 odom 接收队列为最新一帧，避免旧帧排队。没有放宽失联/过期停车阈值。

地图完整且边界内才允许规划；边界外/地图不完整为未知，不能当自由空间。

LaserScan、TF、实际 odom 和障碍图均有健康/过期检查，但当前**没有用 LaserScan 或点云重建碰撞地图**。不要通过 Gazebo GUI 任意加物体后假定它已被纳入安全图；新增几何必须同步场景源/地图。dynamic 的地图保守预留完整移动走廊，不能用于验证对临时空隙的抢行或动态障碍预测。真实轮椅必须先实现经过验证的感知适配器，不能使用这个 oracle。

未实现：3D 点云处理、落差检测、BLE 按住/释放/回中驱动、轮椅端失联释放保障、实机尺寸与最坏制动/通信时延测量。测试限定平整无落差地面。`Ble2WChair` 的 tap 或积分 odom 没有接入，本工程 odom 来自 Gazebo 真实模型位姿。

旧 Float32MultiArray 无采集时间/源序号；眼动包装只能监测接收年龄，不能证明相机帧龄。真实眼动误触发率也未在此次合成测试中量化。

## 配置、测试与记录

控制参数：`src/wheelchair_control/config/control.yaml`；眼动参数：`src/eye_shared_control/config/eye.yaml`。为避免运行中改变制动预算，控制节点拒绝动态改参，需停止后修改配置并以 DISARMED 重启。自定义控制配置用 `config:=/绝对路径/file.yaml`。

构建：

```bash
cd ~/wheelchair_shared_ws
source scripts/env.sh
colcon build --symlink-install --base-paths src
```

纯逻辑/几何/眼动回归：

```bash
source ~/wheelchair_shared_ws/scripts/env.sh
cd ~/wheelchair_shared_ws
ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 -m pytest tests src/eye_shared_control/test src/wheelchair_sim/test/test_scene_geometry.py -q
```

真实 Gazebo 端到端验证：

```bash
source ~/wheelchair_shared_ws/scripts/env.sh
ROS_DOMAIN_ID=97 IGN_PARTITION=wheelchair_validation ROS_LOCALHOST_ONLY=1 \
  /usr/bin/python3 ~/wheelchair_shared_ws/scripts/validate_shared.py --scenario b1
```

其他 scenario：`office`（大办公室保护/碰撞对照）、`eye`、`assist`、`sensor`、`pause`、`replay`、`executor_exit`、`rotation_wall`、`narrow`、`both_blocked`、`dynamic`。验证脚本只启动并关闭自己的隔离仿真；不会连接 BLE/真实底盘。结果为 `artifacts/result-*.json`，日志为 `artifacts/gazebo-*.log`。成功、失败和未测范围见 [TEST_RESULTS.md](TEST_RESULTS.md)。

实验记录：`scripts/record.sh -o /保存路径/实验名` 保存 rosbag，包含眼动源、意图、动作、状态、候选原因、实际 odom、障碍图、激光和碰撞接触。`/wheelchair/decision_trace` 逐周期给出来源时间/序号、候选安全性、累计参考与介入原因。


## 办公室资产与可视化构建

显式选择办公室：

```bash
~/wheelchair_shared_ws/scripts/run_sim.sh scene:=clearpath_office
```

原始资源固定在 `vendor/cpr_gazebo` 的提交 `509afbeec942c6aad376a5a76a24330f1c56dc17`，运行所需模型/纹理及许可证位于 `src/wheelchair_sim/models/clearpath_office`。可用 `scripts/import_clearpath_office.py` 重建碰撞网格与障碍图。原始视觉模型不变；派生碰撞模型限制当前轮椅保护高度，窄门可能因安全余量而被保守拒绝。

`wheelchair_scan_viz` 是本地 Gazebo GUI 插件，直接订阅 Gazebo 原生 LaserScan，并通过场景传感器实体获取实际世界位姿；Fortress 的该消息 world_pose 为单位变换，不能直接用于显示。500 ms 未收到新扫描会隐藏旧点。构建头文件可通过 `scripts/install_scan_viz_deps.py` 安装到已有用户目录运行库，不使用 sudo。重新构建时先关闭使用插件的 Gazebo：

```bash
cd ~/wheelchair_shared_ws
/usr/bin/python3 scripts/install_scan_viz_deps.py
source scripts/env.sh
colcon build --symlink-install --base-paths src
```


### 轮椅尺寸与低位雷达（2026-10-13）

当前是简化电动轮椅，实体外廓长 1.00 m、宽 0.60 m、最高 0.85 m；座面高 0.48 m，扶手顶高 0.70 m。座面和靠背比例参考 [WHILL C2 官方手册](https://www.whill.inc/product-literature/au/model-c2-user-manual.pdf) 的可调尺寸范围，但不是 C2 精确复刻，动力学仍为差速底盘。车轮中心间距 0.50 m，与 Gazebo 驱动参数一致。

避障仍用 x[-0.45,0.55]、y[-0.35,0.35] 的保守矩形，完整包住实体，并在左右各留 5 cm。外观和物理碰撞体同步调整。办公室几何地图仍保守覆盖 0.04–1.30 m 高度，未随外观缩小而删减障碍。

雷达相对地面投影 base_link 的位置为 (0.48, 0, 0.45) m，安装在座椅前方。SDF 与静态 TF 共用位置常量。启动时模型有 5 mm 离地间隙，落地稳定后扫描高度约 0.45 m。低位单线雷达会被自身座椅/靠背遮挡部分后向视野，也无法看见不穿过扫描平面的障碍；当前避障仍使用仿真场景几何地图，不是仅凭这一圈扫描进行感知。


### 办公室空旷区域异常停车修复（2026-10-13）

在 (-2.45,-1.23) 附近实测复现：规划器仍判定通行，但 Python 多边形距离计算阻塞回调，部分动作到达时已经消耗约 90 ms 的 150 ms TTL，执行器随后报 ACTION_STALE，控制器锁定为 EXECUTOR_FAULT。早期记录也出现过 EXECUTOR_STALE。此问题与雷达车体回波无关。

现缓存不可变多边形，使用 Shapely/GEOS 批量计算完整几何距离，并用包围盒下界剔除不可能影响最小距离的候选；保留 Python 标量计算作为差分测试参照。没有增大 TTL、缩小轮廓、减少预测采样或关闭停车保护。decision_trace 增加 compute_ms 和 executor_reason，便于区分障碍停车与链路故障。

运行依赖（本机已安装；新机器先安装）：

```bash
/usr/bin/python3 -m pip install --user -r ~/wheelchair_shared_ws/requirements-runtime.txt
```

诊断脚本 scripts/diagnose_office_stop.py 会通过正常键盘意图接口让**仿真**轮椅运动，运行前应关闭键盘节点，避免两个输入会话冲突。它记录动作/执行反馈/里程计的接收间隔、源时间年龄、状态原因、计算耗时和接触记录，最终发送 STOP。其 --expect-pass / --expect-blocked 参数分别检查自由运动和遇障停止。


### EYETLE RITnet 双目眼动一键启动

关闭旧轮椅仿真后运行（会同时启动真实 Gazebo 办公室、眼动共享控制和 RITnet 双目预览）：

```bash
cd /home/whill/projects/EYETLE
./start_eye_turtle.sh --gazebo --ritnet --eyes both \
  --calibration-file /home/whill/projects/EYETLE/state/ritnet-calibration.json
```

等价入口是 `./start_eye_wheelchair.sh --ritnet --eyes both --calibration-file ...`。原 `start_eye_turtle.sh` 不加 `--gazebo` 仍启动 turtlesim。

新入口将两个工作空间的 ROS domain 统一为 64（可用 WHEELCHAIR_ROS_DOMAIN_ID 覆盖），固定 `input_source:=eye`，不启动旧的 turtlesim 速度控制器。数据路径：RITnet `/eye_tracking/signals` → eye_intent → `/wheelchair/eye_intent` → intent_mux → 公共避障和执行器 → Gazebo。相机识别和仿真分别使用各自环境，避免 TensorRT 环境和 Gazebo 库相互污染。

在眼动识别有效时，双眨眼确认后，看中间前进、看左/右转向（注视250ms确认）。长闭眼650ms、识别失效或遇障都会停车。正常行驶无5秒限时；暂停后睁眼后再短眨眼一次恢复。故障锁止必须先由底层确认可恢复。按 Esc 关闭 RITnet 预览或在启动终端 Ctrl+C 会同时关闭整套眼动仿真；任一主进程退出时启动器会关闭另一方。

可另开 `~/wheelchair_shared_ws/scripts/keyboard.sh`，空格可紧急停车；眼动模式下 W/A/D 不接管方向。要切回键盘驾驶，关闭眼动仿真再用 `run_sim.sh` 启动。`scripts/status.sh` 可查看当前意图、执行动作和停车原因。

真实相机接口联调与合成眼动动作测试分别记录；相机成功加载标定不代表每一帧的眼睛识别都有效，valid=0时轮椅保持停止。


眼动＋有限绕障辅助（先关闭旧仿真）：

```bash
cd /home/whill/projects/EYETLE
WHEELCHAIR_MODE=bounded_assist ./start_eye_turtle.sh --gazebo --ritnet --eyes both \
  --calibration-file /home/whill/projects/EYETLE/state/ritnet-calibration.json
```

默认仍为 stop_only。bounded_assist 只在前进意图下尝试小幅侧偏，受15°、0.25m、2秒边界约束；无安全候选则停车，不保证绕过整堵墙。可同时设置 WHEELCHAIR_SCENE=slight_offset 使用有限绕障测试场景。


### 窄走廊辅助恢复修复

原日志在 (2.93485,0.97368)、朝向0.29968 rad处反复出现 ASSIST_TIME_LIMIT：先前辅助的开始时间和已使用标记被错误地带到了重新确认后的新授权。现在只在禁用STOP握手完成并接受新的用户授权时重置介入时间/次数预算，持续前进心跳不会续命。累计15°/0.25m空间参考保留，碰撞和停车预测不变。

重新确认不等于必然前进。如果继续自动纠偏会越过累计范围，仍需用户主动选择安全转向，对正走廊后再前进。本例的原始日志没有显示末段LEFT/RIGHT请求，重复的是FORWARD；不能据此认定车身物理上无法转向。

可复现命令（先关闭其他Gazebo实例）：

```bash
source ~/wheelchair_shared_ws/scripts/env.sh
ROS_DOMAIN_ID=97 IGN_PARTITION=wheelchair_validation ROS_LOCALHOST_ONLY=1 \
  /usr/bin/python3 ~/wheelchair_shared_ws/scripts/validate_shared.py --scenario corridor
```


### 新模式：直通道/门洞辅助

```bash
~/wheelchair_shared_ws/scripts/run_sim.sh input_source:=keyboard mode:=corridor_assist scene:=clearpath_office
~/wheelchair_shared_ws/scripts/keyboard.sh
```

两条命令分别在两个终端运行。新模式自动选择 narrow_control.yaml；旧 stop_only / bounded_assist 默认参数仍保留。新模式核对实体0.60m宽，保留0.06m硬间距和0.025m模型误差，另以0.15m作为舒适评分。先在安全位置对正，再在固定通道参考下多次微调；明确转向、STOP、故障都取消旧计划。仅支持已知静态地图下的轴对齐直通道/门洞，不自动转90°弯或选择分叉。可选 scripts/visualize_safety.sh 打开决策对应的RViz图层。

完整实现、误差预算、能力测量、最终验收和未覆盖项见上方“窄通道改进实现与启动”。


## 2026-10-13 出口靠墙人工微转

`corridor_assist` 模式下，A/D 请求的常规转向空间不足时，会检查并执行同方向 0.10 s 短脉冲，停稳后重评。短脉冲由执行器强制限时，心跳不会延长；到达安全转向边界时显示 `USER_TURN_LIMIT_REACHED` 并保持停车，可直接选 W。真正危险的方向仍报 `BLOCKED_TURN`，此时停稳后按 R 再选择可行方向。

本次实际办公室卡点 `(10.9067, 0.8737)` 可通过“R（若已锁定）→ A，等待微转停稳 → W”脱困。该顺序只针对这个已验证位置，不代表所有靠墙姿态都应左转。没有新增自动倒车。

消息 `DiscreteAction` 新增 `max_run_ms`（0 表示无附加脉冲上限）；更新后须同时重启规划器和执行器，避免混用旧消息定义。完整回归 145 项通过；原卡点 Gazebo 重放见 `artifacts/result-exit_recovery.json`。运行记录见 TEST_RESULTS.md。

## 办公室导航 RViz2 诊断

仿真运行时在新终端执行 `~/wheelchair_shared_ws/scripts/navigation_rviz.sh`。显示办公室地图、局部代价地图、雷达、轮廓、规划/实际轨迹和停车原因。详见 [使用说明](docs/NAVIGATION_RVIZ.md)。
