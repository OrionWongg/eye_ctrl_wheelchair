# 实验记录：2026-10-12

> 当前 Nav2 路径控制改造尚未完成整体验收，已按用户要求停止继续调试。最新状态、已通过场景与未解决问题见 [Nav2 改造总结](../projects/WHEELCHAIR_NAV2_DISCRETE_PATH_CONTROL_IMPLEMENTATION.md)。下文既有通过结果属于对应历史版本，不能作为当前改造版本的全量通过证据。


环境：本机 Jetson ARM64、Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress 6.18.0。软件阶段，未连接 BLE/真实轮椅，未自动打开眼动摄像头。

## 自动化测试

纯逻辑、ROS 输入接口、几何与眼动状态机测试：**45 项通过**；运行命令见 README，最终结果在 `artifacts/unit-tests.log`。覆盖固定档位、线/角动量与制动、脚踏轮廓扫掠、未知边界、两步恢复、重复/过期/重放、会话切换、停止超时、方向转换、累计辅助范围及眼动单/双眨眼、停留/滞回、失效和长闭眼。

## 真实 Gazebo 端到端

所有测试使用独立 domain 97、partition `wheelchair_validation`，从真实 Gazebo 模型位姿获得 odom，使用实体碰撞和接触传感器。

| scenario | 验证结果 |
| --- | --- |
| b1 | 持续前进后停车；旧 FORWARD 不解除锁定；重新确认可转向；左右切换先停车；用户 STOP 生效 |
| eye | 合成双眨眼→FORWARD→invalid→STOP；信号恢复不自动启动 |
| assist | 实际执行固定右转后再前进，累计角度/横移在限制内，达到介入时间上限后停止 |
| sensor | 障碍图发布进程暂停后停车；恢复地图也不能用旧意图解锁 |
| pause | Gazebo 暂停/恢复撤销授权，不执行旧运动状态 |
| replay | 重复/旧序号运动消息被拒绝，后续普通心跳不能解除锁定 |
| executor_exit | 杀死执行器后底盘看门狗停车；重新启动执行器不恢复旧意图 |
| rotation_wall | 初始车身可停放，但脚踏旋转扫掠不安全，拒绝左转 |
| narrow | 固定速度安全直行，危险原地旋转被拒绝 |
| both_blocked | 无安全动作时保持停车，不反向、不自动脱困 |
| dynamic | 真实障碍物移动，使用完整运动走廊的保守占用图，停车且无碰撞 |

11 个场景均 PASS，机器人接触次数均 **0**。所有执行器 Twist 样本只落在以下集合，没有缩小速度放行：

```text
(v, ω) = (0,0)、(0.372,0)、(0,+0.389)、(0,-0.403)
```

`b1` 记录：停车 x≈1.010 m；前墙表面 x=1.900 m，车体前缘相对 base_link 为 +0.55 m，所以该次停车车体前缘间隙约 **0.34 m**。停车位置会受调度、观测年龄和制动过程影响，不是一个硬编码距离。

`assist` 记录：朝向约 -0.0897 rad（-5.14°），横移约 -0.0191 m；这证明一次有限侧偏和后续前进，不代表绕过整个障碍或到达目标点。

每项完整数据见 `artifacts/result-*.json`，过程日志见 `artifacts/gazebo-*.log`。仿真子模块另做了**故意绕过规划器**的接触阳性对照，5090 条重复接触消息且物理车体被墙限制，确认 contacts 有效；这不计入受保护测试的零碰撞结果。

## 启动与关停

三维 GUI 已实际打开并显示通用轮椅与墙体，截图为 `artifacts/gazebo-wheelchair.png`。最新 replay 与 executor_exit 验证已正常退出；早期个别日志里的退出时上下文销毁异常已通过受控信号与有界 spin 修复。

## 实验边界

- 当前是已知几何 oracle，不是实时雷达/点云碰撞地图；不要把手工向 Gazebo 添加的物体当成自动已被保护。
- 动态场景占满运动走廊，不验证动态空隙通行或突然闯入可接受停车距离内的避碰能力。
- 本机 360° 激光/TF/odom 为仿真输入；受保护测试里的“sensor”注入对象是障碍图发布源，不是实体雷达。
- 眼动验收输入为合成信号，未量化真实误触发/漏检率。
- 实机执行器、失联释放保障、制动参数和传感器布局尚未验收。

## 键盘确认计时修复

修复从 R 开始计算 0.5 秒运动确认超时导致稍后按 W 即撤权的问题，改为首次发布运动请求时起算；保留 5 秒总授权、真实确认超时、状态失联及安全阻挡停车。增加延迟 1 秒按 W、持续 W、确认缺失停车和 R 不延长授权的回归。针对键盘接口运行 3 项测试通过（含原测试与 2 项新增测试）。终端现在显示具体撤权原因并抑制方向键重复刷屏。

## 键盘直接控制（后续用户要求）

新的键盘交互替代前述初始 R 确认和键盘 5 秒限制；眼动策略不变。键盘接口 6 项回归通过，覆盖直接 W、超过 5 秒保持、遇障长按不能解锁、R 恢复不执行旧方向、主动停车后直接继续、状态超时和确认缺失仍停车。

真实 Gazebo 使用 `scripts/validate_direct_keyboard.py` 调用实际 Keyboard 节点验证，结果 `artifacts/result-direct-keyboard.json` 为 PASS、碰撞 0。隔离运行命令：

```bash
source ~/wheelchair_shared_ws/scripts/env.sh
ROS_DOMAIN_ID=98 IGN_PARTITION=wheelchair_keyboard_test ROS_LOCALHOST_ONLY=1 \
  /usr/bin/python3 ~/wheelchair_shared_ws/scripts/validate_direct_keyboard.py
```

## 开源办公室及雷达点显示

- Clearpath Office（BSD-3-Clause），固定提交 `509afbeec942c6aad376a5a76a24330f1c56dc17`。
- 原始视觉网格范围约 44.01 × 25.45 m，8 张原始贴图；派生碰撞高度带 0.04～1.30 m，障碍图 1068 个矩形。
- 原生 Gazebo 点显示插件已实际加载；使用实体的世界位姿修正单线雷达显示高度，自动订阅真实扫描。
- `--scenario office` 实测 PASS：保护开启时停车 x=2.675423 m，碰撞 0。隔离仿真绕过保护的对照产生 6458 条重复接触，证明碰撞网格有效。
- 大办公室测试应独占运行一套 Gazebo；两套大型仿真并行时曾触发时延停车。保留原看门狗阈值，采用地图缓存、局部候选筛选及最新一帧接收减少负载。
- 详细结果：`artifacts/result-office.json`；新的几何和 ROS 回归：`artifacts/office-tests.log`。

办公室接入后的完整回归：53 项通过。最终 GUI 截图：`artifacts/clearpath-office-points.png`。

## 2026-10-13 低位雷达与轮椅比例

实体外廓 1.00×0.60×0.85 m，座面 0.48 m；雷达 TF 与 SDF 共用 (0.48,0,0.45) m。54 项 pytest 通过，包括碰撞体/外观尺寸、车轮间距一致性检查。Gazebo 实测 TF=(0.48,0,0.45)，720 射线正常。办公室持续前进保护测试 PASS，停车 x=2.6431 m，保护期间碰撞 0；独立测试中关闭保护后的物理碰撞正控产生 6155 条重复接触报告。详见 artifacts/result-office.json、artifacts/low-lidar-runtime.json。

## 2026-10-13 原位置异常停车复现与修复

实际在 x≈-2.45,y≈-1.23 处复现 ACTION_STALE：前一条动作到达时年龄约89 ms，下一条还未到达就超过150 ms租期。规划结果仍安全、执行反馈约50 ms周期、里程计正常，无碰撞。记录 office-stop-before-detail.json。

改为缓存原多边形并调用 GEOS 批量精确距离，不改变安全余量、动作TTL或预测采样。62项测试通过，含随机旋转凸多边形差分测试与接触/安全边界附近比较。

同一Gazebo场景和原区域，连续左转35秒，694条运动判定全部PASS，最大计算22.44 ms；再实际前进到(-1.969,-1.089)，运动期间无故障，最大计算17.90 ms。继续持续前进23秒，遇真实障碍在(2.598,0.232)正常BLOCKED_FORWARD并停稳，运动期间无FAULT、接触0。未移除车体回波，证明本次停车原因是计算引起的动作过期而非车体回波。详见 artifacts/office-stop-fix-summary.json 及对应原始事件记录。

## 2026-10-13 EYETLE RITnet → Gazebo 集成

新增 EYETLE start_eye_turtle.sh 的 --gazebo 入口及 start_eye_wheelchair.sh，由 scripts/run_eye_sim.py 统一监管 Gazebo 和 RITnet，固定眼动输入，通信域一致，保留原 turtlesim 模式。脚本语法和Python编译检查通过。

真实双目相机采集300帧，成功加载2眼标定，约19.67 FPS；报告 artifacts/eye-gazebo-ritnet-report.json。运行中的GUI联调接收226条真实信号、342条眼动意图、342条选中意图，眼动与选中意图session一致。该次采样valid帧为0，故未声称完成真人眼动驾驶；系统保持INVALID_INPUT/STOP。记录 artifacts/eye-gazebo-live-topics.json。

独立ROS domain97的真实Gazebo合成眼动闭环测试PASS：双眨眼后前进，valid=0后停车，信号恢复不自行起步，碰撞0。记录 artifacts/result-eye.json。真实信号链路验证与合成眼动动作验证分开记录。

## 2026-10-13 窄走廊辅助状态恢复

从 .ros/log/python3_157986_1791872891832.log 提取原始状态到 artifacts/corridor-original-trace.json。实际卡点 (2.9348465,0.9736821)、yaw0.2996809；首次辅助后 ASSIST_ALREADY_USED，随后用户多次新授权均立即 ASSIST_TIME_LIMIT。末段请求记录为FORWARD，未记录LEFT/RIGHT被拒绝。

修复新授权继承旧介入计时/已用标记的问题，仅在有效的禁用STOP→新授权转换且安全层已解除锁定时更新时间/次数预算。累计空间参考保持，不让心跳/单独STOP刷新预算，不放宽碰撞余量。

新增5项回归覆盖原坐标重新确认、旧FORWARD不能解锁、累计空间边界不重置、明确左右转向不受旧计时锁定及持续授权仍有时间限制。真实Gazebo --scenario corridor PASS：复现初次辅助停止后，新确认不再被旧计时拦截；此时剩余空间预算仍可能NO_USEFUL_SAFE_ASSIST，用户明确右转对正后实际通过1.629m，碰撞0，输出仍只有固定档位和STOP。没有宣称仅保持FORWARD即可自动通过整条走廊。结果 artifacts/result-corridor.json。

## 最终验收摘要

115 项测试通过；以下为最终参数下的实际 Gazebo 单次案例，不是统计完成率。所有案例机器人接触数为 0。

| 案例 | 结果 | 前进距离 | 最大规划耗时 |
|---|---|---:|---:|
| passage080 | 通过 | 2.134 m | 12.0 ms |
| passage090 | 通过 | 2.115 m | 12.9 ms |
| passage100 | 通过 | 2.123 m | 16.5 ms |
| passage110 | 通过 | 2.122 m | 13.6 ms |
| door090 | 通过 | 1.938 m | 29.5 ms |
| hard_reject | INITIAL_HARD_BOUNDARY | — | 2.7 ms |
| exit_reject | PASSAGE_BLOCKED_COLLISION_OR_UNKNOWN | — | 8.6 ms |
| office_original | PASSAGE_END_OR_BRANCH | 1.640 m | 20.9 ms |
| legacy_b1 | 通过 | — | 见日志 |
| legacy_assist | 通过 | — | 见日志 |

实现与误差预算见 ../projects/WHEELCHAIR_NARROW_PASSAGE_IMPLEMENTATION.md。开发中较小硬余量的结果仅保留在 artifacts/narrow-preliminary，最终验收采用 margin=0.06、model_error=0.025、0.60m实体轮廓。

## 2026-10-13 开放区域反复停车修复

修复 passage=None 且车体正在运动时错误返回 PASSAGE_WAIT_OBSERVATION_STOP 的逻辑。真实Gazebo smooth测试PASS：连续前进窗口巡航速度0.372m/s，起步后多余STOP为0，遇障正常停车，机器人接触0；见 artifacts/result-smooth.json。新增移动中未识别通道、障碍仍阻挡、既有可信范围扩展保留停稳检查的回归。

## 侧门连续性和人工选路

side_choice：直行经过侧门3.03m；明确右转再前进，进入门内1.45m，出口停车，碰撞0。真实办公室原卡点前进2.10m通过侧门缺口，碰撞0，硬余量0.092806m保持。新增单侧开口连接、双侧交叉/障碍拒绝连接、用户新方向不吸附旧轴、选择垂直门洞和延伸观察保留固定参考的回归。

## 2026-10-13 狭长走廊偏心停车修复

修复前离线复现：0.82/0.85/0.90 m 平行走廊内，部分厘米级偏心姿态直行及停车包络安全，但中心线纠偏候选全部被拒后无条件锁定 `PASSAGE_NO_EXECUTABLE_CORRECTION`。修复后无可行纠偏时继续执行完整直行、门洞穿越、通道终点和权限检查；姿态稳定且直行仍安全时不反复停车重试，横向偏移变化 5 mm 或朝向变化达到最小有效修正角时重评。没有放宽硬余量，没有新增倒车。

新增 10 项回归：三种宽度双侧偏心连续前进、新障碍停车、朝向漂移重评、通道终点拒绝、短前进安全但门洞出口受阻仍拒绝。完整回归 134 passed（5.03 s）。

隔离 ROS domain 97 / IGN partition wheelchair_validation 的真实 Gazebo 验证：

```bash
source scripts/env.sh
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=wheelchair_validation /usr/bin/python3 scripts/validate_shared.py --scenario passage --test-scene passage_090 --heading 0 --lateral .05
```

结果 PASS：前进 2.117238 m，接触 0，最小实测间距 0.100 m > 硬余量 0.092806 m，最大规划耗时 12.12 ms，输出档位仅 STOP/FORWARD。证据：artifacts/result-corridor-offcenter.json、trace-corridor-offcenter.json、gazebo-corridor-offcenter.log。当前用户卡点没有运行中的位姿/诊断记录，因此此结果证明上述可复现缺陷已修复，不代表确认了用户每一种卡住情况；盲端无掉头空间仍需后续倒车脱困功能。

## 2026-10-13 实际走廊出口靠墙脱困

本次从运行中的仿真读取精确卡点 `(10.906735444657267, 0.873704798336599, -0.039533730318275256)`；日志先报 `PASSAGE_UNCERTAIN_BLOCKED`，之后反复 `BLOCKED_TURN`。默认 0.30 s 左转不安全，0.10 s 左转有安全空间，右转仍不安全。原始记录：artifacts/stuck-exit-live.json、stuck-exit-history.json。

在 corridor_assist 中增加人工同方向短转向，使用结构化 `DiscreteAction.max_run_ms` 指定单次 RUNNING 上限；执行器按本地时间强制结束，续包不能延长或撤销上限，须经 STOP 才能重新起转。预测保留调度、传输和制动余量，并使用带时间戳的 STARTING/STOP 反馈界定可能起转时间；没有减小车体或硬安全间距。每次脉冲等待新停车阶段及传输/停稳窗口后重评；人工微调不套用通道评分的最小纠偏收益阈值。到达可转边界后返回 `USER_TURN_LIMIT_REACHED` 并保持 STOP，可直接选择 W；没有自动替换成人工未请求的反向或前进。

新增测试覆盖实际卡点、危险反向拒绝、指令取消、启动/停稳/超时、未知边界、新鲜与过期执行反馈、执行器时限不能被续包绕过、受限预测及硬边界。接口和控制包 colcon 构建通过；完整回归 145 passed（6.13 s）。

实测命令：`ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=wheelchair_validation /usr/bin/python3 scripts/validate_shared.py --scenario exit_recovery`（先 source scripts/env.sh）。验收期间暂时暂停用户原仿真进程以避免双办公室实例竞争资源，结束后恢复，原车位姿保留。双实例开发试跑出现过 EXECUTOR_STALE / ACTION_STALE，保护机制保留，没有放宽超时。

最终原位重放 PASS：同方向左微转 0.041886 rad（约 2.40°），朝向从 -0.039534 修正至 0.002352 rad，随后直接 W 前进 1.514627 m；接触 0，最小间距 0.102201 m > 硬余量 0.092806 m，最大计算 17.85 ms。证据：artifacts/result-exit_recovery.json、trace-exit_recovery.json、gazebo-exit_recovery.log。此为明确人工 A→W 的原位脱困验证，未声称 W 自动选向或具有倒车能力。

当前用户窗口已同步重载执行器和规划器，保留原位姿；新进程由 artifacts/reload-exit-control.py 监管，原 launch 退出时一并结束。重载前后只读记录：artifacts/exit-reload-before.json、exit-reload-after.json。当前停稳且 READY_TO_REARM，需用户在键盘按 R 后选 A，再在微转停稳后选 W。


## 2026-10-14 意图确认与有限任务交付

按 ../projects/WHEELCHAIR_INTENT_SHARED_CONTROL_IMPLEMENTATION_PLAN.md 实施，记录见 ../projects/WHEELCHAIR_INTENT_SHARED_CONTROL_IMPLEMENTATION.md。新增候选/确认事务、结构化事件/ACK/源健康、独立任务管理、任务参考锁定和有限转角；旧 gaze_joystick 保留。

构建 wheelchair_interfaces、eye_shared_control、wheelchair_control 通过。最终完整回归 **278 passed in 7.37 s**（含 EYETLE 反馈解析/显示测试）；汇总 artifacts/intent-validation-summary.json。

最终任务验收为 **17/17 PASS**，合成眼动为 **6/6 PASS**，机器人接触均为0。任务结果逐案保留，正确拒绝不当作通行成功。默认前进最多3m/10s，包含观测年龄、管理器与下层动作失效后制动余量，实际提前停车；重复确认事件、子动作和心跳不会重置预算。

| 项目 | 最终实测/判据 | 记录 |
|---|---|---|
| 默认有限前进 | 前进2.759m，约9.00s接受→终止，停稳保持 | artifacts/intent-tasks/continue_budget/result.json |
| 有限左转 | 目标15°，实测13.161°，在2°容差内；使用100/200ms限时脉冲，完成后不前进 | artifacts/intent-tasks/heading/result.json |
| 任务替换 | 先停车，锁定新停稳朝向后执行右转，使用100/200/300ms时限 | artifacts/intent-tasks/replace/result.json |
| 3秒时间预算 | 实际前进0.468m后TASK_TIME_LIMIT停稳 | artifacts/intent-tasks/time_budget/result.json |
| 5°偏角走廊 | 任务窗口内自动对正至-0.111°并前进1.206m，随后取消停稳；未宣称每次一定走完3m | artifacts/intent-tasks/corridor_align/result.json |
| 正常门洞 | 前进2.073m，PASSAGE_EXIT_CONFIRM_DIRECTION后保持停车 | artifacts/intent-tasks/door_pass/result.json |
| 分叉/门洞出口堵塞 | 分别PASSAGE_END_OR_BRANCH、PASSAGE_BLOCKED_COLLISION_OR_UNKNOWN | artifacts/intent-tasks/{branch,blocked_door}/result.json |
| 取消、失效、四节点中断 | 都按各自看门狗停车，健康恢复不自动运动 | artifacts/intent-tasks/result.json |
| 事件重放、管理器重启 | 不重复转向、不恢复旧任务 | artifacts/intent-tasks/{replay,manager_restart}/result.json |
| 合成眼动链路 | 无候选/未确认/单眨眼无运动，确认有限任务，健康扫视保持任务，长闭眼/invalid取消 | artifacts/intent-eye/result.json |

延迟统计见 artifacts/intent-eye/latency.json（同机单调时间，当前合成脚本小样本，非人群统计）：接收→预览 n=19，P50/P95/max=0.564/0.744/0.911 ms；确认→ACK n=3，P50/P95/max=429.589/438.205/439.162 ms；ACK→实际起动 n=3，P50/P95/max=299.998/481.260/501.401 ms；内部停车转换 n=3，P50/P95/max=400.031/400.032/400.032 ms；闭眼起点→实际停稳 n=1，P50/P95/max=1616.535/1616.535/1616.535 ms。停止包含650ms识别阈值及实际制动，不称瞬时急停。旧Float32输入没有采集时间，source_to_preview为null。

真实双摄采集150帧、收到151条源时间戳消息（含退出invalid），约15.71FPS，加载两眼标定；有效双眼样本0，确认运动事件0。修复同一帧退出通知重复映射wall时间造成的亚微秒回退，保留原读帧时间戳。结果 artifacts/intent-camera/{result,tracker,observations}.json 只证明新源时间戳消息→健康适配→仿真任务状态链路，不证明真人驾驶成功。

旧交互实际Gazebo eye案例与原办公室出口exit_recovery也重新验证PASS；后者前进1.509m、接触0，见artifacts/result-eye.json和result-exit_recovery.json。

实现期间对候选丢失CANCEL、替换跳过STOP、源样本被心跳变新、时钟回跳后的odom水位、任务ID跨输入会话冲突、预测角度越权、限时脉冲制动中错误匀速等问题增加了回归。对硬25°边界不可达情形单元验证明确阻挡，未放宽角度、TTL或硬安全间距。仿真放置服务用有界等待并核对实际odom；测试会显式报告初始姿态设置失败。

真人佩戴误提交/漏确认统计、阅读观察干扰、主观负担与两模式对照未完成；模板见 docs/INTENT_EXPERIMENTS.md。真实轮椅、固件、三维注视选物和实时障碍融合不在本次交付。


运行环境收尾：2026-10-14 10:31旧TurtleBot目录被移入回收站，轮椅依赖的262MB Gazebo runtime已单独复制到本工作空间vendor/runtime并重定位CLI配置，没有恢复旧TurtleBot工程。ScanPoints、interfaces、sim、control、eye包均重建通过；scripts/env.sh只加载/opt ROS、本runtime和本工作空间local_setup。迁移后本节17任务+6眼动+278测试+双摄传输均重新通过。每个任务案例使用独立随机IGN_PARTITION，清理等待整个自有进程组退出；最终无残留仿真/控制进程。


## 2026-10-14：directional_intent 键盘先行避障控制

实现独立方向事件模式，保留旧遥杆及确认式模式。完成键盘事件/健康、幂等与替换、软件停稳恢复、地图入口几何身份、最近可达入口选择、完整转入及事件消耗后的巡航。参数、接口与启动方法见 [方向意图说明](docs/DIRECTIONAL_INTENT.md)。

- 五个 ROS 包 `colcon build --symlink-install --base-paths src` 构建通过。
- 旧 `intent_confirm` 的 Gazebo `heading` 兼容回归通过：实际转角 13.84°（15°目标的 ±2°容差内）、物理接触 0；见 [旧模式回归结果](artifacts/directional-legacy/result.json)。
- 全部单元/接口回归：**294 passed（13.35 s）**，包含 25 项新增方向意图测试；执行命令如下。
- Gazebo：**22/22 场景最终通过，物理接触 0**。最终汇总包含修正后针对受影响场景的复测，不表示开发过程从未失败。此前的超时与重复停车容差问题记录于 `artifacts/directional/development-failures.json`。
- 本次场景最低实际轮廓间距 **0.1432 m**；最大规划计算耗时 **44.61 ms**，原 50 ms 超时保护保留。这些是本机本次观测，不是全环境或任意负载保证。

```bash
cd ~/wheelchair_shared_ws
source scripts/env.sh
/usr/bin/python3 -m pytest tests src/eye_shared_control/test src/wheelchair_sim/test/test_scene_geometry.py -q
/usr/bin/python3 scripts/validate_directional.py --case all
```

| 检查组 | 场景与结果 |
|---|---|
| 入口任务 | early_left、near_left、right：先接近再转入，整车通过后巡航；PASS |
| 入口选择 | two_left、blocked_first：最近可达入口，锁定后不切换；PASS |
| 搜索边界 | no_entry、behind_entry、narrow_turn：无入口/身后入口/不可执行旋转不盲目转向；PASS |
| 事件语义 | open_area、repeat、replace、w_during_turn：一次偏转、重复幂等、反向停稳替换、W 不重置；PASS |
| 停车 | stop、stop_turn、stop_traverse：接近、旋转、穿门阶段 STOP 清除授权，A 不恢复；PASS |
| 巡航 | branch、straight_door：无唯一延续停车，直行过门后继续；PASS |
| 失效 | input_exit、manager_exit、planner_exit、executor_exit：分别中断输入健康/进程，实际停稳；PASS |
| 真实键盘进程 | keyboard_exit：伪终端运行正式键盘脚本，W 启动、A 选入口、X 退出并实际停车；PASS |

[完整结果 JSON](artifacts/directional/result.json)；[实际轨迹图](artifacts/directional/trajectories.png)。每场景目录保存 Gazebo 日志和逐事件/逐周期 trace，结果含实际路径长度、动作切换/停车次数、最低预测与实际间距、输入事件数、入口 ID 及停止原因。输入事件计数包括验收脚本的启动前 A 拒绝检查和收尾 STOP，不等同于真人操作负担。`keyboard_exit` 的 `input_events` 仅计验收器收尾事件，正式键盘进程另外实际执行了 W/A/X。

开发修正：入口转向将 2°终点容差与额外 5°有限预测范围分离，制动触发使用终点允许区间；旧模式范围和安全余量保持不变。名义入口预检保留位置容差，实际停稳位姿复核不再重复叠加一次容差。直行门洞在整车通过后重建巡航参考，不沿用旧版出口确认停车。

限制：当前是正交直走廊/矩形开口的保守运动基元规划；任意曲线走廊、全局绕路、倒车及实时行人融合不在本轮实现内。键盘阶段完成，眼动事件适配及真人主观连续性/误事件评估尚未完成。

## 2026-10-16 自动防卡与有限脱困

- 完整单元/接口回归 **364 passed**，`wheelchair_control` 构建成功。
- 办公室最终四项仿真通过：巡航/停止/继续、停车盲区转向拒绝、自动脱困尝试耗尽后停车、脱困运动中 STOP 中断。
- 物理接触 0，无运行期 PLANNING_TIMEOUT_FAULT / ACTION_STALE，最大计算耗时 32.61 ms。
- 新增当前扫描可观测约束，旧大厅原地转向“完整地图空旷即允许”的假设被收紧；转向扫入未知后方时拒绝。无倒车动作，无人工确认，无固定速度修改。
- 最终证据及源码哈希：`artifacts/automatic-recovery-20261016/verification.json`；操作说明：`docs/AUTOMATIC_RECOVERY.md`。阶段性失败保留，仅 final-* 目录为最终验收。

## 2026-10-16 可见区域误判修复

撤销人为 180° 掩码，以实际扫描回波保留可见侧后方，仍拒绝实际遮挡后方的扫掠。整体凸包拒绝改为逐段扫掠复核，保留采样间旋转余量。

完整回归 **374 passed**、控制包构建成功、四项办公室仿真 PASS，碰撞 0、无运行期计算超时或 ACTION_STALE，最大控制计算耗时 31.60 ms。证据：`artifacts/turn-observability-fix-20261016/verification.json`。现有座椅后方真实盲区并未消除，不声称所有原地转向恢复。

## 2026-10-16 撤销普通导航额外整路预检

普通新路径/追加路径不再要求整段当前可观测以及远端全速停车预检；实际路径服务不再附加完整门洞接近—转向—离开预检。每次实际动作的碰撞、停车轨迹和真实未知区域检查保留。修复实际动作拒绝后切换脱困时引用已清空路径的问题，并避免相同位姿的重复几何计算。

完整回归 **378 passed**、控制包构建成功、四项办公室仿真 PASS；碰撞 0，无运行期计算超时或 ACTION_STALE，最大计算耗时 40.25 ms。证据：`artifacts/remove-route-preflight-20261016/verification.json`。真实后方盲区导致的原地转向拒绝仍存在，验收未把拒绝当作转向成功。

## 2026-10-16 最终恢复：撤回全部本轮脱困相关改动

按用户要求，当前源码与配置逐文件恢复到 `artifacts/recovery-fix-20261016/control-before/`，接口与 `interfaces-before/` 一致，保留原大厅 A/D/U 修复。自动脱困、额外可观测性限制、额外整路预检及对应测试已归档撤回。以上本日脱困相关实现与验证均为历史，不代表当前运行版本。

恢复后 **350 项回归通过**，接口和控制包重新构建成功；已验证生成接口不再包含 REVERSE/RECOVER_CONFIRMED 常量，运行环境不再加载新增模块。

真实办公室仿真：停车左转、停车右转、停车 180° 掉头全部 PASS，均实际完成转向、停留后才由新 W 前进，STOP 能停车；碰撞 0，无运行期计算超时或 ACTION_STALE。最终汇总及源码哈希：`artifacts/restore-hall-turn-20261016/verification.json`。

## 2026-10-16 受阻转向的前方位置调整试验

在恢复的大厅版本上增加限定失败分支：原地转向受阻后，停车并通过现有路径服务检查前方 20/40/60 cm 的前进＋转向组合。默认完整轮廓、速度、安全余量及超时不变。无解停车并在键盘显示需要协助。

**363 项回归通过**；控制/仿真包构建成功；九项仿真/键盘验收 PASS、碰撞 0，无活动任务计算超时或 ACTION_STALE，最大计算耗时 45.39 ms。组合左右转分别前移约 36.7/36.6 cm，组合掉头前移约 54.4 cm；原办公室 A/D/U 均直接原地完成，未触发位置调整。

说明：`docs/TURN_ADJUSTMENT.md`；证据：`artifacts/turn-adjustment-20261016/verification.json`。未恢复此前撤回的通用脱困、额外可观测性掩码或普通路线提前防卡检查。

## 2026-10-16 Smac State Lattice Humble 可选接口试验

新增独立 `/wheelchair/smac/compute_discrete_path` 及健康话题，通过显式 `path_planner:=smac_lattice` 选择；默认仍为原离散后端。使用官方 Humble 1.1.20 插件和自定义前进/原地旋转基元；直达/锁定路线精确验证，需要绕行时才调用 Smac。

**368 项回归通过**，规划/控制包构建成功；独立服务 11/11 验收通过。新后端仿真功能 6/7 通过，实际 `path_obstacle` 绕障返回 `PATH_NO_EXECUTABLE_PATH`，明确未通过；原后端两项对照通过。所有最终仿真接触计数为零。部分 Python 节点在 SIGINT 关闭阶段记录回调转换异常，原后端对照也出现，不能把功能通过写成整套关闭流程完全干净。

默认后端不切换，新接口仅供实验。总结：`../projects/WHEELCHAIR_SMAC_STATE_LATTICE_HUMBLE_SUMMARY.md`；证据：`artifacts/smac-lattice-20261016/verification.json`。
