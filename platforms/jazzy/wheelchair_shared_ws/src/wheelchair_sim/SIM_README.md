# Gazebo Fortress 通用轮椅仿真

此包只提供真实物理仿真、传感器和已知场景障碍地图，不包含眼动、规划器或动作执行器。模型为自行生成的**通用轮椅占位模型**，不是 WHILL 厂家 CAD，不代表实机质量/制动数据。完整三维碰撞包含座椅、靠背、扶手、脚踏、差速轮及低摩擦支撑球轮；默认最大平面轮廓 x=[-0.45,0.55]、y=[-0.35,0.35] m。支撑球轮简化真实万向轮，不用于验证侧翻或坡道。

```bash
source ~/wheelchair_shared_ws/scripts/env.sh
source ~/wheelchair_shared_ws/install/setup.bash
export ROS_DOMAIN_ID=96 IGN_PARTITION=wheelchair_sim_dev
ros2 launch wheelchair_sim simulation.launch.py scene:=straight_wall gui:=false
```

使用既有本地 rootless Gazebo 6.18，模型由纯 SDF 生成，无在线模型下载。软件渲染/无桌面环境可用 `xvfb-run -a` 包装启动命令；GPU 激光需要渲染上下文。

接口：

- `/wheelchair/sim/cmd_vel`: ROS Twist 单向桥接到同名独立 Gazebo 话题，只由底盘独立看门狗发布；执行器发布 `/wheelchair/sim/drive`，看门狗 20 ms 周期按单调时间检测，超过 0.20 s 无输入即发零。
- `/wheelchair/odom`: 30 Hz `nav_msgs/Odometry`，Gazebo **OdometryPublisher 读取物理模型位姿**，非轮速或指令积分。frame=`odom`, child=`base_link`。
- `/wheelchair/scan`: 15 Hz、720 点、360°、0.06–12 m 激光；frame=`base_scan`，静态 TF 平移 (0.10,0,1.15) m。低障碍不可由此安装高度可靠检测，已知地图包含其几何。
- `/clock`、动态 odom TF、静态雷达 TF。
- `/wheelchair/sim/contacts`: 各障碍 contact sensor 汇入同一 `ros_gz_interfaces/Contacts`；碰撞判断应筛选轮椅与障碍接触。
- `/wheelchair/obstacle_map`: 20 Hz `ObstacleMap`，仿真时间戳、启动 UUID 与递增序号，坐标系 odom。

地图与 SDF 障碍由 `scenes.py` 唯一数据源生成；机器人自身、地面排除，碰撞高度内所有障碍包含在地图中；边界矩形 [-4,6]×[-4,4] 外部必须按未知区域拒绝。`complete=true` 表示该边界内的仿真完整碰撞表示。**这是 simulation-only oracle，不能接实机或宣称点云/雷达已实现三维障碍重建。**

场景：`straight_wall` 直行墙体，`rotation_wall` 贴墙旋转，`narrow` 窄通道，`slight_offset` 偏置低障碍，`both_blocked` 前方及左右阻挡，`dynamic` 横向运动障碍。动态障碍通过 Gazebo set_pose 服务更新；地图预先持续保守占用其完整运动走廊，避免服务与 ROS 地图非原子更新留下空窗，代价是不会放行暂时空出的走廊。动态模型是离散位置更新而非受力行人，不能据此声称验证了不可避免碰撞的物理保证。

底盘 DiffDrive 具有轮速加速度上限，启动/停止授权、固定前进 0.372 m/s、左转 0.389 rad/s、右转 -0.403 rad/s 及丢包停车由独立离散执行器负责。独立底盘看门狗覆盖执行器退出后的超时停车；看门狗自身进程死亡仍不具有 Gazebo 插件内失联保障。仅在室内平整无落差地面测试。

## 已执行验证（2026-10-12）

- `PYTHONPATH=src/wheelchair_sim python3 -m pytest -q src/wheelchair_sim/test`：2 项几何/oracle 及真实 odom 配置回归通过。
- `ROS_DOMAIN_ID=96 IGN_PARTITION=wheelchair_sim_dev xvfb-run -a ros2 launch wheelchair_sim simulation.launch.py gui:=false scene:=straight_wall` 实际启动成功。
- 同域运行 `python3 src/wheelchair_sim/test/ros_smoke.py`：真实 odom frame=odom/child=base_link，scan frame=base_scan、720 点，地图 5 障碍，全部通过。
- 独立看门狗物理测试：drive 发布固定前进 1 s 后停发，实际位移 0.4382 m，最后实际速度 0；最后发布循环结束后 0.164 s 首次观测零指令（定时器此前已累计输入年龄）。
- **接触检测阳性对照**：在独立开发域绕过规划器持续发布前进以故意撞墙，收到 5090 个 contact 条目，物理 x 被墙约束在 1.3500 m。这只证明完整实体碰撞和 contact 桥接生效，不是防碰撞通过记录；安全规划测试应由上层另行执行。
